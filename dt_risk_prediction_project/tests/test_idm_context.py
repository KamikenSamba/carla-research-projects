import csv
import json
import logging
import math
from types import SimpleNamespace as NS

import pytest

import Run_DT_Risk_V1 as baseline
import Run_DT_Risk_V2_IDM as runner
from Estimate_IDM_T import main as fit_main
from dt_risk_common import CsvWriter, actor_state_fieldnames, actor_state_from_actor, to_row
from idm_context import FollowingContextCollector
from idm_history import estimate_headway, read_history
from idm_model import IDMParameters
from idm_snapshot import load_traits


def vector(x=0, y=0, z=0):
    return NS(x=x, y=y, z=z)


class Transform:
    def __init__(self, x=0, y=0, yaw=0):
        self.location = vector(x, y)
        self.rotation = NS(yaw=yaw, roll=0, pitch=0)

    def get_forward_vector(self):
        angle = math.radians(self.rotation.yaw)
        return vector(math.cos(angle), math.sin(angle))

    def transform(self, local):
        forward = self.get_forward_vector()
        return vector(self.location.x + forward.x * local.x - forward.y * local.y,
                      self.location.y + forward.y * local.x + forward.x * local.y,
                      local.z)


class Actor:
    type_id = "vehicle.test"

    def __init__(self, actor_id, x, role, y=0, yaw=0, speed=8):
        self.id, self.x, self.y, self.yaw, self.speed = actor_id, x, y, yaw, speed
        self.attributes = {"role_name": role}
        self.bounding_box = NS(location=vector(), extent=vector(2, 1, 1),
                               rotation=NS(roll=0, pitch=0, yaw=0))
        self.at_light = False
        self.commands = []

    def get_transform(self):
        return Transform(self.x, self.y, self.yaw)

    def get_velocity(self):
        forward = self.get_transform().get_forward_vector()
        return vector(self.speed * forward.x, self.speed * forward.y)

    def is_at_traffic_light(self):
        return self.at_light

    def apply_control(self, *args):
        raise AssertionError("observer must not control")

    def set_autopilot(self, *args):
        raise AssertionError("observer must not register/unregister autopilot")

    def destroy(self):
        raise AssertionError("observer must not destroy")


class Waypoint:
    road_id, section_id, lane_width = 1, 0, 3.5

    def __init__(self, world, x, lane=-1):
        self.world, self.x, self.lane_id = world, x, lane
        self.transform = Transform(x, (-lane - 1) * 3.5)
        self.is_junction = world.boundary == "inside_junction"

    def next(self, distance):
        if self.world.boundary == "end":
            return []
        candidate = Waypoint(self.world, self.x + distance, self.lane_id)
        if self.world.boundary == "junction":
            candidate.is_junction = True
        if self.world.boundary == "transition":
            candidate.section_id = 1
        return [candidate, candidate] if self.world.boundary == "fork" else [candidate]

    def get_landmarks(self, distance, stop_at_junction=False):
        self.world.landmark_queries.append(distance)
        if isinstance(self.world.landmarks, Exception):
            raise self.world.landmarks
        return self.world.landmarks


class World:
    def __init__(self, actors=None, dataset=None):
        self.actors = actors if actors is not None else [Actor(1, 0, "follower"), Actor(2, 26, "leader")]
        self.frame, self.boundary, self.landmarks = 0, "", []
        self.settings = NS(synchronous_mode=False, fixed_delta_seconds=None)
        self.applied, self.landmark_queries = [], []
        self.dataset = dataset
        self.map = NS(name="Town10HD_Opt", get_waypoint=self.waypoint)

    def waypoint(self, location, **kwargs):
        return Waypoint(self, location.x, -1 - round(location.y / 3.5))

    def get_map(self):
        return self.map

    def get_actors(self):
        return NS(filter=lambda _: list(self.actors))

    def get_settings(self):
        return NS(**vars(self.settings))

    def apply_settings(self, settings):
        self.settings = settings
        self.applied.append(settings.synchronous_mode)

    def tick(self):
        self.frame += 1
        if self.dataset:
            row = self.dataset[min(max(self.frame - 2, 0), len(self.dataset) - 1)]
            self.actors[0].x = self.frame * 0.4
            self.actors[0].speed = row["speed_mps"]
            self.actors[1].x = self.actors[0].x + row["gap_m"] + 4
            self.actors[1].speed = row["leader_speed_mps"]
        return self.frame

    def get_snapshot(self):
        return NS(frame=self.frame, timestamp=NS(elapsed_seconds=self.frame * 0.05))


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def observe(observer, world, frame):
    world.frame = frame
    rows = [to_row(actor_state_from_actor(actor, phase="capture", frame=frame,
                 sim_time_s=frame * 0.05, logical_actor_id="role:" + actor.attributes["role_name"]))
            for actor in world.actors]
    observer(world, world.get_snapshot(), world.actors, rows)
    return rows


def connect(monkeypatch, world):
    tm_calls = []
    tm = NS(set_synchronous_mode=lambda enabled: tm_calls.append(enabled))
    monkeypatch.setattr(baseline, "connect_client", lambda *args: NS(
        get_world=lambda: world, get_trafficmanager=lambda _: tm))
    return tm_calls


def test_opt_in_preserves_existing_capture_bytes_ticks_and_settings(tmp_path, monkeypatch):
    captures = []
    for enabled in [False, True]:
        directory = tmp_path / str(enabled)
        directory.mkdir()
        world = World()
        tm_calls = connect(monkeypatch, world)
        args = runner.parse_args(["--mode", "capture", "--duration", "0.15", "--snapshot-at", "0.1",
                                  *(["--collect-following-context"] if enabled else [])])
        runner.capture_current_state(args, directory, logging.getLogger("test"))
        captures.append((directory / "capture_states.csv").read_bytes())
        assert world.frame == 4 and world.applied == [True, False]
        assert world.settings.synchronous_mode is False and world.settings.fixed_delta_seconds is None
        assert tm_calls == [True, False]
        assert [a.id for a in world.actors] == [1, 2]
        if enabled:
            capture = read_csv(directory / "capture_states.csv")
            context = read_csv(directory / "following_context.csv")
            assert [(r["logical_actor_id"], r["carla_frame"]) for r in capture] == [
                (r["logical_actor_id"], r["carla_frame"]) for r in context]
            assert list(capture[0]) == actor_state_fieldnames()
            assert len(read_history(directory / "capture_states.csv", directory / "following_context.csv")) == 6
            meta = json.loads((directory / "following_context.meta.json").read_text())
            assert meta["status"] == "completed" and meta["ticks"] == 3 and meta["rows"] == 6
        else:
            assert not (directory / "following_context.csv").exists()
    assert captures[0] == captures[1]


def test_bbox_offsets_and_nearest_forward_leader(tmp_path):
    actors = [Actor(1, 0, "follower"), Actor(2, 14, "leader"), Actor(3, 30, "far"),
              Actor(4, 10, "adjacent", y=3.5), Actor(5, 8, "oncoming", yaw=180), Actor(6, -5, "behind")]
    actors[0].bounding_box.location.x = 1
    actors[1].bounding_box.location.x = -1
    world = World(actors)
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        observe(observer, world, 1)
    follower = read_csv(tmp_path / "following_context.csv")[0]
    assert follower["leader_id"] == "2" and float(follower["gap_m"]) == pytest.approx(8)
    assert follower["leader_speed_mps"] == "8.0"
    assert all(a.commands == [] for a in actors)


def test_bbox_yaw_projects_length_onto_lane(tmp_path):
    world = World([Actor(1, 0, "follower"), Actor(2, 14, "leader")])
    world.actors[0].bounding_box.rotation.yaw = 90
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        observe(observer, world, 1)
    assert float(read_csv(tmp_path / "following_context.csv")[0]["gap_m"]) == pytest.approx(11)


@pytest.mark.parametrize("boundary,reason", [("end", "route_end"), ("fork", "ambiguous_route"),
    ("junction", "junction_ahead"), ("transition", "lane_transition"), ("inside_junction", "junction")])
def test_route_exclusions(tmp_path, boundary, reason):
    world = World()
    world.boundary = boundary
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        observe(observer, world, 1)
        observe(observer, world, 2)
    row = read_csv(tmp_path / "following_context.csv")[2]
    assert row["fallback_reason"] == reason


def test_lane_change_proxy_hold_and_actor_switch_are_not_normal(tmp_path):
    world = World()
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        for frame in range(1, 46):
            if frame == 42:
                world.actors[0].y = 0.7
            if frame == 43:
                world.actors[0].y = 0
            if frame == 45:
                world.actors[0].id = 100
            observe(observer, world, frame)
    follower = [r for r in read_csv(tmp_path / "following_context.csv") if r["logical_actor_id"] == "role:follower"]
    assert follower[0]["lane_change"] == ""
    assert follower[40]["lane_change"] == "False" and follower[40]["signal_stop"] == "False"
    assert follower[41]["lane_change"] == "True" and follower[43]["lane_change"] == "True"
    assert follower[44]["lane_change"] == ""
    assert follower[44]["fallback_reason"] == "actor_identity_or_history_reset"


def test_lane_key_transition_and_disappearance_reset_history(tmp_path):
    world = World()
    leader = world.actors[1]
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        for frame in range(1, 45):
            if frame == 42:
                world.actors[0].y = 3.5
                world.actors.remove(leader)
            if frame == 44:
                world.actors.append(leader)
            observe(observer, world, frame)
    rows = read_csv(tmp_path / "following_context.csv")
    changed = next(r for r in rows if r["source_actor_id"] == "1" and r["carla_frame"] == "42")
    assert changed["lane_change"] == "True" and changed["lane_id"] == "-2"
    returned = next(r for r in rows if r["source_actor_id"] == "2" and r["carla_frame"] == "44")
    assert returned["fallback_reason"] == "actor_identity_or_history_reset"


@pytest.mark.parametrize("condition,flag,reason", [
    ("red", "True", "signal_stop"), ("landmark", "", "regulatory_landmark_unknown"),
    ("missing", "", "signal_observation_unknown"), ("rpc_error", "", "signal_observation_unknown"),
    ("invalid_flag", "", "signal_observation_unknown"),
    ("invalid_landmarks", "", "signal_observation_unknown")])
def test_signal_detection_unknowns_and_history_hold(tmp_path, condition, flag, reason):
    world = World()
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        for frame in range(1, 46):
            if frame == 43:
                if condition == "red":
                    world.actors[0].at_light = True
                elif condition == "invalid_flag":
                    world.actors[0].at_light = None
                elif condition == "invalid_landmarks":
                    world.landmarks = False
                else:
                    world.landmarks = [object()] if condition == "landmark" else None if condition == "missing" else RuntimeError("RPC")
            if frame == 44:
                world.actors[0].at_light = False
                world.landmarks = []
            observe(observer, world, frame)
    rows = [r for r in read_csv(tmp_path / "following_context.csv") if r["source_actor_id"] == "1"]
    assert rows[41]["signal_stop"] == "False"
    assert rows[42]["signal_stop"] == flag and rows[42]["fallback_reason"] == reason
    assert rows[43]["signal_stop"] == "" and rows[43]["fallback_reason"] == "signal_history_guard"


def test_positive_signal_detection_survives_missing_landmarks(tmp_path):
    world = World()
    world.actors[0].at_light, world.landmarks = True, None
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        observe(observer, world, 1)
    assert read_csv(tmp_path / "following_context.csv")[0]["signal_stop"] == "True"


def test_collected_signal_and_lane_flags_are_excluded_after_csv_join(tmp_path):
    world = World()
    capture = tmp_path / "capture_states.csv"
    with FollowingContextCollector(tmp_path, 0.05) as observer, CsvWriter(capture, actor_state_fieldnames()) as writer:
        for frame in range(1, 101):
            world.actors[0].at_light = 43 <= frame <= 50
            world.actors[0].y = 0.7 if frame >= 93 else 0
            writer.writerows(observe(observer, world, frame))
    joined = read_history(capture, tmp_path / "following_context.csv")
    result = estimate_headway([r for r in joined if r["source_actor_id"] == "1"], IDMParameters(18))
    assert result["exclusions"]["signal_stop"] == 8
    assert result["exclusions"]["lane_change"] == 8
    assert result["exclusions"]["invalid_context"] > 0
    assert not result["accepted"]


@pytest.mark.parametrize("failure", ["missing_bbox", "tilt", "speed_mismatch", "empty_bbox", "orientation_mismatch"])
def test_unobserved_candidate_prevents_normal_fleet_inference(tmp_path, failure):
    world = World()
    with FollowingContextCollector(tmp_path, 0.05) as observer:
        observe(observer, world, 1)
        if failure == "missing_bbox":
            del world.actors[1].bounding_box
        elif failure == "tilt":
            world.actors[1].bounding_box.rotation.pitch = 2
        elif failure == "empty_bbox":
            world.actors[1].bounding_box.extent.x = 0
        else:
            original = observer._sample
            def mismatch(actor, carla_map, row):
                if actor.id == 2:
                    row = dict(row, **({"yaw": 99} if failure == "orientation_mismatch" else {"speed_mps": 99}))
                return original(actor, carla_map, row)
            observer._sample = mismatch
        observe(observer, world, 2)
    rows = read_csv(tmp_path / "following_context.csv")
    assert rows[2]["fallback_reason"] == "incomplete_vehicle_observation"
    assert rows[3]["fallback_reason"] == "vehicle_observation_unknown"


@pytest.mark.parametrize("failure", ["frame", "logical_duplicate", "source_duplicate", "actor_missing", "time"])
def test_invalid_tick_keys_abort_and_preserve_failure_metadata(tmp_path, failure):
    world = World()
    with pytest.raises((ValueError, RuntimeError)):
        with FollowingContextCollector(tmp_path, 0.05) as observer:
            rows = observe(observer, world, 1)
            world.frame = 2
            for row in rows:
                row.update(carla_frame=2, sim_time_s=0.1)
            if failure == "frame":
                rows[0]["carla_frame"] = 3
            elif failure == "logical_duplicate":
                rows[1]["logical_actor_id"] = rows[0]["logical_actor_id"]
            elif failure == "source_duplicate":
                rows[1]["source_actor_id"] = rows[0]["source_actor_id"]
            elif failure == "actor_missing":
                world.actors.pop()
            else:
                rows[0]["sim_time_s"] = 99
            observer(world, world.get_snapshot(), world.actors, rows)
    meta = json.loads((tmp_path / "following_context.meta.json").read_text())
    assert meta["status"] == "failed" and meta["rows"] == 2
    assert len(read_csv(tmp_path / "following_context.csv")) == 2


def test_duplicate_frame_and_file_overwrite_are_rejected(tmp_path):
    world = World()
    with pytest.raises(ValueError, match="duplicate"):
        with FollowingContextCollector(tmp_path, 0.05) as observer:
            observe(observer, world, 1)
            observe(observer, world, 1)
    before = (tmp_path / "following_context.csv").read_bytes()
    with pytest.raises(FileExistsError):
        with FollowingContextCollector(tmp_path, 0.05):
            pass
    assert (tmp_path / "following_context.csv").read_bytes() == before


def test_world_advancing_mid_observation_fails_capture_and_restores_settings(tmp_path, monkeypatch):
    world = World()
    connect(monkeypatch, world)
    args = runner.parse_args(["--mode", "capture", "--duration", "0.15", "--collect-following-context"])
    original = world.actors[0].is_at_traffic_light
    def advance():
        world.frame += 1
        return original()
    world.actors[0].is_at_traffic_light = advance
    with pytest.raises(RuntimeError, match="advanced"):
        runner.capture_current_state(args, tmp_path, logging.getLogger("test"))
    assert world.applied == [True, False] and world.settings.synchronous_mode is False
    assert json.loads((tmp_path / "following_context.meta.json").read_text())["status"] == "failed"
    with pytest.raises(ValueError, match="incomplete or failed"):
        read_history(tmp_path / "capture_states.csv", tmp_path / "following_context.csv")


def test_capture_context_estimator_and_v2_e2e(tmp_path, monkeypatch):
    from test_idm_history import following
    world = World(dataset=following(1.8, measured=False))
    connect(monkeypatch, world)
    args = runner.parse_args(["--mode", "capture", "--duration", "12", "--snapshot-at", "10",
                              "--idm-desired-speed-mps", "18", "--collect-following-context"])
    snapshot = runner.capture_current_state(args, tmp_path, logging.getLogger("test"))
    capture, context = tmp_path / "capture_states.csv", tmp_path / "following_context.csv"
    joined = read_history(capture, context)
    assert len(joined) >= 480 and world.frame == 1 + len(joined) // 2
    assert all(r["observed_acceleration_mps2"] == "" for r in joined)
    fitted = tmp_path / "fit"
    assert fit_main(["--history", str(capture), "--context", str(context), "--snapshot", str(snapshot),
                     "--idm-desired-speed-mps", "18", "--replace-snapshot-headway",
                     "--output-dir", str(fitted)]) == 0
    traits = load_traits(fitted / "actor_traits.json")
    assert traits["role:follower"]["desired_time_headway_s"] == pytest.approx(1.8, abs=0.06)
    diagnostics = json.loads((fitted / "headway_diagnostics.json").read_text())
    follower = diagnostics["actors"]["role:follower"]
    assert follower["adopted"] and follower["valid_samples"] > 140 and follower["rmse_mps2"] < 0.1
    assert follower["future_samples_excluded"] > 0
    assert not diagnostics["actors"]["role:leader"]["adopted"]
    predict = tmp_path / "predict"
    predict.mkdir()
    prediction_args = runner.parse_args(["--mode", "predict", "--snapshot", str(snapshot),
                                         "--actor-traits", str(fitted / "actor_traits.json")])
    received = {}
    monkeypatch.setattr(runner, "run_idm_prediction", lambda a, d, s, l, p: received.update(p) or {})
    runner.run_prediction(prediction_args, predict, snapshot, logging.getLogger("test"))
    assert received["role:follower"].desired_time_headway_s == traits["role:follower"]["desired_time_headway_s"]
    # Metadata guards against accidental use of a truncated sidecar.
    lines = context.read_text(encoding="utf-8-sig").splitlines()
    context.write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="keys or metadata"):
        read_history(capture, context)


def test_predict_mode_rejects_capture_flag():
    with pytest.raises(SystemExit):
        runner.parse_args(["--mode", "predict", "--snapshot", "unused.csv", "--collect-following-context"])


def test_disabled_v2_keeps_existing_three_argument_capture_interface(tmp_path, monkeypatch):
    called = []
    from test_idm import old_row
    snapshot = tmp_path / "snapshot_states.csv"
    runner.save_snapshot(snapshot, [dict(old_row(), desired_time_headway_s=None, desired_speed_mps=None)])
    monkeypatch.setattr(baseline, "capture_current_state", lambda a, d, l: called.append(True) or snapshot)
    runner.capture_current_state(runner.parse_args(["--mode", "capture"]), tmp_path, logging.getLogger("test"))
    assert called == [True] and not (tmp_path / "following_context.csv").exists()
