from dataclasses import asdict, replace
import csv
import json
import logging
import math
from types import SimpleNamespace as NS

import pytest

import Run_DT_Risk_V2_IDM as runner
import idm_controller
from dt_risk_common import ActorState, actor_state_fieldnames
from idm_snapshot import read_snapshot, resolve_parameters
from idm_model import IDMParameters, compute_desired_gap, compute_idm_acceleration
from idm_leader import LaneVehicle, find_leader, trace_lane
from idm_controller import VehicleController, ControlConfig, IDMFleetController


@pytest.fixture
def parameters():
    return IDMParameters(desired_speed_mps=5.0)


def test_free_road_and_equilibrium(parameters):
    assert compute_idm_acceleration(3.0, parameters) > 0
    assert compute_idm_acceleration(5.0, parameters) == pytest.approx(0)
    assert compute_idm_acceleration(6.0, parameters) < 0
    assert compute_idm_acceleration(0.0, parameters) == parameters.max_accel_mps2


def test_closing_speed_and_headway(parameters):
    assert compute_idm_acceleration(5, parameters, 8, 2) < 0
    gaps = [compute_desired_gap(4, 0, replace(parameters, desired_time_headway_s=t))
            for t in [0.8, 1.5, 2.5]]
    assert gaps == sorted(gaps) and len(set(gaps)) == 3
    assert compute_desired_gap(0, 0, parameters) == parameters.minimum_gap_m
    assert compute_desired_gap(2, -100, parameters) == parameters.minimum_gap_m


@pytest.mark.parametrize("gap", [0, -1, 1e-300, 0.001, 0.1])
def test_tiny_gap_is_finite_and_bounded(parameters, gap):
    acceleration = compute_idm_acceleration(5, parameters, gap, 0)
    assert math.isfinite(acceleration)
    assert acceleration == -parameters.max_decel_mps2


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, -1])
def test_invalid_speed_rejected(parameters, value):
    with pytest.raises(ValueError):
        compute_idm_acceleration(value, parameters)


def test_nonfinite_gap_rejected_even_at_extreme_speed(parameters):
    with pytest.raises(ValueError):
        compute_idm_acceleration(1e300, parameters, math.nan, 0)
    assert compute_idm_acceleration(1e300, parameters) == -parameters.max_decel_mps2


def test_control_config_normalizes_numeric_values(parameters):
    config = ControlConfig(speed_kp="3.0")
    controller = VehicleController(parameters, 0.05, config)
    assert controller.command(3, 1, 0)[1] > 0
    with pytest.raises(ValueError):
        controller.command(3, math.nan, 0)


@pytest.mark.parametrize("field,value", [("desired_speed_mps", 0), ("max_accel_mps2", 0),
    ("comfortable_decel_mps2", -1), ("max_decel_mps2", math.inf), ("desired_time_headway_s", math.nan)])
def test_invalid_parameters(parameters, field, value):
    with pytest.raises(ValueError):
        replace(parameters, **{field: value})


def old_row():
    return asdict(ActorState("snapshot", 42, 2.0, 7, "follower", "autopilot", "vehicle.test",
                            0, 0, 0.3, 0, 0, 0, 3, 0, 0, 3, True))


def test_old_csv_json_and_v1_schema_unchanged(tmp_path):
    row = old_row()
    csv_path = tmp_path / "old.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=actor_state_fieldnames())
        writer.writeheader()
        writer.writerow(row)
    restored = read_snapshot(csv_path)
    assert restored[0]["desired_time_headway_s"] is None
    assert "desired_time_headway_s" not in actor_state_fieldnames()
    json_path = tmp_path / "snapshot.json"
    json_path.write_text(json.dumps({"actors": [row]}), encoding="utf-8")
    assert read_snapshot(json_path) == restored
    runner.save_snapshot(tmp_path / "v2.csv", restored)
    assert read_snapshot(tmp_path / "v2.json") == restored
    assert read_snapshot(tmp_path / "v2.csv") == restored


def test_trait_precedence_and_observed_speed():
    args = runner.parse_args(["--mode", "capture", "--default-idm-headway", "2.5"])
    row = old_row()
    p = resolve_parameters(row, args, {})
    assert p.desired_speed_mps == 3
    assert p.desired_time_headway_s == 2.5
    row.update(desired_time_headway_s=0.8, desired_speed_mps=6)
    assert resolve_parameters(row, args, {}).desired_time_headway_s == 0.8
    traits = {"follower": {"desired_time_headway_s": 1.5, "desired_speed_mps": 7}}
    assert resolve_parameters(row, args, traits).desired_speed_mps == 7
    assert resolve_parameters(row, args, traits).desired_time_headway_s == 1.5
    row.update(desired_time_headway_s=None, desired_speed_mps=None, speed_mps=0)
    assert resolve_parameters(row, args, {}).desired_speed_mps == args.idm_min_desired_speed_mps
    args.idm_desired_speed_mps = 10
    assert resolve_parameters(row, args, {}).desired_speed_mps == 10


@pytest.mark.parametrize("changes", [{"speed_mps": "nan"}, {"desired_time_headway_s": -1}, {"x": "inf"}])
def test_bad_snapshot_rejected(tmp_path, changes):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps([dict(old_row(), **changes)]), encoding="utf-8")
    with pytest.raises(ValueError):
        read_snapshot(path)


def test_duplicate_actor_rejected(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps([old_row(), old_row()]), encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate"):
        read_snapshot(path)


def vehicle(actor_id, x, y=0, **kwargs):
    return LaneVehicle(actor_id, x, y, kwargs.get("fx", 1), kwargs.get("fy", 0),
                       kwargs.get("lane", (1, 0, -1)), 2, kwargs.get("speed", 3),
                       kwargs.get("junction", False))


def test_leader_excludes_side_oncoming_behind_and_junction():
    follower = vehicle(1, 0)
    candidates = [vehicle(1, 0), vehicle(2, 12), vehicle(3, 8), vehicle(4, -3),
                  vehicle(5, 5, 3.5, lane=(1, 0, -2)), vehicle(6, 5, fx=-1),
                  vehicle(7, 5, junction=True), vehicle(8, 5, 3),
                  vehicle(9, 5, lane=(1, 1, -1)), vehicle(10, 5, lane=(2, 0, -1))]
    leader = find_leader(follower, candidates, [(0, 0), (20, 0)], 1.5)
    assert leader.vehicle.actor_id == 3
    assert leader.gap_m == 4
    assert find_leader(replace(follower, junction=True), candidates, [(0, 0), (20, 0)], 1.5) is None
    assert find_leader(follower, [vehicle(2, 50)], [(0, 0), (20, 0)], 1.5) is None


def test_curved_path_uses_arc_length():
    leader = find_leader(vehicle(1, 0), [vehicle(2, 10, 8, fx=0, fy=1)],
                         [(0, 0), (10, 0), (10, 10)], 1.5)
    assert leader.gap_m == pytest.approx(14)


def test_overlap_retained_as_leader():
    leader = find_leader(vehicle(1, 0), [vehicle(2, 3)], [(0, 0), (20, 0)], 1.5)
    assert leader.gap_m == -1


def test_control_filter_feedforward_bounds_and_emergency(parameters):
    controller = VehicleController(parameters, 0.05, ControlConfig())
    target, throttle, brake, steer = controller.command(3, 1.5, 100)
    assert controller.acceleration == pytest.approx(0.45)
    assert target == pytest.approx(3.0225)
    assert throttle > 0 and brake == 0 and steer == 0.75
    for _ in range(100):
        target, throttle, brake, steer = controller.command(0, -4, -100)
        assert target >= 0 and 0 <= throttle <= 1 and 0 <= brake <= 1
        assert throttle * brake == 0 and abs(steer) <= 0.75
    assert controller.command(0, -4, 0, True)[1:3] == (0, 1)


@pytest.mark.parametrize("arguments", [["--fixed-delta-seconds", "0"], ["--prediction-seconds", "nan"],
    ["--idm-max-accel-mps2", "-1"], ["--default-idm-headway", "inf"], ["--snapshot-at", "11"]])
def test_cli_rejects_invalid_config(arguments):
    with pytest.raises(SystemExit):
        runner.parse_args(["--mode", "capture", *arguments])


def test_world_settings_restored_after_control_failure():
    order = []
    original = NS(synchronous_mode=False, fixed_delta_seconds=None)
    class World:
        def get_settings(self):
            return NS(**vars(original))
        def apply_settings(self, settings):
            order.append(("settings", settings.synchronous_mode, settings.fixed_delta_seconds))
    class Owned:
        actors, sensors = [1], [2]
        def cleanup(self, logger):
            order.append(("cleanup",))
    with pytest.raises(RuntimeError):
        with runner.PredictionWorldMode(World(), 0.05, Owned(), logging.getLogger("test")):
            raise RuntimeError("control failed")
    assert order == [("settings", True, 0.05), ("cleanup",), ("settings", False, None)]


def test_sensor_tracked_before_listen_failure(monkeypatch):
    class Sensor:
        def listen(self, callback):
            raise RuntimeError("listen failed")
    sensor = Sensor()
    world = NS(spawn_actor=lambda *a, **kw: sensor,
               get_blueprint_library=lambda: NS(find=lambda _: object()))
    owned = runner.CreatedActors()
    monkeypatch.setattr(runner.baseline, "import_carla", lambda: NS(Transform=lambda: None))
    with pytest.raises(RuntimeError, match="listen failed"):
        runner.attach_collision_sensors(world, {"follower": NS(id=1)}, owned,
            [], [], 0, lambda: 0, NS(), logging.getLogger("test"))
    assert owned.sensors == [sensor]


class Waypoint:
    road_id, section_id, lane_id, lane_width = 1, 0, -1, 3.5
    is_junction = False

    def __init__(self, x=0):
        self.x = x
        self.transform = NS(location=NS(x=x, y=0, z=0))

    def next(self, distance):
        return [Waypoint(self.x + distance)]


class FakeActor:
    type_id = "vehicle.test"
    attributes = {"role_name": "autopilot"}
    bounding_box = NS(location=NS(x=0, y=0, z=0), extent=NS(x=2, y=1, z=1))

    def __init__(self, actor_id, x, speed=3):
        self.id, self.x, self.speed = actor_id, x, speed
        self.autopilot, self.commands, self.destroyed = [], [], False

    def get_transform(self):
        location = self.get_location()
        return NS(location=location, rotation=NS(yaw=0, pitch=0, roll=0),
                  get_forward_vector=lambda: NS(x=1, y=0, z=0),
                  transform=lambda local: NS(x=self.x + local.x, y=local.y, z=local.z))

    def get_location(self):
        return NS(x=self.x, y=0, z=0.3)

    def get_velocity(self):
        return NS(x=self.speed, y=0, z=0)

    def set_autopilot(self, enabled, *args):
        self.autopilot.append(enabled)

    def apply_control(self, command):
        self.commands.append(command)

    def destroy(self):
        self.destroyed = True
        return True


class FakeWorld:
    def __init__(self, actors):
        self.actors = actors
        self.settings = NS(synchronous_mode=False, fixed_delta_seconds=None)
        self.applied = []
        self.frame = 0
        self.map = NS(name="Town10HD_Opt", get_waypoint=lambda location, **kw: Waypoint(location.x))

    def get_map(self):
        return self.map

    def get_actors(self):
        return NS(filter=lambda _: self.actors)

    def get_settings(self):
        return NS(**vars(self.settings))

    def apply_settings(self, settings):
        self.settings = settings
        self.applied.append(settings)

    def tick(self):
        self.frame += 1
        return self.frame

    def get_snapshot(self):
        return NS(frame=self.frame, timestamp=NS(elapsed_seconds=self.frame * 0.05))


def test_fleet_free_road_leader_and_logged_fallback(monkeypatch, parameters):
    follower = FakeActor(1, 0)
    external = FakeActor(2, 10, 2)
    world = FakeWorld([follower])
    monkeypatch.setattr(idm_controller, "import_carla", lambda: NS(VehicleControl=lambda **kw: NS(**kw)))
    fleet = IDMFleetController(world, {"follower": follower}, {"follower": parameters},
                              0.05, ControlConfig(), logging.getLogger("test"))
    row = fleet.step(0)[0]
    assert row["leader_id"] == "" and row["idm_acceleration_mps2"] > 0
    assert row["throttle"] > 0 and row["brake"] == 0
    world.actors.append(external)
    row = fleet.step(0.05)[0]
    assert row["leader_id"] == 2 and row["gap_m"] == 6
    assert row["relative_speed_mps"] == 1
    waypoint = Waypoint()
    waypoint.is_junction = True
    world.map.get_waypoint = lambda *args, **kw: waypoint
    row = fleet.step(0.10)[0]
    assert row["fallback_reason"] == "junction" and row["brake"] == 1
    assert row["throttle"] == 0 and row["simulation_time"] == 0.10
    assert follower.autopilot == [False] and external.autopilot == []


@pytest.mark.parametrize("kind", ["fork", "end", "junction", "transition"])
def test_route_boundary_fallback(kind):
    waypoint = Waypoint()
    candidate = Waypoint(2)
    if kind == "junction":
        candidate.is_junction = True
    if kind == "transition":
        candidate.section_id = 1
    waypoint.next = lambda _: [] if kind == "end" else [candidate, candidate] if kind == "fork" else [candidate]
    path, reason = trace_lane(waypoint, 80)
    assert path == [(0, 0)]
    assert reason == {"fork": "ambiguous_route", "end": "route_end", "junction": "junction_ahead",
                      "transition": "lane_transition"}[kind]


@pytest.mark.parametrize("fail_control", [False, True])
def test_prediction_ticks_logs_and_cleanup(monkeypatch, tmp_path, fail_control):
    args = runner.parse_args(["--mode", "predict", "--snapshot", "unused.csv", "--prediction-seconds", "0.1"])
    actor = FakeActor(1, 0)
    world = FakeWorld([actor])
    monkeypatch.setattr(runner, "connect_client", lambda *a: NS(get_world=lambda: world))
    def spawn(world, rows, created, logger):
        created.add_actor(actor)
        return {"follower": actor}
    monkeypatch.setattr(runner, "spawn_from_snapshot", spawn)
    monkeypatch.setattr(runner, "attach_collision_sensors", lambda *a: None)
    monkeypatch.setattr(idm_controller, "import_carla", lambda: NS(VehicleControl=lambda **kw: NS(**kw)))
    if fail_control:
        actor.apply_control = lambda _: (_ for _ in ()).throw(RuntimeError("control failed"))
    snapshot = tmp_path / "snapshot.csv"
    runner.save_snapshot(snapshot, read_compatible_row())
    if fail_control:
        with pytest.raises(RuntimeError, match="control failed"):
            runner.run_prediction(args, tmp_path, snapshot, logging.getLogger("test"))
    else:
        summary = runner.run_prediction(args, tmp_path, snapshot, logging.getLogger("test"))
        assert summary["reconstructed_actor_count"] == 1
        with (tmp_path / "idm_states.csv").open(encoding="utf-8-sig") as stream:
            rows = list(csv.DictReader(stream))
        assert [float(row["simulation_time"]) for row in rows] == pytest.approx([0, 0.05])
        assert world.frame == 3 and len(actor.commands) == 2
        with (tmp_path / "prediction_states.csv").open(encoding="utf-8-sig") as stream:
            rows = list(csv.DictReader(stream))
        assert all(row["autopilot_enabled"] == "False" for row in rows)
    assert actor.destroyed and actor.autopilot == [False]
    assert world.settings.synchronous_mode is False and world.settings.fixed_delta_seconds is None


def read_compatible_row():
    return [dict(old_row(), desired_time_headway_s=None, desired_speed_mps=None)]


def test_tm_dispatch_uses_v1_and_common_risk_detector(monkeypatch, tmp_path):
    args = runner.parse_args(["--mode", "predict", "--snapshot", "unused.csv", "--prediction-controller", "tm"])
    snapshot = tmp_path / "snapshot.csv"
    runner.save_snapshot(snapshot, read_compatible_row())
    received = []
    monkeypatch.setattr(runner.baseline, "run_prediction", lambda a, d, s, l: received.append(read_snapshot(s)) or {"tm": True})
    assert runner.run_prediction(args, tmp_path, snapshot, logging.getLogger("test")) == {"tm": True}
    assert received[0][0]["vx"] == 3
    assert runner.detect_near_misses is runner.baseline.detect_near_misses


def test_headway_changes_following_equilibrium(parameters):
    final_gaps = []
    leader_speed, dt = 12 / 3.6, 0.05
    for headway in [0.8, 1.5, 2.5]:
        p = replace(parameters, desired_time_headway_s=headway)
        v, gap = 5.0, 30.0
        for _ in range(2400):
            acceleration = compute_idm_acceleration(v, p, gap, leader_speed)
            next_v = max(0.0, v + acceleration * dt)
            gap += (leader_speed - (v + next_v) / 2) * dt
            v = next_v
        expected = (p.minimum_gap_m + leader_speed * headway) / math.sqrt(1 - (leader_speed / p.desired_speed_mps) ** 4)
        assert v == pytest.approx(leader_speed, abs=0.01)
        assert gap == pytest.approx(expected, abs=0.03)
        final_gaps.append(gap)
    assert final_gaps[0] < final_gaps[1] < final_gaps[2]
