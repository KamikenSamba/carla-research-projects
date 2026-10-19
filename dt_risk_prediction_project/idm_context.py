"""Read-only, tick-aligned following context. No CARLA import or tick/control calls."""
from __future__ import annotations

from collections import Counter
import csv
from dataclasses import replace
import json
import math
from time import perf_counter
from types import SimpleNamespace

from idm_history import CONTEXT_FIELDS
from idm_leader import find_leader, path_projection, sample_vehicle, trace_lane
from idm_model import finite


FIELDS = ["logical_actor_id", "carla_frame", "sim_time_s", "source_actor_id",
          *CONTEXT_FIELDS, "observed_acceleration_mps2"]
READ_ERRORS = (AttributeError, RuntimeError, TypeError, ValueError, OverflowError)


class FollowingContextCollector:
    """One capture/episode per instance and file. Unknown flags serialize blank.

    Lane-change is a conservative geometry/history proxy, not driver intention.
    Any forward landmark is treated as potentially regulatory (unknown), avoiding
    country-specific sign-type assumptions. Signal exclusion persists for 2 s.
    """
    def __init__(self, run_dir, fixed_delta_seconds, search_m=80.0):
        self.path = run_dir / "following_context.csv"
        self.meta_path = self.path.with_suffix(".meta.json")
        self.dt = finite(fixed_delta_seconds, "fixed delta", 0)
        self.search_m = finite(search_m, "search distance", 0)
        if self.dt <= 0 or self.search_m < 30:
            raise ValueError("positive dt and search distance >= 30 m required")
        self.previous, self.reasons = {}, Counter()
        self.last_frame, self.row_count, self.tick_count = None, 0, 0
        self.map = None
        self.observation_seconds, self.maximum_observation_seconds = 0.0, 0.0

    def __enter__(self):
        if self.meta_path.exists():
            raise FileExistsError(self.meta_path)
        self.file = self.path.open("x", encoding="utf-8-sig", newline="")
        try:
            self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
            self.writer.writeheader()
            self.file.flush()
            self._metadata("running")
        except BaseException:
            self.file.close()
            raise
        return self

    def _metadata(self, status, error=None):
        payload = dict(schema_version=1, status=status, rows=self.row_count,
                       ticks=self.tick_count, last_frame=self.last_frame,
                       fixed_delta_seconds=self.dt, search_m=self.search_m,
                       signal_hold_s=2.0, lane_change_hold_s=1.0,
                       lane_warmup_s=0.3, reasons=dict(self.reasons), error=error)
        payload.update(observation_seconds=self.observation_seconds,
                       maximum_observation_seconds=self.maximum_observation_seconds)
        self.meta_path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    def __exit__(self, exc_type, exc, tb):
        try:
            self.file.flush()
        finally:
            self.file.close()
        self._metadata("completed" if exc_type is None else "failed", None if exc is None else str(exc))

    @staticmethod
    def _sample(actor, carla_map, row):
        # Freeze getter values once, then reuse the existing geometry adapter.
        tf, velocity = actor.get_transform(), actor.get_velocity()
        view = SimpleNamespace(id=actor.id, bounding_box=actor.bounding_box,
                               get_transform=lambda: tf, get_velocity=lambda: velocity)
        vehicle, waypoint = sample_vehicle(view, carla_map)
        if vehicle is None:
            raise ValueError("off_driving_lane")
        for name in ("x", "y", "z"):
            if abs(finite(getattr(tf.location, name), name) - finite(row[name], name)) > 1e-5:
                raise ValueError("capture_state_mismatch")
            if abs(finite(getattr(velocity, name), name) - finite(row["v" + name], name)) > 1e-5:
                raise ValueError("capture_velocity_mismatch")
        for name in ("roll", "pitch", "yaw"):
            if abs(finite(getattr(tf.rotation, name), name) - finite(row[name], name)) > 1e-5:
                raise ValueError("capture_orientation_mismatch")
        if abs(finite(vehicle.speed_mps, "speed", 0) - finite(row["speed_mps"], "speed", 0)) > 1e-5:
            raise ValueError("capture_speed_mismatch")
        finite(vehicle.x, "bbox x")
        finite(vehicle.y, "bbox y")
        for name in ("x", "y", "z"):
            finite(getattr(actor.bounding_box.extent, name), "bbox extent", 0)
        if actor.bounding_box.extent.x <= 0 or actor.bounding_box.extent.y <= 0:
            raise ValueError("empty_bbox")
        if type(vehicle.junction) is not bool:
            raise ValueError("unknown_junction_state")
        # Existing helper transforms bbox centre. Project its rectangular extent
        # onto the local lane tangent, also accounting for bbox yaw rotation.
        rotation = actor.bounding_box.rotation
        if any(abs(finite(value, "tilt")) > 1 for value in
               (rotation.roll, rotation.pitch, tf.rotation.roll, tf.rotation.pitch)):
            raise ValueError("unsupported_bbox_tilt")
        heading = math.radians(finite(tf.rotation.yaw, "yaw") + finite(rotation.yaw, "bbox yaw"))
        lane_forward = waypoint.transform.get_forward_vector()
        ux, uy = finite(lane_forward.x, "lane forward"), finite(lane_forward.y, "lane forward")
        norm = math.hypot(ux, uy)
        if norm < 1e-6:
            raise ValueError("invalid_lane_heading")
        ux, uy = ux / norm, uy / norm
        half = (abs(math.cos(heading) * ux + math.sin(heading) * uy) * actor.bounding_box.extent.x
                + abs(-math.sin(heading) * ux + math.cos(heading) * uy) * actor.bounding_box.extent.y)
        return replace(vehicle, half_length_m=half), waypoint

    def _signal(self, actor, waypoint, speed, time, state):
        try:
            at_light = actor.is_at_traffic_light()
            if type(at_light) is not bool:
                raise ValueError("unknown traffic-light detection")
            if at_light:
                state["last_signal"] = time
                return True, "signal_stop"
            lookahead = max(30.0, speed * speed / 4.0 + 2 * speed + 2)
            landmarks = waypoint.get_landmarks(lookahead, stop_at_junction=False)
            if not isinstance(landmarks, (list, tuple)):
                raise ValueError("unknown landmarks")
            if landmarks:
                state["last_signal"] = time
                return None, "regulatory_landmark_unknown"
        except READ_ERRORS:
            state["last_signal"] = time
            return None, "signal_observation_unknown"
        if time - state["last_signal"] < 2.0 - 1e-9:
            return None, "signal_history_guard"
        return False, ""

    def _row(self, actor, capture, sample, candidates, time, frame, incomplete):
        row = dict.fromkeys(FIELDS, "")
        row.update({key: capture[key] for key in ("logical_actor_id", "carla_frame", "sim_time_s", "source_actor_id")})
        logical = capture["logical_actor_id"]
        if sample is None:
            self.previous.pop(logical, None)
            row["fallback_reason"] = "vehicle_observation_unknown"
            return row
        vehicle, waypoint = sample
        lane = vehicle.lane
        row.update(road_id=lane[0], section_id=lane[1], lane_id=lane[2], is_junction=vehicle.junction)
        old = self.previous.get(logical)
        continuous = (old is not None and old["source"] == actor.id
                      and frame == old["frame"] + 1
                      and 0 < time - old["time"] <= 1.5 * self.dt + 1e-9)
        state = dict(old) if continuous else dict(stable_since=time, last_signal=time, last_change=-math.inf)
        path, boundary = trace_lane(waypoint, self.search_m)
        projection = path_projection(vehicle.x, vehicle.y, path)
        reason = ""
        lane_change = None
        if projection is None:
            reason = boundary or "no_route"
        else:
            width = finite(waypoint.lane_width, "lane width", 0)
            if width <= 0:
                raise ValueError("unknown_lane_width")
            alignment = vehicle.forward_x * projection[2] + vehicle.forward_y * projection[3]
            changed = continuous and old["lane"] != lane
            unstable = projection[0] > width * 0.15 or alignment < 0.98
            if changed or unstable:
                state["stable_since"] = state["last_change"] = time
                lane_change = True
            elif time - state["last_change"] < 1.0 - 1e-9:
                lane_change = True
            elif continuous and time - state["stable_since"] >= 0.3 - 1e-9:
                lane_change = False
            if alignment < 0.5:
                reason = "heading_mismatch"
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
            remaining = length - projection[1] - vehicle.half_length_m
            if boundary and remaining <= max(6.0, vehicle.speed_mps ** 2 / 8 + 2 + vehicle.speed_mps * self.dt):
                reason = boundary
            leader = find_leader(vehicle, candidates, path, width * 0.45)
            if leader is not None:
                row.update(leader_id=leader.vehicle.actor_id, leader_speed_mps=leader.vehicle.speed_mps, gap_m=leader.gap_m)
                if leader.gap_m <= 0:
                    reason = "overlapping_leader"
        signal_stop, signal_reason = self._signal(actor, waypoint, vehicle.speed_mps, time, state)
        if vehicle.junction:
            reason = "junction"
        if incomplete:
            reason = "incomplete_vehicle_observation"
        if not continuous:
            reason = "actor_identity_or_history_reset"
        row.update(lane_change=lane_change, signal_stop=signal_stop,
                   fallback_reason=reason or signal_reason or ("lane_history_unknown" if lane_change is None else ""))
        self.previous[logical] = dict(state, source=actor.id, frame=frame, time=time, lane=lane)
        # Acceleration intentionally blank: estimator differentiates speed within
        # valid windows; no acceleration norm, extra sensor or control request.
        return row

    def __call__(self, world, snapshot, vehicles, capture_rows):
        started = perf_counter()
        frame = int(snapshot.frame)
        if self.last_frame is not None and frame <= self.last_frame:
            raise ValueError("duplicate or nonmonotonic capture frame")
        if int(world.get_snapshot().frame) != frame:
            raise RuntimeError("World advanced before context observation")
        keys = [(r["logical_actor_id"], int(r["carla_frame"])) for r in capture_rows]
        if any(not isinstance(key[0], str) or not key[0].strip() for key in keys):
            raise ValueError("missing logical actor key")
        sources = [int(r["source_actor_id"]) for r in capture_rows]
        if len(set(keys)) != len(keys) or len(set(sources)) != len(sources):
            raise ValueError("duplicate logical or source actor key")
        if any(key[1] != frame for key in keys):
            raise ValueError("capture/context frame mismatch")
        actors = {actor.id: actor for actor in vehicles}
        if len(actors) != len(vehicles) or set(actors) != set(sources):
            raise ValueError("capture actor set mismatch")
        times = [finite(r["sim_time_s"], "capture time") for r in capture_rows]
        if times and any(abs(t - times[0]) > 1e-9 for t in times):
            raise ValueError("capture times differ within frame")
        if self.map is None:
            self.map = world.get_map()
        samples = {}
        for row in capture_rows:
            try:
                samples[row["source_actor_id"]] = self._sample(actors[row["source_actor_id"]], self.map, row)
            except READ_ERRORS:
                samples[row["source_actor_id"]] = None
        candidates = [sample[0] for sample in samples.values() if sample is not None]
        incomplete = len(candidates) != len(vehicles)
        output = []
        for capture in capture_rows:
            try:
                row = self._row(actors[capture["source_actor_id"]], capture,
                                samples[capture["source_actor_id"]], candidates,
                                float(capture["sim_time_s"]), frame, incomplete)
            except READ_ERRORS:
                self.previous.pop(capture["logical_actor_id"], None)
                row = dict.fromkeys(FIELDS, "")
                row.update({key: capture[key] for key in ("logical_actor_id", "carla_frame", "sim_time_s", "source_actor_id")})
                row["fallback_reason"] = "context_observation_unknown"
            output.append(row)
        if int(world.get_snapshot().frame) != frame:
            raise RuntimeError("World advanced during context observation")
        self.writer.writerows(output)
        self.file.flush()
        self.row_count += len(output)
        self.tick_count += 1
        self.last_frame = frame
        self.reasons.update(row["fallback_reason"] or
                            ("lane_change" if row["lane_change"] is True else
                             "no_leader" if row["leader_id"] == "" else "context_valid") for row in output)
        elapsed = perf_counter() - started
        self.observation_seconds += elapsed
        self.maximum_observation_seconds = max(self.maximum_observation_seconds, elapsed)
        return output
