"""Pure lane/path geometry and a small CARLA adapter."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LaneVehicle:
    actor_id: int
    x: float
    y: float
    forward_x: float
    forward_y: float
    lane: tuple[int, int, int]
    half_length_m: float
    speed_mps: float
    junction: bool = False


@dataclass(frozen=True)
class Leader:
    vehicle: LaneVehicle
    gap_m: float


def path_projection(x: float, y: float, path: list[tuple[float, float]]):
    best, travelled = None, 0.0
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        ux, uy = dx / length, dy / length
        raw = (x - a[0]) * ux + (y - a[1]) * uy
        along = max(0.0, min(length, raw))
        distance = math.hypot(x - a[0] - along * ux, y - a[1] - along * uy)
        candidate = (distance, travelled + along, ux, uy, raw >= 0 and raw <= length)
        if best is None or candidate[0] < best[0]:
            best = candidate
        travelled += length
    return best


def find_leader(follower: LaneVehicle, vehicles: list[LaneVehicle],
                path: list[tuple[float, float]], corridor_m: float) -> Leader | None:
    if follower.junction:
        return None
    origin = path_projection(follower.x, follower.y, path)
    if origin is None:
        return None
    selected = None
    for other in vehicles:
        if other.actor_id == follower.actor_id or other.lane != follower.lane or other.junction:
            continue
        projected = path_projection(other.x, other.y, path)
        if projected is None:
            continue
        distance, progress, ux, uy, within = projected
        separation = progress - origin[1]
        if (not within or separation <= 0 or distance > corridor_m
                or other.forward_x * ux + other.forward_y * uy < 0.5):
            continue
        gap = separation - follower.half_length_m - other.half_length_m
        if selected is None or (gap, other.actor_id) < (selected.gap_m, selected.vehicle.actor_id):
            selected = Leader(other, gap)
    return selected


def lane_key(waypoint):
    return (waypoint.road_id, waypoint.section_id, waypoint.lane_id)


def sample_vehicle(actor, carla_map):
    tf = actor.get_transform()
    waypoint = carla_map.get_waypoint(tf.location, project_to_road=False)
    if waypoint is None:
        return None, None
    # Bounding boxes may have a nonzero local center; do not assume actor origin.
    center = tf.transform(actor.bounding_box.location)
    forward = tf.get_forward_vector()
    velocity = actor.get_velocity()
    speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    vehicle = LaneVehicle(actor.id, center.x, center.y, forward.x, forward.y,
                          lane_key(waypoint), actor.bounding_box.extent.x, speed,
                          waypoint.is_junction)
    return vehicle, waypoint


def trace_lane(waypoint, maximum_distance_m: float, step_m: float = 2.0):
    """Trace an unambiguous same-lane path, stopping before junctions or forks."""
    point = waypoint.transform.location
    path = [(point.x, point.y)]
    reason = ""
    origin_lane = lane_key(waypoint)
    for _ in range(math.ceil(maximum_distance_m / step_m)):
        following = waypoint.next(step_m)
        if len(following) != 1:
            reason = "route_end" if not following else "ambiguous_route"
            break
        candidate = following[0]
        if candidate.is_junction:
            reason = "junction_ahead"
            break
        if lane_key(candidate) != origin_lane:
            reason = "lane_transition"
            break
        waypoint = candidate
        point = waypoint.transform.location
        path.append((point.x, point.y))
    return path, reason
