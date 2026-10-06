"""Custom longitudinal and lateral control; never enables TM autopilot."""
from __future__ import annotations

from dataclasses import dataclass, fields
import math

from dt_risk_common import import_carla, iter_vehicle_actors
from idm_model import PID, compute_desired_gap, compute_idm_acceleration, finite
from idm_leader import find_leader, sample_vehicle, trace_lane, path_projection


@dataclass(frozen=True)
class ControlConfig:
    speed_kp: float = 3.0
    speed_ki: float = 0.35
    speed_kd: float = 0.05
    steering_kp: float = 1.25
    steering_ki: float = 0.02
    steering_kd: float = 0.12
    acceleration_filter_alpha: float = 0.30
    route_lookahead_m: float = 6.0
    leader_search_m: float = 80.0
    maximum_steer: float = 0.75

    def __post_init__(self):
        for field in fields(self):
            object.__setattr__(self, field.name, finite(getattr(self, field.name), field.name, 0.0))
        if not 0 < self.acceleration_filter_alpha <= 1 or not 0 < self.maximum_steer <= 1:
            raise ValueError("filter alpha and maximum steer must be in (0, 1]")
        if self.route_lookahead_m <= 0 or self.leader_search_m < self.route_lookahead_m:
            raise ValueError("search distance must be >= positive lookahead")


LOG_FIELDS = ["simulation_time", "actor_id", "logical_actor_id", "leader_id",
              "speed_mps", "leader_speed_mps", "gap_m", "relative_speed_mps",
              "desired_time_headway_s", "desired_gap_m", "idm_acceleration_mps2",
              "filtered_acceleration_mps2", "target_speed_mps", "throttle", "brake",
              "steer", "road_id", "section_id", "lane_id", "fallback_reason"]


class VehicleController:
    def __init__(self, parameters, dt, config: ControlConfig):
        self.parameters, self.dt, self.config = parameters, dt, config
        self.speed_pid = PID(config.speed_kp, config.speed_ki, config.speed_kd, dt, 3.0)
        self.steering_pid = PID(config.steering_kp, config.steering_ki, config.steering_kd, dt, 1.0)
        self.acceleration = 0.0

    def command(self, speed, acceleration, steering_error, emergency=False):
        p = self.parameters
        speed = finite(speed, "speed", 0.0)
        acceleration = finite(acceleration, "acceleration")
        steering_error = finite(steering_error, "steering_error")
        acceleration = max(-p.max_decel_mps2, min(p.max_accel_mps2, acceleration))
        if emergency:
            self.acceleration = -p.max_decel_mps2
            self.speed_pid.reset()
        else:
            alpha = self.config.acceleration_filter_alpha
            self.acceleration = alpha * acceleration + (1 - alpha) * self.acceleration
        target = max(0.0, speed + self.acceleration * self.dt)
        feed_forward = self.acceleration / (p.max_accel_mps2 if self.acceleration >= 0 else p.max_decel_mps2)
        effort = self.speed_pid.step(target - speed) + feed_forward
        throttle, brake = min(1.0, max(0.0, effort)), min(1.0, max(0.0, -effort))
        if emergency:
            throttle, brake = 0.0, 1.0
        steer = max(-self.config.maximum_steer,
                    min(self.config.maximum_steer, self.steering_pid.step(steering_error)))
        return target, throttle, brake, steer


class IDMFleetController:
    def __init__(self, world, logical_to_actor, parameters, dt, config, logger):
        self.world, self.actors, self.logger = world, logical_to_actor, logger
        self.map, self.config = world.get_map(), config
        self.controllers = {logical: VehicleController(parameters[logical], dt, config)
                            for logical in logical_to_actor}
        self.last_reason = {}
        for actor in logical_to_actor.values():
            actor.set_autopilot(False)

    def step(self, simulation_time):
        carla = import_carla()
        # Read all states before applying any controls, including external vehicles.
        sampled = {actor.id: sample_vehicle(actor, self.map)
                   for actor in iter_vehicle_actors(self.world)}
        vehicles = [vehicle for vehicle, _ in sampled.values() if vehicle is not None]
        rows = []
        for logical, actor in sorted(self.actors.items()):
            controller = self.controllers[logical]
            p = controller.parameters
            vehicle, waypoint = sampled.get(actor.id, (None, None))
            leader, reason, steering_error = None, "", 0.0
            velocity = actor.get_velocity()
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
            if vehicle is None:
                reason = "off_driving_lane"
            elif waypoint.is_junction:
                reason = "junction"
            else:
                path, boundary = trace_lane(waypoint, self.config.leader_search_m)
                leader = find_leader(vehicle, vehicles, path, waypoint.lane_width * 0.45)
                projection = path_projection(vehicle.x, vehicle.y, path)
                if projection is None:
                    reason = boundary or "no_route"
                elif vehicle.forward_x * projection[2] + vehicle.forward_y * projection[3] < 0.5:
                    reason = "heading_mismatch"
                else:
                    path_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
                    remaining = path_length - projection[1] - vehicle.half_length_m
                    stopping_distance = speed ** 2 / (2 * p.max_decel_mps2) + p.minimum_gap_m + speed * controller.dt
                    if boundary and remaining <= max(self.config.route_lookahead_m, stopping_distance):
                        reason = boundary
                    # Lane center heading plus cross-track correction, in radians.
                    heading = math.atan2(projection[3], projection[2])
                    yaw = math.radians(actor.get_transform().rotation.yaw)
                    heading_error = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
                    # Use the lookahead waypoint direction to anticipate curves.
                    following = waypoint.next(self.config.route_lookahead_m)
                    if len(following) == 1 and not following[0].is_junction:
                        target = following[0].transform.location
                        steering_error = (math.atan2(target.y - vehicle.y, target.x - vehicle.x)
                                          - yaw + math.pi) % (2 * math.pi) - math.pi
                    else:
                        steering_error = heading_error
            gap = None if leader is None else leader.gap_m
            leader_speed = 0.0 if leader is None else leader.vehicle.speed_mps
            acceleration = compute_idm_acceleration(speed, p, gap, leader_speed)
            if gap is not None and gap <= 0:
                reason = "overlapping_leader"
            target, throttle, brake, steer = controller.command(speed, acceleration, steering_error, bool(reason))
            actor.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
            if reason and self.last_reason.get(logical) != reason:
                self.logger.warning("IDM fallback actor=%s simulation_time=%.3f reason=%s", actor.id, simulation_time, reason)
            self.last_reason[logical] = reason
            rows.append(dict(zip(LOG_FIELDS, [simulation_time, actor.id, logical,
                "" if leader is None else leader.vehicle.actor_id, speed,
                "" if leader is None else leader_speed, "" if gap is None else gap,
                "" if leader is None else speed - leader_speed, p.desired_time_headway_s,
                compute_desired_gap(speed, 0.0 if leader is None else speed - leader_speed, p),
                acceleration, controller.acceleration, target, throttle, brake, steer,
                "" if waypoint is None else waypoint.road_id,
                "" if waypoint is None else waypoint.section_id,
                "" if waypoint is None else waypoint.lane_id, reason])))
        return rows
