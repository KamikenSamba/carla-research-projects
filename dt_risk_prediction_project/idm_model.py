"""Server-free IDM mathematics. All inputs and outputs use SI units."""
from __future__ import annotations

from dataclasses import dataclass, fields
import math


def finite(value: float, name: str, minimum: float | None = None) -> float:
    value = float(value)
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f"{name} must be finite" + (f" and >= {minimum}" if minimum is not None else ""))
    return value


@dataclass(frozen=True)
class IDMParameters:
    desired_speed_mps: float
    desired_time_headway_s: float = 1.5
    minimum_gap_m: float = 2.0
    max_accel_mps2: float = 1.5
    comfortable_decel_mps2: float = 2.0
    acceleration_exponent: float = 4.0
    max_decel_mps2: float = 4.0

    def __post_init__(self):
        for field in fields(self):
            value = finite(getattr(self, field.name), field.name, 0.0)
            object.__setattr__(self, field.name, value)
            if field.name not in {"desired_time_headway_s", "minimum_gap_m"} and value == 0:
                raise ValueError(f"{field.name} must be positive")


def compute_desired_gap(speed_mps: float, relative_speed_mps: float, parameters: IDMParameters) -> float:
    v = finite(speed_mps, "speed_mps", 0.0)
    dv = finite(relative_speed_mps, "relative_speed_mps")
    root = math.sqrt(parameters.max_accel_mps2) * math.sqrt(parameters.comfortable_decel_mps2)
    dynamic = v * parameters.desired_time_headway_s + v * dv / (2 * root)
    finite(dynamic, "dynamic gap")
    return finite(parameters.minimum_gap_m + max(0.0, dynamic), "desired_gap_m", 0.0)


def compute_idm_acceleration(speed_mps: float, parameters: IDMParameters,
                             gap_m: float | None = None, leader_speed_mps: float = 0.0) -> float:
    """Return bounded acceleration; None gap means free road, <=0 means overlap."""
    v = finite(speed_mps, "speed_mps", 0.0)
    leader_v = finite(leader_speed_mps, "leader_speed_mps", 0.0)
    gap = None if gap_m is None else finite(gap_m, "gap_m")
    try:
        free = (v / parameters.desired_speed_mps) ** parameters.acceleration_exponent
        interaction = 0.0
        if gap is not None:
            if gap <= 0:
                return -parameters.max_decel_mps2
            desired = compute_desired_gap(v, v - leader_v, parameters)
            interaction = (desired / max(0.001, gap)) ** 2
        raw = parameters.max_accel_mps2 * (1.0 - free - interaction)
    except OverflowError:
        return -parameters.max_decel_mps2
    return max(-parameters.max_decel_mps2, min(parameters.max_accel_mps2, raw))


class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, integral_limit: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = finite(dt, "dt", 0.0)
        if self.dt == 0:
            raise ValueError("dt must be positive")
        self.limit = integral_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None

    def step(self, error: float) -> float:
        error = finite(error, "PID error")
        derivative = 0.0 if self.previous is None else (error - self.previous) / self.dt
        self.integral = max(-self.limit, min(self.limit, self.integral + error * self.dt))
        self.previous = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
