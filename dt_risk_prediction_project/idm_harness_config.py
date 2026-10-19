"""Server-free experiment policy. Values are fixed before observing a fit."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Scenario:
    map_name: str = "Town10HD_Opt"
    blueprint: str = "vehicle.tesla.model3"
    dt: float = 0.05
    duration: float = 15.0
    snapshot_at: float = 10.0
    tm_port: int = 8000
    seed: int = 42
    road: int = 10
    section: int = 0
    lane: int = 1
    follower_s: float = 120.0
    leader_s: float = 108.0
    end_s: float = 64.0
    spawn_height: float = 0.4
    follower_speed: float = 2.2
    following_distance: float = 5.0
    warmup_speed: float = 1.4
    warmup_seconds: float = 6.0
    warmup_distance: float = 6.0
    ready_seconds: float = 1.0
    leader_speed_limit: float = 1.9
    follower_speed_limit: float = 2.5
    minimum_speed: float = 1.0
    stop_seconds: float = 0.5
    minimum_gap: float = 2.0
    minimum_ttc: float = 1.0
    end_clearance: float = 6.0
    nominal_front_extent: float = 2.42511
    lateral_limit: float = 0.3
    heading_limit: float = 5.0
    deviation_seconds: float = 0.25
    leader_missing_seconds: float = 0.25
    invalid_seconds: float = 0.5
    minimum_valid_ratio: float = 0.9
    minimum_moving_seconds: float = 12.0
    profile: tuple = ((0, 1.4), (2, 1.4), (4, 1.8), (5, 1.8),
                      (7, 1.3), (9, 1.3), (11, 1.7), (15, 1.7))

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)) and (isinstance(value, bool) or not math.isfinite(value)):
                raise ValueError(f"invalid finite numeric setting: {key}")
        if (self.map_name != "Town10HD_Opt" or self.blueprint != "vehicle.tesla.model3"
                or self.dt != 0.05 or self.duration != 15 or self.snapshot_at != 10
                or (self.road, self.section, self.lane) != (10, 0, 1)
                or (self.follower_s, self.leader_s, self.end_s) != (120, 108, 64)):
            raise ValueError("only the audited 56 m Town10HD scenario is supported")
        if not isinstance(self.tm_port, int) or not 1024 <= self.tm_port <= 65535:
            raise ValueError("invalid TM port")
        if not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError("invalid seed")
        positive = (self.spawn_height, self.follower_speed, self.following_distance,
                    self.warmup_speed, self.warmup_seconds, self.warmup_distance,
                    self.ready_seconds, self.minimum_speed, self.stop_seconds,
                    self.minimum_gap, self.minimum_ttc, self.end_clearance,
                    self.nominal_front_extent, self.lateral_limit, self.heading_limit,
                    self.deviation_seconds, self.leader_missing_seconds, self.invalid_seconds)
        if min(positive) <= 0 or not 0 < self.minimum_valid_ratio <= 1:
            raise ValueError("invalid safety threshold")
        if (self.minimum_speed != 1.0 or self.minimum_gap < 2 or self.end_clearance < 6
                or self.leader_speed_limit > 1.9 or self.follower_speed_limit > 2.5
                or self.warmup_distance > 6):
            raise ValueError("outside the audited speed/gap/clearance envelope")
        if not 0 < self.minimum_moving_seconds <= self.duration - 2:
            raise ValueError("invalid moving duration")
        if self.ready_seconds > self.warmup_seconds or self.leader_speed_limit < self.warmup_speed:
            raise ValueError("warmup cannot meet speed/time envelope")
        if not self.minimum_speed < self.warmup_speed < self.follower_speed <= self.follower_speed_limit:
            raise ValueError("speed envelope incompatible with estimator minimum")
        if not self.profile or self.profile[0][0] != 0 or self.profile[-1][0] != self.duration:
            raise ValueError("profile must cover capture")
        last = -1
        for time, speed in self.profile:
            if not math.isfinite(time) or not math.isfinite(speed) or time <= last:
                raise ValueError("invalid profile time")
            if not self.minimum_speed < speed <= self.leader_speed_limit:
                raise ValueError("profile outside speed envelope")
            last = time
        if self.clearance_budget < self.end_clearance:
            raise ValueError("insufficient route length including warmup and vehicle extent")

    @property
    def clearance_budget(self):
        return (self.follower_s - self.end_s - (self.follower_s - self.leader_s)
                - self.warmup_distance - self.leader_speed_limit * self.duration
                - self.nominal_front_extent)

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()

    def speed_at(self, time, stage):
        if stage == "stage1":
            return self.warmup_speed
        for a, b in zip(self.profile, self.profile[1:]):
            if time <= b[0]:
                fraction = max(0.0, (time - a[0]) / (b[0] - a[0]))
                return a[1] + fraction * (b[1] - a[1])
        return self.profile[-1][1]


def load_scenario(path):
    values = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if "profile" in values:
        values["profile"] = tuple(tuple(pair) for pair in values["profile"])
    return Scenario(**values)
