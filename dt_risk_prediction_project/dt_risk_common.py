from __future__ import annotations

import csv
import json
import logging
import math
import os
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Iterator, Mapping, Sequence

DEFAULT_OUTPUT_ROOT = Path(r"D:\CARLA_DATA\DT_RiskPrediction\runs")
DEFAULT_MAP_NAME = "Town10HD_Opt"
DEFAULT_FIXED_DELTA_SECONDS = 0.05


@dataclass
class ActorState:
    phase: str
    carla_frame: int
    sim_time_s: float
    source_actor_id: int
    logical_actor_id: str
    role_name: str
    blueprint_id: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float
    vy: float
    vz: float
    speed_mps: float
    autopilot_enabled: bool


@dataclass
class PredictionState:
    prediction_frame: int
    prediction_time_s: float
    logical_actor_id: str
    carla_actor_id: int
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    vz: float
    speed_mps: float
    autopilot_enabled: bool


@dataclass
class RiskEvent:
    prediction_id: str
    source_snapshot_time_s: float
    prediction_time_s: float
    risk_type: str
    logical_actor_a: str
    logical_actor_b: str
    carla_actor_a: int
    carla_actor_b: int
    risk_x: float
    risk_y: float
    risk_z: float
    distance_m: float
    relative_speed_mps: float
    impulse_magnitude: float
    note: str


@dataclass
class RiskRegion:
    risk_id: str
    risk_type: str
    center_x: float
    center_y: float
    center_z: float
    radius_m: float
    start_time_s: float
    end_time_s: float
    logical_actor_a: str
    logical_actor_b: str


class CsvWriter:
    def __init__(self, path: Path, fieldnames: Sequence[str]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = path.open("w", newline="", encoding="utf-8-sig")
        self._writer = csv.DictWriter(self._file, fieldnames=fieldnames)
        self._writer.writeheader()

    def writerow(self, row: Mapping[str, object]) -> None:
        self._writer.writerow(dict(row))

    def writerows(self, rows: Iterable[Mapping[str, object]]) -> None:
        for row in rows:
            self.writerow(row)

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "CsvWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def create_run_dir(output_root: Path | str = DEFAULT_OUTPUT_ROOT) -> Path:
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(output_root) / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def setup_logger(run_dir: Path) -> logging.Logger:
    logger = logging.getLogger("dt_risk")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)

    file_handler = logging.FileHandler(run_dir / "run.log", encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    return logger


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def import_carla():
    import carla  # type: ignore

    return carla


def connect_client(host: str, port: int, timeout: float = 10.0):
    carla = import_carla()
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def get_actor_role_name(actor) -> str:
    try:
        return actor.attributes.get("role_name", "")
    except Exception:
        return ""


def is_autopilot_enabled(actor) -> bool:
    try:
        return get_actor_role_name(actor) in {"autopilot", "hero"}
    except Exception:
        return False


def speed_mps(velocity) -> float:
    return math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)


def actor_state_from_actor(actor, *, phase: str, frame: int, sim_time_s: float, logical_actor_id: str) -> ActorState:
    tf = actor.get_transform()
    vel = actor.get_velocity()
    return ActorState(
        phase=phase,
        carla_frame=int(frame),
        sim_time_s=float(sim_time_s),
        source_actor_id=int(actor.id),
        logical_actor_id=logical_actor_id,
        role_name=get_actor_role_name(actor),
        blueprint_id=str(actor.type_id),
        x=float(tf.location.x),
        y=float(tf.location.y),
        z=float(tf.location.z),
        roll=float(tf.rotation.roll),
        pitch=float(tf.rotation.pitch),
        yaw=float(tf.rotation.yaw),
        vx=float(vel.x),
        vy=float(vel.y),
        vz=float(vel.z),
        speed_mps=speed_mps(vel),
        autopilot_enabled=is_autopilot_enabled(actor),
    )


def prediction_state_from_actor(actor, *, frame: int, prediction_time_s: float, logical_actor_id: str) -> PredictionState:
    tf = actor.get_transform()
    vel = actor.get_velocity()
    return PredictionState(
        prediction_frame=int(frame),
        prediction_time_s=float(prediction_time_s),
        logical_actor_id=logical_actor_id,
        carla_actor_id=int(actor.id),
        x=float(tf.location.x),
        y=float(tf.location.y),
        z=float(tf.location.z),
        yaw=float(tf.rotation.yaw),
        vx=float(vel.x),
        vy=float(vel.y),
        vz=float(vel.z),
        speed_mps=speed_mps(vel),
        autopilot_enabled=is_autopilot_enabled(actor),
    )


def iter_vehicle_actors(world) -> list:
    return list(world.get_actors().filter("vehicle.*"))


def logical_id_for_actor(actor, index: int) -> str:
    role_name = get_actor_role_name(actor).strip()
    if role_name and role_name not in {"autopilot", "scenario", "background"}:
        return f"role:{role_name}"
    return f"vehicle_{index:03d}_source_{actor.id}"


def read_actor_states_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def actor_state_fieldnames() -> list[str]:
    return list(ActorState.__dataclass_fields__.keys())


def prediction_state_fieldnames() -> list[str]:
    return list(PredictionState.__dataclass_fields__.keys())


def risk_event_fieldnames() -> list[str]:
    return list(RiskEvent.__dataclass_fields__.keys())


def risk_region_fieldnames() -> list[str]:
    return list(RiskRegion.__dataclass_fields__.keys())


def to_row(instance) -> dict[str, object]:
    return asdict(instance)


class SynchronousMode:
    def __init__(self, world, traffic_manager, fixed_delta_seconds: float, logger: logging.Logger) -> None:
        self.world = world
        self.traffic_manager = traffic_manager
        self.fixed_delta_seconds = fixed_delta_seconds
        self.logger = logger
        self.original_settings = None
        self.original_tm_sync = None

    def __enter__(self):
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        self.world.apply_settings(settings)
        try:
            self.traffic_manager.set_synchronous_mode(True)
            self.original_tm_sync = False
        except Exception as exc:
            self.logger.warning("Traffic Manager sync setup failed: %s", exc)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            if self.original_settings is not None:
                self.world.apply_settings(self.original_settings)
        finally:
            try:
                self.traffic_manager.set_synchronous_mode(False)
            except Exception:
                pass


def current_python_description() -> str:
    return sys.version.replace("\n", " ")


def carla_module_description() -> dict[str, object]:
    carla = import_carla()
    return {
        "module_file": getattr(carla, "__file__", "unknown"),
        "module_version": getattr(carla, "__version__", "unknown"),
    }


def get_available_map_names(client) -> list[str]:
    maps = []
    for item in client.get_available_maps():
        name = str(item).replace("\\", "/").split("/")[-1]
        maps.append(name)
    return sorted(set(maps))


def distance_xy(a, b) -> float:
    dx = float(a.x) - float(b.x)
    dy = float(a.y) - float(b.y)
    return math.sqrt(dx * dx + dy * dy)


def relative_speed_toward_each_other(loc_a, vel_a, loc_b, vel_b) -> float:
    dx = float(loc_b.x) - float(loc_a.x)
    dy = float(loc_b.y) - float(loc_a.y)
    dist = math.sqrt(dx * dx + dy * dy)
    if dist <= 1e-6:
        return 0.0
    nx = dx / dist
    ny = dy / dist
    rvx = float(vel_a.x) - float(vel_b.x)
    rvy = float(vel_a.y) - float(vel_b.y)
    return max(0.0, rvx * nx + rvy * ny)
