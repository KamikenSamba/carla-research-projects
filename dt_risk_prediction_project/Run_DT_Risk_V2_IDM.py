from __future__ import annotations

import argparse
import logging
import math
import signal
from pathlib import Path
from typing import Any

from dt_risk_common import (
    DEFAULT_FIXED_DELTA_SECONDS,
    DEFAULT_MAP_NAME,
    DEFAULT_OUTPUT_ROOT,
    CsvWriter,
    RiskEvent,
    RiskRegion,
    actor_state_from_actor,
    carla_module_description,
    connect_client,
    create_run_dir,
    current_python_description,
    prediction_state_fieldnames,
    prediction_state_from_actor,
    read_actor_states_csv,
    risk_event_fieldnames,
    risk_region_fieldnames,
    setup_logger,
    to_row,
    write_json,
)


import json
import subprocess
from dataclasses import asdict, fields
import Run_DT_Risk_V1 as baseline
from Run_DT_Risk_V1 import ensure_requested_map, spawn_from_snapshot, detect_near_misses
from idm_model import finite
from idm_snapshot import IDMActorState, read_snapshot, load_traits, resolve_parameters
from idm_controller import IDMFleetController, ControlConfig, LOG_FIELDS

def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a digital twin and compare TM / IDM risk prediction (V2)."
    )
    parser.add_argument("--mode", choices=("capture", "predict", "all"), required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--map", default=DEFAULT_MAP_NAME)
    parser.add_argument("--reload-world", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--snapshot-at", type=float, default=None)
    parser.add_argument("--prediction-seconds", type=float, default=8.0)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--fixed-delta-seconds", type=float, default=DEFAULT_FIXED_DELTA_SECONDS)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--tm-seed", type=int, default=42)
    parser.add_argument("--near-miss-distance-m", type=float, default=4.0)
    parser.add_argument("--min-closing-speed-mps", type=float, default=1.0)
    parser.add_argument("--near-miss-cooldown-s", type=float, default=1.0)
    parser.add_argument("--collision-region-radius-m", type=float, default=3.0)
    parser.add_argument("--near-miss-region-radius-m", type=float, default=4.0)
    parser.add_argument("--prediction-controller", choices=("tm", "idm"), default="idm")
    parser.add_argument("--default-idm-headway", type=float, default=1.5)
    parser.add_argument("--idm-desired-speed-mps", type=float)
    parser.add_argument("--idm-min-desired-speed-mps", type=float, default=1.0)
    parser.add_argument("--idm-minimum-gap-m", type=float, default=2.0)
    parser.add_argument("--idm-max-accel-mps2", type=float, default=1.5)
    parser.add_argument("--idm-comfortable-decel-mps2", type=float, default=2.0)
    parser.add_argument("--idm-acceleration-exponent", type=float, default=4.0)
    parser.add_argument("--idm-max-decel-mps2", type=float, default=4.0)
    parser.add_argument("--actor-traits", type=Path)
    parser.add_argument("--control-config", type=Path, help="JSON overrides of ControlConfig fields")
    args = parser.parse_args(argv)
    try:
        for name in ("fixed_delta_seconds", "prediction_seconds", "duration", "idm_min_desired_speed_mps"):
            if finite(getattr(args, name), name, 0.0) == 0:
                raise ValueError(f"{name} must be positive")
        if args.fixed_delta_seconds > 0.1:
            raise ValueError("fixed delta must be <= 0.1 s (CARLA default physics substeps)")
        finite(args.default_idm_headway, "default_idm_headway", 0.0)
        for name in ("near_miss_distance_m", "min_closing_speed_mps", "near_miss_cooldown_s",
                     "collision_region_radius_m", "near_miss_region_radius_m"):
            finite(getattr(args, name), name, 0.0)
        if args.snapshot_at is not None:
            finite(args.snapshot_at, "snapshot_at", 0.0)
            if args.snapshot_at > args.duration:
                raise ValueError("snapshot_at must be <= duration")
        if args.mode == "predict" and args.snapshot is None:
            raise ValueError("--snapshot is required for --mode predict")
        args.traits = load_traits(args.actor_traits)
        control_values = {} if args.control_config is None else json.loads(args.control_config.read_text(encoding="utf-8-sig"))
        args.control = ControlConfig(**control_values)
        resolve_parameters({"logical_actor_id": "", "speed_mps": 0.0}, args, {})
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    return args


class CreatedActors(baseline.CreatedActors):
    def add_sensor(self, sensor):
        if sensor not in self.sensors:
            super().add_sensor(sensor)


class SensorTrackingWorld:
    """Track sensors before listen() so callback setup failures cannot leak them."""
    def __init__(self, world, created):
        self.world, self.created = world, created

    def get_blueprint_library(self):
        return self.world.get_blueprint_library()

    def spawn_actor(self, *args, **kwargs):
        sensor = self.world.spawn_actor(*args, **kwargs)
        self.created.add_sensor(sensor)
        return sensor


def attach_collision_sensors(world, logical_to_actor, created, *args):
    return baseline.attach_collision_sensors(SensorTrackingWorld(world, created), logical_to_actor, created, *args)


class PredictionWorldMode:
    """IDM owns world ticks only; leave Traffic Manager settings untouched."""
    def __init__(self, world, dt, created, logger):
        self.world, self.dt, self.created, self.logger = world, dt, created, logger
        self.original = None

    def __enter__(self):
        self.original = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = max(1, math.ceil(self.dt / 0.01))
        try:
            self.world.apply_settings(settings)
        except BaseException:
            self.world.apply_settings(self.original)
            raise
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.created.cleanup(self.logger)
            self.created.actors.clear()
            self.created.sensors.clear()
        finally:
            self.world.apply_settings(self.original)


def save_snapshot(path, rows):
    with CsvWriter(path, [field.name for field in fields(IDMActorState)]) as writer:
        writer.writerows(rows)
    write_json(path.with_suffix(".json"), {"schema_version": 2, "actors": rows})


def capture_current_state(args, run_dir, logger):
    path = baseline.capture_current_state(args, run_dir, logger)
    rows = read_snapshot(path)
    unknown = set(args.traits) - {row["logical_actor_id"] for row in rows}
    if unknown:
        raise ValueError(f"trait overrides reference unknown actors: {sorted(unknown)}")
    for row in rows:
        parameters = resolve_parameters(row, args, args.traits)
        row["desired_time_headway_s"] = parameters.desired_time_headway_s
        row["desired_speed_mps"] = parameters.desired_speed_mps
    save_snapshot(path, rows)
    return path


def run_prediction(args, run_dir, snapshot_path, logger):
    rows = read_snapshot(snapshot_path)
    unknown = set(args.traits) - {row["logical_actor_id"] for row in rows}
    if unknown:
        raise ValueError(f"trait overrides reference unknown actors: {sorted(unknown)}")
    parameters = {row["logical_actor_id"]: resolve_parameters(row, args, args.traits) for row in rows}
    normalized = run_dir / "input_snapshot.csv"
    save_snapshot(normalized, rows)
    write_json(run_dir / "resolved_idm_parameters.json", {logical: asdict(p) for logical, p in parameters.items()})
    if args.prediction_controller == "tm":
        return baseline.run_prediction(args, run_dir, normalized, logger)
    return run_idm_prediction(args, run_dir, normalized, logger, parameters)


def run_idm_prediction(args: argparse.Namespace, run_dir: Path, snapshot_path: Path, logger: logging.Logger, parameters) -> dict[str, Any]:
    snapshot_rows = read_actor_states_csv(snapshot_path)
    if not snapshot_rows:
        logger.warning("Snapshot has no rows: %s", snapshot_path)

    source_snapshot_time_s = 0.0
    if snapshot_rows:
        source_snapshot_time_s = float(snapshot_rows[0].get("sim_time_s", 0.0) or 0.0)

    client = connect_client(args.host, args.port, args.timeout)
    world = client.get_world()
    world = ensure_requested_map(client, world, args.map, args.reload_world, logger)
    created = CreatedActors()
    events: list[RiskEvent] = []
    regions: list[RiskRegion] = []
    prediction_start_time = 0.0

    reconstructed_path = run_dir / "reconstructed_states.csv"
    prediction_path = run_dir / "prediction_states.csv"
    risk_events_path = run_dir / "risk_events.csv"
    risk_regions_path = run_dir / "risk_regions.csv"

    logical_to_actor: dict[str, Any] = {}
    near_miss_cooldown: dict[tuple[str, str], float] = {}

    try:
        with PredictionWorldMode(world, args.fixed_delta_seconds, created, logger):
            logical_to_actor = spawn_from_snapshot(world, snapshot_rows, created, logger)
            logger.info("Reconstructed %d vehicles from snapshot.", len(logical_to_actor))

            with CsvWriter(reconstructed_path, [field.name for field in fields(IDMActorState)]) as reconstructed_writer:
                frame = world.tick()
                snap = world.get_snapshot()
                carla_frame, sim_time = int(snap.frame), float(snap.timestamp.elapsed_seconds)
                for logical, actor in sorted(logical_to_actor.items()):
                    state = actor_state_from_actor(
                        actor,
                        phase="reconstructed",
                        frame=carla_frame,
                        sim_time_s=0.0,
                        logical_actor_id=logical,
                    )
                    row = to_row(state)
                    row.update(desired_time_headway_s=parameters[logical].desired_time_headway_s,
                               desired_speed_mps=parameters[logical].desired_speed_mps)
                    reconstructed_writer.writerow(row)

            attach_collision_sensors(
                world,
                logical_to_actor,
                created,
                events,
                regions,
                source_snapshot_time_s,
                lambda: prediction_start_time,
                args,
                logger,
            )

            fleet = IDMFleetController(world, logical_to_actor, parameters, args.fixed_delta_seconds, args.control, logger)

            with CsvWriter(prediction_path, prediction_state_fieldnames()) as prediction_writer, CsvWriter(run_dir / "idm_states.csv", LOG_FIELDS) as idm_writer:
                steps = max(1, int(round(args.prediction_seconds / args.fixed_delta_seconds)))
                for _ in range(steps):
                    idm_writer.writerows(fleet.step(prediction_start_time))
                    frame = world.tick()
                    snap = world.get_snapshot()
                    prediction_start_time = float(snap.timestamp.elapsed_seconds - sim_time)
                    detect_near_misses(
                        logical_to_actor,
                        events,
                        regions,
                        source_snapshot_time_s,
                        prediction_start_time,
                        args,
                        near_miss_cooldown,
                    )
                    for logical, actor in sorted(logical_to_actor.items()):
                        state = prediction_state_from_actor(actor, frame=int(snap.frame),
                            prediction_time_s=prediction_start_time, logical_actor_id=logical)
                        state.autopilot_enabled = False
                        prediction_writer.writerow(to_row(state))

            with CsvWriter(risk_events_path, risk_event_fieldnames()) as event_writer:
                event_writer.writerows(to_row(event) for event in events)
            with CsvWriter(risk_regions_path, risk_region_fieldnames()) as region_writer:
                region_writer.writerows(to_row(region) for region in regions)
    finally:
        created.cleanup(logger)

    logger.info("prediction_states.csv: %s", prediction_path)
    logger.info("risk_events.csv: %s", risk_events_path)
    logger.info("risk_regions.csv: %s", risk_regions_path)

    return {
        "reconstructed_actor_count": len(logical_to_actor),
        "collision_count": sum(1 for event in events if event.risk_type == "collision"),
        "near_miss_count": sum(1 for event in events if event.risk_type == "near_miss"),
        "output_files": [
            str(run_dir / "idm_states.csv"),
            str(reconstructed_path),
            str(prediction_path),
            str(risk_events_path),
            str(risk_regions_path),
        ],
    }


def build_config_summary(args, run_dir):
    config = baseline.build_config_summary(args, run_dir)
    config.update({key: str(value) if isinstance(value, Path) else value
                   for key, value in vars(args).items() if key not in {"control", "traits"}})
    config["control"] = asdict(args.control)
    config["traits"] = args.traits
    config["tm_seed"] = args.tm_seed
    try:
        repo = Path(__file__).resolve().parent.parent
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo, capture_output=True, text=True, check=True)
        status = subprocess.run(["git", "status", "--porcelain"], cwd=repo, capture_output=True, text=True, check=True)
        config["source_commit"] = commit.stdout.strip()
        config["source_dirty"] = bool(status.stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        config["source_commit"] = "unknown"
    return config


def main() -> int:
    args = parse_args()
    run_dir = create_run_dir(args.output_root)
    logger = setup_logger(run_dir)
    interrupted = False

    def handle_signal(signum, frame):
        nonlocal interrupted
        interrupted = True
        logger.warning("Interrupted by signal %s. Cleaning up...", signum)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)

    config = build_config_summary(args, run_dir)
    write_json(run_dir / "config_used.json", config)
    logger.info("Run directory: %s", run_dir)

    summary: dict[str, Any] = {
        "run_dir": str(run_dir),
        "python_version": current_python_description(),
        "prediction_controller": args.prediction_controller,
        "map_name": args.map,
        "fixed_delta_seconds": args.fixed_delta_seconds,
        "errors_or_warnings": [],
    }

    try:
        summary["carla_python_module"] = carla_module_description()
        snapshot_path = args.snapshot
        if args.mode in {"capture", "all"}:
            snapshot_path = capture_current_state(args, run_dir, logger)
            summary["snapshot_path"] = str(snapshot_path)

        if args.mode in {"predict", "all"}:
            if snapshot_path is None:
                raise RuntimeError("--snapshot is required for --mode predict.")
            prediction_summary = run_prediction(args, run_dir, snapshot_path, logger)
            summary.update(prediction_summary)

        if interrupted:
            summary["errors_or_warnings"].append("Interrupted by user.")
        summary["completed"] = True
        return_code = 0
    except KeyboardInterrupt:
        summary["completed"] = False
        summary["errors_or_warnings"].append("Interrupted by user.")
        return_code = 130
    except Exception as exc:
        logger.exception("DT risk run failed.")
        summary["completed"] = False
        summary["errors_or_warnings"].append(str(exc))
        return_code = 1
    finally:
        try:
            summary["run_log"] = str(run_dir / "run.log")
            summary["output_files"] = sorted(str(path) for path in run_dir.glob("*"))
            write_json(run_dir / "summary.json", summary)
        except Exception as exc:
            logger.error("Failed to write summary.json: %s", exc)

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
