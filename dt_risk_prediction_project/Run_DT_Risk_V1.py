from __future__ import annotations

import argparse
import itertools
import logging
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

from dt_risk_common import (
    DEFAULT_FIXED_DELTA_SECONDS,
    DEFAULT_MAP_NAME,
    DEFAULT_OUTPUT_ROOT,
    CsvWriter,
    RiskEvent,
    RiskRegion,
    SynchronousMode,
    actor_state_fieldnames,
    actor_state_from_actor,
    carla_module_description,
    connect_client,
    create_run_dir,
    current_python_description,
    distance_xy,
    get_available_map_names,
    import_carla,
    iter_vehicle_actors,
    logical_id_for_actor,
    prediction_state_fieldnames,
    prediction_state_from_actor,
    read_actor_states_csv,
    relative_speed_toward_each_other,
    risk_event_fieldnames,
    risk_region_fieldnames,
    setup_logger,
    to_row,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a CARLA digital-twin snapshot and run Autopilot risk prediction."
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
    return parser.parse_args()


class CreatedActors:
    def __init__(self) -> None:
        self.actors: list[Any] = []
        self.sensors: list[Any] = []

    def add_actor(self, actor) -> None:
        self.actors.append(actor)

    def add_sensor(self, sensor) -> None:
        self.sensors.append(sensor)

    def cleanup(self, logger: logging.Logger) -> None:
        for sensor in self.sensors:
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
            except Exception as exc:
                logger.warning("Failed to destroy sensor %s: %s", getattr(sensor, "id", "?"), exc)
        for actor in self.actors:
            try:
                actor.destroy()
            except Exception as exc:
                logger.warning("Failed to destroy actor %s: %s", getattr(actor, "id", "?"), exc)


def ensure_requested_map(client, world, map_name: str, reload_world: bool, logger: logging.Logger):
    current_map = world.get_map().name
    current_short = current_map.replace("\\", "/").split("/")[-1]
    if current_short == map_name and not reload_world:
        return world

    if not reload_world:
        logger.warning(
            "Current map is %s, requested map is %s. Continuing without world reload.",
            current_short,
            map_name,
        )
        return world

    available = get_available_map_names(client)
    if map_name not in available:
        raise RuntimeError(f"Requested map {map_name} is not available. Available maps: {available}")

    logger.info("Reloading world: %s", map_name)
    try:
        client.set_timeout(60.0)
    except Exception:
        pass
    return client.load_world(map_name)


def frame_time(world_snapshot) -> tuple[int, float]:
    return int(world_snapshot.frame), float(world_snapshot.timestamp.elapsed_seconds)


def capture_current_state(args: argparse.Namespace, run_dir: Path, logger: logging.Logger) -> Path:
    client = connect_client(args.host, args.port, args.timeout)
    world = client.get_world()
    world = ensure_requested_map(client, world, args.map, False, logger)
    traffic_manager = client.get_trafficmanager(args.tm_port)

    capture_path = run_dir / "capture_states.csv"
    snapshot_path = run_dir / "snapshot_states.csv"
    snapshot_at = args.snapshot_at if args.snapshot_at is not None else args.duration
    snapshot_rows: list[dict[str, object]] = []
    source_mapping: dict[int, str] = {}

    logger.info("Capturing vehicle states for %.3f seconds.", args.duration)
    logger.info("Snapshot target time: %.3f seconds.", snapshot_at)

    with CsvWriter(capture_path, actor_state_fieldnames()) as capture_writer:
        with SynchronousMode(world, traffic_manager, args.fixed_delta_seconds, logger):
            start_frame = world.tick()
            start_snapshot = world.get_snapshot()
            _start_frame, start_time = frame_time(start_snapshot)
            end_time = start_time + args.duration
            snapshot_target = start_time + snapshot_at
            snapshot_taken = False

            while True:
                frame = world.tick()
                snap = world.get_snapshot()
                carla_frame, sim_time = frame_time(snap)
                vehicles = iter_vehicle_actors(world)
                vehicles.sort(key=lambda actor: actor.id)

                for index, actor in enumerate(vehicles, start=1):
                    logical_id = source_mapping.setdefault(actor.id, logical_id_for_actor(actor, index))
                    state = actor_state_from_actor(
                        actor,
                        phase="capture",
                        frame=carla_frame,
                        sim_time_s=sim_time - start_time,
                        logical_actor_id=logical_id,
                    )
                    capture_writer.writerow(to_row(state))

                if not snapshot_taken and sim_time >= snapshot_target:
                    snapshot_rows = []
                    for index, actor in enumerate(vehicles, start=1):
                        logical_id = source_mapping.setdefault(actor.id, logical_id_for_actor(actor, index))
                        state = actor_state_from_actor(
                            actor,
                            phase="snapshot",
                            frame=carla_frame,
                            sim_time_s=sim_time - start_time,
                            logical_actor_id=logical_id,
                        )
                        snapshot_rows.append(to_row(state))
                    snapshot_taken = True
                    logger.info("Saved in-memory snapshot with %d vehicles.", len(snapshot_rows))

                if sim_time >= end_time:
                    break

        capture_writer.flush()

    with CsvWriter(snapshot_path, actor_state_fieldnames()) as snapshot_writer:
        snapshot_writer.writerows(snapshot_rows)

    logger.info("capture_states.csv: %s", capture_path)
    logger.info("snapshot_states.csv: %s", snapshot_path)
    if not snapshot_rows:
        logger.warning("Snapshot contains no vehicles. Start traffic before running capture.")
    return snapshot_path


def select_blueprint(blueprint_library, blueprint_id: str):
    try:
        return blueprint_library.find(blueprint_id)
    except Exception:
        matches = blueprint_library.filter("vehicle.*")
        if not matches:
            raise RuntimeError("No vehicle blueprints are available.")
        return matches[0]


def spawn_from_snapshot(world, snapshot_rows: list[dict[str, str]], created: CreatedActors, logger: logging.Logger):
    carla = import_carla()
    blueprint_library = world.get_blueprint_library()
    logical_to_actor: dict[str, Any] = {}

    for row in snapshot_rows:
        logical_id = row["logical_actor_id"]
        blueprint = select_blueprint(blueprint_library, row["blueprint_id"])
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", f"dt_risk:{logical_id}")

        rotation = carla.Rotation(
            roll=float(row["roll"]),
            pitch=float(row["pitch"]),
            yaw=float(row["yaw"]),
        )
        base_z = max(float(row["z"]), 0.2)
        actor = None
        for z_offset in (0.0, 0.3, 0.7, 1.2):
            location = carla.Location(
                x=float(row["x"]),
                y=float(row["y"]),
                z=base_z + z_offset,
            )
            actor = world.try_spawn_actor(blueprint, carla.Transform(location, rotation))
            if actor is not None:
                if z_offset > 0.0:
                    logger.warning(
                        "Spawned %s with z offset %.2f m to avoid initial overlap.",
                        logical_id,
                        z_offset,
                    )
                break
        if actor is None:
            logger.warning("Failed to spawn %s at snapshot pose.", logical_id)
            continue

        created.add_actor(actor)
        logical_to_actor[logical_id] = actor
        try:
            actor.set_target_velocity(
                carla.Vector3D(float(row["vx"]), float(row["vy"]), float(row["vz"]))
            )
        except Exception as exc:
            logger.warning("Failed to set initial velocity for %s: %s", logical_id, exc)

    return logical_to_actor


def attach_collision_sensors(world, logical_to_actor: dict[str, Any], created: CreatedActors, events: list[RiskEvent], regions: list[RiskRegion], source_snapshot_time_s: float, start_time_getter, args: argparse.Namespace, logger: logging.Logger) -> None:
    carla = import_carla()
    blueprint = world.get_blueprint_library().find("sensor.other.collision")

    actor_id_to_logical = {actor.id: logical for logical, actor in logical_to_actor.items()}
    seen_collision_pairs: set[tuple[int, int, int]] = set()

    def make_callback(owner_logical: str, owner_actor):
        def callback(event) -> None:
            other = event.other_actor
            other_logical = actor_id_to_logical.get(getattr(other, "id", -1), f"external:{getattr(other, 'id', -1)}")
            impulse = event.normal_impulse
            impulse_mag = math.sqrt(impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z)
            elapsed = start_time_getter()
            bucket = int(elapsed * 10)
            pair_key = tuple(sorted((int(owner_actor.id), int(getattr(other, "id", -1))))) + (bucket,)
            if pair_key in seen_collision_pairs:
                return
            seen_collision_pairs.add(pair_key)
            loc = owner_actor.get_location()
            risk_id = f"collision_{len(events) + 1:04d}"
            events.append(
                RiskEvent(
                    prediction_id=risk_id,
                    source_snapshot_time_s=source_snapshot_time_s,
                    prediction_time_s=elapsed,
                    risk_type="collision",
                    logical_actor_a=owner_logical,
                    logical_actor_b=other_logical,
                    carla_actor_a=int(owner_actor.id),
                    carla_actor_b=int(getattr(other, "id", -1)),
                    risk_x=float(loc.x),
                    risk_y=float(loc.y),
                    risk_z=float(loc.z),
                    distance_m=0.0,
                    relative_speed_mps=0.0,
                    impulse_magnitude=float(impulse_mag),
                    note=f"other_type={getattr(other, 'type_id', 'unknown')}",
                )
            )
            regions.append(
                RiskRegion(
                    risk_id=risk_id,
                    risk_type="collision",
                    center_x=float(loc.x),
                    center_y=float(loc.y),
                    center_z=float(loc.z),
                    radius_m=float(args.collision_region_radius_m),
                    start_time_s=elapsed,
                    end_time_s=elapsed,
                    logical_actor_a=owner_logical,
                    logical_actor_b=other_logical,
                )
            )
        return callback

    for logical, actor in logical_to_actor.items():
        sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=actor)
        sensor.listen(make_callback(logical, actor))
        created.add_sensor(sensor)


def detect_near_misses(logical_to_actor: dict[str, Any], events: list[RiskEvent], regions: list[RiskRegion], source_snapshot_time_s: float, prediction_time_s: float, args: argparse.Namespace, cooldown: dict[tuple[str, str], float]) -> None:
    items = sorted(logical_to_actor.items(), key=lambda item: item[0])
    for (logical_a, actor_a), (logical_b, actor_b) in itertools.combinations(items, 2):
        loc_a = actor_a.get_location()
        loc_b = actor_b.get_location()
        dist = distance_xy(loc_a, loc_b)
        if dist > args.near_miss_distance_m:
            continue
        rel_speed = relative_speed_toward_each_other(
            loc_a,
            actor_a.get_velocity(),
            loc_b,
            actor_b.get_velocity(),
        )
        if rel_speed < args.min_closing_speed_mps:
            continue
        pair = tuple(sorted((logical_a, logical_b)))
        last_time = cooldown.get(pair, -1e9)
        if prediction_time_s - last_time < args.near_miss_cooldown_s:
            continue
        cooldown[pair] = prediction_time_s

        center_x = (float(loc_a.x) + float(loc_b.x)) / 2.0
        center_y = (float(loc_a.y) + float(loc_b.y)) / 2.0
        center_z = (float(loc_a.z) + float(loc_b.z)) / 2.0
        risk_id = f"near_miss_{len(events) + 1:04d}"
        events.append(
            RiskEvent(
                prediction_id=risk_id,
                source_snapshot_time_s=source_snapshot_time_s,
                prediction_time_s=prediction_time_s,
                risk_type="near_miss",
                logical_actor_a=logical_a,
                logical_actor_b=logical_b,
                carla_actor_a=int(actor_a.id),
                carla_actor_b=int(actor_b.id),
                risk_x=center_x,
                risk_y=center_y,
                risk_z=center_z,
                distance_m=float(dist),
                relative_speed_mps=float(rel_speed),
                impulse_magnitude=0.0,
                note="xy distance and closing speed threshold",
            )
        )
        regions.append(
            RiskRegion(
                risk_id=risk_id,
                risk_type="near_miss",
                center_x=center_x,
                center_y=center_y,
                center_z=center_z,
                radius_m=float(args.near_miss_region_radius_m),
                start_time_s=prediction_time_s,
                end_time_s=prediction_time_s,
                logical_actor_a=logical_a,
                logical_actor_b=logical_b,
            )
        )


def run_prediction(args: argparse.Namespace, run_dir: Path, snapshot_path: Path, logger: logging.Logger) -> dict[str, Any]:
    snapshot_rows = read_actor_states_csv(snapshot_path)
    if not snapshot_rows:
        logger.warning("Snapshot has no rows: %s", snapshot_path)

    source_snapshot_time_s = 0.0
    if snapshot_rows:
        source_snapshot_time_s = float(snapshot_rows[0].get("sim_time_s", 0.0) or 0.0)

    client = connect_client(args.host, args.port, args.timeout)
    world = client.get_world()
    world = ensure_requested_map(client, world, args.map, args.reload_world, logger)
    traffic_manager = client.get_trafficmanager(args.tm_port)
    try:
        traffic_manager.set_random_device_seed(args.tm_seed)
    except Exception:
        pass

    created = CreatedActors()
    events: list[RiskEvent] = []
    regions: list[RiskRegion] = []
    prediction_start_time = 0.0

    def elapsed_prediction_time() -> float:
        return prediction_start_time

    reconstructed_path = run_dir / "reconstructed_states.csv"
    prediction_path = run_dir / "prediction_states.csv"
    risk_events_path = run_dir / "risk_events.csv"
    risk_regions_path = run_dir / "risk_regions.csv"

    logical_to_actor: dict[str, Any] = {}
    near_miss_cooldown: dict[tuple[str, str], float] = {}

    try:
        with SynchronousMode(world, traffic_manager, args.fixed_delta_seconds, logger):
            logical_to_actor = spawn_from_snapshot(world, snapshot_rows, created, logger)
            logger.info("Reconstructed %d vehicles from snapshot.", len(logical_to_actor))

            with CsvWriter(reconstructed_path, actor_state_fieldnames()) as reconstructed_writer:
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
                    reconstructed_writer.writerow(to_row(state))

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

            for logical, actor in logical_to_actor.items():
                try:
                    actor.set_autopilot(True, args.tm_port)
                    try:
                        traffic_manager.vehicle_percentage_speed_difference(actor, 0.0)
                    except Exception:
                        pass
                except Exception as exc:
                    logger.warning("Failed to enable autopilot for %s: %s", logical, exc)

            with CsvWriter(prediction_path, prediction_state_fieldnames()) as prediction_writer:
                steps = max(1, int(round(args.prediction_seconds / args.fixed_delta_seconds)))
                for _ in range(steps):
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
                        prediction_writer.writerow(
                            to_row(
                                prediction_state_from_actor(
                                    actor,
                                    frame=int(snap.frame),
                                    prediction_time_s=prediction_start_time,
                                    logical_actor_id=logical,
                                )
                            )
                        )

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
            str(reconstructed_path),
            str(prediction_path),
            str(risk_events_path),
            str(risk_regions_path),
        ],
    }


def build_config_summary(args: argparse.Namespace, run_dir: Path) -> dict[str, Any]:
    return {
        "mode": args.mode,
        "host": args.host,
        "port": args.port,
        "map": args.map,
        "reload_world": args.reload_world,
        "duration": args.duration,
        "snapshot_at": args.snapshot_at,
        "prediction_seconds": args.prediction_seconds,
        "fixed_delta_seconds": args.fixed_delta_seconds,
        "near_miss_distance_m": args.near_miss_distance_m,
        "min_closing_speed_mps": args.min_closing_speed_mps,
        "output_root": str(args.output_root),
        "run_dir": str(run_dir),
    }


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
        "carla_python_module": carla_module_description(),
        "map_name": args.map,
        "fixed_delta_seconds": args.fixed_delta_seconds,
        "errors_or_warnings": [],
    }

    try:
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
