"""Vehicle ground-contact audit and deterministic physics settling.

This module changes scenario initialization only.  It has no OGM dependency.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .phase1b5_diagnostics import SettleMonitor, clearance, transform_record


VEHICLE_LABELS = {"Car", "Truck", "Bus", "Motorcycle", "Bicycle"}


def _vector(value):
    return [float(value.x), float(value.y), float(value.z)]


def _location(carla, xyz):
    return carla.Location(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def road_query(world, x, y, *, start_z=5.0, end_z=-5.0):
    """Keep collision surface and waypoint reference as distinct values."""
    import carla
    start = np.array([x, y, start_z], dtype=float)
    end = np.array([x, y, end_z], dtype=float)
    hits = []
    available = hasattr(world, "cast_ray")
    if available:
        try:
            for hit in world.cast_ray(_location(carla, start), _location(carla, end)):
                point = np.array(_vector(hit.location))
                hits.append({"z": float(point[2]), "label": str(hit.label),
                             "distance": float(np.linalg.norm(point-start))})
            hits.sort(key=lambda item: item["distance"])
        except RuntimeError as exc:
            available = False
            hits = [{"error": str(exc)}]
    road = next((hit for hit in hits if hit.get("label") == "Roads"), None)
    waypoint = world.get_map().get_waypoint(
        carla.Location(x=float(x), y=float(y), z=.5), project_to_road=True)
    return {
        "cast_ray_available": available,
        "road_surface_z": None if road is None else road["z"],
        "waypoint_reference_z": None if waypoint is None else float(waypoint.transform.location.z),
        "first_hit": hits[0] if hits else None,
        "hits": hits,
        "road_surface_definition": "first Roads-labelled World.cast_ray hit from above",
    }


def actor_contact_record(world, actor, role, stage, requested_state):
    tf = actor.get_transform()
    vertices = np.asarray([_vector(v) for v in actor.bounding_box.get_world_vertices(tf)], dtype=float)
    bottom, top = float(vertices[:, 2].min()), float(vertices[:, 2].max())
    query = road_query(world, tf.location.x, tf.location.y,
                       start_z=max(5.0, top+3.0))
    control = actor.get_control()
    return {
        "role": role, "blueprint": actor.type_id, "actor_id": int(actor.id),
        "stage": stage, "transform": transform_record(tf),
        "physics_requested": bool(requested_state["physics"]),
        "gravity_requested": bool(requested_state["gravity"]),
        "velocity": _vector(actor.get_velocity()),
        "angular_velocity": _vector(actor.get_angular_velocity()),
        "control": {"throttle": float(control.throttle), "brake": float(control.brake),
                    "hand_brake": bool(control.hand_brake)},
        "obb_bottom_z": bottom, "obb_top_z": top,
        "road_surface_z": query["road_surface_z"],
        "waypoint_reference_z": query["waypoint_reference_z"],
        "clearance": clearance(bottom, query["road_surface_z"]),
        "road_query": query,
    }


def settle_vehicle(world, actor, role, dt, output_dir, *, max_seconds=10.0,
                   velocity_threshold=.01, delta_z_threshold=.001,
                   consecutive_required=20, freeze_after=True):
    import carla
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    actor.set_simulate_physics(True)
    actor.set_enable_gravity(True)
    actor.set_target_velocity(carla.Vector3D())
    actor.set_target_angular_velocity(carla.Vector3D())
    actor.apply_control(carla.VehicleControl(throttle=0., brake=1., hand_brake=True))
    monitor = SettleMonitor(velocity_threshold=velocity_threshold,
                            delta_z_threshold=delta_z_threshold,
                            consecutive_required=consecutive_required,
                            minimum_frames=consecutive_required)
    rows = []
    converged = False
    for _ in range(round(max_seconds/dt)):
        frame = world.tick()
        tf = actor.get_transform(); velocity = actor.get_velocity()
        converged = monitor.update(tf.location.z, velocity.z)
        rows.append({"frame": int(frame), "elapsed_seconds": monitor.frames*dt,
                     **transform_record(tf), "vz": float(velocity.z),
                     "consecutive_stable": int(monitor.consecutive)})
        if converged:
            break
    with (output_dir/f"{role}_settle_trace.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    if not converged:
        raise RuntimeError(f"{role} did not settle within {max_seconds} seconds")
    settled = transform_record(actor.get_transform())
    final_velocity = _vector(actor.get_velocity())
    if freeze_after:
        actor.set_target_velocity(carla.Vector3D())
        actor.set_target_angular_velocity(carla.Vector3D())
        actor.set_simulate_physics(False)
        world.tick()
    result = {
        "role": role, "converged": True, "settled_transform": settled,
        "settle_elapsed_frames": int(monitor.frames),
        "settle_elapsed_seconds": float(monitor.frames*dt),
        "final_velocity_before_freeze": final_velocity,
        "settle_threshold_vz": float(velocity_threshold),
        "settle_threshold_dz": float(delta_z_threshold),
        "settle_required_frames": int(consecutive_required),
        "physics_during_settle": True,
        "physics_during_measurement": not freeze_after,
        "grounding_method": "physics settle: gravity ON, zero velocities, brake=1, hand_brake ON; then physics OFF",
    }
    (output_dir/f"{role}_settle_result.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def ground_all_vehicles(world, actors_by_role, configured_transforms, dt, output_dir,
                        *, settings=None):
    """Audit and settle every supplied vehicle with one common procedure."""
    settings = dict(settings or {})
    output_dir = Path(output_dir); output_dir.mkdir(parents=True, exist_ok=True)
    records = {}
    for role, actor in actors_by_role.items():
        before = actor_contact_record(world, actor, role, "before_settle",
                                      {"physics": False, "gravity": True})
        settle = settle_vehicle(
            world, actor, role, dt, output_dir,
            max_seconds=float(settings.get("max_seconds", 10.0)),
            velocity_threshold=float(settings.get("velocity_threshold_mps", .01)),
            delta_z_threshold=float(settings.get("delta_z_threshold_m", .001)),
            consecutive_required=int(settings.get("consecutive_required_frames", 20)),
            freeze_after=True,
        )
        after = actor_contact_record(world, actor, role, "after_settle_static",
                                     {"physics": False, "gravity": True})
        records[role] = {
            "blueprint": actor.type_id,
            "original_spawn_transform": configured_transforms[role],
            "observed_before": before,
            "settled_transform": settle["settled_transform"],
            "observed_after": after,
            "road_surface_z": after["road_surface_z"],
            "clearance_before": before["clearance"],
            "clearance_after": after["clearance"],
            **{key: settle[key] for key in (
                "grounding_method", "settle_threshold_vz", "settle_threshold_dz",
                "settle_required_frames", "settle_elapsed_frames",
                "settle_elapsed_seconds", "physics_during_settle",
                "physics_during_measurement")},
        }
    metadata = {
        "schema_version": "1.0", "grounded": True,
        "all_vehicle_roles": list(actors_by_role),
        "vehicle_actor_count": len(actors_by_role),
        "actors": records,
    }
    (output_dir/"grounding_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    return metadata


def transforms_reproducible(a, b, *, xy_tolerance=.001, z_tolerance=.001,
                            yaw_tolerance=.01):
    delta = {key: abs(float(a[key])-float(b[key])) for key in ("x", "y", "z", "yaw")}
    passed = (delta["x"] <= xy_tolerance and delta["y"] <= xy_tolerance and
              delta["z"] <= z_tolerance and delta["yaw"] <= yaw_tolerance)
    return passed, delta
