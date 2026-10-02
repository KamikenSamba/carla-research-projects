"""Phase-1 3D height-slab ray carving for the cooperative OGM.

This module intentionally contains no policy changes to log-odds values,
thresholds, decay, communication, or fusion.  Callers provide those existing
values and the legacy grid helpers explicitly.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


@dataclass(frozen=True)
class RayGridUpdates:
    free_cells: tuple[tuple[int, int], ...]
    occupied_cell: tuple[int, int] | None
    endpoint_in_slab: bool
    endpoint_out_of_bounds: bool = False
    ray_out_of_bounds: bool = False


def ray_segment_in_height_slab(sensor_xyz, hit_xyz, z_min, z_max):
    """Return the part of S+t(H-S), 0<=t<=1, strictly inside a z slab.

    Boundary points may appear as returned geometric endpoints, but only when
    the open interval immediately next to them lies inside the slab.  The
    endpoints are used for XY rasterisation, not as occupied z samples.
    """
    sensor = np.asarray(sensor_xyz, dtype=np.float64)
    hit = np.asarray(hit_xyz, dtype=np.float64)
    if sensor.shape != (3,) or hit.shape != (3,):
        raise ValueError("sensor_xyz and hit_xyz must have shape (3,)")
    if not np.isfinite(sensor).all() or not np.isfinite(hit).all():
        raise ValueError("ray endpoints must be finite")
    z_min, z_max = float(z_min), float(z_max)
    if not np.isfinite([z_min, z_max]).all() or z_min >= z_max:
        raise ValueError("height slab bounds must be finite and increasing")

    dz = hit[2] - sensor[2]
    if dz == 0.0:
        if z_min < sensor[2] < z_max:
            return sensor.copy(), hit.copy()
        return None

    t_a = (z_min - sensor[2]) / dz
    t_b = (z_max - sensor[2]) / dz
    t_enter = max(0.0, min(t_a, t_b))
    t_exit = min(1.0, max(t_a, t_b))
    if t_enter >= t_exit:
        return None
    direction = hit - sensor
    return sensor + t_enter * direction, sensor + t_exit * direction


def trace_height_slab_ray(
    sensor_xyz,
    hit_xyz,
    z_min,
    z_max,
    lidar_range,
    world_to_grid: Callable[[float, float], tuple[int, int]],
    in_bounds: Callable[[int, int], bool],
    bresenham: Callable[[int, int, int, int], Iterable[tuple[int, int]]],
) -> RayGridUpdates:
    """Project only the ray portion in the occupancy-height slab.

    The legacy XY contract is preserved: the sensor origin and raw return
    endpoint must both be in the Grid and the return must be within range.
    """
    sensor = np.asarray(sensor_xyz, dtype=np.float64)
    hit = np.asarray(hit_xyz, dtype=np.float64)
    if sensor.shape != (3,) or hit.shape != (3,):
        raise ValueError("sensor_xyz and hit_xyz must have shape (3,)")
    if not np.isfinite(sensor).all() or not np.isfinite(hit).all():
        raise ValueError("ray endpoints must be finite")
    lidar_range = float(lidar_range)
    if not np.isfinite(lidar_range) or lidar_range <= 0:
        raise ValueError("lidar_range must be finite and positive")

    origin_cell = world_to_grid(float(sensor[0]), float(sensor[1]))
    endpoint_cell = world_to_grid(float(hit[0]), float(hit[1]))
    endpoint_in_slab = bool(float(z_min) < hit[2] < float(z_max))
    if not in_bounds(*origin_cell):
        return RayGridUpdates((), None, endpoint_in_slab, ray_out_of_bounds=True)
    if not in_bounds(*endpoint_cell):
        return RayGridUpdates((), None, endpoint_in_slab,
                              endpoint_out_of_bounds=True, ray_out_of_bounds=True)
    delta_xy = hit[:2] - sensor[:2]
    if float(delta_xy @ delta_xy) > lidar_range * lidar_range:
        return RayGridUpdates((), None, endpoint_in_slab)

    segment = ray_segment_in_height_slab(sensor, hit, z_min, z_max)
    if segment is None:
        return RayGridUpdates((), None, endpoint_in_slab)
    start, end = segment
    start_cell = world_to_grid(float(start[0]), float(start[1]))
    end_cell = endpoint_cell if endpoint_in_slab else world_to_grid(float(end[0]), float(end[1]))
    if not in_bounds(*start_cell) or not in_bounds(*end_cell):
        # Both original XY endpoints are in the rectangular Grid, so this is
        # only possible through a nonstandard legacy mapping convention.
        return RayGridUpdates((), None, endpoint_in_slab, ray_out_of_bounds=True)

    cells = tuple(bresenham(*start_cell, *end_cell))
    if endpoint_in_slab:
        # Preserve intermediate-Free / endpoint-Occupied and never apply both
        # updates to the endpoint cell.
        return RayGridUpdates(cells[:-1], endpoint_cell, True)
    return RayGridUpdates(cells, None, False)


def update_from_points_height_slab(
    points_xyz_world,
    sensor_xyz_world,
    target_logodds,
    *,
    z_min,
    z_max,
    lidar_range,
    free_logodds,
    occupied_logodds,
    free_scale,
    logodds_min,
    logodds_max,
    world_to_grid,
    in_bounds,
    bresenham,
):
    """Apply Phase-1 updates using caller-supplied, unchanged OGM constants."""
    points = np.asarray(points_xyz_world)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points_xyz_world must have shape (N, >=3)")
    if not np.isfinite(points[:, :3]).all():
        raise ValueError("points_xyz_world contains NaN or Inf")
    for hit in points[:, :3]:
        updates = trace_height_slab_ray(
            sensor_xyz_world, hit, z_min, z_max, lidar_range,
            world_to_grid, in_bounds, bresenham,
        )
        for cx, cy in updates.free_cells:
            target_logodds[cy, cx] = np.clip(
                target_logodds[cy, cx] + free_scale * free_logodds,
                logodds_min, logodds_max,
            )
        if updates.occupied_cell is not None:
            cx, cy = updates.occupied_cell
            target_logodds[cy, cx] = np.clip(
                target_logodds[cy, cx] + occupied_logodds,
                logodds_min, logodds_max,
            )
