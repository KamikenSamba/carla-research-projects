"""Sensor-label-agnostic OGM update engine.

Phase 1b.6 is a refactor only.  Callers construct configs from their existing
constants and provide the unchanged grid conversion/rasterisation functions.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np

from .height_slab_free_space import trace_height_slab_ray


@dataclass(frozen=True)
class OGMUpdateConfig:
    z_min: float
    z_max: float
    lidar_range: float
    free_logodds: float
    occupied_logodds: float
    free_scale: float
    logodds_min: float
    logodds_max: float
    known_free_threshold: float
    occupied_threshold: float
    decay_rate: float
    initial_logodds: float = 0.0


@dataclass(frozen=True)
class OGMGridContract:
    world_to_grid: Callable[[float, float], tuple[int, int]]
    in_bounds: Callable[[int, int], bool]
    bresenham: Callable[[int, int, int, int], Iterable[tuple[int, int]]]


@dataclass
class OGMUpdateStats:
    raw_returns: int = 0
    z_pass: int = 0
    z_reject: int = 0
    free_updates: int = 0
    occupied_updates: int = 0
    extra_free_updates: int = 0
    free_touched_cells: int = 0
    occupied_touched_cells: int = 0

    def as_dict(self) -> dict[str, int]:
        return {name: int(getattr(self, name)) for name in self.__dataclass_fields__}


@dataclass(frozen=True)
class OGMUpdateResult:
    z_pass_mask: np.ndarray
    free_touched: np.ndarray
    occupied_touched: np.ndarray
    stats: OGMUpdateStats


def validate_config(config: OGMUpdateConfig) -> None:
    values = tuple(float(getattr(config, name)) for name in config.__dataclass_fields__)
    if not np.isfinite(values).all():
        raise ValueError("OGM config contains NaN or Inf")
    if config.z_min >= config.z_max:
        raise ValueError("z_min must be below z_max")
    if config.lidar_range <= 0:
        raise ValueError("lidar_range must be positive")
    if config.logodds_min >= config.logodds_max:
        raise ValueError("logodds_min must be below logodds_max")


def z_filter_mask(points_world: np.ndarray, config: OGMUpdateConfig) -> np.ndarray:
    points = np.asarray(points_world)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points_world must have shape (N, >=3)")
    if not np.isfinite(points[:, :3]).all():
        raise ValueError("points_world contains NaN or Inf")
    return (points[:, 2] > config.z_min) & (points[:, 2] < config.z_max)


def update_valid_returns(
    logodds: np.ndarray,
    points_world: np.ndarray,
    sensor_origin_world,
    grid: OGMGridContract,
    config: OGMUpdateConfig,
    *,
    free_touched: np.ndarray | None = None,
    occupied_touched: np.ndarray | None = None,
) -> OGMUpdateStats:
    """Apply the unchanged standard Cooperative valid-return update."""
    validate_config(config)
    points = np.asarray(points_world)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points_world must have shape (N, >=3)")
    if not np.isfinite(points[:, :3]).all():
        raise ValueError("points_world contains NaN or Inf")
    ox, oy = sensor_origin_world[0], sensor_origin_world[1]
    ix0, iy0 = grid.world_to_grid(ox, oy)
    stats = OGMUpdateStats()
    if not grid.in_bounds(ix0, iy0):
        return stats

    range_sq = config.lidar_range * config.lidar_range
    for xw, yw, _zw in points[:, :3]:
        dx = xw - ox
        dy = yw - oy
        if dx * dx + dy * dy > range_sq:
            continue
        ix1, iy1 = grid.world_to_grid(xw, yw)
        if not grid.in_bounds(ix1, iy1):
            continue
        for cx, cy in grid.bresenham(ix0, iy0, ix1, iy1):
            if cx == ix1 and cy == iy1:
                logodds[cy, cx] = np.clip(
                    logodds[cy, cx] + config.occupied_logodds,
                    config.logodds_min, config.logodds_max,
                )
                stats.occupied_updates += 1
                if occupied_touched is not None:
                    occupied_touched[cy, cx] = True
            else:
                logodds[cy, cx] = np.clip(
                    logodds[cy, cx] + config.free_scale * config.free_logodds,
                    config.logodds_min, config.logodds_max,
                )
                stats.free_updates += 1
                if free_touched is not None:
                    free_touched[cy, cx] = True
    return stats


def update_extra_free_from_rejected_returns(
    logodds: np.ndarray,
    rejected_points_world: np.ndarray,
    sensor_origin_world,
    grid: OGMGridContract,
    config: OGMUpdateConfig,
    *,
    free_touched: np.ndarray | None = None,
    free_update_counts: np.ndarray | None = None,
) -> OGMUpdateStats:
    """Apply the unchanged Phase 1a rejected-return height-slab Free update."""
    points = np.asarray(rejected_points_world)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("rejected_points_world must have shape (N, >=3)")
    stats = OGMUpdateStats()
    for hit in points[:, :3]:
        updates = trace_height_slab_ray(
            sensor_origin_world, hit, config.z_min, config.z_max,
            config.lidar_range, grid.world_to_grid, grid.in_bounds,
            grid.bresenham,
        )
        for cx, cy in updates.free_cells:
            logodds[cy, cx] = np.clip(
                logodds[cy, cx] + config.free_scale * config.free_logodds,
                config.logodds_min, config.logodds_max,
            )
            stats.free_updates += 1
            stats.extra_free_updates += 1
            if free_touched is not None:
                free_touched[cy, cx] = True
            if free_update_counts is not None:
                free_update_counts[cy, cx] += 1
    return stats


def update_lidar_measurement(
    logodds: np.ndarray,
    points_world: np.ndarray,
    sensor_origin_world,
    grid: OGMGridContract,
    config: OGMUpdateConfig,
    *,
    include_rejected_free: bool = False,
) -> OGMUpdateResult:
    """Filter and update one measurement without consulting a sensor label."""
    points = np.asarray(points_world)
    passed = z_filter_mask(points, config)
    free_touched = np.zeros(logodds.shape, dtype=bool)
    occupied_touched = np.zeros(logodds.shape, dtype=bool)
    stats = OGMUpdateStats(
        raw_returns=len(points), z_pass=int(passed.sum()),
        z_reject=int((~passed).sum()),
    )
    valid_stats = update_valid_returns(
        logodds, points[passed, :3], sensor_origin_world, grid, config,
        free_touched=free_touched, occupied_touched=occupied_touched,
    )
    stats.free_updates += valid_stats.free_updates
    stats.occupied_updates += valid_stats.occupied_updates
    if include_rejected_free:
        extra = update_extra_free_from_rejected_returns(
            logodds, points[~passed, :3], sensor_origin_world, grid, config,
            free_touched=free_touched,
        )
        stats.free_updates += extra.free_updates
        stats.extra_free_updates += extra.extra_free_updates
    stats.free_touched_cells = int(free_touched.sum())
    stats.occupied_touched_cells = int(occupied_touched.sum())
    return OGMUpdateResult(passed, free_touched, occupied_touched, stats)


def apply_decay(logodds: np.ndarray, dt: float, config: OGMUpdateConfig) -> None:
    """Apply the existing in-place linear decay and clipping expression."""
    if config.decay_rate <= 0 or dt <= 0:
        return
    logodds += (config.initial_logodds - logodds) * (config.decay_rate * dt)
    np.clip(logodds, config.logodds_min, config.logodds_max, out=logodds)


def probability_from_logodds(logodds: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-logodds))


def known_masks(logodds: np.ndarray, config: OGMUpdateConfig):
    probability = probability_from_logodds(logodds)
    occupied = probability >= config.occupied_threshold
    free = probability <= config.known_free_threshold
    unknown = ~(occupied | free)
    return free, occupied, unknown


def classify_logodds(logodds: np.ndarray, config: OGMUpdateConfig) -> np.ndarray:
    free, occupied, _unknown = known_masks(logodds, config)
    labels = np.zeros(logodds.shape, dtype=np.uint8)
    labels[free] = 1
    labels[occupied] = 2
    return labels

