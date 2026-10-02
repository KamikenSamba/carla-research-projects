"""Opt-in, read-only diagnostics for the cooperative occupancy-grid pipeline.

The collector deliberately owns no OGM update operation.  It observes the same
inputs as the current inverse sensor model and writes only private diagnostic
arrays and artifacts.
"""
from __future__ import annotations

import csv
import json
import math
import re
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

import numpy as np

from .grid_contract import GridContract
from .logodds import FREE_TH, OCC_TH
from .risk_priority import load_road_mask


FRAME_COLUMNS = [
    "carla_frame", "sensor_name", "timestamp", "raw_return_count",
    "world_transformed_count", "z_filter_pass_count", "z_filter_reject_count",
    "z_below_lower_count", "z_inside_slab_count", "z_above_upper_count",
    "world_z_min", "world_z_max", "world_z_mean", "world_z_median",
    "world_z_p05", "world_z_p25", "world_z_p75", "world_z_p95",
    "free_ray_count", "occupied_endpoint_count",
    "free_cell_update_count_total", "free_cell_update_count_unique",
    "occupied_cell_update_count_total", "occupied_cell_update_count_unique",
    "grid_out_of_bounds_endpoint_count", "grid_out_of_bounds_ray_count",
    "rejected_return_count", "rejected_ray_potential_free_updates_total",
    "rejected_ray_potential_free_cells_unique",
    "phase1_added_free_updates_total", "phase1_added_free_cells_unique",
]


def _finite_float(value: object, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _z_statistics(z: np.ndarray) -> dict[str, float | None]:
    if z.size == 0:
        return {name: None for name in ("min", "max", "mean", "median", "p05", "p25", "p75", "p95")}
    if not np.isfinite(z).all():
        raise ValueError("WORLD z values contain NaN or Inf")
    q = np.percentile(z, [5, 25, 75, 95])
    return {
        "min": float(np.min(z)), "max": float(np.max(z)),
        "mean": float(np.mean(z)), "median": float(np.median(z)),
        "p05": float(q[0]), "p25": float(q[1]),
        "p75": float(q[2]), "p95": float(q[3]),
    }


def _classify(logodds: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not np.isfinite(logodds).all():
        raise ValueError("logodds contains NaN or Inf")
    probability = 1.0 / (1.0 + np.exp(-logodds))
    occupied = probability >= OCC_TH
    free = probability <= FREE_TH
    return occupied, free, ~(occupied | free)


@dataclass
class _SensorState:
    shape: tuple[int, int]
    free_touched: np.ndarray = field(init=False)
    free_update_count: np.ndarray = field(init=False)
    occupied_touched: np.ndarray = field(init=False)
    potential_free: np.ndarray = field(init=False)
    original_valid_free: np.ndarray = field(init=False)
    phase1_added_free: np.ndarray = field(init=False)
    phase1_added_free_update_count: np.ndarray = field(init=False)
    frames: list[dict[str, object]] = field(default_factory=list)
    world_z_chunks: list[np.ndarray] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.free_touched = np.zeros(self.shape, dtype=bool)
        self.free_update_count = np.zeros(self.shape, dtype=np.uint32)
        self.occupied_touched = np.zeros(self.shape, dtype=bool)
        self.potential_free = np.zeros(self.shape, dtype=bool)
        self.original_valid_free = np.zeros(self.shape, dtype=bool)
        self.phase1_added_free = np.zeros(self.shape, dtype=bool)
        self.phase1_added_free_update_count = np.zeros(self.shape, dtype=np.uint32)


class OGMDiagnostics:
    """Accumulate per-measurement evidence without changing OGM arrays."""

    def __init__(
        self,
        output_dir: str | Path,
        grid: GridContract,
        road_mask_path: str | Path,
        road_mask_metadata_path: str | Path,
        *,
        z_min_world: float,
        z_max_world: float,
        lidar_range: float,
        world_to_grid: Callable[[float, float], tuple[int, int]],
        in_bounds: Callable[[int, int], bool],
        bresenham: Callable[[int, int, int, int], Iterable[tuple[int, int]]],
        experiment: dict[str, object] | None = None,
        free_space_height_slab: bool = False,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.grid = grid
        self.road_mask, self.road_mask_metadata = load_road_mask(
            road_mask_path, road_mask_metadata_path, grid
        )
        if self.road_mask.shape != grid.shape:
            raise ValueError("road mask/Grid shape mismatch")
        self.z_min = _finite_float(z_min_world, "z_min_world")
        self.z_max = _finite_float(z_max_world, "z_max_world")
        self.lidar_range = _finite_float(lidar_range, "lidar_range")
        if self.z_min >= self.z_max or self.lidar_range <= 0:
            raise ValueError("invalid z filter or LiDAR range")
        self.world_to_grid = world_to_grid
        self.in_bounds = in_bounds
        self.bresenham = bresenham
        self.experiment = dict(experiment or {})
        self.free_space_height_slab = bool(free_space_height_slab)
        self.sensors = {name: _SensorState(grid.shape) for name in ("ego", "rsu")}
        self.lidar_config: dict[str, object] = {}
        self._lock = threading.Lock()

    def set_lidar_config(self, config: dict[str, object]) -> None:
        # JSON round-trip also rejects non-serializable CARLA proxy objects.
        self.lidar_config = json.loads(json.dumps(config, allow_nan=False))

    def observe_measurement(
        self,
        sensor_name: str,
        *,
        carla_frame: int,
        timestamp: float,
        raw_return_count: int,
        points_world: np.ndarray,
        z_pass_mask: np.ndarray,
        origin_xy_world: tuple[float, float],
        origin_xyz_world: tuple[float, float, float] | None = None,
    ) -> None:
        state = self._state(sensor_name)
        points = np.asarray(points_world)
        passed = np.asarray(z_pass_mask, dtype=bool)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points_world must have shape (N, >=3)")
        if passed.shape != (points.shape[0],):
            raise ValueError("z_pass_mask shape mismatch")
        if int(raw_return_count) != points.shape[0]:
            raise ValueError("raw/world-transformed return count mismatch")
        if not np.isfinite(points).all():
            raise ValueError("WORLD points contain NaN or Inf")
        frame = int(carla_frame)
        stamp = _finite_float(timestamp, "timestamp")
        z = points[:, 2]
        expected = (z > self.z_min) & (z < self.z_max)
        if not np.array_equal(passed, expected):
            raise ValueError("z_pass_mask differs from configured current z filter")

        if self.free_space_height_slab:
            if origin_xyz_world is None:
                raise ValueError("Phase 1 diagnostics require origin_xyz_world")
            valid_metrics, valid_free, valid_occ, valid_counts = self._trace_phase1(
                points[passed], origin_xyz_world)
            rejected_metrics, rejected_free, _, rejected_counts = self._trace_phase1(
                points[~passed], origin_xyz_world)
            current_free = valid_free | rejected_free
            current_counts = valid_counts + rejected_counts
        else:
            valid_metrics, valid_free, valid_occ = self._trace(points[passed], origin_xy_world)
            rejected_metrics, rejected_free, _ = self._trace(points[~passed], origin_xy_world)
            current_free = valid_free
            current_counts = None
        z_stats = _z_statistics(z)
        row: dict[str, object] = {
            "carla_frame": frame, "sensor_name": sensor_name, "timestamp": stamp,
            "raw_return_count": int(raw_return_count),
            "world_transformed_count": int(points.shape[0]),
            "z_filter_pass_count": int(passed.sum()),
            "z_filter_reject_count": int((~passed).sum()),
            "z_below_lower_count": int((z <= self.z_min).sum()),
            "z_inside_slab_count": int(passed.sum()),
            "z_above_upper_count": int((z >= self.z_max).sum()),
            **{f"world_z_{key}": value for key, value in z_stats.items()},
            "free_ray_count": valid_metrics["valid_ray_count"] + (
                rejected_metrics["valid_ray_count"] if self.free_space_height_slab else 0),
            "occupied_endpoint_count": valid_metrics["occupied_endpoint_count"],
            "free_cell_update_count_total": valid_metrics["free_updates_total"] + (
                rejected_metrics["free_updates_total"] if self.free_space_height_slab else 0),
            "free_cell_update_count_unique": int(current_free.sum()),
            "occupied_cell_update_count_total": valid_metrics["occupied_endpoint_count"],
            "occupied_cell_update_count_unique": int(valid_occ.sum()),
            "grid_out_of_bounds_endpoint_count": valid_metrics["out_of_bounds_endpoint_count"],
            "grid_out_of_bounds_ray_count": valid_metrics["out_of_bounds_ray_count"],
            "rejected_return_count": int((~passed).sum()),
            "rejected_ray_potential_free_updates_total": rejected_metrics["free_updates_total"],
            "rejected_ray_potential_free_cells_unique": int(rejected_free.sum()),
            "phase1_added_free_updates_total": (
                rejected_metrics["free_updates_total"] if self.free_space_height_slab else 0),
            "phase1_added_free_cells_unique": int(rejected_free.sum()) if self.free_space_height_slab else 0,
        }
        with self._lock:
            state.frames.append(row)
            state.world_z_chunks.append(z.astype(np.float64, copy=True))
            state.free_touched |= current_free
            state.occupied_touched |= valid_occ
            state.potential_free |= rejected_free
            state.original_valid_free |= valid_free
            if self.free_space_height_slab:
                state.phase1_added_free |= rejected_free
                state.phase1_added_free_update_count += rejected_counts
                state.free_update_count += current_counts
            else:
                # Re-run just the accepted traversals to retain multiplicity.
                # This touches only the private uint32 diagnostic array.
                self._add_free_counts(state.free_update_count, points[passed], origin_xy_world)

    def _trace(self, points: np.ndarray, origin_xy: tuple[float, float]):
        free = np.zeros(self.grid.shape, dtype=bool)
        occupied = np.zeros(self.grid.shape, dtype=bool)
        metrics = dict(valid_ray_count=0, occupied_endpoint_count=0, free_updates_total=0,
                       out_of_bounds_endpoint_count=0, out_of_bounds_ray_count=0)
        ox, oy = map(float, origin_xy)
        ix0, iy0 = self.world_to_grid(ox, oy)
        origin_ok = self.in_bounds(ix0, iy0)
        range_sq = self.lidar_range * self.lidar_range
        for point in points:
            xw, yw = float(point[0]), float(point[1])
            if (xw - ox) ** 2 + (yw - oy) ** 2 > range_sq:
                continue
            ix1, iy1 = self.world_to_grid(xw, yw)
            if not self.in_bounds(ix1, iy1):
                metrics["out_of_bounds_endpoint_count"] += 1
                metrics["out_of_bounds_ray_count"] += 1
                continue
            if not origin_ok:
                metrics["out_of_bounds_ray_count"] += 1
                continue
            metrics["valid_ray_count"] += 1
            metrics["occupied_endpoint_count"] += 1
            for cx, cy in self.bresenham(ix0, iy0, ix1, iy1):
                if cx == ix1 and cy == iy1:
                    occupied[cy, cx] = True
                else:
                    free[cy, cx] = True
                    metrics["free_updates_total"] += 1
        return metrics, free, occupied

    def _add_free_counts(self, target: np.ndarray, points: np.ndarray, origin_xy: tuple[float, float]) -> None:
        ox, oy = map(float, origin_xy)
        ix0, iy0 = self.world_to_grid(ox, oy)
        if not self.in_bounds(ix0, iy0):
            return
        range_sq = self.lidar_range * self.lidar_range
        for point in points:
            xw, yw = float(point[0]), float(point[1])
            if (xw - ox) ** 2 + (yw - oy) ** 2 > range_sq:
                continue
            ix1, iy1 = self.world_to_grid(xw, yw)
            if not self.in_bounds(ix1, iy1):
                continue
            for cx, cy in self.bresenham(ix0, iy0, ix1, iy1):
                if cx == ix1 and cy == iy1:
                    break
                if target[cy, cx] != np.iinfo(target.dtype).max:
                    target[cy, cx] += 1

    def _trace_phase1(self, points: np.ndarray, origin_xyz: tuple[float, float, float]):
        from .height_slab_free_space import trace_height_slab_ray

        free = np.zeros(self.grid.shape, dtype=bool)
        occupied = np.zeros(self.grid.shape, dtype=bool)
        counts = np.zeros(self.grid.shape, dtype=np.uint32)
        metrics = dict(valid_ray_count=0, occupied_endpoint_count=0, free_updates_total=0,
                       out_of_bounds_endpoint_count=0, out_of_bounds_ray_count=0)
        for point in points:
            updates = trace_height_slab_ray(
                origin_xyz, point[:3], self.z_min, self.z_max, self.lidar_range,
                self.world_to_grid, self.in_bounds, self.bresenham,
            )
            metrics["out_of_bounds_endpoint_count"] += int(updates.endpoint_out_of_bounds)
            metrics["out_of_bounds_ray_count"] += int(updates.ray_out_of_bounds)
            if updates.free_cells or updates.occupied_cell is not None:
                metrics["valid_ray_count"] += 1
            for cx, cy in updates.free_cells:
                free[cy, cx] = True
                counts[cy, cx] += 1
                metrics["free_updates_total"] += 1
            if updates.occupied_cell is not None:
                cx, cy = updates.occupied_cell
                occupied[cy, cx] = True
                metrics["occupied_endpoint_count"] += 1
        return metrics, free, occupied, counts

    def finalize(self, ego_logodds: np.ndarray, rsu_logodds: np.ndarray,
                 gt_occupied_mask: np.ndarray | None = None) -> dict[str, object]:
        arrays = {"ego": np.asarray(ego_logodds), "rsu": np.asarray(rsu_logodds)}
        for name, arr in arrays.items():
            if arr.shape != self.grid.shape:
                raise ValueError(f"{name} logodds/Grid shape mismatch")

        # Guard the central Phase-0 invariant: artifact generation must not
        # mutate either caller-owned OGM array.
        before = {name: arr.copy() for name, arr in arrays.items()}
        if gt_occupied_mask is not None:
            gt_occupied_mask = np.asarray(gt_occupied_mask)
            if gt_occupied_mask.shape != self.grid.shape or gt_occupied_mask.dtype != np.bool_:
                raise ValueError("GT occupied mask must be bool with Grid shape")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        summaries: dict[str, object] = {}
        for name, arr in arrays.items():
            state = self.sensors[name]
            occupied, free, unknown = _classify(arr)
            observed = state.free_touched | state.occupied_touched
            np.save(self.output_dir / f"{name}_free_touched_mask.npy", state.free_touched)
            np.save(self.output_dir / f"{name}_free_update_count.npy", state.free_update_count)
            np.save(self.output_dir / f"{name}_observed_by_current_model.npy", observed)
            np.save(self.output_dir / f"{name}_unknown_mask.npy", unknown)
            np.save(self.output_dir / f"{name}_potential_free_from_z_rejected.npy", state.potential_free)
            np.save(self.output_dir / f"{name}_free_from_original_valid_return.npy", state.original_valid_free)
            np.save(self.output_dir / f"{name}_free_from_z_rejected_return.npy", state.phase1_added_free)
            np.save(self.output_dir / f"{name}_phase1_added_free_update_count.npy",
                    state.phase1_added_free_update_count)
            false_free = ((gt_occupied_mask & free) if gt_occupied_mask is not None
                          else np.zeros(self.grid.shape, dtype=bool))
            np.save(self.output_dir / f"{name}_false_free_mask.npy", false_free)
            summaries[name] = self._sensor_summary(
                state, occupied, free, unknown, observed, gt_occupied_mask, false_free)
            self._render(name, arr, unknown, state, observed)

        if gt_occupied_mask is not None:
            np.save(self.output_dir / "actor_ground_truth_occupied_mask.npy", gt_occupied_mask)
        np.save(self.output_dir / "ego_logodds.npy", arrays["ego"])
        np.save(self.output_dir / "rsu_logodds.npy", arrays["rsu"])

        rows = [row for name in ("ego", "rsu") for row in self.sensors[name].frames]
        rows.sort(key=lambda row: (int(row["carla_frame"]), str(row["sensor_name"])))
        with (self.output_dir / "ogm_diagnostics_frames.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FRAME_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        (self.output_dir / "lidar_config.json").write_text(
            json.dumps(self.lidar_config, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8"
        )
        invariant = {f"{name}_logodds_unchanged_by_finalize": bool(np.array_equal(before[name], arr))
                     for name, arr in arrays.items()}
        summary = {
            "schema_version": "1.0",
            "phase": ("Phase 1 height-slab Free ray" if self.free_space_height_slab
                      else "Phase 0 observation only"),
            "experiment": self.experiment, "grid": self.grid.metadata(),
            "z_filter": {"lower_exclusive_m": self.z_min, "upper_exclusive_m": self.z_max},
            "known_thresholds": {"occupied_probability_gte": OCC_TH, "free_probability_lte": FREE_TH},
            "arrays": self._array_manifest(), "sensors": summaries,
            "invariance": invariant,
            "free_space_height_slab_enabled": self.free_space_height_slab,
            "interpretation_note": (
                "unknown_untouched_by_current_model is not a no-return classification; it may include "
                "occlusion, FOV, sampling, drop-off, Grid limits, and z-filter effects."
            ),
        }
        (self.output_dir / "ogm_diagnostics_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8"
        )
        if not all(invariant.values()):
            raise RuntimeError("diagnostic finalization changed an OGM array")
        return summary

    def _sensor_summary(self, state, occupied, free, unknown, observed,
                        gt_occupied_mask, false_free):
        potential = state.potential_free
        touched = state.free_touched
        road = self.road_mask
        totals = {key: int(sum(int(row[key]) for row in state.frames)) for key in (
            "raw_return_count", "world_transformed_count", "z_filter_pass_count", "z_filter_reject_count",
            "free_ray_count", "occupied_endpoint_count", "free_cell_update_count_total",
            "occupied_cell_update_count_total", "grid_out_of_bounds_endpoint_count",
            "grid_out_of_bounds_ray_count", "rejected_return_count",
            "rejected_ray_potential_free_updates_total")}
        totals.update(
            measurement_count=len(state.frames),
            free_cell_update_count_unique=int(touched.sum()),
            occupied_cell_update_count_unique=int(state.occupied_touched.sum()),
            rejected_ray_potential_free_cells_unique=int(potential.sum()),
            potential_free_but_current_unknown_cells=int((potential & unknown).sum()),
            potential_free_and_current_known_free_cells=int((potential & free).sum()),
            potential_free_and_current_occupied_cells=int((potential & occupied).sum()),
            free_touched_and_known_free=int((touched & free).sum()),
            free_touched_but_unknown=int((touched & unknown).sum()),
            free_touched_but_occupied=int((touched & occupied).sum()),
            unknown_total=int(unknown.sum()),
            unknown_but_observed=int((unknown & observed).sum()),
            unknown_untouched_by_current_model=int((unknown & ~observed).sum()),
            known_free_total=int(free.sum()), occupied_total=int(occupied.sum()),
        )
        added = state.phase1_added_free
        totals.update(
            phase1_added_free_updates_total=int(state.phase1_added_free_update_count.sum()),
            phase1_added_free_cells_unique=int(added.sum()),
            phase1_added_free_to_known_free=int((added & free).sum()),
            phase1_added_free_still_unknown=int((added & unknown).sum()),
            phase1_added_free_to_occupied=int((added & occupied).sum()),
        )
        all_z = (np.concatenate(state.world_z_chunks) if state.world_z_chunks
                 else np.empty(0, dtype=np.float64))
        totals["world_z_distribution"] = _z_statistics(all_z)
        totals["z_filter_reject_rate"] = (
            totals["z_filter_reject_count"] / totals["raw_return_count"]
            if totals["raw_return_count"] else 0.0
        )
        totals["road"] = {
            "road_total_cells": int(road.sum()), "road_unknown": int((road & unknown).sum()),
            "road_known_free": int((road & free).sum()), "road_occupied": int((road & occupied).sum()),
            "road_free_touched_but_unknown": int((road & touched & unknown).sum()),
            "road_potential_free_but_current_unknown": int((road & potential & unknown).sum()),
            "road_unknown_untouched_by_current_model": int((road & unknown & ~observed).sum()),
            "road_phase1_added_free_cells_unique": int((road & added).sum()),
            "road_phase1_added_free_to_known_free": int((road & added & free).sum()),
            "road_phase1_added_free_still_unknown": int((road & added & unknown).sum()),
            "road_phase1_added_free_to_occupied": int((road & added & occupied).sum()),
        }
        gt_count = int(gt_occupied_mask.sum()) if gt_occupied_mask is not None else 0
        false_count = int(false_free.sum())
        totals["vehicle_ground_truth"] = {
            "available": gt_occupied_mask is not None,
            "gt_occupied_cells": gt_count,
            "false_free_cells": false_count,
            "false_free_rate": false_count / gt_count if gt_count else 0.0,
        }
        buckets: dict[str, dict[str, int]] = {}
        for count in range(1, 10):
            mask = state.free_update_count == count
            buckets[str(count)] = {"known_free": int((mask & free).sum()),
                                   "unknown": int((mask & unknown).sum()),
                                   "occupied": int((mask & occupied).sum())}
        mask = state.free_update_count >= 10
        buckets["10+"] = {"known_free": int((mask & free).sum()),
                           "unknown": int((mask & unknown).sum()),
                           "occupied": int((mask & occupied).sum())}
        totals["free_update_count_final_state"] = buckets
        unknown_total = totals["unknown_total"]
        totals["phase0_unknown_cause_candidates"] = {
            "A_z_rejected_potential_free_current_unknown": totals["potential_free_but_current_unknown_cells"],
            "B_free_touched_but_unknown": totals["free_touched_but_unknown"],
            "C_unknown_untouched_by_current_model": totals["unknown_untouched_by_current_model"],
            "shares_of_unknown": {
                "A": totals["potential_free_but_current_unknown_cells"] / unknown_total if unknown_total else 0.0,
                "B": totals["free_touched_but_unknown"] / unknown_total if unknown_total else 0.0,
                "C": totals["unknown_untouched_by_current_model"] / unknown_total if unknown_total else 0.0,
            },
            "overlap_note": "A is potential evidence and can overlap B or C; B and C are disjoint.",
        }
        return totals

    def _array_manifest(self):
        result = {}
        for name, state in self.sensors.items():
            for suffix, arr in (("free_touched_mask", state.free_touched),
                                ("free_update_count", state.free_update_count),
                                ("observed_by_current_model", state.free_touched | state.occupied_touched),
                                ("potential_free_from_z_rejected", state.potential_free)):
                result[f"{name}_{suffix}.npy"] = {"dtype": str(arr.dtype), "shape": list(arr.shape)}
            result[f"{name}_unknown_mask.npy"] = {"dtype": "bool", "shape": list(self.grid.shape)}
            result[f"{name}_free_from_original_valid_return.npy"] = {"dtype": "bool", "shape": list(self.grid.shape)}
            result[f"{name}_free_from_z_rejected_return.npy"] = {"dtype": "bool", "shape": list(self.grid.shape)}
            result[f"{name}_phase1_added_free_update_count.npy"] = {"dtype": "uint32", "shape": list(self.grid.shape)}
            result[f"{name}_false_free_mask.npy"] = {"dtype": "bool", "shape": list(self.grid.shape)}
        return result

    def _render(self, name, logodds, unknown, state, observed):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        probability = 1.0 / (1.0 + np.exp(-logodds))
        panels = [
            (probability, f"{name.upper()} final OGM probability", "viridis", 0.0, 1.0),
            (unknown, "Unknown", "gray_r", 0.0, 1.0),
            (state.free_touched, "Free touched", "Blues", 0.0, 1.0),
            (state.free_touched & unknown, "Free touched but Unknown", "Oranges", 0.0, 1.0),
            (state.potential_free, "Potential Free from z-rejected", "Purples", 0.0, 1.0),
            (unknown & ~observed, "Untouched Unknown", "Reds", 0.0, 1.0),
        ]
        fig, axes = plt.subplots(2, 3, figsize=(15, 9), constrained_layout=True)
        extent = self.grid.extent
        for axis, (data, title, cmap, lo, hi) in zip(axes.flat, panels):
            image = axis.imshow(data, origin="lower", extent=extent, cmap=cmap, vmin=lo, vmax=hi,
                                interpolation="nearest")
            axis.set_title(title)
            axis.set_xlabel("WORLD X [m]")
            axis.set_ylabel("WORLD Y [m]")
            fig.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
        fig.suptitle(f"{name.upper()} Unknown diagnostics (Phase 0)")
        fig.savefig(self.output_dir / f"{name}_unknown_diagnostics.png", dpi=140)
        plt.close(fig)

    def _state(self, sensor_name: str) -> _SensorState:
        try:
            return self.sensors[sensor_name]
        except KeyError as exc:
            raise ValueError(f"sensor_name must be 'ego' or 'rsu', got {sensor_name!r}") from exc


def blueprint_attributes(blueprint, explicitly_set: set[str]) -> dict[str, object]:
    """Capture requested CARLA attributes and whether this code set each one."""
    names = ["channels", "range", "rotation_frequency", "points_per_second",
             "upper_fov", "lower_fov", "noise_stddev", "dropoff_general_rate",
             "dropoff_intensity_limit", "dropoff_zero_intensity"]
    result: dict[str, object] = {}
    for name in names:
        if blueprint.has_attribute(name):
            attr = blueprint.get_attribute(name)
            raw = str(attr)
            match = re.search(r"(?:^|,)value=([^)]*)", raw)
            scalar = match.group(1) if match else raw
            try:
                value: object = float(scalar) if any(ch in scalar.lower() for ch in (".", "e")) else int(scalar)
            except ValueError:
                value = scalar
            result[name] = {"value": value,
                            "source": "explicitly_set_by_coop_comm" if name in explicitly_set else "CARLA_blueprint_default"}
        else:
            result[name] = {"value": None, "source": "attribute_not_available"}
    return result
