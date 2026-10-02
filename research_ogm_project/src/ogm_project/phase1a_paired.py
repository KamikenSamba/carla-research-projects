"""Same-measurement paired Phase-0 / Phase-1a shadow OGM evaluation."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .ogm_engine import (
    OGMGridContract, OGMUpdateConfig,
    update_extra_free_from_rejected_returns,
    update_valid_returns,
)
from .logodds import FREE_TH, OCC_TH


LABELS = ("free", "unknown", "occupied")


def classify(logodds):
    if not np.isfinite(logodds).all():
        raise ValueError("shadow logodds contains NaN or Inf")
    p = 1.0 / (1.0 + np.exp(-logodds))
    labels = np.full(logodds.shape, 1, dtype=np.uint8)
    labels[p <= FREE_TH] = 0
    labels[p >= OCC_TH] = 2
    return labels, p


def transition_matrix(before, after, subset=None):
    before, after = np.asarray(before), np.asarray(after)
    if before.shape != after.shape:
        raise ValueError("transition inputs must have equal shape")
    mask = np.ones(before.shape, dtype=bool) if subset is None else np.asarray(subset)
    if mask.shape != before.shape or mask.dtype != np.bool_:
        raise ValueError("transition subset must be bool with label shape")
    return {src: {dst: int((mask & (before == i) & (after == j)).sum())
                  for j, dst in enumerate(LABELS)}
            for i, src in enumerate(LABELS)}


class Phase1aPairedComparison:
    def __init__(self, output_dir, grid, road_mask, *, z_min, z_max,
                 lidar_range, free_logodds, occupied_logodds,
                 logodds_min, logodds_max, world_to_grid, in_bounds,
                 bresenham, experiment=None):
        self.output_dir = Path(output_dir)
        self.grid = grid
        self.road = np.asarray(road_mask)
        if self.road.shape != grid.shape or self.road.dtype != np.bool_:
            raise ValueError("paired Road Mask must be bool with Grid shape")
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.lidar_range = float(lidar_range)
        self.free_logodds, self.occupied_logodds = float(free_logodds), float(occupied_logodds)
        self.logodds_min, self.logodds_max = float(logodds_min), float(logodds_max)
        self.world_to_grid, self.in_bounds, self.bresenham = world_to_grid, in_bounds, bresenham
        self.experiment = dict(experiment or {})
        self.phase0 = {n: np.zeros(grid.shape, dtype=np.float32) for n in ("ego", "rsu")}
        self.phase1a = {n: np.zeros(grid.shape, dtype=np.float32) for n in ("ego", "rsu")}
        self.extra_count = {n: np.zeros(grid.shape, dtype=np.uint32) for n in ("ego", "rsu")}
        self.raw_counts = {n: dict(measurements=0, raw=0, z_pass=0, z_reject=0,
                                   occupied_endpoint_updates=0) for n in ("ego", "rsu")}
        self.frames = {n: [] for n in ("ego", "rsu")}
        self.shared_decay_calls = {n: 0 for n in ("ego", "rsu")}

    def decay(self, sensor, dt, rate, decay_function):
        dt = float(dt)
        decay_function(self.phase0[sensor], dt, rate)
        decay_function(self.phase1a[sensor], dt, rate)
        self.shared_decay_calls[sensor] += 1

    def _config(self, free_scale, decay_rate=0.0):
        return OGMUpdateConfig(
            self.z_min, self.z_max, self.lidar_range, self.free_logodds,
            self.occupied_logodds, float(free_scale), self.logodds_min,
            self.logodds_max, FREE_TH, OCC_TH, float(decay_rate),
        )

    def update(self, sensor, points_world, z_pass_mask, origin_xyz, free_scale,
               legacy_update, *, frame, timestamp):
        points = np.asarray(points_world)
        passed = np.asarray(z_pass_mask, dtype=bool)
        if points.ndim != 2 or points.shape[1] < 3 or passed.shape != (len(points),):
            raise ValueError("paired measurement shape mismatch")
        if not np.isfinite(points).all() or not np.isfinite(float(timestamp)):
            raise ValueError("paired measurement contains NaN or Inf")
        expected = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
        if not np.array_equal(passed, expected):
            raise ValueError("paired z mask differs from current legacy filter")

        valid = points[passed, :3]
        origin_xy = (float(origin_xyz[0]), float(origin_xyz[1]))
        config = self._config(free_scale)
        grid = OGMGridContract(self.world_to_grid, self.in_bounds, self.bresenham)
        # ``legacy_update`` remains in the API for saved replay callers, but
        # Phase 1b.6 routes both valid-return grids through the Common Engine.
        if valid.size:
            update_valid_returns(self.phase0[sensor], valid, origin_xyz, grid, config)
            update_valid_returns(self.phase1a[sensor], valid, origin_xyz, grid, config)

        extra = update_extra_free_from_rejected_returns(
            self.phase1a[sensor], points[~passed, :3], origin_xyz, grid, config,
            free_update_counts=self.extra_count[sensor])
        added_updates = extra.extra_free_updates

        stats = self.raw_counts[sensor]
        stats["measurements"] += 1; stats["raw"] += len(points)
        stats["z_pass"] += int(passed.sum()); stats["z_reject"] += int((~passed).sum())
        stats["occupied_endpoint_updates"] += self._legacy_occupied_count(valid, origin_xy)
        self.frames[sensor].append({"frame": int(frame), "timestamp": float(timestamp),
                                    "extra_free_updates": added_updates})

    def _legacy_occupied_count(self, valid_points, origin_xy):
        if not self.in_bounds(*self.world_to_grid(*origin_xy)):
            return 0
        result = 0
        ox, oy = origin_xy
        for hit in valid_points:
            if (float(hit[0]) - ox) ** 2 + (float(hit[1]) - oy) ** 2 > self.lidar_range ** 2:
                continue
            if self.in_bounds(*self.world_to_grid(float(hit[0]), float(hit[1]))):
                result += 1
        return result

    def finalize(self, production_ego, production_rsu, gt_masks):
        production = {"ego": np.asarray(production_ego), "rsu": np.asarray(production_rsu)}
        for name in ("ego", "rsu"):
            if not np.array_equal(production[name], self.phase0[name]):
                raise RuntimeError(f"{name} Phase-0 shadow differs from production legacy OGM")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        combined_gt = np.zeros(self.grid.shape, dtype=bool)
        for mask in gt_masks.values():
            mask = np.asarray(mask)
            if mask.shape != self.grid.shape or mask.dtype != np.bool_:
                raise ValueError("actor GT mask/Grid mismatch")
            combined_gt |= mask

        result = {
            "schema_version": "1.0", "phase": "Phase 1a paired shadow comparison",
            "paired_same_measurements": True, "paired_same_decay": True,
            "phase0_shadow_matches_production": True,
            "grid": self.grid.metadata(), "experiment": self.experiment,
            "sensors": {}, "actor_gt_cells": {k: int(v.sum()) for k, v in gt_masks.items()},
        }
        occupied_rows = []
        target_rows = []
        for name in ("ego", "rsu"):
            log0, log1 = self.phase0[name], self.phase1a[name]
            delta = log1 - log0
            labels0, p0 = classify(log0); labels1, p1 = classify(log1)
            matrix = transition_matrix(labels0, labels1)
            road_matrix = transition_matrix(labels0, labels1, self.road)
            extra = self.extra_count[name]
            false0, false1 = combined_gt & (labels0 == 0), combined_gt & (labels1 == 0)
            actor_false = {}
            for actor_name, mask in gt_masks.items():
                cells = int(mask.sum())
                f0, f1 = int((mask & (labels0 == 0)).sum()), int((mask & (labels1 == 0)).sum())
                actor_false[actor_name] = dict(
                    gt_cells=cells, phase0_false_free_cells=f0, phase1a_false_free_cells=f1,
                    phase0_false_free_rate=f0 / cells if cells else 0.0,
                    phase1a_false_free_rate=f1 / cells if cells else 0.0)
            counts0 = self._counts(labels0); counts1 = self._counts(labels1)
            road0 = self._counts(labels0, self.road); road1 = self._counts(labels1, self.road)
            delta_positive = int((delta > 0).sum())
            result["sensors"][name] = {
                "measurement_counts": self.raw_counts[name],
                "shared_decay_calls": self.shared_decay_calls[name],
                "phase0": counts0, "phase1a": counts1,
                "road_phase0": road0, "road_phase1a": road1,
                "transition_matrix": matrix, "road_transition_matrix": road_matrix,
                "unknown_to_free": matrix["unknown"]["free"],
                "occupied_to_unknown": matrix["occupied"]["unknown"],
                "occupied_to_free": matrix["occupied"]["free"],
                "free_to_unknown": matrix["free"]["unknown"],
                "free_to_occupied": matrix["free"]["occupied"],
                "road_unknown_to_free": road_matrix["unknown"]["free"],
                "extra_free_updates_total": int(extra.sum()),
                "extra_free_cells_unique": int((extra > 0).sum()),
                "road_extra_free_cells_unique": int((self.road & (extra > 0)).sum()),
                "delta": {"positive_cells": delta_positive,
                          "zero_cells": int((delta == 0).sum()),
                          "negative_cells": int((delta < 0).sum()),
                          "min": float(delta.min()), "max": float(delta.max())},
                "delta_nonpositive_sanity": delta_positive == 0,
                "false_free": {"gt_occupied_cells": int(combined_gt.sum()),
                               "phase0_false_free_cells": int(false0.sum()),
                               "phase1a_false_free_cells": int(false1.sum()),
                               "phase0_false_free_rate": float(false0.sum() / combined_gt.sum()) if combined_gt.any() else 0.0,
                               "phase1a_false_free_rate": float(false1.sum() / combined_gt.sum()) if combined_gt.any() else 0.0,
                               "by_actor": actor_false},
            }
            np.save(self.output_dir / f"{name}_logodds_phase0.npy", log0)
            np.save(self.output_dir / f"{name}_logodds_phase1a.npy", log1)
            np.save(self.output_dir / f"{name}_logodds_delta.npy", delta)
            np.save(self.output_dir / f"{name}_extra_free_update_count.npy", extra)
            np.save(self.output_dir / f"{name}_extra_free_touched_mask.npy", extra > 0)
            self._occupied_rows(name, labels0, labels1, log0, log1, extra, combined_gt, occupied_rows)
            self._target_rows(name, labels0, labels1, p0, p1, extra, gt_masks, target_rows)
            self._render(name, log0, log1, labels0, labels1, extra, delta)

        self._write_csv("occupied_transition_cells.csv", occupied_rows,
                        ["sensor", "world_x", "world_y", "ix", "iy", "transition",
                         "phase0_logodds", "phase1a_logodds", "extra_free_count", "road", "actor_gt_overlap"])
        self._write_csv("target_phase1a_check.csv", target_rows,
                        ["sensor", "world_x", "world_y", "ix", "iy", "phase0_probability",
                         "phase1a_probability", "phase0_label", "phase1a_label",
                         "extra_free_count", "GT_target_cell"])
        path = self.output_dir / "phase1a_paired_comparison.json"
        path.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        return result

    def _counts(self, labels, subset=None):
        mask = np.ones(labels.shape, dtype=bool) if subset is None else subset
        return {label: int((mask & (labels == i)).sum()) for i, label in enumerate(LABELS)}

    def _occupied_rows(self, sensor, a, b, log0, log1, extra, gt, rows):
        changed = (a == 2) & (b != 2)
        for iy, ix in np.argwhere(changed):
            rows.append(dict(sensor=sensor, world_x=self.grid.x_min_m + (ix + .5) * self.grid.resolution_m,
                             world_y=self.grid.y_min_m + (iy + .5) * self.grid.resolution_m,
                             ix=int(ix), iy=int(iy), transition=f"occupied_to_{LABELS[int(b[iy, ix])]}",
                             phase0_logodds=float(log0[iy, ix]), phase1a_logodds=float(log1[iy, ix]),
                             extra_free_count=int(extra[iy, ix]), road=bool(self.road[iy, ix]),
                             actor_gt_overlap=bool(gt[iy, ix])))

    def _target_rows(self, sensor, a, b, p0, p1, extra, gt_masks, rows):
        target = next((mask for key, mask in gt_masks.items() if "target" in key.lower()), None)
        if target is None:
            return
        vicinity = target.copy()
        ys, xs = np.where(target)
        for iy, ix in zip(ys, xs):
            vicinity[max(0, iy-2):min(self.grid.ny, iy+3), max(0, ix-2):min(self.grid.nx, ix+3)] = True
        for iy, ix in np.argwhere(vicinity):
            rows.append(dict(sensor=sensor, world_x=self.grid.x_min_m + (ix + .5) * self.grid.resolution_m,
                             world_y=self.grid.y_min_m + (iy + .5) * self.grid.resolution_m,
                             ix=int(ix), iy=int(iy), phase0_probability=float(p0[iy, ix]),
                             phase1a_probability=float(p1[iy, ix]), phase0_label=LABELS[int(a[iy, ix])],
                             phase1a_label=LABELS[int(b[iy, ix])], extra_free_count=int(extra[iy, ix]),
                             GT_target_cell=bool(target[iy, ix])))

    def _write_csv(self, name, rows, fields):
        with (self.output_dir / name).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)

    def _render(self, name, log0, log1, labels0, labels1, extra, delta):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap

        p0 = 1 / (1 + np.exp(-log0)); p1 = 1 / (1 + np.exp(-log1))
        transition = np.zeros(labels0.shape, dtype=np.uint8)
        transition[(labels0 == 1) & (labels1 == 0)] = 1
        transition[(labels0 == 2) & (labels1 == 1)] = 2
        transition[(labels0 == 2) & (labels1 == 0)] = 3
        panels = [(p0, "Phase 0 shadow OGM", "viridis", 0, 1),
                  (p1, "Phase 1a shadow OGM", "viridis", 0, 1),
                  (labels0 == 1, "Phase 0 Unknown", "gray_r", 0, 1),
                  (labels1 == 1, "Phase 1a Unknown", "gray_r", 0, 1),
                  (extra > 0, "Extra Free cells", "Blues", 0, 1),
                  (transition, "Transitions: U→F blue, O→U orange, O→F red",
                   ListedColormap(["white", "blue", "orange", "red"]), 0, 3),
                  (delta, "log-odds delta (Phase1a - Phase0)", "coolwarm", -4, 4)]
        fig, axes = plt.subplots(2, 4, figsize=(19, 9), constrained_layout=True)
        for axis in axes.flat: axis.set_visible(False)
        for axis, (data, title, cmap, lo, hi) in zip(axes.flat, panels):
            axis.set_visible(True)
            image = axis.imshow(data, origin="lower", extent=self.grid.extent, cmap=cmap,
                                vmin=lo, vmax=hi, interpolation="nearest")
            axis.set_title(title); axis.set_xlabel("WORLD X [m]"); axis.set_ylabel("WORLD Y [m]")
            fig.colorbar(image, ax=axis, fraction=.046, pad=.04)
        fig.suptitle(f"{name.upper()} paired Phase 0 / Phase 1a")
        fig.savefig(self.output_dir / f"{name}_phase0_vs_phase1a_paired.png", dpi=140)
        plt.close(fig)
