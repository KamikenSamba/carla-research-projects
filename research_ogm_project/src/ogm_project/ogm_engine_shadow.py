"""Legacy/Common paired shadow comparison for Phase 1b.6."""
from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from .height_slab_free_space import trace_height_slab_ray
from .ogm_engine import (
    OGMGridContract, OGMUpdateConfig, apply_decay, known_masks,
    update_lidar_measurement,
)


def config_diff(ego: OGMUpdateConfig, rsu: OGMUpdateConfig) -> dict:
    a, b = asdict(ego), asdict(rsu)
    return {key: {"ego": a[key], "rsu": b[key]} for key in a if a[key] != b[key]}


class OGMEngineShadowComparison:
    """Run untouched legacy calls and the Common Engine from the same inputs."""

    def __init__(self, output_dir, shape, grid: OGMGridContract,
                 ego_config: OGMUpdateConfig, rsu_config: OGMUpdateConfig,
                 legacy_update, legacy_decay):
        self.output_dir = Path(output_dir)
        self.shape = tuple(shape)
        self.grid = grid
        self.configs = {"ego": ego_config, "rsu": rsu_config}
        self.legacy_update = legacy_update
        self.legacy_decay = legacy_decay
        self.arrays = {
            impl: {phase: {name: np.zeros(self.shape, dtype=np.float32)
                           for name in ("ego", "rsu")}
                   for phase in ("phase0", "phase1a")}
            for impl in ("legacy", "common")
        }
        self.counts = {
            impl: {phase: {name: self._empty_counts() for name in ("ego", "rsu")}
                   for phase in ("phase0", "phase1a")}
            for impl in ("legacy", "common")
        }
        self.measurements = {"ego": 0, "rsu": 0}

    @staticmethod
    def _empty_counts():
        return dict(raw_returns=0, z_pass=0, z_reject=0, free_updates=0,
                    occupied_updates=0, extra_free_updates=0,
                    free_touched_cells=0, occupied_touched_cells=0)

    @staticmethod
    def _add(target, values):
        for key, value in values.items():
            target[key] += int(value)

    def decay(self, name, dt):
        cfg = self.configs[name]
        for phase in ("phase0", "phase1a"):
            self.legacy_decay(self.arrays["legacy"][phase][name], dt, cfg.decay_rate)
            apply_decay(self.arrays["common"][phase][name], dt, cfg)

    def _legacy_counts_and_extra(self, phase, name, points, passed, origin):
        cfg = self.configs[name]
        arr = self.arrays["legacy"][phase][name]
        free_touched = np.zeros(self.shape, dtype=bool)
        occ_touched = np.zeros(self.shape, dtype=bool)
        counts = dict(raw_returns=len(points), z_pass=int(passed.sum()),
                      z_reject=int((~passed).sum()), free_updates=0,
                      occupied_updates=0, extra_free_updates=0,
                      free_touched_cells=0, occupied_touched_cells=0)
        origin_cell = self.grid.world_to_grid(origin[0], origin[1])
        if self.grid.in_bounds(*origin_cell):
            for xw, yw, _zw in points[passed, :3]:
                dx, dy = xw-origin[0], yw-origin[1]
                if dx*dx+dy*dy > cfg.lidar_range*cfg.lidar_range:
                    continue
                endpoint = self.grid.world_to_grid(xw, yw)
                if not self.grid.in_bounds(*endpoint):
                    continue
                cells = tuple(self.grid.bresenham(*origin_cell, *endpoint))
                for cx, cy in cells[:-1]:
                    free_touched[cy, cx] = True
                    counts["free_updates"] += 1
                cx, cy = cells[-1]
                occ_touched[cy, cx] = True
                counts["occupied_updates"] += 1
        if phase == "phase1a":
            for hit in points[~passed, :3]:
                updates = trace_height_slab_ray(
                    origin, hit, cfg.z_min, cfg.z_max, cfg.lidar_range,
                    self.grid.world_to_grid, self.grid.in_bounds,
                    self.grid.bresenham,
                )
                for cx, cy in updates.free_cells:
                    arr[cy, cx] = np.clip(
                        arr[cy, cx] + cfg.free_scale * cfg.free_logodds,
                        cfg.logodds_min, cfg.logodds_max,
                    )
                    free_touched[cy, cx] = True
                    counts["free_updates"] += 1
                    counts["extra_free_updates"] += 1
        counts["free_touched_cells"] = int(free_touched.sum())
        counts["occupied_touched_cells"] = int(occ_touched.sum())
        return counts

    def update(self, name, points_world, sensor_origin_world):
        cfg = self.configs[name]
        points = np.asarray(points_world)
        passed = (points[:, 2] > cfg.z_min) & (points[:, 2] < cfg.z_max)
        valid = points[passed, :3]
        for phase in ("phase0", "phase1a"):
            legacy_arr = self.arrays["legacy"][phase][name]
            if valid.size:
                self.legacy_update(valid, sensor_origin_world[:2], legacy_arr,
                                   free_scale=cfg.free_scale)
            legacy_counts = self._legacy_counts_and_extra(
                phase, name, points, passed, sensor_origin_world)
            self._add(self.counts["legacy"][phase][name], legacy_counts)

            result = update_lidar_measurement(
                self.arrays["common"][phase][name], points,
                sensor_origin_world, self.grid, cfg,
                include_rejected_free=(phase == "phase1a"),
            )
            self._add(self.counts["common"][phase][name], result.stats.as_dict())
        self.measurements[name] += 1

    def finalize(self, *, encode_grid, fuse, road_mask=None, risk=None,
                 production_ego=None, production_rsu=None):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        result = {"measurements": dict(self.measurements), "checks": {},
                  "intermediate_counts": {}, "config_diff": config_diff(
                      self.configs["ego"], self.configs["rsu"])}
        for name in ("ego", "rsu"):
            for phase in ("phase0", "phase1a"):
                legacy = self.arrays["legacy"][phase][name]
                common = self.arrays["common"][phase][name]
                key = f"{name}_{phase}_bit_exact"
                result["checks"][key] = bool(np.array_equal(legacy, common))
                legacy_masks = known_masks(legacy, self.configs[name])
                common_masks = known_masks(common, self.configs[name])
                result["checks"][f"{name}_{phase}_labels_bit_exact"] = all(
                    np.array_equal(a, b) for a, b in zip(legacy_masks, common_masks))
                counts_equal = self.counts["legacy"][phase][name] == self.counts["common"][phase][name]
                result["checks"][f"{name}_{phase}_counts_identical"] = counts_equal
                result["intermediate_counts"][f"{name}_{phase}"] = {
                    "legacy": self.counts["legacy"][phase][name],
                    "common": self.counts["common"][phase][name],
                }
                np.save(self.output_dir/f"{name}_{phase}_legacy.npy", legacy)
                np.save(self.output_dir/f"{name}_{phase}_common.npy", common)

        legacy_rsu = self.arrays["legacy"]["phase0"]["rsu"]
        common_rsu = self.arrays["common"]["phase0"]["rsu"]
        result["checks"]["communication_payload_bit_exact"] = (
            encode_grid(legacy_rsu) == encode_grid(common_rsu))
        legacy_fused = fuse(self.arrays["legacy"]["phase0"]["ego"], legacy_rsu)
        common_fused = fuse(self.arrays["common"]["phase0"]["ego"], common_rsu)
        result["checks"]["fusion_bit_exact"] = bool(np.array_equal(legacy_fused, common_fused))
        if production_ego is not None:
            result["checks"]["production_ego_phase0_matches_legacy"] = bool(
                np.array_equal(np.asarray(production_ego), self.arrays["legacy"]["phase0"]["ego"]))
        if production_rsu is not None:
            result["checks"]["production_rsu_phase0_matches_legacy"] = bool(
                np.array_equal(np.asarray(production_rsu), self.arrays["legacy"]["phase0"]["rsu"]))

        legacy_ego_masks = known_masks(self.arrays["legacy"]["phase0"]["ego"], self.configs["ego"])
        common_ego_masks = known_masks(self.arrays["common"]["phase0"]["ego"], self.configs["ego"])
        legacy_rsu_masks = known_masks(legacy_rsu, self.configs["rsu"])
        common_rsu_masks = known_masks(common_rsu, self.configs["rsu"])
        result["checks"]["ego_unknown_identical"] = bool(np.array_equal(legacy_ego_masks[2], common_ego_masks[2]))
        result["checks"]["rsu_known_identical"] = bool(np.array_equal(
            ~legacy_rsu_masks[2], ~common_rsu_masks[2]))
        if road_mask is not None and risk is not None:
            legacy_priority = np.asarray(risk) * legacy_ego_masks[2] * (~legacy_rsu_masks[2]) * np.asarray(road_mask)
            common_priority = np.asarray(risk) * common_ego_masks[2] * (~common_rsu_masks[2]) * np.asarray(road_mask)
            result["checks"]["priority_bit_exact"] = bool(np.array_equal(legacy_priority, common_priority))

        result["all_pass"] = all(result["checks"].values())
        (self.output_dir/"config_diff.json").write_text(
            json.dumps(result["config_diff"], indent=2, ensure_ascii=False), encoding="utf-8")
        (self.output_dir/"shadow_comparison.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        return result
