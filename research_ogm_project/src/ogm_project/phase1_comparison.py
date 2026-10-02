"""Validated Phase-0/Phase-1 aggregate and image comparison artifacts."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .grid_contract import GridContract, validate_same_grid


def _load_summary(directory: Path):
    return json.loads((directory / "ogm_diagnostics_summary.json").read_text(encoding="utf-8"))


def _delta(before, after):
    change = int(before) - int(after)
    return {"before": int(before), "after": int(after), "reduction": change,
            "reduction_ratio": change / int(before) if int(before) else 0.0}


def compare_phase_runs(phase0_dir, phase1_dir, output_dir):
    phase0_dir, phase1_dir, output_dir = map(Path, (phase0_dir, phase1_dir, output_dir))
    before = _load_summary(phase0_dir)
    after = _load_summary(phase1_dir)
    if before.get("free_space_height_slab_enabled"):
        raise ValueError("comparison baseline must have Phase 1 disabled")
    if not after.get("free_space_height_slab_enabled"):
        raise ValueError("comparison after run must have Phase 1 enabled")
    grid0, grid1 = GridContract.from_metadata(before["grid"]), GridContract.from_metadata(after["grid"])
    validate_same_grid(grid0, grid1)
    if before.get("experiment", {}).get("scenario_id") != after.get("experiment", {}).get("scenario_id"):
        raise ValueError("Phase-0/Phase-1 scenario mismatch")

    result = {
        "schema_version": "1.0", "comparison": "Phase 0 vs Phase 1 height-slab Free ray",
        "phase0_directory": str(phase0_dir.resolve()), "phase1_directory": str(phase1_dir.resolve()),
        "grid": grid0.metadata(), "sensors": {},
    }
    for name in ("ego", "rsu"):
        b, a = before["sensors"][name], after["sensors"][name]
        result["sensors"][name] = {
            "unknown": _delta(b["unknown_total"], a["unknown_total"]),
            "road_unknown": _delta(b["road"]["road_unknown"], a["road"]["road_unknown"]),
            "known_free": {"before": int(b["known_free_total"]), "after": int(a["known_free_total"]),
                           "change": int(a["known_free_total"] - b["known_free_total"])},
            "road_known_free": {"before": int(b["road"]["road_known_free"]),
                                "after": int(a["road"]["road_known_free"]),
                                "change": int(a["road"]["road_known_free"] - b["road"]["road_known_free"])},
            "occupied": {"before": int(b["occupied_total"]), "after": int(a["occupied_total"]),
                         "change": int(a["occupied_total"] - b["occupied_total"])},
            "false_free": {"before": b["vehicle_ground_truth"], "after": a["vehicle_ground_truth"]},
            "phase1_added_free": {
                "updates_total": int(a["phase1_added_free_updates_total"]),
                "cells_unique": int(a["phase1_added_free_cells_unique"]),
                "to_known_free": int(a["phase1_added_free_to_known_free"]),
                "still_unknown": int(a["phase1_added_free_still_unknown"]),
                "to_occupied": int(a["phase1_added_free_to_occupied"]),
                "road": {
                    "cells_unique": int(a["road"]["road_phase1_added_free_cells_unique"]),
                    "to_known_free": int(a["road"]["road_phase1_added_free_to_known_free"]),
                    "still_unknown": int(a["road"]["road_phase1_added_free_still_unknown"]),
                    "to_occupied": int(a["road"]["road_phase1_added_free_to_occupied"]),
                },
            },
        }
        _render(name, phase0_dir, phase1_dir, output_dir, grid0)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "phase1_comparison.json"
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    return str(path)


def _render(name, phase0_dir, phase1_dir, output_dir, grid):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    log0 = np.load(phase0_dir / f"{name}_logodds.npy", allow_pickle=False)
    log1 = np.load(phase1_dir / f"{name}_logodds.npy", allow_pickle=False)
    unk0 = np.load(phase0_dir / f"{name}_unknown_mask.npy", allow_pickle=False)
    unk1 = np.load(phase1_dir / f"{name}_unknown_mask.npy", allow_pickle=False)
    added = np.load(phase1_dir / f"{name}_free_from_z_rejected_return.npy", allow_pickle=False)
    false_free = np.load(phase1_dir / f"{name}_false_free_mask.npy", allow_pickle=False)
    for label, array in (("phase0 logodds", log0), ("phase1 logodds", log1),
                         ("phase0 unknown", unk0), ("phase1 unknown", unk1),
                         ("phase1 added free", added), ("false free", false_free)):
        if array.shape != grid.shape:
            raise ValueError(f"{label} shape/Grid mismatch")
    probability0 = 1.0 / (1.0 + np.exp(-log0))
    probability1 = 1.0 / (1.0 + np.exp(-log1))
    panels = [(probability0, "Phase 0 OGM", "viridis", 0, 1),
              (probability1, "Phase 1 OGM", "viridis", 0, 1),
              (unk0, "Phase 0 Unknown", "gray_r", 0, 1),
              (unk1, "Phase 1 Unknown", "gray_r", 0, 1),
              (added, "Free from z-rejected returns", "Blues", 0, 1),
              (false_free, "Vehicle-GT False Free", "Reds", 0, 1)]
    fig, axes = plt.subplots(2, 3, figsize=(15, 9), constrained_layout=True)
    for axis, (data, title, cmap, lo, hi) in zip(axes.flat, panels):
        image = axis.imshow(data, origin="lower", extent=grid.extent, interpolation="nearest",
                            cmap=cmap, vmin=lo, vmax=hi)
        axis.set_title(title); axis.set_xlabel("WORLD X [m]"); axis.set_ylabel("WORLD Y [m]")
        fig.colorbar(image, ax=axis, fraction=.046, pad=.04)
    fig.suptitle(f"{name.upper()} Phase 0 vs Phase 1")
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{name}_phase0_vs_phase1.png", dpi=140)
    plt.close(fig)
