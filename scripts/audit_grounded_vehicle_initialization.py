"""Run two all-vehicle physics-settle trials for priority_crossing_v1."""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/"src"))
import carla
from run_real_priority_integration import load_json, short_map, spawn_vehicle
from ogm_project.phase1b5_diagnostics import transform_record
from ogm_project.vehicle_grounding import ground_all_vehicles, transforms_reproducible


def cleanup(world):
    for actor in list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("sensor.*")):
        try: actor.destroy()
        except RuntimeError: pass
    world.tick()


def plot_grounding(path, trials):
    import matplotlib.pyplot as plt
    roles = tuple(trials[0]["actors"])
    fig, axes = plt.subplots(1, len(roles), figsize=(6*len(roles), 5), squeeze=False)
    for ax, role in zip(axes[0], roles):
        first = trials[0]["actors"][role]
        before, after = first["observed_before"], first["observed_after"]
        road = after["road_surface_z"]
        ax.axhline(road, color="black", label="Roads cast-ray surface")
        for x, record, color, label in ((0, before, "tab:red", "legacy spawn"),
                                         (1, after, "tab:green", "grounded")):
            ax.plot([x, x], [record["obb_bottom_z"], record["obb_top_z"]], lw=12,
                    color=color, alpha=.55, label=label)
            ax.scatter([x], [record["transform"]["z"]], color=color, marker="x", s=80)
        ax.set_xticks([0, 1], ["legacy", "grounded"]); ax.set_ylabel("WORLD z [m]")
        ax.set_title(f"{role}\norigin x marker / OBB vertical span"); ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT/"configs"/"priority_crossing_v1.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1"); parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args(argv); cfg = load_json(args.config); root = args.output_dir.resolve()
    if root.exists() and any(root.iterdir()): raise ValueError(f"output directory must be empty: {root}")
    root.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port); client.set_timeout(60); world = client.get_world()
    if short_map(world) != cfg["map_name"]: world = client.load_world(cfg["map_name"])
    original = world.get_settings(); settings = world.get_settings(); settings.synchronous_mode = True
    settings.fixed_delta_seconds = cfg["fixed_delta_seconds"]; world.apply_settings(settings)
    trials = []
    try:
        for trial in range(2):
            cleanup(world); random.seed(cfg["random_seed"]); np.random.seed(cfg["random_seed"])
            spawn_cfg={role:dict(cfg[role]) for role in ('ego','target')}
            if cfg.get('grounded'):
                for role in ('ego','target'):
                    legacy=cfg[role]['grounding_provenance']['legacy_original_transform']
                    for key in ('x','y','z','pitch','yaw','roll'):spawn_cfg[role][key]=legacy[key]
            actors = {
                "ego": spawn_vehicle(world, spawn_cfg["ego"], "priority_ego"),
                "target": spawn_vehicle(world, spawn_cfg["target"], "priority_target"),
            }
            world.tick()
            configured = {role: {**transform_record(actor.get_transform()),
                                 "configured_z": float(cfg[role]["z"])}
                          for role, actor in actors.items()}
            result = ground_all_vehicles(world, actors, configured,
                cfg["fixed_delta_seconds"], root/f"trial_{trial+1}",settings=cfg.get('grounding'))
            trials.append(result)
        comparisons = {}
        for role in trials[0]["actors"]:
            passed, delta = transforms_reproducible(
                trials[0]["actors"][role]["settled_transform"],
                trials[1]["actors"][role]["settled_transform"])
            comparisons[role] = {"passed": passed, "absolute_delta": delta,
                "trial_1": trials[0]["actors"][role]["settled_transform"],
                "trial_2": trials[1]["actors"][role]["settled_transform"]}
        summary = {"schema_version": "1.0", "source_scenario": cfg["scenario_name"],
                   "vehicle_roles": list(trials[0]["actors"]), "trials": trials,
                   "reproducibility": comparisons,
                   "all_reproducible": all(item["passed"] for item in comparisons.values())}
        (root/"grounding_reproducibility.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        plot_grounding(root/"all_vehicle_floating_vs_grounded.png", trials)
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        if not summary["all_reproducible"]: raise RuntimeError("settled transforms were not reproducible")
    finally:
        cleanup(world); world.apply_settings(original)


if __name__ == "__main__": main()
