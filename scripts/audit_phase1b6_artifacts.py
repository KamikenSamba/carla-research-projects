"""Audit Phase 1b.6 pre/post CARLA artifacts and fail on any invariant."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pre", type=Path)
    parser.add_argument("post", type=Path)
    args = parser.parse_args(argv)
    pre, post = args.pre.resolve(), args.post.resolve()
    pre_shadow = read_json(pre/"ogm_engine_shadow"/"shadow_comparison.json")
    post_shadow = read_json(post/"ogm_engine_shadow"/"shadow_comparison.json")
    paired = read_json(post/"phase1a_paired"/"phase1a_paired_comparison.json")

    checks = {
        "pre_shadow_all_pass": bool(pre_shadow["all_pass"]),
        "post_shadow_all_pass": bool(post_shadow["all_pass"]),
        "config_diff_only_free_scale_decay_rate": set(post_shadow["config_diff"]) == {
            "free_scale", "decay_rate"},
        "ego_measurements_20": post_shadow["measurements"]["ego"] == 20,
        "rsu_measurements_20": post_shadow["measurements"]["rsu"] == 20,
    }
    for sensor in ("ego", "rsu"):
        for phase in ("phase0", "phase1a"):
            key = f"{sensor}_{phase}_pre_post_bit_exact"
            checks[key] = bool(np.array_equal(
                np.load(pre/"phase1a_paired"/f"{sensor}_logodds_{phase}.npy"),
                np.load(post/"phase1a_paired"/f"{sensor}_logodds_{phase}.npy"),
            ))
    expected = {"ego": 46, "rsu": 60}
    new_false_free = {}
    for sensor in ("ego", "rsu"):
        target = paired["sensors"][sensor]["false_free"]["by_actor"]["priority_target"]
        value = target["phase1a_false_free_cells"] - target["phase0_false_free_cells"]
        new_false_free[sensor] = value
        checks[f"{sensor}_new_false_free_preserved"] = value == expected[sensor]

    result = {
        "schema_version": "1.0",
        "phase": "Phase 1b.6 Common OGM Engine artifact audit",
        "pre_run": str(pre), "post_run": str(post),
        "checks": checks, "new_target_false_free": new_false_free,
        "passed": all(checks.values()),
    }
    output = post/"ogm_engine_shadow"/"phase1b6_audit.json"
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
