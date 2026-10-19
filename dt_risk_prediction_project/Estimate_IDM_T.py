"""Estimate actor headways offline and export the existing V2 traits format."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from idm_history import FitConfig, estimate_traits, read_history
from idm_snapshot import load_traits, read_snapshot, resolve_parameters


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--history", type=Path, required=True)
    parser.add_argument("--context", type=Path, help="Optional context CSV keyed by logical ID and CARLA frame")
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--actor-traits", type=Path, help="Explicit traits, protected from estimated values")
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; never overwrites results")
    parser.add_argument("--replace-snapshot-headway", action="store_true",
                        help="Explicitly allow estimates to replace snapshot T, never explicit traits")
    parser.add_argument("--idm-desired-speed-mps", type=float, required=True,
                        help="Fixed v0 (not estimated); explicit traits/snapshot v0 retain precedence")
    parser.add_argument("--default-idm-headway", type=float, default=1.5)
    parser.add_argument("--idm-min-desired-speed-mps", type=float, default=1.0)
    parser.add_argument("--idm-minimum-gap-m", type=float, default=2.0)
    parser.add_argument("--idm-max-accel-mps2", type=float, default=1.5)
    parser.add_argument("--idm-comfortable-decel-mps2", type=float, default=2.0)
    parser.add_argument("--idm-acceleration-exponent", type=float, default=4.0)
    parser.add_argument("--idm-max-decel-mps2", type=float, default=4.0)
    parser.add_argument("--minimum-t-s", type=float, default=0.3)
    parser.add_argument("--maximum-t-s", type=float, default=4.0)
    args = parser.parse_args(argv)
    try:
        # Validate defaults even when all snapshot rows override them.
        resolve_parameters(dict(logical_actor_id="", speed_mps=0), args, {})
        snapshot = read_snapshot(args.snapshot)
        if not snapshot:
            raise ValueError("target snapshot is empty")
        explicit = load_traits(args.actor_traits)
        parameters = {r["logical_actor_id"]: resolve_parameters(r, args, explicit) for r in snapshot}
        rows = read_history(args.history, args.context)
        traits, diagnostics = estimate_traits(
            rows, parameters, explicit,
            {r["logical_actor_id"]: r["desired_time_headway_s"] for r in snapshot},
            args.replace_snapshot_headway,
            FitConfig(minimum_t_s=args.minimum_t_s, maximum_t_s=args.maximum_t_s),
            cutoff_times={r["logical_actor_id"]: r["sim_time_s"] for r in snapshot})
        diagnostics["inputs"] = {name: str(getattr(args, name)) if getattr(args, name) else None
                                 for name in ("history", "context", "snapshot", "actor_traits")}
        diagnostics["replace_snapshot_headway"] = args.replace_snapshot_headway
        diagnostics["input_sha256"] = {
            name: hashlib.sha256(getattr(args, name).read_bytes()).hexdigest()
            for name in ("history", "context", "snapshot", "actor_traits") if getattr(args, name)}
        # New directory required: do not overwrite input data or prior results.
        args.output_dir.mkdir(parents=True, exist_ok=False)
        for name, payload in (("actor_traits.json", traits), ("headway_diagnostics.json", diagnostics)):
            (args.output_dir / name).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    print(f"Accepted estimates: {sum(d['adopted'] for d in diagnostics['actors'].values())}/{len(parameters)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
