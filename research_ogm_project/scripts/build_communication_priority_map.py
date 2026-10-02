"""Generate a Communication Priority Map offline; never transmits data."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from ogm_project.risk_priority import build_priority


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ogm-dir', type=Path, required=True)
    p.add_argument('--risk-dir', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--require-experiment-match', action='store_true')
    args = p.parse_args(argv)
    try:
        result = build_priority(args.ogm_dir, args.risk_dir, args.output_dir,
                                require_experiment_match=args.require_experiment_match)
    except (ValueError, OSError) as exc:
        p.error(f'{type(exc).__name__}: {exc}')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
