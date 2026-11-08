"""Validate a saved Source/Prediction bundle without CARLA or forecast scoring."""
import argparse
import json
from pathlib import Path

from dt_gt_manifest import validate


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    if args.output.exists():parser.error('new output path required')
    result=validate(args.manifest)
    # Exclusive create also protects an input/output created after the check.
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({k:result[k] for k in ('status','errors','independent_forecast_accuracy_enabled')}))
    return 2 if result['errors'] else 0


if __name__=='__main__':raise SystemExit(main())
