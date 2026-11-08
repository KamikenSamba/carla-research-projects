"""Convert saved bounded V2 observations offline. Never imports a CARLA client."""
import argparse
import json
from pathlib import Path

from dt_gt_adapter import adapt


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--prediction-run',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,required=True,help='new directory outside Repository and both input runs')
    args=parser.parse_args(argv)
    try:result=adapt(args.source_run,args.prediction_run,args.output_root)
    except (OSError,ValueError,RuntimeError) as error:parser.error(str(error))
    print(json.dumps(dict(status=result['status'],errors=result['errors']),ensure_ascii=False))
    return 0 if result['status']=='CONTRACT_VALID_PENDING_APPROVAL' else 2


if __name__=='__main__':raise SystemExit(main())
