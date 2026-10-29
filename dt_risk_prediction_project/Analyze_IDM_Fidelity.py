"""Analyze saved V2 validation evidence without CARLA, GPU or server access."""
import argparse
import json
from idm_fidelity import analyze


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root',required=True)
    parser.add_argument('--output-root',required=True,help='new directory outside source evidence')
    parser.add_argument('--plots',action='store_true',help='requires matplotlib; JSON/CSV need only stdlib')
    args=parser.parse_args(argv)
    summary=analyze(args.evidence_root,args.output_root,args.plots)
    print(json.dumps(dict(rows=summary['total_rows'],error=summary['aggregate_error']),allow_nan=False))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
