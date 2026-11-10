"""Offline Snapshot -> two-vehicle IDM -> future CSV/plot. No CARLA import."""
import argparse
import json
from pathlib import Path

from idm_model import IDMParameters
from idm_dt_mvp import initial_state, predict, save


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--raw-prefix',type=Path,required=True,help='reader stops after both F0 bbox observations')
    p.add_argument('--state-prefix',type=Path,required=True,help='existing pair_raw F0 state prefix; no future read')
    p.add_argument('--source-run-id',required=True,help='exact same resolved Source directory name')
    p.add_argument('--follower-id',required=True);p.add_argument('--leader-id',required=True)
    p.add_argument('--input-kind',choices=['recorded_snapshot','synthetic'],required=True)
    p.add_argument('--follower-v0',type=float,required=True);p.add_argument('--leader-v0',type=float,required=True)
    p.add_argument('--headways',type=float,nargs='+',default=[1.,2.])
    p.add_argument('--leader-T',type=float,default=1.)
    p.add_argument('--s0',type=float,default=2.);p.add_argument('--a',type=float,default=.3)
    p.add_argument('--b',type=float,default=1.5);p.add_argument('--max-decel',type=float,default=2.)
    p.add_argument('--dt',type=float,default=.05);p.add_argument('--horizon',type=float,default=6.)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--plots',action='store_true')
    args=p.parse_args(argv)
    # Existing evidence is never an output destination, even under a fresh child name.
    out=args.output.resolve()
    for path in (args.snapshot,args.raw_prefix,args.state_prefix):
        if out.is_relative_to(path.resolve().parent):raise ValueError('output must be outside input directories')
    initial=initial_state(args.snapshot,args.raw_prefix,args.follower_id,args.leader_id,
                          source_run_id=args.source_run_id,state_prefix_path=args.state_prefix)
    parameters=lambda v,T:IDMParameters(v,T,args.s0,args.a,args.b,4.,args.max_decel)
    prediction=predict(initial,parameters(args.follower_v0,args.headways[0]),
        parameters(args.leader_v0,args.leader_T),headways=args.headways,dt=args.dt,horizon=args.horizon)
    result=save(out,initial,prediction,input_kind=args.input_kind,plots=args.plots)
    print(json.dumps(dict(output=str(out),comparison=result['comparison'],CARLA_connected=False)))
    return 0


if __name__=='__main__':raise SystemExit(main())
