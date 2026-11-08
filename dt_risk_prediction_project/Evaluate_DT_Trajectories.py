"""Compare a completed V2 fleet run and its replay, offline and read-only."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from idm_fidelity import csv_rows, write_csv
from idm_snapshot import read_snapshot
from idm_v2_reconstruction import physics_settings
from dt_trajectory_evaluation import evaluate


def assess(source_run,replay_run,output,horizons=(1.,2.,3.,5.)):
    source,replay,out=Path(source_run),Path(replay_run),Path(output)
    if out.exists() or any(out.resolve().is_relative_to(p.resolve()) for p in (source,replay)):
        raise ValueError('new output outside original runs required')
    hashes={}
    def path(root,name):
        p=root/name;hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();return p
    def read(root,name):return json.loads(path(root,name).read_text(encoding='utf-8-sig'))
    env=[];snapshots=[];frames=[];states=[];configs=[];parameters=[];launch=[]
    for root in (source,replay):
        result=read(root,'result.json')
        if (result.get('status')!='completed' or result.get('worker_exit_code')!=0 or result.get('supervised') is not True
            or result.get('fatal_stderr') or result.get('timed_out') or not result.get('cleanup',{}).get('complete')
            or result.get('collision_events')!=0):raise ValueError('completed collision-free run with verified cleanup required')
        env.append(read(root,'environment.json'))
        configs.append(read(root,'config_used.json')['scenario'])
        parameters.append(read(root,'resolved_idm_parameters.json'))
        launch.append(read(root,'launch.json'))
        snap=read_snapshot(path(root,'initial_snapshot.json'));snapshots.append({r['logical_actor_id']:r for r in snap})
        if {r['logical_actor_id'] for r in csv_rows(path(root,'prediction_states.csv'))}!=set(snapshots[-1]):
            raise ValueError('trajectory/snapshot logical ID sets differ')
        if len({r['carla_frame'] for r in snap})!=1:raise ValueError('snapshot frame mismatch')
        frames.append(snap[0]['carla_frame']);states.append(csv_rows(path(root,'prediction_states.csv')))
        for logical,r in snapshots[-1].items():
            ids={int(s['carla_actor_id']) for s in states[-1] if s['logical_actor_id']==logical}
            if ids!={r['source_actor_id']}:raise ValueError('snapshot/trajectory actor mapping mismatch')
    for field in ('versions','map','weather','initial_actor_ids','original_settings','applied_settings'):
        if env[0][field]!=env[1][field]:raise ValueError('environment mismatch: '+field)
    dt=env[0]['applied_settings']['fixed_delta_seconds']
    if launch[0]['source_commit']!=launch[1]['source_commit']:raise ValueError('different control source commits')
    if configs[0].get('trial')!='fleet' or configs[1].get('trial')!='replay':raise ValueError('fleet and replay roles required')
    if {k:v for k,v in configs[0].items() if k!='trial'}!={k:v for k,v in configs[1].items() if k!='trial'}:
        raise ValueError('different scenario settings')
    if parameters[0]!=parameters[1]:raise ValueError('different per-actor IDM parameters')
    for name in ('vehicle_physics.json','leader_physics.json'):
        if physics_settings(read(source,name))!=physics_settings(read(replay,name)):raise ValueError('physics settings differ')
    if set(snapshots[0])!=set(snapshots[1]):raise ValueError('snapshot logical IDs differ')
    reconstruction=read(replay,'reconstruction_comparison.json')
    if reconstruction.get('within_initial_state_tolerance') is not True:
        raise ValueError('unverified source/reconstruction linkage')
    initial={}
    for logical,a in snapshots[0].items():
        b=snapshots[1][logical]
        link=reconstruction['actors'][logical]
        if (link['source_actor_id']!=a['source_actor_id'] or link['reconstructed_actor_id']!=b['source_actor_id']):
            raise ValueError('wrong source/reconstruction actor linkage')
        if a['blueprint_id']!=b['blueprint_id']:raise ValueError('blueprint mismatch')
        delta=dict(position_error_m=math.dist([a[k] for k in 'xyz'],[b[k] for k in 'xyz']),
                   velocity_error_mps=math.dist([a['v'+k] for k in 'xyz'],[b['v'+k] for k in 'xyz']),
                   yaw_error_deg=abs((b['yaw']-a['yaw']+180)%360-180))
        if delta['position_error_m']>.02 or delta['velocity_error_mps']>.01 or delta['yaw_error_deg']>.2:
            raise ValueError('outside existing reconstruction tolerance')
        initial[logical]=delta
    result=evaluate(states[0],states[1],dt=dt,horizons=horizons,purpose='reconstruction_repeatability',
                    reference_origin_frame=frames[0],prediction_origin_frame=frames[1])
    result.update(source_sha256=hashes,initial_state_errors=initial,
        environment_match=True,parameters_equal=True,physics_internal_state_restored=False,
        limitation='Source and Replay both use the same IDM controller; this is not independent prediction accuracy')
    out.mkdir(parents=True,exist_ok=False)
    for name,rows in [('actor_horizon_metrics.csv',result['per_actor']),('time_errors.csv',result['per_time']),
                      ('paired_errors.csv',result['paired_samples'])]:
        if rows:write_csv(out/name,rows)
    (out/'evaluation.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--replay-run',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--horizons',default='1,2,3,5')
    args=parser.parse_args(argv)
    result=assess(args.source_run,args.replay_run,args.output_root,tuple(float(x) for x in args.horizons.split(',')))
    print(json.dumps(result['per_horizon'],allow_nan=False));return 0


if __name__=='__main__':raise SystemExit(main())
