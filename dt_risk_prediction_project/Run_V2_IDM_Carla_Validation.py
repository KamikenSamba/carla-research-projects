"""Existing V2 IDMFleetController in a bounded CARLA worker; offline default."""
import argparse
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

from idm_carla_validation import ValidationConfig,V2SingleHarness,V2FleetHarness,validation_parameters
from idm_model import compute_idm_acceleration
from idm_pair_exploration import pair_budget
from idm_v2_reconstruction import SnapshotRestorer
from idm_harness_host import listener_pids
from Run_Exploratory_Model3 import native_identity,same_identity,session_valid,IdentityGate
from Run_IDM_Following_Experiment import supervise


def verify_completed(directory,result,c):
    if (result.get('status')!='completed' or not result.get('cleanup',{}).get('complete')
        or result.get('collision_events')!=0 or result.get('route_audit') is not None
        or result.get('physical_bounds') is not None):raise ValueError('incomplete V2 validation')
    rows=list(csv.DictReader((directory/'calibration_observations.csv').open(encoding='utf-8')))
    raw=[json.loads(x) for x in (directory/'calibration_raw.jsonl').read_text(encoding='utf-8').splitlines()]
    if len(rows)!=len(raw) or len(rows)!=result['motion_samples']:raise ValueError('frame count mismatch')
    count=1 if c.trial=='single' else 2
    frames=[int(r['frame']) for r in rows]
    unique=sorted(set(frames))
    if frames!=[f for f in unique for _ in range(count)] or unique!=list(range(unique[0],unique[0]+len(unique))) or frames[-1]!=result['final_frame']:
        raise ValueError('noncontiguous V2 observations')
    by_frame={(int(r['frame']),int(r['actor_id'])):r for r in rows}
    for row,envelope in zip(rows,raw):
        if int(row['command_based_on_frame'])!=int(row['frame'])-1 or envelope['carla_frame']!=int(row['frame']) or not envelope['frame_validated']:
            raise ValueError('command/observation frame mismatch')
        for k in ('throttle','brake','steer'):
            if abs(float(row[k])-float(row['command_'+k]))>1e-5:raise ValueError('control echo mismatch')
    v2=list(csv.DictReader((directory/'v2_validation.csv').open(encoding='utf-8')))
    if len(v2)!=120*count:raise ValueError('incomplete IDM interval')
    for r in v2:
        p=validation_parameters(r['logical_actor_id'])
        gap=None if r['v2_gap_m']=='' else float(r['v2_gap_m'])
        leader=0. if r['v2_leader_speed_mps']=='' else float(r['v2_leader_speed_mps'])
        expected=compute_idm_acceleration(float(r['v2_speed_mps']),p,gap,leader)
        if abs(float(r['v2_idm_acceleration_mps2'])-expected)>1e-10 or r['v2_fallback_reason']:
            raise ValueError('IDM equation/parameter replay mismatch')
        obs=by_frame[(int(r['observation_frame']),int(r['actor_id']))]
        source=by_frame[(int(r['command_based_on_frame']),int(r['actor_id']))]
        if abs(float(r['v2_speed_mps'])-float(source['speed_mps']))>1e-5:
            raise ValueError('IDM calculation did not use the recorded pre-command state')
        if int(r['command_based_on_frame'])!=int(r['observation_frame'])-1:
            raise ValueError('IDM frame mismatch')
        for k in ('throttle','brake','steer'):
            if float(r['actual_'+k])!=float(obs[k]):raise ValueError('IDM actual control log mismatch')
    completion=json.loads((directory/'v2_completion.json').read_text())
    window=[r for r in rows if int(r['frame'])>=completion['stop_onset_frame']]
    if len(window)<11*count or completion['idm_vehicle_count']!=count or completion['final_frame']!=result['final_frame']:raise ValueError('stop incomplete')
    from idm_staged_stop import stopped
    for r in window:
        v={k:float(r[k]) for k in ('speed_planar_mps','v_long_mps','vz','a_long_mps2','throttle','brake')}
        v['hand_brake']=r['hand_brake']=='True'
        if not stopped(v):raise ValueError('invalid stop window')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--server-pid',type=int);p.add_argument('--port',type=int,default=2100)
    p.add_argument('--output-root',type=Path,default=Path('runs'))
    p.add_argument('--wall-timeout',type=float,default=120)
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--stage',default='exploratory_low_speed',choices=['exploratory_low_speed'])
    p.add_argument('--run-dir',type=Path,help=argparse.SUPPRESS)
    p.add_argument('--supervisor-pid',type=int,help=argparse.SUPPRESS)
    a=p.parse_args(argv);a.stage1_result=None
    payload=json.loads(a.config.read_text(encoding='utf-8-sig'));c=ValidationConfig(trial=payload['trial'])
    if payload.get('scenario')!=asdict(c):raise ValueError('scenario differs from fixed exploratory protocol')
    if not a.execute and not a.worker:
        print(json.dumps(dict(status='offline_plan_only',scenario=asdict(c),strict_scene_audit='BLOCKED',
            route_audit=None,physical_bounds=None,requires_authorized_session=True),indent=2));return 0
    if a.wall_timeout!=120:raise ValueError('fixed 120s worker supervisor limit')
    stop_m=None;source_run=None
    if c.trial!='single':
        single=Path(payload['single_run']);first=json.loads((single/'result.json').read_text())
        if (first.get('status')!='completed' or first.get('config_sha256')!=ValidationConfig().fingerprint
            or first.get('worker_exit_code')!=0 or first.get('supervised') is not True):
            raise ValueError('completed supervised single IDM run required')
        verify_completed(single,first,ValidationConfig())
        stop_m=json.loads((single/'v2_completion.json').read_text())['stop_distance_m']
        if not pair_budget(stop_m,(c.front_extent_limit,c.front_extent_limit))['admissible']:
            raise ValueError('pair budget invalid before connection')
    if c.trial=='replay':
        source_run=Path(payload['source_run']);previous=json.loads((source_run/'result.json').read_text())
        if previous.get('config_sha256')!=ValidationConfig(trial='fleet').fingerprint or previous.get('worker_exit_code')!=0 or previous.get('supervised') is not True:
            raise ValueError('completed supervised fleet source required')
        verify_completed(source_run,previous,ValidationConfig(trial='fleet'))
        SnapshotRestorer(None,source_run/'initial_snapshot.json',source_run/'resolved_idm_parameters.json')
    session=payload['session'];session_valid(session,a.server_pid,a.port)
    if not same_identity(native_identity(a.server_pid),session['identity']):raise ValueError('server identity mismatch')
    if a.worker:
        if not a.run_dir or not a.supervisor_pid:raise ValueError('worker requires supervisor')
        gate=IdentityGate(a.server_pid,a.supervisor_pid,a.run_dir/'halt_rpc',session['identity']);gate.check()
        if listener_pids(a.port)!={a.server_pid}:raise ValueError('listener identity mismatch')
        import carla
        client=carla.Client('127.0.0.1',a.port);client.set_timeout(5.)
        harness=(V2SingleHarness(client,carla,c,a.run_dir,gate) if c.trial=='single' else
                 V2FleetHarness(client,carla,c,a.run_dir,gate,single_stop_m=stop_m,source_run=source_run))
        result=harness.run()
        # Do not let a binding destructor attempt RPC after server failure.
        sys.stdout.flush();sys.stderr.flush();os._exit(result['exit_code'])
    return supervise(a,c,entrypoint=Path(__file__),source_files=(
        'Run_V2_IDM_Carla_Validation.py','idm_pair_exploration.py','idm_v2_reconstruction.py','Run_DT_Risk_V1.py','Run_DT_Risk_V2_IDM.py','dt_risk_common.py','Run_Exploratory_Model3.py','idm_carla_validation.py','idm_controller.py','idm_model.py','idm_snapshot.py','idm_leader.py','idm_low_speed.py','idm_reverse_motion.py','idm_staged_stop.py','idm_brake_harness.py','idm_brake_config.py',
        'idm_brake_metrics.py','idm_experiment_harness.py','idm_harness_host.py'),
        verify_completed=lambda directory,result:verify_completed(directory,result,c))


if __name__=='__main__':raise SystemExit(main())
