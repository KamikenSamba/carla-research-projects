"""Offline by default. One bounded manual single/pair research worker per invocation."""
import argparse
from dataclasses import asdict
import csv
import json
import os
from pathlib import Path
import sys

from idm_exploratory_following import ResearchConfig, StabilityHarness, ready
from idm_staged_stop import stopped
from idm_pair_exploration import PairHarness, single_admission, pair_budget
from Run_Exploratory_Model3 import native_identity, same_identity, session_valid, IdentityGate
from idm_harness_host import listener_pids
from Run_IDM_Following_Experiment import supervise


def verify_completed(directory, result, c):
    if (result.get('collision_events') != 0 or not result.get('cleanup', {}).get('complete')
        or result.get('physical_bounds') is not None or result.get('route_audit') is not None):
        raise ValueError('incomplete exploratory result')
    if c.trial=='following':
        rows=list(csv.DictReader((directory/'pair_observations.csv').open(encoding='utf-8')))
        raw=[json.loads(x) for x in (directory/'pair_raw.jsonl').read_text(encoding='utf-8').splitlines()]
        completion=json.loads((directory/'pair_completion.json').read_text())
        if len(rows)!=2*completion['pair_frames'] or len(raw)!=completion['pair_frames']:
            raise ValueError('pair sample count mismatch')
        for i,envelope in enumerate(raw):
            pair=rows[2*i:2*i+2]
            if ({r['logical_id'] for r in pair}!={'leader','follower'} or
                any(int(r['frame'])!=envelope['frame'] or int(r['command_based_on_frame'])!=envelope['frame']-1 for r in pair)
                or (i and envelope['frame']!=raw[i-1]['frame']+1)):
                raise ValueError('pair frame/role mismatch')
            for r in pair:
                expected=next(x for x in envelope['rows'] if x['logical_id']==r['logical_id'])
                for k in ('throttle','brake','steer'):
                    if abs(float(r[k])-float(r['command_'+k]))>1e-5 or float(r[k])!=expected[k]:
                        raise ValueError('pair actual/command mismatch')
        if completion['continuous_stop_s']!=.5 or completion['final_frame']!=result['final_frame']:
            raise ValueError('pair stop incomplete')
        for r in rows[-22:]:
            observed={k:float(r[k]) for k in ('speed_planar_mps','v_long_mps','vz','a_long_mps2','throttle','brake')}
            observed['hand_brake']=r['hand_brake']=='True'
            if not stopped(observed):raise ValueError('pair stop window invalid')
        return
    rows = list(csv.DictReader((directory/'calibration_observations.csv').open(encoding='utf-8')))
    raw = [json.loads(x) for x in (directory/'calibration_raw.jsonl').read_text(encoding='utf-8').splitlines()]
    if len(rows) != len(raw) or len(rows) != result['motion_samples']:
        raise ValueError('raw/CSV sample mismatch')
    for i, (r, obs) in enumerate(zip(rows, raw)):
        if (int(r['frame']) != obs['carla_frame'] or not obs['frame_validated']
            or int(r['command_based_on_frame']) != int(r['frame'])-1
            or (i and int(r['frame']) != int(rows[i-1]['frame'])+1)):
            raise ValueError('frame mismatch')
        for k in ('throttle','brake','steer'):
            if abs(float(r[k])-float(r['command_'+k])) > 1e-5:
                raise ValueError('actual/command control mismatch')
    stability = json.loads((directory/'stability_result.json').read_text())
    window = [r for r in rows if stability['first_frame'] <= int(r['frame']) <= stability['final_frame']]
    if len(window) < 41 or not all(ready({k:float(r[k]) for k in
        ('speed_planar_mps','a_long_mps2','vz')}) for r in window):
        raise ValueError('two second readiness not demonstrated')
    stop = json.loads((directory/'separated_stop_result.json').read_text())
    window = [r for r in rows if int(r['frame']) >= stop['stop_onset_frame']]
    if len(window) < 11 or int(window[-1]['frame']) != result['final_frame']:
        raise ValueError('continuous stop missing')
    for r in window:
        values={k:float(r[k]) for k in ('speed_planar_mps','v_long_mps','vz','a_long_mps2','throttle','brake')}
        values['hand_brake']=r['hand_brake']=='True'
        if not stopped(values):raise ValueError('invalid stopped sample')


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
    payload=json.loads(a.config.read_text(encoding='utf-8-sig'))
    c=ResearchConfig(trial=payload['trial'])
    if payload.get('scenario')!=asdict(c):raise ValueError('fixed exploratory config mismatch')
    if not a.execute and not a.worker:
        print(json.dumps(dict(status='offline_plan_only',scenario=asdict(c),physical_bounds=None,route_audit=None)))
        return 0
    if a.wall_timeout!=120:raise ValueError('fixed worker deadline')
    stop_m=None
    if c.trial=='following':
        source=Path(payload['single_run'])
        result=json.loads((source/'result.json').read_text())
        stability=json.loads((source/'stability_result.json').read_text())
        stop=json.loads((source/'separated_stop_result.json').read_text()) if (source/'separated_stop_result.json').exists() else {}
        if (result.get('config_sha256')!=ResearchConfig().fingerprint or
            not single_admission(result,stability,stop)):
            raise ValueError('completed stable single and stop evidence required')
        verify_completed(source,result,ResearchConfig())
        stop_m=stop['stop_distance_m']
        if not pair_budget(stop_m,(c.front_extent_limit,c.front_extent_limit))['admissible']:
            raise ValueError('pair distance budget inadmissible before RPC')
    session=payload['session'];session_valid(session,a.server_pid,a.port)
    if not same_identity(native_identity(a.server_pid),session['identity']):raise ValueError('identity mismatch')
    if a.worker:
        gate=IdentityGate(a.server_pid,a.supervisor_pid,a.run_dir/'halt_rpc',session['identity']);gate.check()
        if listener_pids(a.port)!={a.server_pid}:raise ValueError('listener mismatch')
        import carla
        client=carla.Client('127.0.0.1',a.port);client.set_timeout(5.)
        harness=(PairHarness(client,carla,c,a.run_dir,gate,single_stop_m=stop_m) if c.trial=='following'
                 else StabilityHarness(client,carla,c,a.run_dir,gate))
        result=harness.run()
        sys.stdout.flush();sys.stderr.flush();os._exit(result['exit_code'])
    return supervise(a,c,entrypoint=Path(__file__),source_files=(
        'Run_Exploratory_Following.py','idm_exploratory_following.py','idm_low_speed.py',
        'idm_reverse_motion.py','idm_staged_stop.py','idm_brake_harness.py','idm_brake_config.py',
        'idm_brake_metrics.py','Run_Exploratory_Model3.py','idm_pair_exploration.py'),
        verify_completed=lambda directory,result:verify_completed(directory,result,c))


if __name__=='__main__':raise SystemExit(main())
