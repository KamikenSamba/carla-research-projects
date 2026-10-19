"""Server-free brake metrics. Unknowns stay null; incomplete trials cannot be qualified."""
import csv
import json
from pathlib import Path
import statistics

from idm_brake_config import finite, load_config

FIELDS = ['phase','frame','sim_time_s','world_time_s','actor_id','x','y','z','vx','vy','vz',
 'forward_x','forward_y','forward_z','yaw','pitch','roll','speed_mps','speed_planar_mps','v_long_mps',
 'ax','ay','az','a_long_mps2','a_fd_long_mps2','a_api_minus_fd_mps2',
 'throttle','brake','hand_brake','steer','reverse','manual_gear_shift','gear',
 'route_progress_m','route_lateral_m','signed_lateral_m','heading_error_deg','front_clearance_m',
 'road_id','section_id','lane_id','at_traffic_light','control_read_at_frame',
 'command_sequence','command_based_on_frame','command_throttle','command_brake','command_steer',
 'brake_request_frame','brake_request_time_s','brake_request_progress_m','brake_request_speed_mps']
BOOL_FIELDS={'hand_brake','reverse','manual_gear_shift','at_traffic_light'}
INT_FIELDS={'frame','actor_id','road_id','section_id','lane_id','control_read_at_frame',
            'command_sequence','command_based_on_frame','brake_request_frame','gear'}


def brake_matched(row,c):
    return abs(row['brake']-c.brake)<=1e-5 and abs(row['throttle'])<=1e-5 and row['hand_brake'] is False


def stop_ready(row,c):
    return (row['speed_planar_mps']<c.stopped_speed and abs(row['v_long_mps'])<c.stopped_speed
            and abs(row['vz'])<c.stopped_speed and abs(row['a_long_mps2'])<=c.ready_acceleration
            and brake_matched(row,c))


def stats(values):
    return dict(n=len(values),mean=statistics.mean(values) if values else None,
                minimum=min(values) if values else None,maximum=max(values) if values else None,
                temporal_population_std=statistics.pstdev(values) if values else None,
                interpretation='within-trial time variation, not between-trial uncertainty or a bound')


def analyze(rows,c,*,completed,abort_reason=None):
    required=['frame','sim_time_s','world_time_s','actor_id','speed_planar_mps','v_long_mps','vz',
              'a_long_mps2','route_progress_m','throttle','brake','command_sequence','command_based_on_frame',
              'x','y','z','vx','vy','forward_x','forward_y','forward_z','yaw','pitch','roll','steer',
              'route_lateral_m','heading_error_deg','front_clearance_m','command_throttle','command_brake','command_steer']
    if not rows or any(not finite(r.get(k)) for r in rows for k in required):
        raise ValueError('empty or unknown/nonfinite measurements')
    for i,r in enumerate(rows):
        if (r['control_read_at_frame']!=r['frame'] or r['command_based_on_frame']!=r['frame']-1
                or r['actor_id']!=rows[0]['actor_id'] or r['phase'] not in ('settle','launch','braking','stop_confirm')):
            raise ValueError('measurement/command identity mismatch')
        if i and (r['frame']!=rows[i-1]['frame']+1 or
                  abs(r['sim_time_s']-rows[i-1]['sim_time_s']-c.dt)>1e-6 or
                  abs(r['world_time_s']-rows[i-1]['world_time_s']-c.dt)>1e-5):
            raise ValueError('noncontiguous frames/timestamps')
    safety_errors=[];count=0
    for r in rows:
        count=count+1 if abs(r['a_long_mps2'])>c.acceleration_sustained else 0
        if abs(r['a_long_mps2'])>c.acceleration_peak or count>=2:safety_errors.append('acceleration_gate')
        if max(r['speed_planar_mps'],abs(r['v_long_mps']))>c.speed_limit:safety_errors.append('speed_gate')
        if (r['route_lateral_m']>c.lateral_limit or abs(r['heading_error_deg'])>c.heading_limit
                or r['front_clearance_m']<c.end_clearance or r['v_long_mps']<-.02):safety_errors.append('route_gate')
        if (tuple(r[k] for k in ('road_id','section_id','lane_id'))!=(c.road,c.section,c.lane)
                or r['at_traffic_light'] is not False):safety_errors.append('lane_signal_gate')
        if (any(type(r[k]) is not bool or r[k] for k in ('hand_brake','reverse','manual_gear_shift'))
                or not 0<=r['throttle']<=c.throttle_cap+1e-5 or not 0<=r['brake']<=1
                or abs(r['steer'])>c.steer_cap+1e-5):safety_errors.append('control_gate')
        if not 0<=r['command_throttle']<=c.throttle_cap+1e-9 or not 0<=r['command_brake']<=c.brake+1e-9:
            safety_errors.append('command_envelope')
    braking=[r for r in rows if r['phase'] in ('braking','stop_confirm')]
    first_control=onset=onset_confirmed=stop=stop_confirmed=None
    request=None;errors=list(safety_errors)
    if braking:
        first=braking[0];request={k:first[k] for k in ('brake_request_frame','brake_request_time_s',
                                                     'brake_request_progress_m','brake_request_speed_mps')}
        if any(not finite(v) for v in request.values()):raise ValueError('unknown brake request reference')
        reference=next((r for r in rows if r['frame']==request['brake_request_frame']),None)
        if (reference is None or reference['phase']!='launch' or first['frame']!=reference['frame']+1
                or abs(reference['route_progress_m']-request['brake_request_progress_m'])>1e-6
                or abs(reference['speed_planar_mps']-request['brake_request_speed_mps'])>1e-6
                or abs(reference['sim_time_s']-request['brake_request_time_s'])>1e-6):
            raise ValueError('brake reference not an observed pre-command state')
        if abs(request['brake_request_speed_mps']-c.target_speed)>c.ready_speed_tolerance+1e-9:
            errors.append('brake_start_speed_outside_design')
        launch=[r for r in rows if r['phase']=='launch' and r['frame']<=reference['frame']]
        ready=[r for r in launch if (reference['frame']-r['frame'])*c.dt<=c.ready_seconds+1e-9]
        if (len(ready)!=round(c.ready_seconds/c.dt)+1 or any(
            abs(r['speed_planar_mps']-c.target_speed)>c.ready_speed_tolerance+1e-9 or
            abs(r['a_long_mps2'])>c.ready_acceleration or abs(r['vz'])>=.05 for r in ready)):
            errors.append('launch_not_continuously_ready')
        candidate=None;stopped=[]
        for r in braking:
            if any(r[k]!=v for k,v in request.items()):raise ValueError('brake request changed within trial')
            if first_control is None and brake_matched(r,c):first_control=r
            if first_control is not None and not brake_matched(r,c):errors.append('brake_not_held')
            decreasing=(r['speed_planar_mps']<=request['brake_request_speed_mps']-c.deceleration_speed_drop
                        and r['a_long_mps2']<=-c.deceleration_threshold)
            if onset is None:
                if decreasing and candidate is not None and r['frame']==candidate['frame']+1:
                    onset,onset_confirmed=candidate,r
                candidate=r if decreasing else None
            if stop_ready(r,c):
                stopped.append(r)
                if stop is None and (r['frame']-stopped[0]['frame'])*c.dt+1e-9>=c.stopped_seconds:
                    stop,stop_confirmed=stopped[0],r
            else:
                stopped=[]
                if stop_confirmed is not None:errors.append('motion_after_stop_confirmation')
        if completed and stop_confirmed and rows[-1]['frame']!=stop_confirmed['frame']:
            errors.append('unexpected_samples_after_stop_confirmation')
        if first_control is None:errors.append('brake_control_not_observed')
        if onset is None:errors.append('deceleration_not_confirmed')
        if stop is None:errors.append('sustained_stop_not_observed')
        for event,name in ((first_control,'control'),(onset_confirmed,'deceleration_confirmation')):
            if event is not None and (event['frame']-request['brake_request_frame'])*c.dt>c.response_timeout+1e-9:
                errors.append(name+'_response_timeout')
    else:errors.append('brake_not_commanded')
    if not completed:errors.append(abort_reason or 'trial_incomplete')
    decel=[-r['a_long_mps2'] for r in braking if brake_matched(r,c) and
           r['speed_planar_mps']>c.stopped_speed and r['a_long_mps2']<0]
    differences=[abs(r['a_api_minus_fd_mps2']) for r in rows if finite(r.get('a_api_minus_fd_mps2'))]
    def delta(event):return event['frame']-request['brake_request_frame'] if event and request else None
    result=dict(schema_version=1,valid=not errors,censored=not completed or stop is None,
        invalid_reasons=sorted(set(errors)),actor_id=rows[0]['actor_id'],samples=len(rows),
        brake_request=request,first_control_frame=first_control['frame'] if first_control else None,
        first_deceleration_frame=onset['frame'] if onset else None,
        deceleration_confirmed_frame=onset_confirmed['frame'] if onset_confirmed else None,
        command_to_control_frames=delta(first_control),command_to_deceleration_frames=delta(onset),
        control_to_deceleration_frames=onset['frame']-first_control['frame'] if onset and first_control else None,
        command_to_control_observed_s=delta(first_control)*c.dt if first_control else None,
        command_to_deceleration_observed_s=delta(onset)*c.dt if onset else None,
        stop_onset_frame=stop['frame'] if stop else None,stop_confirmed_frame=stop_confirmed['frame'] if stop_confirmed else None,
        stop_time_from_request_s=delta(stop)*c.dt if stop else None,
        stop_confirmation_time_from_request_s=delta(stop_confirmed)*c.dt if stop_confirmed else None,
        stop_distance_from_request_m=stop['route_progress_m']-request['brake_request_progress_m'] if stop else None,
        stop_distance_from_control_m=stop['route_progress_m']-first_control['route_progress_m'] if stop and first_control else None,
        distance_observed_since_request_m=braking[-1]['route_progress_m']-request['brake_request_progress_m'] if braking else None,
        acceleration_min_mps2=min(r['a_long_mps2'] for r in rows),acceleration_max_mps2=max(r['a_long_mps2'] for r in rows),
        moving_brake_deceleration_mps2=stats(decel),api_fd_absolute_difference_mps2=stats(differences),
        deceleration_series=[dict(frame=r['frame'],sim_time_s=r['sim_time_s'],speed_mps=r['speed_planar_mps'],
            a_api_mps2=r['a_long_mps2'],a_fd_mps2=r.get('a_fd_long_mps2')) for r in braking],
        sample_interval_s=c.dt,gear_unknown_samples=sum(r.get('gear') is None for r in rows),physical_bounds_qualified=False,
        accepted_braking_lower_bound_mps2=None,accepted_response_upper_bound_s=None,
        limitations=['threshold-defined stop, not mathematical zero velocity',
                    'API and finite difference acceleration share velocity information',
                    'one low-speed manual-control trial does not certify 5m/s or TM response',
                    'first observed control snapshot is not the exact engine actuation instant'])
    return result


def read_validated_run(directory,*,require_supervised=True,result_override=None):
    directory=Path(directory)
    config_path=directory/'config_used.json'
    payload=json.loads(config_path.read_text(encoding='utf-8'))
    from idm_brake_config import BrakeConfig
    c=BrakeConfig(**payload['scenario'])
    result_path=directory/('result.json' if require_supervised else 'worker_result.json')
    result=result_override if result_override is not None else json.loads(result_path.read_text(encoding='utf-8'))
    if (not result.get('calibration_valid') or result.get('stage')!='brake_calibration'
            or result.get('status')!='completed' or result.get('exit_code')!=0 or not result.get('cleanup',{}).get('complete')
            or result.get('config_sha256')!=c.fingerprint or payload.get('config_sha256')!=c.fingerprint
            or (require_supervised and (not result.get('supervised') or result.get('worker_exit_code')!=0
                                       or result.get('fatal_stderr') is not False))):
        raise ValueError('incomplete, unsupervised, or failed calibration is rejected')
    with (directory/'calibration_observations.csv').open(encoding='utf-8-sig',newline='') as f:
        reader=csv.DictReader(f)
        if reader.fieldnames!=FIELDS:raise ValueError('calibration schema mismatch')
        records=[]
        for row in reader:
            parsed={}
            for k,v in row.items():
                if k=='phase':parsed[k]=v
                elif v=='':parsed[k]=None
                elif k in BOOL_FIELDS:
                    if v not in ('True','False'):raise ValueError('unknown boolean')
                    parsed[k]=v=='True'
                else:parsed[k]=int(v) if k in INT_FIELDS else float(v)
            records.append(parsed)
    raw=[json.loads(line) for line in (directory/'calibration_raw.jsonl').read_text(encoding='utf-8').splitlines()]
    if ([r['frame'] for r in records]!=[r['carla_frame'] for r in raw] or any(
        not r['frame_validated'] or r['acquisition_status']!='complete' or len(r['rows'])!=1 for r in raw)):
        raise ValueError('raw/CSV frame mismatch')
    environment=json.loads((directory/'environment.json').read_text(encoding='utf-8'))
    if (not records or records[-1]['frame']!=result.get('final_frame') or
            records[0]['frame']!=environment['initial_sync_frame']+1):
        raise ValueError('result/initial/final frame mismatch')
    for row,envelope in zip(records,raw):
        observed=envelope['rows'][0]
        if observed['actor_id']!=row['actor_id'] or observed['frame']!=row['frame']:
            raise ValueError('raw actor identity mismatch')
        for k in ('throttle','brake','steer','hand_brake','reverse','manual_gear_shift','gear'):
            if observed['control'][k]!=row[k]:raise ValueError('raw control mismatch')
        if observed['a_long_mps2']!=row['a_long_mps2']:raise ValueError('raw acceleration mismatch')
        for k in ('x','y','z'):
            if observed['transform'][k]!=row[k] or observed['velocity'][k]!=row['v'+k]:
                raise ValueError('raw state mismatch')
    ledger=[json.loads(line) for line in (directory/'command_ledger.jsonl').read_text(encoding='utf-8').splitlines()]
    requests={r['sequence']:r for r in ledger if r['operation']=='manual_control' and r.get('outcome')=='requested'}
    returned={r['request_sequence'] for r in ledger if r['operation']=='manual_control' and r.get('outcome')=='call_returned'}
    for row in records:
        command=requests.get(row['command_sequence'])
        if command is None or command['sequence'] not in returned or command['based_on_frame']!=row['command_based_on_frame']:
            raise ValueError('control command ledger mismatch')
        for key in ('throttle','brake','steer'):
            if command['control'][key]!=row['command_'+key]:raise ValueError('control values mismatch')
    metrics=analyze(records,c,completed=True)
    if not metrics['valid']:raise ValueError('invalid calibration: '+','.join(metrics['invalid_reasons']))
    return metrics
