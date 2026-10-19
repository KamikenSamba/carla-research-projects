"""Synthetic single-car brake fixtures; this is not a CARLA physics model."""
import copy
import csv
from dataclasses import asdict,replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

from idm_brake_config import BrakeConfig,load_config
from idm_brake_harness import BrakeHarness,LaunchController
from idm_brake_metrics import analyze,read_validated_run
from Run_Calibrate_Brakes import main
from test_idm_harness import World,Actor,Gate,csv_rows
from test_idm_context import vector


def config():
    c=BrakeConfig()
    a=dict(policy_sha256=c.policy_sha256,opendrive_sha256=hashlib.sha256(b'synthetic').hexdigest(),
           evidence_sha256='f'*64,reviewed_by='TEST FIXTURE',reviewed_at='TEST ONLY',
           evidence_reference='synthetic fixture',scene_review_reference='TEST ONLY',
           dedicated_session_reference='TEST ONLY',carla_version='0.9.16',map_name=c.map_name,
           points=[[i,0,0] for i in range(57)])
    return replace(c,route_audit=a)


def run(directory,*,fault=None,fault_frame=40,control_delay=1,motion_delay=2,setup=None):
    w=World();gate=Gate();commands=[];w.map.to_opendrive=lambda:'synthetic'
    wp=w.map.get_waypoint_xodr
    def waypoint(*args):
        p=wp(*args);p.lane_type='Driving';return p
    w.map.get_waypoint_xodr=waypoint
    original=w.try_spawn_actor
    def spawn(bp,tf):
        a=original(bp,tf);a.control=NS(throttle=0,brake=0,steer=0,hand_brake=False,reverse=False,
                                     manual_gear_shift=False,gear=1)
        a.acceleration=vector();a.get_acceleration=lambda:a.acceleration
        def apply(control):
            commands.append(dict(frame=w.frame,control=control,actor=a.id));w.events.append(('manual_control',a.id))
        a.apply_control=apply
        a.get_physics_control=lambda:NS(mass=1800,center_of_mass=vector(),wheels=[
            NS(radius=30,tire_friction=1,max_brake_torque=1000,max_handbrake_torque=1000) for _ in range(4)])
        return a
    w.try_spawn_actor=spawn
    def tick():
        w.frame+=1;w.time+=.05;w.events.append(('tick',w.frame))
        if len(w.actors)>1:
            a=w.actors[1];old=a.speed
            applied=[cmd for cmd in commands if cmd['frame']<=w.frame-control_delay]
            if applied:a.control=NS(**vars(applied[-1]['control']),gear=1)
            brake_cmd=next((cmd for cmd in commands if cmd['control'].brake>.1),None)
            if brake_cmd and w.frame>=brake_cmd['frame']+motion_delay:
                a.speed=max(0,a.speed-.1)
            elif w.frame>20:
                a.speed=min(1,a.speed+.1)
            a.acceleration=vector((a.speed-old)/.05,0,0)
            a.x+=a.speed*.05
            if fault=='never_stop' and brake_cmd:
                a.speed=.8;a.acceleration=vector(-.3,0,0) if w.frame<brake_cmd['frame']+6 else vector()
            if fault=='no_deceleration' and brake_cmd:a.speed=1;a.acceleration=vector()
            if fault=='no_control' and brake_cmd:a.control.brake=0
            if fault=='stop_wobble' and brake_cmd and w.frame==brake_cmd['frame']+15:
                a.speed=.1;a.acceleration=vector((a.speed-old)/.05,0,0)
            if fault=='ready_dip' and w.frame==50:a.speed=.85;a.acceleration=vector()
            if fault=='launch_stall' and w.frame>20:a.speed=0;a.acceleration=vector()
            if w.frame==fault_frame:
                if fault=='peak':a.acceleration=vector(5.01,0,0)
                elif fault=='sustained':w.sustained=True
                elif fault=='unknown_accel':a.get_acceleration=lambda:None
                elif fault=='nonfinite_accel':a.acceleration=vector(float('nan'),0,0)
                elif fault=='speed':a.speed=1.21
                elif fault=='offlane':a.y=5
                elif fault=='heading':a.yaw=6
                elif fault=='endpoint':a.x=37
                elif fault=='handbrake':a.control.hand_brake=True
                elif fault=='missing_control':del a.control.brake
                elif fault=='disappear':w.actors.remove(a)
                elif fault=='external':w.actors.append(NS(id=999,type_id='vehicle.external'))
                elif fault=='server':raise RuntimeError('synthetic server failure')
                elif fault=='getter':a.get_acceleration=lambda:(_ for _ in ()).throw(RuntimeError('getter lost'))
                elif fault=='interleave':w.frame+=1
                elif fault=='settings':w.settings.max_substeps=7
            if getattr(w,'sustained',False):a.acceleration=vector(-3.01,0,0)
        return w.frame
    w.tick=tick
    if setup:setup(w)
    client=NS(get_world=lambda:w,get_client_version=lambda:'0.9.16',get_server_version=lambda:'0.9.16',
              get_trafficmanager=lambda *_:pytest.fail('TM must not be constructed'))
    api=NS(LaneType=NS(Driving='Driving'),VehicleControl=lambda **kw:NS(**kw),
           CityObjectLabel=NS(**{k:k for k in ('Car','Truck','Bus','Motorcycle','Bicycle','Pedestrians')}))
    h=BrakeHarness(client,api,config(),directory,gate)
    return h.run(),w,gate,h,commands


def test_e2e_one_actor_no_tm_no_extra_tick_cleanup_and_metrics(tmp_path):
    result,w,gate,h,commands=run(tmp_path)
    assert result['status']=='completed',result
    metrics=result['metrics'];assert result['calibration_valid'] and metrics['valid']
    assert metrics['command_to_control_frames']==1
    assert metrics['command_to_deceleration_frames']==2
    assert metrics['control_to_deceleration_frames']==1
    assert metrics['moving_brake_deceleration_mps2']['mean']==pytest.approx(2)
    assert metrics['moving_brake_deceleration_mps2']['temporal_population_std']==pytest.approx(0,abs=1e-10)
    assert metrics['acceleration_max_mps2']==pytest.approx(2)
    assert metrics['acceleration_min_mps2']==pytest.approx(-2)
    assert metrics['api_fd_absolute_difference_mps2']['maximum']==pytest.approx(0)
    assert metrics['stop_onset_frame']<metrics['stop_confirmed_frame']
    assert (metrics['stop_confirmed_frame']-metrics['stop_onset_frame'])*.05>=.5
    rows=csv_rows(tmp_path/'calibration_observations.csv')
    assert len(rows)==len(commands)==len([e for e in w.events if e[0]=='tick'])==w.frame
    assert len({r['actor_id'] for r in rows})==1
    assert all(int(r['command_based_on_frame'])+1==int(r['frame'])==int(r['control_read_at_frame']) for r in rows)
    assert not any(e[0] in ('tm_sync','autopilot','seed','path') for e in w.events)
    assert w.actors==[w.external] and vars(w.settings)==vars(w.original) and result['cleanup']['complete']
    assert metrics['accepted_braking_lower_bound_mps2'] is None and not metrics['physical_bounds_qualified']
    assert read_validated_run(tmp_path,require_supervised=False)['valid']
    assert not gate.poisoned


@pytest.mark.parametrize('control,motion',[(1,2),(2,3),(3,4)])
def test_command_control_and_motion_delays_are_distinguished(tmp_path,control,motion):
    result,*_=run(tmp_path,control_delay=control,motion_delay=motion)
    assert result['status']=='completed',result
    assert result['metrics']['command_to_control_frames']==control
    assert result['metrics']['command_to_deceleration_frames']==motion
    assert result['metrics']['control_to_deceleration_frames']==motion-control


def test_stop_distances_are_observed_reference_differences(tmp_path):
    result,w,gate,h,_=run(tmp_path)
    m=result['metrics'];request=m['brake_request']
    onset=next(r for r in h.rows if r['frame']==m['stop_onset_frame'])
    actual_control=next(r for r in h.rows if r['frame']==m['first_control_frame'])
    assert m['stop_distance_from_request_m']==pytest.approx(onset['route_progress_m']-request['brake_request_progress_m'])
    assert m['stop_distance_from_control_m']==pytest.approx(onset['route_progress_m']-actual_control['route_progress_m'])
    assert m['stop_time_from_request_s']==pytest.approx((onset['frame']-request['brake_request_frame'])*.05)
    assert m['stop_distance_from_request_m']>m['stop_distance_from_control_m']


@pytest.mark.parametrize('fault,reason',[('peak','acceleration_peak'),('sustained','acceleration_sustained'),
    ('unknown_accel','acceleration_unknown'),('nonfinite_accel','acceleration_unknown'),
    ('speed','speed_envelope'),('offlane','off_driving_lane'),('heading','route_deviation'),
    ('endpoint','route_endpoint_margin'),('handbrake','hand_brake_reverse_or_manual_gear'),
    ('missing_control','control_unknown'),('disappear','actor_disappeared'),('external','external_actor_appeared'),
    ('settings','world_settings_changed')])
def test_safety_aborts_are_censored_and_only_owned_actor_deleted(tmp_path,fault,reason):
    result,w,gate,h,_=run(tmp_path,fault=fault)
    assert result['status']=='safe_early_stop' and result['reason']==reason,result
    assert result['cleanup']['complete'] and not result['calibration_valid']
    assert result['metrics']['censored'] and w.external in w.actors
    with pytest.raises(ValueError):read_validated_run(tmp_path,require_supervised=False)
    if fault in ('peak','unknown_accel','nonfinite_accel'):
        raw=json.loads((tmp_path/'last_observation.json').read_text())
        assert raw['carla_frame']==40 and raw['frame_validated'] and len(raw['rows'])==1


@pytest.mark.parametrize('fault,reason',[('no_control','control_response_timeout'),
    ('no_deceleration','deceleration_response_timeout'),('never_stop','sustained_stop_timeout'),
    ('launch_stall','launch_not_ready')])
def test_measurement_must_reach_real_observed_stop(tmp_path,fault,reason):
    result,*_=run(tmp_path,fault=fault)
    assert result['status']=='safe_early_stop' and result['reason']==reason,result
    assert not result['calibration_valid'] and result['metrics']['censored']
    assert result['metrics']['stop_distance_from_request_m'] is None


@pytest.mark.parametrize('fault',['server','getter','interleave'])
def test_poison_or_external_tick_suppresses_cleanup_rpc(tmp_path,fault):
    result,w,gate,h,_=run(tmp_path,fault=fault,fault_frame=2)
    assert result['status']==('cleanup_incomplete' if fault=='interleave' else 'server_unavailable'),result
    assert not any(e[0]=='destroy' for e in w.events)
    if fault!='interleave':assert gate.poisoned and gate.calls_after_poison==0
    if fault=='getter':
        raw=json.loads((tmp_path/'last_observation.json').read_text())
        assert raw['carla_frame']==2 and raw['acquisition_status']=='partial' and raw['acquired']['velocity']


def test_unreviewed_default_refused_before_connection_or_os_access(tmp_path,monkeypatch,capsys):
    import Run_Calibrate_Brakes as entry
    monkeypatch.setattr(entry,'listener_pids',lambda *_:pytest.fail('must not inspect/connect'))
    assert main([])==0
    assert json.loads(capsys.readouterr().out)['executable_route'] is False
    with pytest.raises(SystemExit):main(['--execute','--server-pid','1'])
    h=BrakeHarness(NS(get_world=lambda:pytest.fail('must not connect')),None,BrakeConfig(),tmp_path,Gate())
    result=h.run();assert result['status']=='exception' and 'unaudited' in result['reason']


@pytest.mark.parametrize('change',[dict(target_speed=5),dict(brake=.8),dict(dt=.1),dict(throttle_cap=.2),
    dict(ready_seconds=1),dict(acceleration_peak=6),dict(end_clearance=10),dict(speed_limit=None),dict(road=11)])
def test_config_cannot_silently_relax_first_trial(change):
    with pytest.raises(ValueError):replace(BrakeConfig(),**change)


@pytest.mark.parametrize('change',[dict(opendrive_sha256='bad'),dict(scene_review_reference=''),
    dict(dedicated_session_reference=''),dict(points=[]),dict(physical_bounds=dict(braking_lower_bound_mps2=2))])
def test_route_review_is_independent_and_never_prefills_physical_bounds(change):
    c=config()
    with pytest.raises(ValueError):replace(c,route_audit=dict(c.route_audit,**change))


def test_launch_controller_is_bounded_and_slew_limited():
    c=BrakeConfig();controller=LaunchController(c);previous=0
    for speed in [0]*100+[.95]*20+[1.19]*100:
        throttle=controller.step(speed)
        assert 0<=throttle<=.1 and abs(throttle-previous)<=.005+1e-12
        previous=throttle
    assert controller.integral<=.1


@pytest.mark.parametrize('corruption',['raw_frame','csv_frame','ledger','worker','unsupervised'])
def test_replay_rejects_incomplete_or_mismatched_artifacts(tmp_path,corruption):
    result,*_=run(tmp_path)
    assert result['status']=='completed'
    if corruption=='raw_frame':
        p=tmp_path/'calibration_raw.jsonl';lines=p.read_text().splitlines();r=json.loads(lines[0]);r['carla_frame']+=1
        lines[0]=json.dumps(r);p.write_text('\n'.join(lines)+'\n')
    elif corruption=='csv_frame':
        p=tmp_path/'calibration_observations.csv';rs=csv_rows(p);rs[5]['frame']=rs[4]['frame']
        with p.open('w',newline='') as f:
            writer=csv.DictWriter(f,list(rs[0]));writer.writeheader();writer.writerows(rs)
    elif corruption=='ledger':(tmp_path/'command_ledger.jsonl').write_text('')
    elif corruption=='worker':
        p=tmp_path/'worker_result.json';r=json.loads(p.read_text());r['status']='safe_early_stop';p.write_text(json.dumps(r))
    if corruption=='unsupervised':
        with pytest.raises(OSError):read_validated_run(tmp_path)
    else:
        with pytest.raises(ValueError):read_validated_run(tmp_path,require_supervised=False)

@pytest.mark.parametrize('fault',['stop_wobble','ready_dip'])
def test_continuous_windows_restart_after_one_bad_observation(tmp_path,fault):
    result,w,gate,h,_=run(tmp_path,fault=fault)
    assert result['status']=='completed',result
    m=result['metrics']
    if fault=='stop_wobble':assert m['stop_confirmed_frame']>93
    else:assert m['brake_request']['brake_request_frame']>71


def test_offline_reducer_rejects_safety_violation_even_if_completed_claimed(tmp_path):
    result,w,gate,h,_=run(tmp_path)
    records=copy.deepcopy(h.rows);records[30]['a_long_mps2']=5.01
    metrics=analyze(records,h.c,completed=True)
    assert not metrics['valid'] and 'acceleration_gate' in metrics['invalid_reasons']


@pytest.mark.parametrize('fault,reason',[('physics','physics_parameters_unknown'),('hash','route_map_hash_mismatch'),
                                       ('occupied','external_dynamic_actors_present'),('landmarks','regulatory_landmarks')])
def test_preflight_rejects_missing_physics_and_bad_scene(tmp_path,fault,reason):
    def setup(w):
        if fault=='hash':w.map.to_opendrive=lambda:'different'
        elif fault=='occupied':w.actors.append(NS(id=99,type_id='vehicle.external'))
        elif fault=='landmarks':w.landmarks=[NS(id='test')]
        elif fault=='physics':
            original=w.try_spawn_actor
            def spawn(*args):
                a=original(*args);a.get_physics_control=lambda:NS(mass=None,wheels=[]);return a
            w.try_spawn_actor=spawn
    result,w,*_=run(tmp_path,setup=setup)
    assert result['status']=='safe_early_stop' and result['reason']==reason,result
    assert result['cleanup']['complete']
    assert not any(e[0]=='tick' for e in w.events)


def test_failed_owned_actor_destroy_is_never_valid_calibration(tmp_path):
    def setup(w):
        original=w.try_spawn_actor
        def spawn(*args):
            a=original(*args);a.destroy=lambda:False;return a
        w.try_spawn_actor=spawn
    result,w,*_=run(tmp_path,setup=setup)
    assert result['status']=='cleanup_incomplete'
    assert not result['calibration_valid'] and not result['metrics']['valid']
    assert vars(w.settings)==vars(w.original) and w.external in w.actors


@pytest.mark.parametrize('exit_code,fatal,expected',[(3221225477,False,'native_abnormal_exit'),
    (0,True,'native_abnormal_exit'),(0,False,'invalid_result')])
def test_supervisor_uses_new_entry_and_records_native_or_invalid_worker(tmp_path,monkeypatch,exit_code,fatal,expected):
    import Run_Calibrate_Brakes as entry
    import Run_IDM_Following_Experiment as parent
    monkeypatch.setattr(entry,'listener_pids',lambda port:set())
    monkeypatch.setattr(parent,'process_alive',lambda pid:True)
    monkeypatch.setattr(parent,'listener_pids',lambda port:{100})
    monkeypatch.setattr(parent.tempfile,'gettempdir',lambda:str(tmp_path))
    monkeypatch.setattr(parent,'source_metadata',lambda **kw:dict(source_commit='fixture'))
    def popen(command,**kwargs):
        assert Path(command[command.index('--worker')-1]).name=='Run_Calibrate_Brakes.py'
        directory=Path(command[command.index('--run-dir')+1])
        assert command[command.index('--stage')+1]=='brake_calibration'
        if exit_code==0:
            (directory/'worker_result.json').write_text(json.dumps(dict(status='completed',exit_code=0,cleanup=dict(complete=True))))
        if fatal:(directory/'worker_stderr.txt').write_text('Fatal Python error: Access violation')
        return NS(pid=200,returncode=exit_code,poll=lambda:exit_code,wait=lambda:exit_code)
    monkeypatch.setattr(parent.subprocess,'Popen',popen)
    c=config();p=tmp_path/'config.json';p.write_text(json.dumps(asdict(c)))
    code=main(['--execute','--config',str(p),'--server-pid','100','--output-root',str(tmp_path/'output')])
    result=json.loads(next((tmp_path/'output').glob('*/result.json')).read_text())
    assert result['status']==expected and code!=0
    assert result['worker_exit_code']==exit_code
    assert not (tmp_path/'carla-idm-harness-2000.lock').exists()


def test_replay_supervised_complete_run_and_reject_modified_raw(tmp_path,capsys):
    result,w,gate,h,_=run(tmp_path)
    result.update(supervised=True,worker_exit_code=0,fatal_stderr=False)
    (tmp_path/'result.json').write_text(json.dumps(result))
    assert main(['--evaluate-run',str(tmp_path)])==0
    assert json.loads(capsys.readouterr().out)['valid']
    p=tmp_path/'calibration_raw.jsonl';lines=p.read_text().splitlines();row=json.loads(lines[-1])
    row['rows'][0]['control']['brake']=0;lines[-1]=json.dumps(row);p.write_text('\n'.join(lines)+'\n')
    with pytest.raises(ValueError,match='raw control'):read_validated_run(tmp_path)


def test_existing_evidence_directory_is_not_reused(tmp_path):
    marker=tmp_path/'config_used.json';marker.write_text('original evidence')
    with pytest.raises(FileExistsError):run(tmp_path)
    assert marker.read_text()=='original evidence'


def test_cli_validation_never_imports_carla():
    script=Path(__file__).parents[1]/'Run_Calibrate_Brakes.py'
    code="""import builtins,runpy,sys
from pathlib import Path
original=builtins.__import__
def guarded(name,*a,**k):
    if name=='carla':raise AssertionError('CARLA import forbidden')
    return original(name,*a,**k)
builtins.__import__=guarded
sys.path.insert(0,str(Path(sys.argv[1]).parent));sys.argv=[sys.argv[1]]
runpy.run_path(sys.argv[0],run_name='__main__')
"""
    process=subprocess.run([sys.executable,'-c',code,str(script)],capture_output=True,text=True)
    assert process.returncode==0,process.stderr
    assert json.loads(process.stdout)['status']=='validated_offline_no_connection'
