"""Fidelity diagnostics change observability, not IDM/PID or safety behavior."""
import copy
import csv
from dataclasses import asdict
import json
import math
from types import SimpleNamespace as NS

import pytest

from idm_controller import VehicleController, ControlConfig, LOG_FIELDS
from idm_model import PID, compute_idm_acceleration
from idm_carla_validation import ValidationConfig, validation_parameters, BoundedV2Actuator
from idm_experiment_harness import EarlyStop
from idm_fidelity import (acceleration_definitions, analyze, analyze_run, control_float,
                          errors, lag_metrics, number, write_csv)


def legacy_command(controller, speed, acceleration, steering, emergency=False):
    """Independent frozen pre-change implementation: command equality regression."""
    p, c = controller.parameters, controller.config
    acceleration=max(-p.max_decel_mps2,min(p.max_accel_mps2,acceleration))
    if emergency:
        controller.acceleration=-p.max_decel_mps2;controller.speed_pid.reset()
    else:
        controller.acceleration=c.acceleration_filter_alpha*acceleration+(1-c.acceleration_filter_alpha)*controller.acceleration
    target=max(0.,speed+controller.acceleration*controller.dt)
    ff=controller.acceleration/(p.max_accel_mps2 if controller.acceleration>=0 else p.max_decel_mps2)
    effort=controller.speed_pid.step(target-speed)+ff
    th,br=min(1.,max(0.,effort)),min(1.,max(0.,-effort))
    if emergency:th,br=0.,1.
    steer=max(-c.maximum_steer,min(c.maximum_steer,controller.steering_pid.step(steering)))
    return target,th,br,steer


def test_diagnostics_keep_exact_original_commands_and_pid_state():
    p=validation_parameters('solo');cfg=ControlConfig()
    old,new=VehicleController(p,.05,cfg),VehicleController(p,.05,cfg)
    for i in range(180):
        speed=abs(math.sin(i*.1))*.6;accel=math.cos(i*.13)*3;steer=math.sin(i)
        emergency=i in (40,80)
        assert new.command(speed,accel,steer,emergency)==legacy_command(old,speed,accel,steer,emergency)
        d=new.last_diagnostics
        assert d['pid_output']==pytest.approx(d['pid_p']+d['pid_i']+d['pid_d'])
        assert d['effort_before_clamp']==d['pid_output']+d['feed_forward']
        assert vars(new.speed_pid)==vars(old.speed_pid)
    assert 'pid_output' not in LOG_FIELDS  # existing CSV schema is unchanged


def test_bounded_actuator_diagnostics_and_brake_gate_remain_unchanged():
    ctl=BoundedV2Actuator(ValidationConfig())
    assert ctl.step(NS(throttle=1.,brake=0.,steer=0.),0.,'')==(.001,0.,0.)
    d=ctl.last_diagnostics
    assert d['throttle_scaled']==.1 and d['throttle_after_cap']==.1
    assert d['throttle_slew_limited'] and d['throttle_step_limit']==.001
    with pytest.raises(EarlyStop,match='not_qualified'):
        ctl.step(NS(throttle=0.,brake=.01,steer=0.),.11,'')
    assert ctl.throttle==.001  # failure before mutation


@pytest.mark.parametrize('bad',['',None,[],True,'nan','inf','-inf','not a number'])
def test_missing_or_nonfinite_measurements_are_not_zero_filled(bad):
    with pytest.raises((ValueError,TypeError)):number(bad)


def test_error_definition_and_empty_phases():
    e=errors([1.,-1.],[.3,.3])
    assert e['bias']==pytest.approx(-.3)
    assert e['rmse']==pytest.approx(math.sqrt(1.09))
    assert e['mae']==1.
    assert errors([],[])['rmse'] is None
    with pytest.raises(ValueError):errors([1],[])


def state(v, time, forward=1., x=0.):
    return dict(world_time_s=time,vx=v,vy=0.,vz=0.,forward_x=forward,forward_y=0.,forward_z=0.,
                x=x,y=0.,z=0.,ax=(v+.04)/.05,ay=0.,az=0.)


def test_signed_vector_fd_is_not_speed_magnitude_fd_or_position_endpoint():
    a=state(-.04,0.);b=state(.06,.05,x=.001)
    r=acceleration_definitions(a,b)
    assert r['vector_fd']==pytest.approx(2.)
    assert r['magnitude_fd']==pytest.approx(.4)
    assert r['signed_position_interval_speed']==pytest.approx(.02)
    assert r['body_endpoint_speed']==.06
    with pytest.raises(ValueError):acceleration_definitions(a,a)


def test_rotating_basis_signed_speed_fd_differs_from_vector_projection():
    a=state(.5,0.,forward=1.);b=state(.5,.05,forward=-1.)
    r=acceleration_definitions(a,b)
    assert r['vector_fd']==0 and r['signed_speed_fd']==-20


def test_lag_scan_has_fixed_support_and_explicit_shift_sign():
    rows=[dict(raw_idm_a=float(i),api_a=float(i-2)) for i in range(30)]
    scan=lag_metrics(rows,3)
    assert {r['n'] for r in scan}=={24}
    assert min(scan,key=lambda r:r['rmse'])['response_shift_frames']==2
    assert lag_metrics(rows[:4],3)==[]


def source_run(path, n=30):
    path.mkdir(parents=True)
    cfg=ValidationConfig();p=validation_parameters('solo');control_cfg=ControlConfig()
    controller=VehicleController(p,cfg.dt,control_cfg);actuator=BoundedV2Actuator(cfg)
    def native(i):
        s=state(i*.005,i*.05,x=.5*.1*(i*.05)**2)
        s.update(frame=str(100+i),actor_id='7',phase='settle' if i==0 else 'launch',
            sim_time_s=i*.05,speed_mps=i*.005,speed_planar_mps=i*.005,
            control_read_at_frame=100+i,a_long_mps2=.1,a_fd_long_mps2=.1,
            ax=.1,throttle=0.,brake=0.,steer=0.,command_sequence=i,
            command_throttle=0.,command_brake=0.,command_steer=0.)
        return s
    motion=[native(0)];rows=[];ledger=[]
    for i in range(n):
        pre=motion[-1];cur=native(i+1)
        raw=compute_idm_acceleration(pre['speed_mps'],p)
        target,th,br,st=controller.command(pre['speed_mps'],raw,0.)
        bounded=actuator.step(NS(throttle=control_float(th),brake=control_float(br),steer=control_float(st)),pre['speed_planar_mps'],'')
        for key,value in zip(('throttle','brake','steer'),bounded):
            cur[key]=control_float(value);cur['command_'+key]=value
        ledger.append(dict(operation='manual_control',outcome='requested',actor_id=7,sequence=i+1,
            based_on_frame=100+i,control=dict(zip(('throttle','brake','steer'),bounded))))
        motion.append(cur)
        rows.append(dict(observation_frame=101+i,command_based_on_frame=100+i,
            observation_time_s=(i+1)*.05,v2_simulation_time=i*.05,logical_actor_id='solo',actor_id='7',
            v2_speed_mps=pre['speed_mps'],v2_gap_m='',v2_leader_speed_mps='',v2_idm_acceleration_mps2=raw,
            v2_target_speed_mps=target,v2_throttle=th,v2_brake=br,v2_steer=st,v2_fallback_reason='',
            v2_filtered_acceleration_mps2=controller.acceleration,a_long_mps2=.1,a_fd_long_mps2=.1,
            actual_throttle=cur['throttle'],actual_brake=cur['brake'],actual_steer=cur['steer']))
    write_csv(path/'calibration_observations.csv',motion);write_csv(path/'v2_validation.csv',rows)
    (path/'command_ledger.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in ledger))
    for file,data in [('resolved_control_config.json',asdict(control_cfg)),('resolved_idm_parameters.json',{'solo':asdict(p)}),
                       ('config_used.json',{'scenario':asdict(cfg)|dict(drive_seconds=n*.05)}),('launch.json',{'source_commit':'synthetic'})]:
        (path/file).write_text(json.dumps(data))
    for file in ('calibration_raw.jsonl','longitudinal_diagnostics.jsonl'):
        (path/file).write_text('synthetic fixture: numerical cross-checks use CSV; no real-data claim\n')
    return rows,motion


def test_offline_replay_checks_frame_command_and_float32_sink_without_carla(tmp_path,monkeypatch):
    import dt_risk_common
    monkeypatch.setattr(dt_risk_common,'import_carla',lambda:pytest.fail('CARLA imported'))
    path=tmp_path/'run';source_run(path)
    summary,rows=analyze_run(path,'synthetic')
    assert len(rows)==summary['groups']['solo']['frames']==30
    assert all(r['response_frame']==r['source_frame']+1 for r in rows)
    assert summary['groups']['solo']['api_vs_vector_fd']['rmse']<1e-12
    assert all(r['replay_provenance'].startswith('offline_') for r in rows)


@pytest.mark.parametrize('corruption',['frame','duplicate','command','missing','nan','time'])
def test_corrupt_or_missing_inputs_reject_instead_of_fabricating(tmp_path,corruption):
    run=tmp_path/'run';rows,motion=source_run(run)
    if corruption=='frame':rows[4]['command_based_on_frame']-=1
    if corruption=='duplicate':rows.append(copy.copy(rows[0]))
    if corruption=='command':rows[4]['actual_throttle']=.9
    if corruption=='nan':rows[4]['a_long_mps2']='nan'
    if corruption=='time':rows[4]['observation_time_s']+=.05
    if corruption=='missing':
        (run/'resolved_control_config.json').unlink()
    else:
        (run/'v2_validation.csv').unlink();write_csv(run/'v2_validation.csv',rows)
    with pytest.raises((ValueError,FileNotFoundError,KeyError)):analyze_run(run,'synthetic')


def test_cli_outputs_new_only_and_refuses_source_tree_and_overwrites(tmp_path):
    import Analyze_IDM_Fidelity as entry
    root=tmp_path/'evidence';root.mkdir()
    for label in ('single_idm_once','fleet_idm_once','snapshot_replay_once'):
        folder=root/label;folder.mkdir();source_run(folder/'run')
        (folder/'controller_result.json').write_text(json.dumps(dict(successful=True,run_dir=str(folder/'run'))))
    out=tmp_path/'analysis'
    assert entry.main(['--evidence-root',str(root),'--output-root',str(out)])==0
    summary=json.loads((out/'fidelity_summary.json').read_text())
    assert summary['total_rows']==90 and summary['route_audit'] is None and summary['physical_bounds'] is None
    with pytest.raises(ValueError,match='output'):analyze(root,out)
    with pytest.raises(ValueError,match='output'):analyze(root,root/'new')
