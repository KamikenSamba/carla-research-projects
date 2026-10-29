"""Existing V2 core and bounded actuator, without CARLA or a physics claim."""
import copy
import csv
from dataclasses import asdict, replace
import json
import logging
from types import SimpleNamespace as NS
import pytest

import idm_controller
from idm_controller import IDMFleetController, ControlConfig
from idm_carla_validation import ValidationConfig,validation_parameters,BoundedV2Actuator,V2SingleHarness
from idm_experiment_harness import EarlyStop
from idm_brake_metrics import FIELDS
from idm_model import compute_idm_acceleration
from idm_snapshot import read_snapshot
from test_idm import FakeActor,FakeWorld,Waypoint
from test_idm_harness import Gate
import Run_V2_IDM_Carla_Validation as entry


def test_v2_optional_sink_preserves_original_requests_and_avoids_tm_on_fresh_owned_actor(monkeypatch):
    api=NS(VehicleControl=lambda **kw:NS(**kw));monkeypatch.setattr(idm_controller,'import_carla',lambda:api)
    actor=FakeActor(1,0,0);world=FakeWorld([actor]);sent=[]
    fleet=IDMFleetController(world,{'solo':actor},{'solo':validation_parameters('solo')},.05,
        ControlConfig(),logging.getLogger('test'),control_sink=lambda *args:sent.append(args),disable_autopilot=False)
    rows=fleet.step(0.)
    assert len(sent)==1 and not actor.commands and not actor.autopilot
    assert rows[0]['idm_acceleration_mps2']==.3 and rows[0]['leader_id']==''
    assert sent[0][2].throttle==rows[0]['throttle']


def test_distinct_per_vehicle_parameters_and_T_counterfactual_use_existing_equation(monkeypatch):
    monkeypatch.setattr(idm_controller,'import_carla',lambda:NS(VehicleControl=lambda **kw:NS(**kw)))
    follower,leader=FakeActor(1,0,.3),FakeActor(2,20,.3);world=FakeWorld([follower,leader])
    params={k:validation_parameters(k) for k in ('leader','follower')}
    fleet=IDMFleetController(world,{'follower':follower,'leader':leader},params,.05,
        ControlConfig(),logging.getLogger('test'),control_sink=lambda *a:None,disable_autopilot=False)
    rows={r['logical_actor_id']:r for r in fleet.step(0)}
    assert rows['follower']['leader_id']==2 and rows['leader']['leader_id']==''
    assert rows['follower']['desired_time_headway_s']==2 and rows['leader']['desired_time_headway_s']==1
    p=params['follower'];gap=rows['follower']['gap_m']
    assert compute_idm_acceleration(.3,replace(p,desired_time_headway_s=1),gap,.3)>rows['follower']['idm_acceleration_mps2']
    assert compute_idm_acceleration(.3,p)==compute_idm_acceleration(.3,replace(p,desired_time_headway_s=9))


def test_actuator_scaling_slew_and_unqualified_brake_rejection():
    c=ValidationConfig();ctl=BoundedV2Actuator(c);last=0.
    for _ in range(150):
        th,br,steer=ctl.step(NS(throttle=1.,brake=0.,steer=.5),.2,'')
        assert 0<=th<=.10000001 and abs(th-last)<=.00100001 and br==0 and steer==.1
        last=th
    with pytest.raises(EarlyStop,match='transition_not_qualified'):
        ctl.step(NS(throttle=0.,brake=.1,steer=0.),.2,'')
    with pytest.raises(EarlyStop,match='fallback'):
        ctl.step(NS(throttle=0.,brake=1.,steer=0.),0.,'junction')


@pytest.mark.parametrize('field,value',[('acceleration_peak',6),('speed_limit',1),('route_audit',{}),('drive_seconds',60)])
def test_fixed_validation_policy_keeps_safety(field,value):
    with pytest.raises(ValueError):replace(ValidationConfig(),**{field:value})


def test_offline_cli_never_uses_native_identity_or_client(tmp_path,monkeypatch,capsys):
    p=tmp_path/'config.json';p.write_text(json.dumps(dict(trial='single',scenario=asdict(ValidationConfig()))))
    monkeypatch.setattr(entry,'native_identity',lambda *a:pytest.fail('native access'))
    assert entry.main(['--config',str(p)])==0
    assert json.loads(capsys.readouterr().out)['route_audit'] is None


def test_single_synthetic_full_capture_v2_replay_stop_cleanup(tmp_path,monkeypatch):
    monkeypatch.setattr('idm_brake_harness.LaunchController.step',lambda *a:pytest.fail('manual launch PI used in IDM trial'))
    monkeypatch.setattr('idm_carla_validation.time.sleep',lambda *a:None)
    monkeypatch.setattr('idm_low_speed.time.sleep',lambda *a:None)
    api=NS(VehicleControl=lambda **kw:NS(**kw));monkeypatch.setattr(idm_controller,'import_carla',lambda:api)
    actor=FakeActor(11,0,0);actor.type_id='vehicle.tesla.model3';actor.attributes={'role_name':'v2:solo'}
    actor.control=NS(throttle=0.,brake=0.,steer=0.,gear=1,hand_brake=False,reverse=False,manual_gear_shift=False)
    external=NS(id=1,type_id='static.prop.mesh');actors=[external,actor]
    sensor=NS(id=12,is_listening=True,type_id='sensor.other.collision')
    def destroy(a):actors.remove(a);return True
    actor.destroy=lambda:destroy(actor);sensor.destroy=lambda:destroy(sensor);actors.append(sensor)
    sensor.stop=lambda:setattr(sensor,'is_listening',False)
    actor.apply_control=lambda cmd:setattr(actor,'control',NS(gear=1,**vars(cmd)))
    old_settings=NS(synchronous_mode=False,fixed_delta_seconds=None)
    w=NS(frame=1,time=.05,settings=NS(synchronous_mode=True,fixed_delta_seconds=.05))
    w.get_actors=lambda:list(actors);w.get_settings=lambda:copy.copy(w.settings)
    w.apply_settings=lambda s:setattr(w,'settings',copy.copy(s))
    w.get_snapshot=lambda:NS(frame=w.frame,timestamp=NS(elapsed_seconds=w.time))
    wp=Waypoint();wp.road_id=8
    w.map=NS(get_waypoint=lambda *a,**kw:Waypoint(actor.x))
    def tick():
        old=actor.speed;control=actor.control
        desired=0. if control.brake else 2*control.throttle
        actor.speed=max(0.,old+max(-.05,min(.05,desired-old)));actor.accel=(actor.speed-old)/.05
        actor.x+=actor.speed*.05;w.frame+=1;w.time+=.05;return w.frame
    w.tick=tick
    h=V2SingleHarness(None,api,ValidationConfig(),tmp_path,Gate())
    def prepare():
        h.world=w;h.map=w.map;h.actor=actor;h.sensor=sensor;h.initial_ids={1}
        h.owned={'collision':sensor,'vehicle':actor};h.settings_touched=True;h.original=old_settings
        h.last_frame=w.frame;h.last_time=w.time
    h.prepare=prepare
    def observe(phase,cmd):
        r=dict.fromkeys(FIELDS,None);v=actor.speed;ctl=actor.control
        r.update(phase=phase,frame=w.frame,sim_time_s=h.t,world_time_s=w.time,actor_id=actor.id,
            x=actor.x,y=0.,z=0.,roll=0.,pitch=0.,yaw=0.,vx=v,vy=0.,vz=0.,speed_mps=v,
            speed_planar_mps=v,v_long_mps=v,a_long_mps2=actor.accel,a_fd_long_mps2=actor.accel,
            route_progress_m=actor.x,gear=1,hand_brake=False,
            heading_error_deg=0.,signed_lateral_m=0.,
            command_based_on_frame=cmd['based_on_frame'],control_read_at_frame=w.frame)
        r.update({k:getattr(ctl,k) for k in ('throttle','brake','steer')})
        r.update({'command_'+k:cmd['control'][k] for k in ('throttle','brake','steer')})
        h.rows.append(r)
        with (tmp_path/'calibration_observations.csv').open('a',newline='') as f:csv.DictWriter(f,FIELDS).writerow(r)
        with (tmp_path/'calibration_raw.jsonl').open('a') as f:f.write(json.dumps(dict(carla_frame=w.frame,frame_validated=True))+'\n')
        return r
    h.observe=observe
    result=h.run()
    assert result['status']=='completed',(result['status'],result.get('reason'))
    assert result['cleanup']['complete'] and actors==[external] and not actor.autopilot
    assert vars(w.settings)==vars(old_settings)
    entry.verify_completed(tmp_path,result,h.c)
    snapshot=read_snapshot(tmp_path/'initial_snapshot.json')
    assert snapshot[0]['desired_speed_mps']==.5 and snapshot[0]['desired_time_headway_s']==1
