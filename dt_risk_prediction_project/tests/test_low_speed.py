"""Exploratory policy and inherited safety regressions; no CARLA server."""
from dataclasses import asdict,replace
import json
from types import SimpleNamespace as NS
import threading
import pytest

from idm_low_speed import ExploratoryHarness,config_for,local_pose_ok
from idm_brake_config import BrakeConfig
from idm_brake_harness import LaunchController
from idm_experiment_harness import EarlyStop
import Run_Exploratory_Model3 as entry


@pytest.mark.parametrize('trial,target,limit,distance',[('trial1',.5,.7,3),('trial2',1,1.2,10),('calibration',1,1.2,10)])
def test_fixed_protocol_is_not_strict_route_approval(trial,target,limit,distance):
    c=config_for(trial)
    assert (c.target_speed,c.speed_limit,c.drive_distance)==(target,limit,distance)
    assert c.route_audit is None and c.mode!='single_brake_town01_opposite_v3'
    with pytest.raises(ValueError):c.require_executable('brake_calibration')
    c.require_executable('exploratory_low_speed')


@pytest.mark.parametrize('change',[{'speed_limit':2},{'throttle_cap':.2},{'acceleration_peak':6},
    {'road':9},{'follower_s':171},{'lane':1},{'route_audit':{}},{'response_timeout':1},{'maximum_frames':999}])
def test_unauthorized_policy_change_rejected(change):
    with pytest.raises(ValueError):replace(config_for('trial1'),**change)


def test_existing_control_and_safety_values_are_preserved():
    base=asdict(BrakeConfig());c=asdict(config_for('calibration'))
    for k in ('dt','throttle_cap','throttle_slew_per_second','speed_kp','speed_ki','brake','ready_seconds',
        'acceleration_peak','acceleration_sustained','lateral_limit','end_clearance','stopped_seconds','response_timeout'):
        assert c[k]==base[k]
    controller=LaunchController(config_for('trial1'));last=0
    for _ in range(300):
        value=controller.step(0)
        assert 0<=value<=.1 and abs(value-last)<=.005000001
        last=value


def test_collision_is_recorded_and_stops_before_another_tick(tmp_path):
    h=ExploratoryHarness.__new__(ExploratoryHarness)
    h.directory=tmp_path;h.collisions=[];h.collision_lock=threading.Lock()
    h.actor=NS(id=9);h.sensor=NS(is_listening=True)
    h.collision(NS(frame=100,normal_impulse=NS(x=1,y=2,z=3),other_actor=NS(id=77,type_id='static.mesh')))
    with pytest.raises(EarlyStop,match='collision_event'):h.collision_gate()
    rows=[json.loads(x) for x in (tmp_path/'collision_events.jsonl').read_text().splitlines()]
    assert rows[0]['frame']==100 and rows[0]['other_actor_id']==77


def test_missing_collision_listener_fails_closed():
    h=ExploratoryHarness.__new__(ExploratoryHarness)
    h.collisions=[];h.collision_lock=threading.Lock();h.actor=NS(id=1);h.sensor=NS(is_listening=False)
    with pytest.raises(EarlyStop,match='collision_sensor_not_listening'):h.collision_gate()


def test_frame_cap_prevents_extra_tick():
    h=ExploratoryHarness.__new__(ExploratoryHarness)
    h.c=config_for('trial1');h.rows=[{}]*h.c.maximum_frames;h.collision_gate=lambda:None
    with pytest.raises(EarlyStop,match='frame_limit'):h.step('launch',.05,0)


@pytest.mark.parametrize('field,value',[('speed_planar_mps',.06),('vz',.06),('roll',6),('z',1),('a_long_mps2',.6)])
def test_contact_and_stable_pose_check(field,value):
    r=dict(speed_planar_mps=0,vz=0,pitch=0,roll=0,z=0,a_long_mps2=0)
    assert local_pose_ok(r)
    r[field]=value;assert not local_pose_ok(r)


def test_identity_mismatch_is_not_pid_only(monkeypatch,tmp_path):
    witness=dict(pid=12,creation_filetime=123,path='server.exe')
    gate=entry.IdentityGate(12,13,tmp_path/'halt',witness)
    monkeypatch.setattr(entry.HealthGate,'check',lambda self:None)
    monkeypatch.setattr(entry,'native_identity',lambda pid:dict(witness,creation_filetime=124))
    with pytest.raises(entry.ServerUnavailable):gate.check()
    assert gate.poisoned


def test_offline_cli_has_no_native_or_client_access(tmp_path,monkeypatch,capsys):
    path=tmp_path/'config.json';c=config_for('trial1')
    path.write_text(json.dumps(dict(trial='trial1',scenario=asdict(c))))
    monkeypatch.setattr(entry,'native_identity',lambda *_:pytest.fail('offline identity access'))
    assert entry.main(['--config',str(path)])==0
    assert json.loads(capsys.readouterr().out)['route_audit'] is None


def test_execute_requires_new_session_attestation():
    with pytest.raises(ValueError):entry.session_valid({},1,2100)


@pytest.mark.parametrize('overlap,blocking,allowed',[('NONE',False,True),('Poles',False,False),('NONE',True,False),(None,True,False)])
def test_overlap_is_not_misrepresented_as_physical_collision(tmp_path,overlap,blocking,allowed):
    from test_idm_context import Transform,vector
    h=ExploratoryHarness.__new__(ExploratoryHarness);h.directory=tmp_path
    h.call=lambda fn,*args:fn(*args)
    h.api=NS(Location=lambda **kw:NS(**kw),Vector3D=lambda **kw:NS(**kw),CityObjectLabel=NS(Roads='Roads',NONE='NONE'))
    hit=NS(label=overlap,location=vector(0,0,.35))
    h.world=NS(ground_projection=lambda p,d:NS(label='Roads',location=vector(p.x,p.y,0)),
        cast_ray=lambda *args:[hit] if overlap else [],project_point=lambda *args:hit if blocking else None)
    tf=Transform(0);tf.get_right_vector=lambda:vector(0,1,0)
    if allowed:
        h.initial_clearance(tf)
        d=json.loads((tmp_path/'initial_clearance.json').read_text())
        assert d['rays'][0]['untagged_overlap_collision_status']=='UNKNOWN' and d['route_audit'] is None
    else:
        with pytest.raises(EarlyStop):h.initial_clearance(tf)


@pytest.mark.parametrize('fault',[None,'collision','rpc','external_actor'])
def test_mock_physics_lifecycle_and_owned_sensor_cleanup(tmp_path,monkeypatch,fault):
    from test_idm_harness import World,Waypoint,Gate
    from test_idm_context import Transform,vector
    from idm_scene_audit import WEATHER_FIELDS
    import idm_low_speed as module
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    monkeypatch.setattr(module.hashlib,'sha256',lambda _:NS(hexdigest=lambda:
        '97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709'))
    w=World();gate=Gate()
    class WP(Waypoint):
        road_id,section_id,lane_id,lane_width,lane_type=8,0,-1,4.,'Driving'
        def __init__(self,x):self.x=x;self.s=170+x
        @property
        def transform(self):
            t=Transform(self.x);t.get_right_vector=lambda:vector(0,1,0);return t
    w.map=NS(name='Carla/Maps/Town01_Opt',to_opendrive=lambda:'synthetic',
        get_waypoint_xodr=lambda road,lane,s:WP(s-170),get_waypoint=lambda p,**kw:WP(p.x))
    w.ground_projection=lambda p,d:NS(label='Roads',location=vector(p.x,p.y,0))
    w.cast_ray=lambda *args:[]
    w.project_point=lambda *args:None
    w.get_weather=lambda:NS(**{k:0. for k in WEATHER_FIELDS})
    original_spawn=w.try_spawn_actor
    def spawn(bp,tf):
        a=original_spawn(bp,tf);a.acceleration=vector()
        a.control=NS(throttle=0,brake=0,steer=0,hand_brake=False,reverse=False,manual_gear_shift=False,gear=1)
        a.apply_control=lambda ctl:setattr(a,'control',NS(**vars(ctl),gear=1))
        a.get_acceleration=lambda:a.acceleration
        a.get_physics_control=lambda:NS(mass=1800,center_of_mass=vector(),wheels=[
            NS(radius=30,tire_friction=1,max_brake_torque=1000) for _ in range(4)])
        return a
    w.try_spawn_actor=spawn
    class Sensor:
        id=100;is_listening=False
        def listen(self,callback):self.callback=callback;self.is_listening=True
        def stop(self):self.is_listening=False;w.events.append(('sensor_stop',self.id))
        def destroy(self):w.actors.remove(self);w.events.append(('destroy',self.id));return True
    sensor=Sensor()
    def spawn_sensor(*args,**kwargs):w.actors.append(sensor);return sensor
    w.spawn_actor=spawn_sensor
    def tick():
        w.frame+=1;w.time+=.05;w.events.append(('tick',w.frame))
        a=next(x for x in w.actors if getattr(x,'type_id','').startswith('vehicle.'))
        old=a.speed
        if a.control.brake>0:a.speed=max(0,a.speed-.05)
        elif w.frame>40:a.speed=min(.5,a.speed+.05)
        a.acceleration=vector((a.speed-old)/.05,0,0);a.x+=a.speed*.05
        if w.frame==65:
            if fault=='collision':sensor.callback(NS(frame=w.frame,normal_impulse=vector(1,0,0),other_actor=w.external))
            if fault=='rpc':raise RuntimeError('synthetic RPC loss')
            if fault=='external_actor':w.actors.append(NS(id=999,type_id='vehicle.external'))
        return w.frame
    w.tick=tick
    api=NS(Location=lambda **kw:NS(**kw),Vector3D=lambda **kw:NS(**kw),Transform=lambda:NS(),VehicleControl=lambda **kw:NS(**kw),
        CityObjectLabel=NS(Roads='Roads',NONE='NONE'))
    client=NS(get_world=lambda:w,get_client_version=lambda:'0.9.16',get_server_version=lambda:'0.9.16')
    h=ExploratoryHarness(client,api,config_for('trial1'),tmp_path,gate);result=h.run()
    if fault=='rpc':
        assert result['status']=='server_unavailable' and not result['cleanup']['complete']
        assert gate.calls_after_poison==0
    else:
        assert result['status']==('completed' if fault is None else 'safe_early_stop'),result
        assert result['cleanup']['complete']
        destroyed=[x[1] for x in w.events if x[0]=='destroy']
        assert destroyed==[sensor.id,h.actor.id] and w.external in w.actors
        assert vars(w.settings)==vars(w.original)
        if fault=='external_actor':assert any(a.id==999 for a in w.actors)
        if fault is None:
            assert result['distance_m']>=3 and result['maximum_speed_mps']<=.7
            assert [r['frame'] for r in h.rows]==list(range(1,len(h.rows)+1))
            entry.verify_completed(tmp_path,result,h.c)
            with pytest.raises(ValueError):entry.verify_completed(tmp_path,dict(result,motion_samples=1),h.c)
    assert result['route_audit'] is None and result['physical_bounds'] is None
