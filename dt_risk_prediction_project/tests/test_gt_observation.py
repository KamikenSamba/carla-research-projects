"""Synthetic World/Actors drive the existing bounded V2, never a CARLA server."""
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import random
from types import SimpleNamespace as NS

import pytest

from dt_gt_observation import GTObserver, encoded, put, digest, options
from idm_carla_validation import V2FleetHarness, V2SingleHarness, ValidationConfig
from idm_brake_harness import physics_record
from idm_reverse_motion import ReverseMotion
from idm_experiment_harness import save, settings_dict
from test_idm import FakeActor
from test_idm_harness import Gate


def provenance(role, parent=None):
    value=dict(enabled=True,role=role,evidence_kind='synthetic',run_id='mock-'+role,
        protocol='synthetic_existing_bounded_V2',code_commit='a'*40,source_dirty=False,
        seed=dict(value=42,status='applied',components=['mock_random_generator'],evidence='random.Random(42)'),
        inputs=[],inventory_complete=True,parameter_policy='reconstruction_known',split='test',
        model_data=dict(training_source_ids=[],tuning_source_ids=[],evaluation_source_ids=['mock-source'],approach='prescribed'))
    if parent is not None:
        value['parent']={k:v for k,v in parent.items() if k!='_inputs'}
        value['inputs']=parent['_inputs']
    return value


def mock_physics():
    wheel=NS(tire_friction=3.,radius=35.,max_brake_torque=1500.,max_handbrake_torque=3000.,
        damping_rate=.25,max_steer_angle=70.,lat_stiff_max_load=2.,lat_stiff_value=17.,long_stiff_value=1000.,
        position=NS(x=0.,y=0.,z=0.))
    return NS(mass=1845.,drag_coefficient=.3,moi=1.,max_rpm=5000.,use_gear_autobox=True,
        gear_switch_time=.1,clutch_strength=10.,final_ratio=4.,damping_rate_full_throttle=.1,
        damping_rate_zero_throttle_clutch_engaged=.1,damping_rate_zero_throttle_clutch_disengaged=.1,
        center_of_mass=NS(x=0.,y=0.,z=0.),use_sweep_wheel_collision=False,wheels=[copy.deepcopy(wheel) for _ in range(4)],
        torque_curve=[NS(x=0.,y=100.)],steering_curve=[NS(x=0.,y=1.)],
        forward_gears=[NS(ratio=3.,down_ratio=.2,up_ratio=.8)])


def collect(directory, monkeypatch, enabled=True, role='source', parent=None, fault=None, single=False):
    """Actual V2 capture/observe/ledger/tick/gates/stop/cleanup, fake preparation/physics."""
    import idm_controller
    directory=Path(directory);directory.mkdir()
    monkeypatch.setattr('idm_carla_validation.time.sleep',lambda *_:None)
    monkeypatch.setattr('idm_pair_exploration.time.sleep',lambda *_:None)
    monkeypatch.setattr(idm_controller,'import_carla',lambda:NS(VehicleControl=lambda **kw:NS(**kw)))
    random.Random(42).random()  # synthetic seed application, explicitly not engine seeding
    calls=[];commands=[];base=100 if role=='source' else 1000
    initial_time=1000. if role=='source' else 2000.
    world=NS(frame=base,time=initial_time,settings=NS(synchronous_mode=True,fixed_delta_seconds=.05))
    external=NS(id=9000,type_id='static.prop.mesh');live=[external]
    class Waypoint:
        road_id=8;section_id=0;lane_id=-1;lane_width=4.;is_junction=False
        def __init__(self,x):self.transform=NS(location=NS(x=x,y=0.,z=0.),rotation=NS(yaw=0.,pitch=0.,roll=0.))
        def next(self,d):return [Waypoint(self.transform.location.x+d)]
    actors={}
    for i,(logical,x) in enumerate((('solo',0.),) if single else (('follower',0.),('leader',20.))):
        actor=FakeActor(base+i+1,x,0.);actor.type_id='vehicle.tesla.model3';actor.attributes={'role_name':logical}
        actor.bounding_box=NS(location=NS(x=0.,y=0.,z=0.),extent=NS(x=2.,y=1.,z=1.),rotation=NS(yaw=0.,pitch=0.,roll=0.))
        actor.control=NS(throttle=0.,brake=0.,steer=0.,hand_brake=False,reverse=False,manual_gear_shift=False,gear=1)
        actor.acceleration=0.
        actor.get_location=lambda a=actor:NS(x=a.x,y=0.,z=0.)
        actor.get_control=lambda a=actor:a.control
        actor.get_acceleration=lambda a=actor:NS(x=a.acceleration,y=0.,z=0.)
        actor.is_at_traffic_light=lambda:False
        def apply(command,a=actor):
            commands.append((world.frame,a.id,dict(vars(command))));a.control=NS(gear=1,**vars(command))
        actor.apply_control=apply
        actor.destroy=lambda a=actor:live.remove(a) or True
        actors[logical]=actor;live.append(actor)
    sensors={}
    for i,logical in enumerate(actors):
        sensor=NS(id=base+10+i,type_id='sensor.other.collision',is_listening=True)
        sensor.stop=lambda s=sensor:setattr(s,'is_listening',False)
        sensor.destroy=lambda s=sensor:live.remove(s) or True
        sensors[logical]=sensor;live.append(sensor)
    world.get_actors=lambda:calls.append('actors') or list(live)
    world.get_settings=lambda:calls.append('settings') or copy.copy(world.settings)
    world.apply_settings=lambda s:setattr(world,'settings',copy.copy(s))
    world.get_map=lambda:world.map
    world.map=NS(get_waypoint=lambda loc,**kw:Waypoint(loc.x))
    def snapshot():
        calls.append('snapshot')
        return NS(frame=world.frame,timestamp=NS(elapsed_seconds=world.time,delta_seconds=.05),
            find=lambda actor_id:next((a for a in actors.values() if a.id==actor_id),None))
    world.get_snapshot=snapshot
    def tick():
        calls.append('tick');world.frame+=1;world.time+=.05
        for actor in actors.values():
            old=actor.speed;target=max(0.,3*actor.control.throttle-actor.control.brake)
            actor.speed=max(0.,old+max(-.025,min(.025,target-old)))
            actor.acceleration=(actor.speed-old)/.05;actor.x+=actor.speed*.05
        return world.frame
    world.tick=tick
    if single:
        h=V2SingleHarness(None,NS(VehicleControl=lambda **kw:NS(**kw)),ValidationConfig(),directory,Gate())
    else:
        h=V2FleetHarness(None,NS(VehicleControl=lambda **kw:NS(**kw)),ValidationConfig(trial='fleet'),directory,Gate(),single_stop_m=.5)
    if enabled:h.gt_observer=GTObserver(directory,provenance(role,parent))
    old=NS(synchronous_mode=False,fixed_delta_seconds=None)
    def prepare():
        h.world=world;h.map=world.map;h.last_frame=world.frame;h.last_time=world.time
        h.path=[(0.,0.,0.),(60.,0.,0.)];h.route_length=60.;h.initial_ids={external.id}
        h.original=old;h.settings_touched=True;h.expected_settings=settings_dict(world.settings)
        h.owned={**{k+'_sensor':s for k,s in sensors.items()},**({'vehicle':actors['solo']} if single else actors)}
        if single:
            h.actor=actors['solo'];h.sensor=sensors['solo'];h.reverse_monitor=ReverseMotion((1.,0.,0.),.05)
        else:
            for logical,actor in actors.items():
                h.slots[logical]=dict(actor=actor,sensor=sensors[logical],previous_velocity=None,accel_count=0,
                    reverse_monitor=ReverseMotion((1.,0.,0.),.05),controller=None)
            h.select('follower')
        save(directory/'environment.json',dict(versions=['0.9.16','0.9.16'],map='SYNTHETIC_MAP',
            applied_settings=settings_dict(world.settings),weather=dict(cloudiness=0.)))
        save(directory/'local_route.json',dict(points=h.path,route_audit=None))
        if enabled:h.gt_observer.opendrive('<OpenDRIVE synthetic="true"/>',[external],world.frame,world.time)
        for logical,actor in actors.items():
            name='vehicle_physics.json' if logical in ('follower','solo') else 'leader_physics.json';ph=mock_physics()
            save(directory/name,physics_record(ph))
            if enabled:
                h.gt_observer.physics(actor.id,ph,name,world.frame,world.time)
                h.gt_observer.sensor_started(sensors[logical].id,actor.id,world.frame)
        if enabled:h.gt_observer.environment(world.frame,world.time)
    h.prepare=prepare
    capture=h.capture
    def capture_and_fence():
        capture()
        if enabled:h.gt_observer.confirm_synthetic_delivery()
    h.capture=capture_and_fence
    if fault:fault(h)
    result=h.run()
    return h,result,calls,commands,world,live


def test_real_v2_mock_on_off_command_pid_actuator_tick_equivalence(tmp_path,monkeypatch):
    off=collect(tmp_path/'off',monkeypatch,False);on=collect(tmp_path/'on',monkeypatch)
    for run in (off,on):
        h,result,_,_,world,live=run
        assert result['status']=='completed',result
        assert result['cleanup']['complete'] and len(live)==1 and live[0].id==9000
        assert vars(world.settings)==dict(synchronous_mode=False,fixed_delta_seconds=None)
        assert result['route_audit'] is None and result['physical_bounds'] is None
    assert off[2]==on[2]  # includes every mock API call, not only ticks
    assert off[3]==on[3]
    for logical in on[0].fleet.controllers:
        a,b=off[0].fleet.controllers[logical],on[0].fleet.controllers[logical]
        assert vars(a.speed_pid)==vars(b.speed_pid)
        assert vars(a.steering_pid)==vars(b.steering_pid)
        assert vars(off[0].actuators[logical])==vars(on[0].actuators[logical])
    origin=json.loads((tmp_path/'on/gt_origin.json').read_text(encoding='utf-8'))
    assert origin['frame']==140 and origin['simulation_time_s']>1000
    internal=json.loads((tmp_path/'on/gt_internal_state.json').read_text(encoding='utf-8'))
    assert internal['frame']==origin['frame'] and internal['simulation_time_s']==origin['simulation_time_s']
    physics=json.loads((tmp_path/'on/gt_physics_extras.json').read_text(encoding='utf-8'))
    assert all(p['acquisition_frame']==100 and p['acquisition_time_s']==1000. for p in physics.values())
    view=json.loads((tmp_path/'on/gt_input_view/snapshot.json').read_text(encoding='utf-8'))
    assert all(r['desired_time_headway_s'] is None and r['desired_speed_mps'] is None for r in view['actors'])
    assert json.loads((tmp_path/'on/gt_capture_coverage.json').read_text(encoding='utf-8'))['capture_status']=='complete'


def test_unknown_physics_and_real_delivery_cannot_be_certified(tmp_path):
    obs=GTObserver(tmp_path,dict(provenance('source'),evidence_kind='recorded_carla'))
    ph=mock_physics();del ph.wheels[0].long_stiff_value
    obs.physics(1,ph,'vehicle_physics.json')
    assert obs.physics_records['1']['unknown_static']==['wheels/0/long_stiff_value']
    with pytest.raises(ValueError,match='fence unavailable'):obs.confirm_synthetic_delivery()
    obs.finish(dict(status='completed',cleanup=dict(complete=True)),20)
    assert json.loads((tmp_path/'gt_capture_coverage.json').read_text(encoding='utf-8'))['capture_status']=='unknown'


def test_event_metadata_missing_time_and_late_callback_are_not_zero_filled(tmp_path):
    obs=GTObserver(tmp_path,provenance('source'))
    record=obs.collision_metadata(NS(),1)
    assert record['timestamp'] is None
    obs.confirm_synthetic_delivery();obs.collision_metadata(NS(timestamp=1.),1)
    assert 'callback_after_delivery_fence' in obs.errors
    with pytest.raises(OSError):obs.check()


def test_options_require_explicit_metadata_and_no_carla_import():
    assert options({}) is None
    with pytest.raises(ValueError):options(dict(ground_truth_observation={}))
    assert options(dict(ground_truth_observation=provenance('source')))['role']=='source'


@pytest.mark.parametrize('frame,stamp',[(1.5,1.),(1,float('nan')),(True,1.)])
def test_invalid_world_clock_is_not_filled(tmp_path,frame,stamp):
    obs=GTObserver(tmp_path,provenance('source'))
    with pytest.raises(ValueError):obs.clock(NS(frame=frame,timestamp=NS(elapsed_seconds=stamp)))


def test_duplicate_clock_and_existing_receipt_rejected(tmp_path):
    obs=GTObserver(tmp_path,provenance('source'));snapshot=NS(frame=1,timestamp=NS(elapsed_seconds=1.))
    obs.clock(snapshot)
    with pytest.raises(ValueError):obs.clock(snapshot)
    before=(tmp_path/'gt_run_provenance.json').read_bytes()
    with pytest.raises(FileExistsError):GTObserver(tmp_path,provenance('source'))
    assert (tmp_path/'gt_run_provenance.json').read_bytes()==before


@pytest.mark.parametrize('pair',[False,True])
def test_callback_io_error_retains_existing_collision_abort_flag(tmp_path,monkeypatch,pair):
    import threading
    from idm_low_speed import ExploratoryHarness
    from idm_pair_exploration import PairHarness
    from idm_experiment_harness import EarlyStop
    obs=GTObserver(tmp_path,provenance('source'))
    h=NS(directory=tmp_path,gt_observer=obs,actor=NS(id=1),collisions=[],collision_lock=threading.Lock())
    event=NS(frame=10,timestamp=5.,other_actor=NS(id=2,type_id='vehicle.test'),normal_impulse=NS(x=1.,y=0.,z=0.))
    original=Path.open
    def broken(path,*args,**kwargs):
        if path.name=='collision_events.jsonl':raise OSError('synthetic callback I/O')
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'open',broken)
    with pytest.raises(OSError):
        if pair:PairHarness.record_collision(h,event,1)
        else:ExploratoryHarness.collision(h,event)
    assert len(h.collisions)==1 and h.collisions[0]['timestamp']==5.
    assert obs.errors and obs.event_count==1


def test_freeze_error_keeps_partial_evidence_and_no_input_view_manifest(tmp_path,monkeypatch):
    def fault(h):
        original=h.gt_observer.freeze
        def freeze(*args):
            # Append a future frame to the original raw before attempting the freeze.
            path=h.directory/'calibration_raw.jsonl'
            raw=json.loads(path.read_text(encoding='utf-8').splitlines()[-1])
            raw['carla_frame']=h.last_frame+1
            with path.open('a',encoding='utf-8') as stream:stream.write(json.dumps(raw)+'\n')
            return original(*args)
        h.gt_observer.freeze=freeze
    _,result,_,commands,_,live=collect(tmp_path/'run',monkeypatch,single=True,fault=fault)
    assert result['status']=='exception' and result['cleanup']['complete']
    assert 'future raw' in result['reason'] and commands[-1][2]['brake']==.70
    assert len(live)==1 and not (tmp_path/'run/gt_origin.json').exists()


def test_original_collision_payload_and_event_clock_saved_without_rpc(tmp_path):
    import threading
    from idm_pair_exploration import PairHarness
    obs=GTObserver(tmp_path,provenance('source'))
    h=NS(directory=tmp_path,gt_observer=obs,collisions=[],collision_lock=threading.Lock())
    event=NS(frame=42,timestamp=123.5,other_actor=NS(id=2,type_id='vehicle.test'),normal_impulse=NS(x=1.,y=2.,z=3.))
    PairHarness.record_collision(h,event,1)
    row=json.loads((tmp_path/'collision_events.jsonl').read_text(encoding='utf-8'))
    assert row['frame']==42 and row['timestamp']==123.5
    assert row['actor_id']==1 and row['other_actor_id']==2
    assert row['normal_impulse']==dict(x=1.,y=2.,z=3.) and row['gt_event_id']=='callback-1'
    assert len(h.collisions)==1 and not obs.errors
