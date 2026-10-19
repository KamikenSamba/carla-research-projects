"""Server-free lifecycle/contract tests. Kinematics are fixtures, not CARLA physics."""
import copy
import csv
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

from dt_risk_common import actor_state_fieldnames
from Estimate_IDM_T import main as fit_main
from idm_experiment_harness import Harness
from idm_harness_config import Scenario, load_scenario
from idm_harness_host import HealthGate, PortOwnership, ServerUnavailable
from idm_history import read_history
from idm_snapshot import load_traits
from Run_DT_Risk_V2_IDM import parse_args, resolve_parameters
from Run_IDM_Following_Experiment import classify_exit, main, stage_gate
from test_idm_context import Actor as BaseActor, Transform, vector


class Waypoint:
    road_id,section_id,lane_id,lane_width,is_junction = 10,0,1,3.5,False
    def __init__(self, world, x):
        self.world,self.x,self.s=world,x,120-x
    @property
    def transform(self):
        return Transform(self.x)
    def next(self,distance):
        return [Waypoint(self.world,self.x+distance)]
    def get_landmarks(self,*args,**kwargs):
        return self.world.landmarks


class Actor(BaseActor):
    type_id='vehicle.tesla.model3'
    def __init__(self,world,actor_id,x,role):
        super().__init__(actor_id,x,role,speed=0)
        self.world=world
        self.control=NS(throttle=0.3,brake=0.0,hand_brake=False,steer=0.0,reverse=False,
                        gear=1,manual_gear_shift=False)
    def set_autopilot(self,enabled,port):
        self.world.events.append(('autopilot',self.id,enabled,port))
        if self.world.registration_fail and enabled:
            return False
        self.registered=enabled
    def get_control(self):
        return self.control
    def get_acceleration(self):
        return vector()
    def get_traffic_light_state(self):
        return 'Green'
    def destroy(self):
        self.world.events.append(('destroy',self.id))
        self.world.actors.remove(self)
        return True


class World:
    def __init__(self, fault=None, lights=True):
        self.frame=0;self.time=0.0;self.events=[];self.landmarks=[];self.fault=fault
        self.settings=NS(synchronous_mode=False,fixed_delta_seconds=None,substepping=True,
                         max_substep_delta_time=0.01,max_substeps=10,no_rendering_mode=False)
        self.original=copy.copy(self.settings)
        self.external=NS(id=1,type_id='traffic.traffic_light' if lights else 'spectator')
        self.actors=[self.external];self.registration_fail=False;self.actor_count=10
        self.map=NS(name='Carla/Maps/Town10HD_Opt',
            get_waypoint_xodr=lambda road,lane,s:Waypoint(self,120-s),get_waypoint=self.waypoint)
        self.tm=TM(self)
    def waypoint(self,loc,**kwargs):
        return None if abs(loc.y)>2 else Waypoint(self,loc.x)
    def get_map(self):return self.map
    def get_level_bbs(self,label):return []
    def get_actors(self):return list(self.actors)
    def get_settings(self):return copy.copy(self.settings)
    def apply_settings(self,settings):
        self.settings=copy.copy(settings);self.events.append(('settings',settings.synchronous_mode))
    def get_blueprint_library(self):
        class Blueprint:
            def set_attribute(bp,key,value):bp.role=value
        return NS(find=lambda name:Blueprint())
    def try_spawn_actor(self,bp,tf):
        self.actor_count+=1
        actor=Actor(self,self.actor_count,tf.location.x,bp.role)
        self.actors.append(actor);self.events.append(('spawn',actor.id))
        return actor
    def get_snapshot(self):
        return NS(frame=self.frame,timestamp=NS(elapsed_seconds=self.time))
    def tick(self):
        self.events.append(('tick',self.frame+1));self.frame+=1;self.time+=0.05
        self.tm.cycles+=1
        for a in self.actors[1:]:
            if getattr(a,'registered',False):
                a.speed=self.tm.leader_speed
                a.x+=a.speed*0.05
        if self.fault and self.frame==75:
            f=self.actors[1]
            if self.fault=='stopped': self.tm.leader_speed=0;self.fault_active='stopped'
            elif self.fault=='offlane':f.y=5
            elif self.fault=='heading':f.yaw=8
            elif self.fault=='contact':self.actors[2].x=f.x+4.5
            elif self.fault=='disappear':self.actors.remove(f)
            elif self.fault=='handbrake':f.control.hand_brake=True
            elif self.fault=='endpoint':self.actors[2].x=52
            elif self.fault=='overspeed':self.actors[2].speed=3
            elif self.fault=='external_actor':self.actors.append(NS(id=999,type_id='vehicle.external'))
            elif self.fault=='time':self.time+=0.1
            elif self.fault=='interleave':self.frame+=1
            elif self.fault=='server':raise RuntimeError('server RPC lost')
        if getattr(self,'fault_active',None)=='stopped':
            for a in self.actors[1:]:a.speed=0
        return self.frame


class TM:
    def __init__(self,world):self.w=world;self.leader_speed=1.4;self.cycles=0;self.sync=False
    def set_synchronous_mode(self,value):
        self.w.events.append(('tm_sync',value));self.sync=value
    def set_random_device_seed(self,value):self.w.events.append(('seed',value))
    def set_hybrid_physics_mode(self,value):pass
    def set_respawn_dormant_vehicles(self,value):pass
    def auto_lane_change(self,*args):pass
    def distance_to_leading_vehicle(self,*args):pass
    def set_path(self,actor,path):self.w.events.append(('path',actor.id,len(path)))
    def set_desired_speed(self,actor,speed):
        self.w.events.append(('speed',actor.id,speed))
        if actor.attributes['role_name'].endswith('leader'):self.leader_speed=speed/3.6
    def get_next_action(self,actor):return ('LaneFollow',None)


class Ownership:
    def __init__(self,world,remote=False):self.world=world;self.remote=remote
    def require_unused(self):self.world.events.append(('port_unused',))
    def require_self(self):
        self.world.events.append(('port_self',))
        if self.remote:raise ValueError('Remote TM owner')


class Gate:
    def __init__(self):self.poisoned=False;self.calls_after_poison=0
    def check(self):
        if self.poisoned:raise ServerUnavailable('poisoned')
    def call(self,fn,*args,**kw):
        if self.poisoned:self.calls_after_poison+=1
        self.check()
        try:return fn(*args,**kw)
        except RuntimeError as exc:
            self.poisoned=True;raise ServerUnavailable(str(exc)) from exc


def run(tmp_path,stage='stage1',fault=None,lights=True,remote=False,setup=None):
    w=World(fault,lights);gate=Gate()
    if setup:setup(w)
    client=NS(get_world=lambda:w,get_client_version=lambda:'0.9.16',get_server_version=lambda:'0.9.16',
              get_trafficmanager=lambda port:w.tm)
    api=NS(CityObjectLabel=NS(**{k:k for k in ('Car','Truck','Bus','Motorcycle','Bicycle','Pedestrians')}))
    h=Harness(client,api,Scenario(),stage,tmp_path,gate,Ownership(w,remote))
    return h.run(),w,gate,h


def csv_rows(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))


@pytest.mark.parametrize('stage',['stage1','stage2'])
def test_full_offline_e2e_same_frames_single_tick_cleanup_and_traits(tmp_path,stage):
    result,w,gate,h=run(tmp_path,stage)
    assert result['status']=='completed',result
    capture=csv_rows(tmp_path/'capture_states.csv');ctx=csv_rows(tmp_path/'following_context.csv')
    audit=csv_rows(tmp_path/'control_audit.csv')
    key=lambda rows:[(r['logical_actor_id'],r['carla_frame']) for r in rows]
    assert len(capture)==len(ctx)==len(audit)==600
    assert key(capture)==key(ctx)==key(audit)
    assert list(capture[0])==actor_state_fieldnames()
    assert all(r['control_read_at_frame']==r['carla_frame'] for r in audit)
    assert all(int(r['command_based_on_frame'])+1==int(r['carla_frame']) for r in audit)
    assert all(r['seed_applied']=='False' and r['autopilot_registered_verified']=='' for r in audit)
    assert len(read_history(tmp_path/'capture_states.csv',tmp_path/'following_context.csv'))==600
    assert w.frame==20+20+300  # settle + 1s readiness + capture; no observer/cleanup ticks
    assert w.tm.cycles==w.frame
    assert asdict_settings(w.settings)==asdict_settings(w.original)
    assert w.actors==[w.external] and result['cleanup']['complete']
    assert not any(e[0]=='seed' for e in w.events)
    assert w.events.index(('port_self',))<w.events.index(('tm_sync',True))
    for actor in (11,12):
        assert w.events.index(('autopilot',actor,False,8000))<w.events.index(('destroy',actor))
        assert w.events.index(('autopilot',actor,True,8000))<next(i for i,e in enumerate(w.events) if e[0]=='path' and e[1]==actor)
    assert not gate.poisoned
    snapshots=csv_rows(tmp_path/'snapshot_states.csv')
    assert {r['phase'] for r in snapshots}=={'snapshot'}
    assert all(float(r['sim_time_s'])==10 for r in snapshots)
    output=tmp_path/'fit'
    assert fit_main(['--history',str(tmp_path/'capture_states.csv'),'--context',str(tmp_path/'following_context.csv'),
        '--snapshot',str(tmp_path/'snapshot_states.csv'),'--idm-desired-speed-mps','2.2',
        '--replace-snapshot-headway','--output-dir',str(output)])==0
    traits=load_traits(output/'actor_traits.json')
    args=parse_args(['--mode','predict','--snapshot',str(tmp_path/'snapshot_states.csv')])
    for row in snapshots:
        resolved=resolve_parameters(row,args,traits)
        assert resolved.desired_speed_mps==2.2
        assert 0.3<=resolved.desired_time_headway_s<=4
    diag=json.loads((output/'headway_diagnostics.json').read_text())
    assert all(d['future_samples_excluded']==100 for d in diag['actors'].values())
    assert not diag['actors']['role:idm_harness:leader']['adopted']


def asdict_settings(settings):return vars(settings)


@pytest.mark.parametrize('fault,reason',[
    ('stopped','follower_stopped'),('offlane','off_driving_lane'),('heading','route_deviation_follower'),
    ('contact','contact_gap'),('disappear','actor_disappeared'),('handbrake','hand_brake_or_reverse'),
    ('endpoint','route_endpoint_margin'),('overspeed','speed_envelope'),('external_actor','external_actor_appeared')])
def test_early_stops_preserve_reason_and_owned_cleanup(tmp_path,fault,reason):
    result,w,gate,h=run(tmp_path,fault=fault)
    assert result['status']=='safe_early_stop',result
    assert result['reason']==reason
    assert result['final_frame']>=75 and result['sim_time_s']>0
    assert result['cleanup']['complete']
    assert w.external in w.actors and not gate.poisoned
    assert json.loads((tmp_path/'following_context.meta.json').read_text())['status']=='failed'
    with pytest.raises(ValueError,match='incomplete or failed'):
        read_history(tmp_path/'capture_states.csv',tmp_path/'following_context.csv')
    if fault=='handbrake':
        assert json.loads((tmp_path/'last_observation.json').read_text())['rows'][0]['control']['hand_brake'] is True


@pytest.mark.parametrize('fault',['interleave','time'])
def test_interleaving_is_not_clean_success_and_no_further_ticks(tmp_path,fault):
    result,w,gate,h=run(tmp_path,fault=fault)
    assert result['reason'].startswith('unexpected_')
    assert len([e for e in w.events if e[0]=='tick'])==75
    if fault=='interleave':
        assert result['status']=='cleanup_incomplete'
        assert not any(e[0]=='destroy' for e in w.events)


def test_rpc_failure_suppresses_every_cleanup_rpc(tmp_path):
    result,w,gate,h=run(tmp_path,fault='server')
    assert result['status']=='server_unavailable'
    assert gate.poisoned and gate.calls_after_poison==0
    assert not result['cleanup']['attempted']
    assert not any(e[0]=='destroy' for e in w.events)
    assert w.settings.synchronous_mode


def test_remote_owner_rejected_before_settings_seed_or_spawn(tmp_path):
    result,w,gate,h=run(tmp_path,remote=True)
    assert result['status']=='exception'
    assert not any(e[0] in ('settings','tm_sync','spawn','seed') for e in w.events)
    assert w.actors==[w.external]


def test_seed_only_without_external_signal_actors(tmp_path):
    result,w,gate,h=run(tmp_path,lights=False)
    assert result['status']=='completed'
    assert w.events.count(('seed',42))==1
    ledger=[json.loads(line) for line in (tmp_path/'command_ledger.jsonl').read_text().splitlines()]
    assert any(r['operation']=='leader_speed_kmh' and r.get('arguments',[None,0])[1]==pytest.approx(5.04)
               for r in ledger if r.get('outcome')=='requested')


def test_autopilot_explicit_failed_return_cleans_only_owned(tmp_path):
    result,w,gate,h=run(tmp_path,setup=lambda w:setattr(w,'registration_fail',True))
    assert result['status']=='safe_early_stop'
    assert result['reason']=='autopilot_register_failed'
    assert w.actors==[w.external]


def test_existing_vehicle_rejected_before_tm(tmp_path):
    result,w,gate,h=run(tmp_path,setup=lambda w:w.actors.append(NS(id=2,type_id='vehicle.external')))
    assert result['reason']=='external_dynamic_actors_present'
    assert result['cleanup']['complete'] and len(w.actors)==2
    assert not w.events


def test_tm_sync_failure_restores_settings_before_spawn(tmp_path):
    def setup(w):w.tm.set_synchronous_mode=lambda value:False if value else None
    result,w,gate,h=run(tmp_path,setup=setup)
    assert result['reason']=='tm_sync_failed' and result['cleanup']['complete']
    assert w.actors==[w.external]


def test_port_owner_race_is_not_remote_success():
    guard=PortOwnership(12345,inspect=lambda p:{os.getpid()+1})
    with pytest.raises(ValueError,match='already has'):guard.require_unused()
    with pytest.raises(ValueError,match='exclusively'):guard.require_self()


def test_health_gate_never_retries_after_rpc_exception(tmp_path,monkeypatch):
    import idm_harness_host as host
    monkeypatch.setattr(host,'process_alive',lambda pid:True)
    gate=HealthGate(1,2,tmp_path/'halt')
    calls=[]
    def failing():calls.append('RPC');raise RuntimeError('native server unavailable')
    with pytest.raises(ServerUnavailable):gate.call(failing)
    with pytest.raises(ServerUnavailable):gate.call(lambda:calls.append('extra RPC'))
    assert calls==['RPC']


def test_context_mismatch_fails_closed(tmp_path,monkeypatch):
    import idm_experiment_harness as harness
    original=harness.FollowingContextCollector
    class Corrupt(original):
        def __call__(self,*args):
            rows=super().__call__(*args)
            rows[0]['carla_frame']+=1
            return rows
    monkeypatch.setattr(harness,'FollowingContextCollector',Corrupt)
    result,w,gate,h=run(tmp_path)
    assert result['reason']=='capture_context_mismatch'
    assert result['status']=='safe_early_stop' and result['cleanup']['complete']


@pytest.mark.parametrize('code,status',[(0,'completed'),(2,'safe_early_stop'),(1,'exception'),
    (4,'server_unavailable'),(5,'cleanup_incomplete'),(-6,'native_abnormal_exit'),(3221225477,'native_abnormal_exit')])
def test_supervisor_exit_categories(code,status):
    result=dict(exit_code=code,status=status)
    assert classify_exit(code,result)[0]==status
    assert classify_exit(0,None)==('abnormal_exit',1)


def test_stage2_requires_successful_compatible_stage1(tmp_path):
    c=Scenario();p=tmp_path/'result.json'
    with pytest.raises(ValueError):stage_gate('stage2',c,None)
    for status in ('safe_early_stop','native_abnormal_exit','cleanup_incomplete'):
        p.write_text(json.dumps(dict(status=status)))
        with pytest.raises(ValueError):stage_gate('stage2',c,p)
    payload=dict(status='completed',stage='stage1',config_sha256=c.fingerprint,cleanup=dict(complete=True),
                 supervised=True,worker_exit_code=0)
    p.write_text(json.dumps(payload));stage_gate('stage2',c,p)
    payload['config_sha256']='different';p.write_text(json.dumps(payload))
    with pytest.raises(ValueError):stage_gate('stage2',c,p)


@pytest.mark.parametrize('change',[dict(dt=0.1),dict(leader_speed_limit=3),dict(follower_s=121),
    dict(tm_port=True),dict(seed=-1),dict(profile=((0,0.5),(15,1.4))),dict(warmup_seconds=-1)])
def test_invalid_or_insufficient_route_scenario_rejected(change):
    with pytest.raises(ValueError):replace(Scenario(),**change)


def test_profile_integral_and_braking_headroom():
    c=Scenario()
    distance=sum((b[0]-a[0])*(a[1]+b[1])/2 for a,b in zip(c.profile,c.profile[1:]))
    assert distance==pytest.approx(23.3)
    assert c.clearance_budget==pytest.approx(7.07489)
    assert max(c.speed_at(t/20,'stage1') for t in range(301))==1.4
    assert c.speed_at(3,'stage2')==pytest.approx(1.6)
    # At 2.5 m/s, 0.5s reaction + 2m/s² assumed deceleration gives <6m margin.
    assert 2.5*0.5+2.5**2/(2*2)<c.end_clearance


def test_validation_cli_cannot_import_carla_or_connect(tmp_path):
    script=Path(__file__).resolve().parents[1]/'Run_IDM_Following_Experiment.py'
    code="""import builtins,runpy,sys
original=builtins.__import__
def guarded(name,*a,**kw):
    if name=='carla': raise AssertionError('CARLA import forbidden')
    return original(name,*a,**kw)
builtins.__import__=guarded
sys.path.insert(0,str(__import__('pathlib').Path(sys.argv[1]).parent))
sys.argv=[sys.argv[1]]
runpy.run_path(sys.argv[0],run_name='__main__')
"""
    p=subprocess.run([sys.executable,'-c',code,str(script)],capture_output=True,text=True)
    assert p.returncode==0,p.stderr
    assert json.loads(p.stdout)['status']=='validated_offline_no_connection'


def test_static_road_user_box_rejected_before_tm(tmp_path):
    def setup(w):
        w.get_level_bbs=lambda label:[NS(location=vector(20,0,0),extent=vector(2,1,1),rotation=NS(yaw=0))]
    result,w,gate,h=run(tmp_path,setup=setup)
    assert result['reason']=='static_road_user_obstacle'
    assert w.actors==[w.external] and not w.events


def test_failed_destroy_does_not_claim_success(tmp_path):
    def setup(w):
        original=w.try_spawn_actor
        def spawn(*args):
            a=original(*args);a.destroy=lambda:False;return a
        w.try_spawn_actor=spawn
    result,w,gate,h=run(tmp_path,setup=setup)
    assert result['status']=='cleanup_incomplete'
    assert result['cleanup']['remaining_owned']==[11,12]
    assert not result['cleanup']['complete']
    assert result['cleanup']['settings_restored']


def test_missing_control_is_unknown_and_stops(tmp_path):
    def setup(w):
        original=w.try_spawn_actor
        def spawn(*args):
            a=original(*args);del a.control.hand_brake;return a
        w.try_spawn_actor=spawn
    result,w,gate,h=run(tmp_path,setup=setup)
    assert result['status']=='safe_early_stop' and result['reason']=='control_unknown'
    assert json.loads((tmp_path/'last_observation.json').read_text())['rows'][0]['control']['hand_brake'] is None


def test_server_failure_inside_context_does_not_escape_health_gate(tmp_path,monkeypatch):
    import idm_experiment_harness as module
    original=module.FollowingContextCollector
    class Failure(original):
        def __call__(self,world,*args):
            # All world access exposed to the collector must go through the health gate.
            underlying=harness_box[0].world
            underlying.get_snapshot=lambda:(_ for _ in ()).throw(RuntimeError('server lost during observer'))
            return super().__call__(world,*args)
    harness_box=[]
    base=module.Harness.capture
    def capture(h):harness_box.append(h);return base(h)
    monkeypatch.setattr(module.Harness,'capture',capture)
    monkeypatch.setattr(module,'FollowingContextCollector',Failure)
    result,w,gate,h=run(tmp_path)
    assert result['status']=='server_unavailable'
    assert gate.poisoned and gate.calls_after_poison==0
    assert not result['cleanup']['attempted']


@pytest.mark.parametrize('exit_code,expected',[(0,'completed'),(-6,'native_abnormal_exit'),(1,'abnormal_exit')])
def test_supervisor_records_native_process_exit_without_carla(tmp_path,monkeypatch,exit_code,expected):
    import Run_IDM_Following_Experiment as entry
    monkeypatch.setattr(entry,'process_alive',lambda pid:True)
    monkeypatch.setattr(entry,'listener_pids',lambda port:{100})
    monkeypatch.setattr(entry,'source_metadata',lambda:dict(source_commit='fixture'))
    monkeypatch.setattr(entry.tempfile,'gettempdir',lambda:str(tmp_path))
    def popen(command,**kwargs):
        directory=Path(command[command.index('--run-dir')+1])
        if exit_code==0:
            (directory/'worker_result.json').write_text(json.dumps(dict(
                status='completed',exit_code=0,cleanup=dict(complete=True))))
        return NS(pid=200,returncode=exit_code,poll=lambda:exit_code,wait=lambda:exit_code)
    monkeypatch.setattr(entry.subprocess,'Popen',popen)
    c=Scenario();config=tmp_path/'scenario.json';config.write_text(json.dumps(asdict(c)))
    args=NS(stage='stage1',stage1_result=None,server_pid=100,port=2000,output_root=tmp_path/'outputs',
            config=config,wall_timeout=120)
    code=entry.supervise(args,c)
    directories=list((tmp_path/'outputs').iterdir());assert len(directories)==1
    result=json.loads((directories[0]/'result.json').read_text())
    assert result['status']==expected and result['worker_exit_code']==exit_code
    assert code==result['exit_code']
    assert not (tmp_path/'carla-idm-harness-2000.lock').exists()
    assert (directories[0]/'worker_stderr.txt').exists()


def test_health_marker_stops_before_any_rpc(tmp_path,monkeypatch):
    import idm_harness_host as module
    monkeypatch.setattr(module,'process_alive',lambda pid:True)
    marker=tmp_path/'halt';marker.touch();gate=HealthGate(1,2,marker)
    calls=[]
    with pytest.raises(ServerUnavailable):gate.call(lambda:calls.append('RPC'))
    assert not calls and gate.poisoned


def test_run_directory_is_never_overwritten(tmp_path):
    marker=tmp_path/'config_used.json';marker.write_text('existing evidence')
    with pytest.raises(FileExistsError):run(tmp_path)
    assert marker.read_text()=='existing evidence'


def test_partial_spawn_failure_restores_and_deletes_first_owned_actor(tmp_path):
    def setup(w):
        original=w.try_spawn_actor
        def spawn(bp,tf):
            return None if len(w.actors)>1 else original(bp,tf)
        w.try_spawn_actor=spawn
    result,w,gate,h=run(tmp_path,setup=setup)
    assert result['reason']=='spawn_failed' and result['cleanup']['complete']
    assert w.actors==[w.external]
