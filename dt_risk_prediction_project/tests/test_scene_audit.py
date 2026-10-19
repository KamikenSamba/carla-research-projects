"""SYNTHETIC contract tests only: no simulator/physics/real scene certification."""
import ast
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

import idm_scene_reference as reference
import idm_brake_config as policy
from idm_brake_route import ALL_LABELS,geometry_sha256
from idm_scene_audit import ALLOWLIST,AuditAbort,Evidence,Reader,SceneAudit,FILES
import idm_scene_os as host
import Run_Audit_Town01_Scene as cli

XML='<OpenDRIVE><road id="8"><signals/><objects/></road></OpenDRIVE>'
FORBIDDEN={'tick','wait_for_tick','apply_settings','load_world','reload_world','get_trafficmanager',
           'spawn_actor','try_spawn_actor','destroy','apply_control','set_transform',
           'set_target_velocity','set_autopilot','apply_batch_sync','apply_batch',
           'set_weather','get_spectator','enable_environment_objects'}


class Sentinel:
    def __getattr__(self,name):
        if name in FORBIDDEN:
            raise AssertionError('FORBIDDEN API accessed: '+name)
        raise AttributeError(name)


def xyz(x=0,y=0,z=0):return NS(x=x,y=y,z=z)
def tf(x=0,y=0,z=0):return NS(location=xyz(x,y,z),rotation=NS(yaw=0.,pitch=0.,roll=0.))
def pos(s):return [(250-s)*policy.TOWN01_LENGTH_M/195,0.,0.]


class Waypoint(Sentinel):
    road_id,section_id,lane_id,lane_width,is_junction,lane_type=8,0,1,4.,False,'Driving'
    def __init__(self,s):self.s=s;self.transform=tf(*pos(s))
    def next(self,d):return [Waypoint(self.s-d)]


class World(Sentinel):
    id=42
    def __init__(self):
        self.map=Map();self.settings=NS(synchronous_mode=False,fixed_delta_seconds=None,no_rendering_mode=False)
        self.weather=NS(wetness=0.,cloudiness=0.);self.actors=[];self.boxes=[];self.objects=[]
        self.snapshots=iter([(100,5.),(110,5.5)]);self.settings_calls=0;self.actor_calls=0
    def get_map(self):return self.map
    def get_settings(self):self.settings_calls+=1;return self.settings
    def get_weather(self):return self.weather
    def get_actors(self):self.actor_calls+=1;return self.actors
    def get_snapshot(self):
        frame,t=next(self.snapshots)
        return NS(frame=frame,timestamp=NS(elapsed_seconds=t,delta_seconds=.05,platform_timestamp=t+1000))
    def get_level_bbs(self,label):return self.boxes if label=='Static' else []
    def get_environment_objects(self,label):return self.objects


class Map(Sentinel):
    name='/Game/Carla/Maps/Town01_Opt'
    def to_opendrive(self):return XML
    def get_waypoint_xodr(self,road,lane,s):return Waypoint(s)
    def get_all_landmarks(self):return []


class Client(Sentinel):
    def __init__(self):self.world=World()
    def get_server_version(self):return '0.9.16'
    def get_client_version(self):return '0.9.16'
    def get_world(self):return self.world


@pytest.fixture
def synthetic(tmp_path,monkeypatch):
    c=policy.town01_config()
    points=[pos(s) for s in range(250,54,-1)]
    poses=[dict(road=8,section=0,lane=1,s=s,yaw=0.,pitch=0.,roll=0.,width=4.) for s in range(250,54,-1)]
    digest=geometry_sha256(points,poses)
    monkeypatch.setattr(reference,'GEOMETRY_SHA256',digest)
    monkeypatch.setattr(policy,'TOWN01_MAP_SHA256',hashlib.sha256(XML.encode()).hexdigest())
    geometry=dict(points=points,waypoints=poses,geometry_sha256=digest)
    xp=tmp_path/'synthetic.xodr';xp.write_text(XML,encoding='utf-8')
    gp=tmp_path/'synthetic.json';gp.write_text(json.dumps(geometry),encoding='utf-8')
    ref=reference.read_reference(c,xp,gp)
    api=NS(LaneType=NS(Driving='Driving'),CityObjectLabel=NS(**{k:k for k in ALL_LABELS+('Any',)}))
    return NS(c=c,ref=ref,api=api,xml=xp,geometry=gp,points=points,poses=poses)


def audit(tmp_path,s,client=None):
    e=Evidence(tmp_path/'evidence','TEST_SYNTHETIC',synthetic=True)
    a=SceneAudit(client or Client(),s.api,s.c,s.ref,e)
    return a,a.run()


def test_mock_e2e_196_window_no_mutation(tmp_path,synthetic):
    a,result=audit(tmp_path,synthetic)
    assert result['status']=='pass'
    assert result['route_audit'] is None and result['physical_bounds'] is None
    assert result['execute_allowed'] is False
    assert a.e.data['route_geometry']['point_count']==196
    assert a.e.data['route_comparison']['status']=='pass'
    env=a.e.data['environment']
    assert env['snapshot_start']['frame']==100 and env['snapshot_end']['frame']==110
    assert env['single_frame_claim'] is False
    assert sum(x['api']=='map.get_waypoint_xodr' for x in a.reader.ledger)==196
    assert all(x['api'].split('.')[1] not in FORBIDDEN for x in a.reader.ledger)
    for file in FILES:
        record=json.loads((a.e.directory/(file+'.json')).read_text(encoding='utf-8'))
        assert record['synthetic'] is True and record['source_commit']=='TEST_SYNTHETIC'
    assert (a.e.directory/'report_ja.md').is_file()


@pytest.mark.parametrize('field,value,reason',[
    ('road_id',9,'lane/key'),('section_id',1,'lane/key'),('lane_id',-1,'lane/key'),
    ('is_junction',True,'lane/key'),('lane_type','Shoulder','lane/key'),
    ('lane_width',3.5,'reference'),('s',249.,'reference')])
def test_waypoint_mismatch(tmp_path,synthetic,field,value,reason):
    client=Client()
    def waypoint(*args):
        w=Waypoint(args[-1]);setattr(w,field,value);return w
    client.world.map.get_waypoint_xodr=waypoint
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and reason in r['reason']
    assert len(a.e.data['route_geometry']['observed_points'])==1


@pytest.mark.parametrize('change', ['missing','position','rotation','nan','branch','reverse'])
def test_geometry_bad_or_missing(tmp_path,synthetic,change):
    client=Client()
    def waypoint(*args):
        if change=='missing':return None
        w=Waypoint(args[-1])
        if change=='position':w.transform.location.y=1.
        if change=='rotation':w.transform.rotation.yaw=1.
        if change=='nan':w.transform.location.z=float('nan')
        if change=='branch':w.next=lambda d:[Waypoint(w.s-d),Waypoint(w.s-d)]
        if change=='reverse':w.next=lambda d:[Waypoint(w.s+d)]
        return w
    client.world.map.get_waypoint_xodr=waypoint
    a,r=audit(tmp_path,synthetic,client)
    assert r['status'] in ('fail','unknown')
    if change=='missing':assert a.e.data['route_geometry']['status']=='unknown'


@pytest.mark.parametrize('change',['server_version','client_version','map','hash','sync','actor'])
def test_preflight_fail_closed(tmp_path,synthetic,change):
    client=Client()
    if change=='server_version':client.get_server_version=lambda:'0.9.15'
    if change=='client_version':client.get_client_version=lambda:'0.9.15'
    if change=='map':client.world.map.name='Town10HD_Opt'
    if change=='hash':client.world.map.to_opendrive=lambda:'wrong'
    if change=='sync':client.world.settings.synchronous_mode=True
    if change=='actor':client.world.actors=[NS(id=1,type_id='vehicle.tesla.model3',attributes={})]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail'
    assert not any(x['api']=='map.get_waypoint_xodr' for x in a.reader.ledger)


def test_unreviewed_prop_actor_refused(tmp_path,synthetic):
    client=Client();client.world.actors=[NS(id=99,type_id='static.prop.box01',attributes={})]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and 'unreviewed actor type' in r['reason']
    assert not any(x['api']=='map.get_waypoint_xodr' for x in a.reader.ledger)


@pytest.mark.parametrize('pair',[[(100,5.),(99,5.5)],[(100,5.),(110,4.)],[(100,5.),(110,5.)],[(100,5.),(100,5.5)]])
def test_snapshot_inconsistency(tmp_path,synthetic,pair):
    client=Client();client.world.snapshots=iter(pair)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'snapshot window' in r['reason']


@pytest.mark.parametrize('change',['settings','actors','episode','weather','map'])
def test_changed_acquisition_state(tmp_path,synthetic,change):
    client=Client();w=client.world
    if change=='settings':
        def settings():
            w.settings_calls+=1
            return NS(synchronous_mode=False,fixed_delta_seconds=None if w.settings_calls==1 else .05)
        w.get_settings=settings
    if change=='actors':
        def actors():
            w.actor_calls+=1
            return [] if w.actor_calls==1 else [NS(id=2,type_id='spectator',attributes={})]
        w.get_actors=actors
    if change=='episode':
        n=[0]
        def world():n[0]+=1;w.id=42 if n[0]==1 else 43;return w
        client.get_world=world
    if change=='weather':
        n=[0]
        def weather():n[0]+=1;return NS(wetness=float(n[0]))
        w.get_weather=weather
    if change=='map':
        n=[0]
        def map_get():n[0]+=1;return w.map if n[0]==1 else NS(name='Town04')
        w.get_map=map_get
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'changed' in r['reason']


def box(x=50,y=0,z=1):return NS(location=xyz(x,y,z),extent=xyz(.5,.5,.5),rotation=NS(yaw=0.,pitch=0.,roll=0.))


@pytest.mark.parametrize('source',['level','environment','landmark','trigger','stop','affected','xml'])
def test_obstacle_regulatory_abort(tmp_path,synthetic,source):
    client=Client();w=client.world
    if source=='level':w.boxes=[box()]
    if source=='environment':w.objects=[NS(id=5,type='Static',bounding_box=box())]
    if source=='landmark':w.map.get_all_landmarks=lambda:[NS(id='sign',transform=tf(50,0))]
    if source in ('trigger','stop','affected'):
        actor=NS(id=6,type_id='traffic.traffic_light',attributes={},
                 trigger_volume=box(x=0),get_transform=lambda:tf(50,0 if source=='trigger' else 50),
                 get_state=lambda:'Red',get_stop_waypoints=lambda:[],get_affected_lane_waypoints=lambda:[])
        if source=='stop':actor.get_stop_waypoints=lambda:[Waypoint(200)]
        if source=='affected':actor.get_affected_lane_waypoints=lambda:[Waypoint(200)]
        w.actors=[actor]
    if source=='xml':
        xml='<OpenDRIVE><road id="8"><signals><signal s="100"/></signals></road></OpenDRIVE>'
        w.map.to_opendrive=lambda:xml
        synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail'
    assert any(term in r['reason'] for term in ('obstacle','landmark','trigger','waypoint','regulatory'))


def test_elevated_box_records_z_without_false_ground_obstacle(tmp_path,synthetic):
    client=Client();client.world.boxes=[box(z=10)]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass'
    rec=a.e.data['static_obstacles']['observed'][0]
    assert rec['xy_overlap'] and not rec['z_overlap'] and rec['z_min']==9.5


def test_traffic_light_state_change_invalidates_window(tmp_path,synthetic):
    client=Client();states=iter(['Red','Green'])
    client.world.actors=[NS(id=7,type_id='traffic.traffic_light',attributes={},
        trigger_volume=box(x=0),get_transform=lambda:tf(50,50),get_state=lambda:next(states),
        get_stop_waypoints=lambda:[],get_affected_lane_waypoints=lambda:[])]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'traffic actor state changed' in r['reason']


def test_rpc_failure_poison_no_more_calls(tmp_path,synthetic):
    client=Client()
    def fail():raise RuntimeError('synthetic RPC failure')
    client.world.get_weather=fail
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and a.reader.poisoned
    assert a.reader.ledger[-1]['api']=='world.get_weather' and a.reader.ledger[-1]['status']=='failed'
    count=len(a.reader.ledger)
    with pytest.raises(AuditAbort):a.reader.call('client',client,'get_world')
    assert len(a.reader.ledger)==count


@pytest.mark.parametrize('method',sorted(FORBIDDEN))
def test_allowlist_rejects_before_attribute_access(tmp_path,method):
    reader=Reader(Evidence(tmp_path,'SYNTHETIC',synthetic=True))
    with pytest.raises(AuditAbort,match='outside read-only allowlist'):
        reader.call('world',Sentinel(),method)
    assert reader.ledger==[]


def test_default_cli_no_carla_or_server(monkeypatch,tmp_path):
    monkeypatch.setitem(sys.modules,'carla',NS(Client=lambda *a:pytest.fail('connection forbidden')))
    monkeypatch.setattr(cli,'supervise',lambda *a:pytest.fail('worker forbidden in offline mode'))
    monkeypatch.setattr(cli,'inspect_os',lambda *a:pytest.fail('offline does not inspect a server'))
    assert cli.main(['--output-root',str(tmp_path)])==0
    path=next(tmp_path.glob('town01_scene_audit_*'))
    result=json.loads((path/'audit_results.json').read_text())['data']
    assert result['mode']=='offline' and result['reference_status']=='unknown'
    assert json.loads((path/'api_call_ledger.json').read_text())['data']['status']=='unknown'


@pytest.mark.parametrize('flags',[['--connect-readonly'],['--host','localhost'],['--port','2000'],['--server-pid','123']])
def test_cli_missing_target_rejected_before_worker(monkeypatch,tmp_path,flags):
    monkeypatch.setattr(cli,'supervise',lambda *a:pytest.fail('worker must not run'))
    assert cli.main(flags+['--output-root',str(tmp_path)])==2


def test_reference_independent_of_approval_and_geometry_hash(tmp_path,synthetic):
    assert synthetic.c.route_audit is None and synthetic.ref['kind']=='offline_reference_not_approval'
    assert len(synthetic.ref['points'])==196
    assert synthetic.ref['measured_length_m']!=195
    data=json.loads(synthetic.geometry.read_text());data['points'][5][0]+=.01
    synthetic.geometry.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='hash'):reference.read_reference(synthetic.c,synthetic.xml,synthetic.geometry)


def test_reference_wrong_map_and_missing(tmp_path,synthetic):
    synthetic.xml.write_text('different map')
    with pytest.raises(ValueError,match='OpenDRIVE'):reference.read_reference(synthetic.c,synthetic.xml,synthetic.geometry)
    with pytest.raises(ValueError,match='both'):reference.read_reference(synthetic.c,None,None)


def target(tmp_path):
    t=dict(host='127.0.0.1',port=2100,tm_port=8100,server_pid=123,
           server_started_utc='2026-10-20T00:00:00+00:00',session_reference=str(tmp_path/'session.json'))
    review=dict(t);review.pop('session_reference')
    review.update(dedicated_session_declared=True,operator='SYNTHETIC',reviewed_at='2026-10-20T01:00:00Z')
    Path(t['session_reference']).write_text(json.dumps(review))
    return t


def test_explicit_session_validation_and_os_pid(tmp_path):
    t=target(tmp_path);assert host.validate_target(t)['operator']=='SYNTHETIC'
    observed=dict(pid=123,name='CarlaUE4-Win64-Shipping.exe',started_utc=t['server_started_utc'],tcp=[
        dict(LocalPort=2100,RemotePort=0,State='Listen',OwningProcess=123)])
    assert host.verify_os(t,observed)['external_clients_absent']=='unknown'
    for key,value in [('pid',124),('started_utc','2026-10-19T00:00:00Z'),('name','other.exe')]:
        bad=copy.deepcopy(observed);bad[key]=value
        with pytest.raises(ValueError):host.verify_os(t,bad)
    for row in [dict(LocalPort=2100,RemotePort=0,State='Listen',OwningProcess=999),
                dict(LocalPort=8100,RemotePort=0,State='Listen',OwningProcess=999),
                dict(LocalPort=2100,RemotePort=100,State='Established',OwningProcess=123)]:
        bad=copy.deepcopy(observed);bad['tcp'].append(row)
        with pytest.raises(ValueError):host.verify_os(t,bad)


def test_connection_requires_reference_even_with_explicit_target(tmp_path,monkeypatch):
    t=target(tmp_path);flags=['--connect-readonly','--output-root',str(tmp_path)]
    for k,v in t.items():flags.extend(['--'+k.replace('_','-'),str(v)])
    monkeypatch.setattr(cli,'supervise',lambda *a:pytest.fail('no reference, no worker'))
    assert cli.main(flags)==2


def test_native_monitor_preserves_partial_evidence(tmp_path):
    e=Evidence(tmp_path,'SYNTHETIC',synthetic=True)
    e.write('route_geometry',dict(status='unknown',observed_points=[[1,2,3]]))
    command=[sys.executable,'-c','import sys; sys.stderr.write("Fatal synthetic native failure"); sys.exit(3)']
    assert cli.supervise(tmp_path/'unused.json',tmp_path,command=command)==1
    assert e.data['route_geometry']['observed_points']==[[1,2,3]]
    assert json.loads((tmp_path/'supervisor_result.json').read_text())['native_failure']
    assert json.loads((tmp_path/'audit_results.json').read_text())['data']['execute_allowed'] is False


def test_zero_exit_without_completed_marker_is_invalid(tmp_path):
    Evidence(tmp_path,'SYNTHETIC',synthetic=True)
    assert cli.supervise(tmp_path/'unused.json',tmp_path,command=[sys.executable,'-c','pass'])==1
    result=json.loads((tmp_path/'audit_results.json').read_text())['data']
    assert result['reason']=='worker incomplete' and result['status']=='unknown'
    manifest=json.loads((tmp_path/'artifact_hashes.json').read_text())
    assert manifest['audit_results.json']==hashlib.sha256((tmp_path/'audit_results.json').read_bytes()).hexdigest()


def test_timeout_only_terminates_owned_audit_child(tmp_path):
    Evidence(tmp_path,'SYNTHETIC',synthetic=True)
    assert cli.supervise(tmp_path/'unused.json',tmp_path,command=[sys.executable,'-c','import time; time.sleep(5)'],timeout=.1)==1
    result=json.loads((tmp_path/'supervisor_result.json').read_text())
    assert result['timed_out'] and not result['recovery_rpc_attempted']


def test_worker_preconditions_and_single_constructor_are_synthetic(tmp_path,synthetic,monkeypatch):
    import idm_harness_host
    t=target(tmp_path)
    flags=['--connect-readonly','--reference-opendrive',str(synthetic.xml),
           '--reference-geometry',str(synthetic.geometry)]
    for k,v in t.items():flags.extend(['--'+k.replace('_','-'),str(v)])
    args=cli.parser().parse_args(flags)
    directory=tmp_path/'worker';directory.mkdir()
    payload=tmp_path/'payload.json'
    payload.write_text(json.dumps(dict(args=vars(args),directory=str(directory),commit='SYNTHETIC',supervisor_pid=999,synthetic=True)))
    observed=dict(pid=123,name='CarlaUE4.exe',started_utc=t['server_started_utc'],tcp=[
        dict(LocalPort=2100,RemotePort=0,State='Listen',OwningProcess=123)])
    monkeypatch.setattr(cli,'inspect_os',lambda t:observed)
    monkeypatch.setattr(idm_harness_host,'HealthGate',lambda *a:NS(check=lambda:None,call=lambda fn,*a:fn(*a)))
    calls=[]
    def factory(host,port):calls.append((host,port));return Client()
    monkeypatch.setitem(sys.modules,'carla',NS(Client=factory,LaneType=synthetic.api.LaneType,
                                           CityObjectLabel=synthetic.api.CityObjectLabel))
    monkeypatch.setattr(sys,'argv',['synthetic_worker',str(payload)])
    class WorkerExit(BaseException):pass
    monkeypatch.setattr(cli.os,'_exit',lambda code:(_ for _ in ()).throw(WorkerExit(code)))
    with pytest.raises(WorkerExit) as exc:cli.worker()
    assert exc.value.args==(0,) and calls==[('127.0.0.1',2100)]
    result=json.loads((directory/'audit_results.json').read_text())['data']
    assert result['status']=='pass' and result['route_audit'] is None
    ledger=json.loads((directory/'api_call_ledger.json').read_text())['data']
    assert ledger[0]['api']=='transport.Client' and ledger[0]['status']=='returned'


def test_bad_os_does_not_import_or_construct_client(tmp_path,synthetic,monkeypatch):
    t=target(tmp_path)
    flags=['--connect-readonly','--reference-opendrive',str(synthetic.xml),'--reference-geometry',str(synthetic.geometry)]
    for k,v in t.items():flags.extend(['--'+k.replace('_','-'),str(v)])
    args=cli.parser().parse_args(flags)
    directory=tmp_path/'worker';directory.mkdir()
    payload=tmp_path/'payload.json'
    payload.write_text(json.dumps(dict(args=vars(args),directory=str(directory),commit='SYNTHETIC',supervisor_pid=999,synthetic=True)))
    monkeypatch.setattr(cli,'inspect_os',lambda t:(_ for _ in ()).throw(ValueError('PID gone')))
    monkeypatch.setitem(sys.modules,'carla',NS(Client=lambda *a:pytest.fail('must not construct Client')))
    monkeypatch.setattr(sys,'argv',['synthetic_worker',str(payload)])
    class WorkerExit(BaseException):pass
    monkeypatch.setattr(cli.os,'_exit',lambda code:(_ for _ in ()).throw(WorkerExit(code)))
    with pytest.raises(WorkerExit) as exc:cli.worker()
    assert exc.value.args==(2,)
    assert 'PID gone' in json.loads((directory/'audit_results.json').read_text())['data']['reason']


def test_static_no_mutation_or_harness_import():
    root=Path(cli.__file__).parent
    for name in ('Run_Audit_Town01_Scene.py','idm_scene_audit.py','idm_scene_reference.py','idm_scene_os.py'):
        tree=ast.parse((root/name).read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node,ast.Attribute):assert node.attr not in FORBIDDEN
            if isinstance(node,ast.ImportFrom):
                assert node.module not in ('idm_experiment_harness','idm_brake_harness','Run_Calibrate_Brakes')
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='call':
                # Dynamic dispatch names used in the engine must be literal allowlisted pairs.
                if len(node.args)>=3 and isinstance(node.args[0],ast.Constant) and isinstance(node.args[2],ast.Constant):
                    if node.args[0].value in ALLOWLIST:
                        assert node.args[2].value in ALLOWLIST[node.args[0].value]
    # Reused geometry functions have no hidden imports/remote methods.
    import idm_brake_route
    import inspect
    for fn in ('angle','box_vertices','rotate','hull','overlaps','ribbons','segment_distance','geometry_sha256','validate_review_geometry'):
        tree=ast.parse(inspect.getsource(getattr(idm_brake_route,fn)))
        assert not any(isinstance(n,(ast.Import,ast.ImportFrom)) for n in ast.walk(tree))
        assert not any(isinstance(n,ast.Attribute) and n.attr in FORBIDDEN for n in ast.walk(tree))
