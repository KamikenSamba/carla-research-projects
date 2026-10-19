"""Synthetic Town01 contracts, not CARLA physics or real scene approval."""
import copy
from dataclasses import asdict,replace
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

import idm_brake_config as policy
from idm_brake_config import BrakeConfig,town01_config,load_config
from idm_brake_route import geometry_sha256,ribbons,overlaps,box_vertices,ALL_LABELS
from idm_brake_metrics import read_validated_run
import test_brake_calibration as legacy
from test_idm_context import Transform,vector

XML='<OpenDRIVE><road id="8"><signals/><objects/></road></OpenDRIVE>'


def position(s):
    i=250-s;x=i*policy.TOWN01_LENGTH_M/195
    return [x,.25*(i/195)**2,0]


def heading(s):
    i=250-s
    return math.degrees(math.atan(.5*(i/195)/policy.TOWN01_LENGTH_M))


def reviewed(c=None):
    c=c or town01_config()
    points=[position(s) for s in range(250,54,-1)]
    meta=[dict(s=s,road=8,section=0,lane=1,yaw=heading(s),pitch=0,roll=0,width=4.)
          for s in range(250,54,-1)]
    audit=dict(policy_sha256=c.policy_sha256,opendrive_sha256=policy.TOWN01_MAP_SHA256,
        evidence_sha256='f'*64,map_name=c.map_name,carla_version='0.9.16',
        reviewed_by='SYNTHETIC TEST ONLY',reviewed_at='TEST ONLY',evidence_reference='TEST ONLY',
        scene_review_reference='TEST ONLY',dedicated_session_reference='TEST ONLY',
        points=points,waypoints=meta,geometry_sha256=geometry_sha256(points,meta))
    return replace(c,route_audit=audit)


@pytest.fixture
def synthetic_policy(monkeypatch):
    # The mock XML is deliberately NOT the real map. This shim is test-scoped only.
    monkeypatch.setattr(policy,'TOWN01_MAP_SHA256',hashlib.sha256(XML.encode()).hexdigest())
    return reviewed()


def run_town01(tmp_path,monkeypatch,c,setup=None,**kwargs):
    monkeypatch.setattr(legacy,'config',lambda:c)
    original=legacy.BrakeHarness
    def factory(client,api,*args):
        for label in ALL_LABELS+('NONE',):setattr(api.CityObjectLabel,label,label)
        api.Location=lambda **kw:NS(**kw)
        return original(client,api,*args)
    monkeypatch.setattr(legacy,'BrakeHarness',factory)
    def configure(w):
        class WP:
            road_id,section_id,lane_id,lane_width,is_junction,lane_type=8,0,1,4.,False,'Driving'
            def __init__(self,s):self.s=s
            @property
            def transform(self):
                p=position(self.s);return Transform(p[0],p[1],heading(self.s))
            def next(self,d):return [WP(self.s-d)]
            def get_landmarks(self,*args,**kw):return w.landmarks
        w.external.type_id='spectator'
        w.map.name='Carla/Maps/Town01_Opt'
        w.map.to_opendrive=lambda:XML
        w.map.get_waypoint_xodr=lambda road,lane,s:WP(s) if (road,lane)==(8,1) else None
        w.map.get_waypoint=lambda p,**kw:WP(250-p.x*195/policy.TOWN01_LENGTH_M)
        w.map.get_all_landmarks=lambda:[]
        w.get_environment_objects=lambda label:[]
        if setup:setup(w)
    return legacy.run(tmp_path,setup=configure,**kwargs)


def box(x,y,z=1,ex=.02,ey=.02,ez=.1,yaw=0):
    return NS(location=vector(x,y,z),extent=vector(ex,ey,ez),
              rotation=NS(yaw=yaw,pitch=0,roll=0))


def test_v1_defaults_fingerprint_and_distance_contract_unchanged():
    c=BrakeConfig()
    assert c.fingerprint=='450fd09aa784158b5da5371baeb0bdd531015573af13016b9ca94c7a075cd24f'
    assert c.policy_sha256=='5591a52eeb194ff32c3eb898aec25cea55ac5f1e456a8324dfdb0675fe914e9b'
    assert c.route_point_count==57 and not c.extended_route_audit
    assert c.distance_report()==dict(assumptions_only=True,v_mps=1.2,tau_s=.5,a_mps2=5,b_mps2=.5,
        stopping_distance_m=14.915000000000001,required_route_m=43.34011,
        margin_m=12.659889999999997,accepted_braking_lower_bound=None,accepted_response_upper_bound=None)


def test_new_policy_changes_only_explicit_map_route_distance_fields():
    old=asdict(BrakeConfig());new=asdict(town01_config())
    assert {k for k in old if old[k]!=new[k]}=={'mode','map_name','road','follower_s','end_s','planned_route_length'}
    c=load_config(Path(__file__).parents[1]/'configs/brake_calibration_town01_v2.json')
    assert c.route_audit is None and c.route_point_count==196 and c.extended_route_audit
    assert c.policy_sha256!=BrakeConfig().policy_sha256 and c.fingerprint!=BrakeConfig().fingerprint


@pytest.mark.parametrize('change',[dict(map_name='Town04'),dict(road=10),dict(section=1),dict(lane=-1),
    dict(follower_s=249),dict(end_s=54),dict(planned_route_length=195),dict(target_speed=2),
    dict(throttle_cap=.2),dict(brake=.8),dict(acceleration_peak=6),dict(ready_seconds=1)])
def test_new_policy_is_fixed_not_arbitrary_route_or_relaxed_control(change):
    with pytest.raises(ValueError):replace(town01_config(),**change)


@pytest.mark.parametrize('field,value',[('map_name','Town10HD_Opt'),('policy_sha256','a'*64),
    ('opendrive_sha256','5d883b799f634030af92be1e9d79d107845540ba04338e8c60e095be1aef7be7'),
    ('geometry_sha256','a'*64),('points',[[0,0,0]]*57),('waypoints',[]),
    ('physical_bounds',{'braking_lower_bound':2})])
def test_new_review_rejects_cross_map_or_corrupted_evidence(field,value):
    c=reviewed()
    with pytest.raises(ValueError):replace(c,route_audit=dict(c.route_audit,**{field:value}))


def test_review_requires_all_poses_and_coordinate_consistency():
    c=reviewed()
    for mutation in ('pose','width','s','direction','distance'):
        a=copy.deepcopy(c.route_audit)
        if mutation=='pose':a['waypoints'][10]['pitch']=10
        elif mutation=='width':a['waypoints'][10]['width']=3.5
        elif mutation=='s':a['waypoints'][10]['s']=230
        elif mutation=='direction':a['waypoints'][10]['yaw']=180
        elif mutation=='distance':a['points'][10][0]+=.1
        a['geometry_sha256']=geometry_sha256(a['points'],a['waypoints'])
        with pytest.raises(ValueError):replace(c,route_audit=a)


def test_coordinate_distance_not_integer_s_difference():
    c=reviewed();report=c.distance_report()
    length=sum(math.dist(p,q) for p,q in zip(c.route_audit['points'],c.route_audit['points'][1:]))
    assert length!=195 and report['route_length_m']==pytest.approx(length)
    assert report['route_length_source']=='reviewed_coordinates'
    assert [r['required_route_m'] for r in report['sensitivity']]==pytest.approx([43.34011,57.03011,109.00511])
    live=c.distance_report(route_length_m=195.00316911583323)
    assert live['route_length_source']=='live_waypoint_coordinates'
    assert [r['margin_m'] for r in live['sensitivity']]==pytest.approx([151.663059,137.973059,85.998059])
    assert report['accepted_braking_lower_bound'] is None


def test_unreviewed_new_policy_refused_before_any_os_or_client(tmp_path,monkeypatch,capsys):
    import Run_Calibrate_Brakes as entry
    p=tmp_path/'new.json';p.write_text(json.dumps(asdict(town01_config())))
    monkeypatch.setattr(entry,'listener_pids',lambda *_:pytest.fail('OS/RPC forbidden'))
    assert entry.main(['--config',str(p)])==0
    assert not json.loads(capsys.readouterr().out)['executable_route']
    with pytest.raises(SystemExit):entry.main(['--config',str(p),'--execute','--server-pid','1'])


def test_new_mock_e2e_frames_cleanup_and_replay(tmp_path,monkeypatch,synthetic_policy):
    result,w,gate,h,commands=run_town01(tmp_path,monkeypatch,synthetic_policy)
    assert result['status']=='completed',result
    assert len(h.route)==196 and h.route_length!=195
    assert len(h.rows)==len(commands)==w.frame
    assert all(r['command_based_on_frame']+1==r['frame']==r['control_read_at_frame'] for r in h.rows)
    assert len({cmd['actor'] for cmd in commands})==1
    assert not any(e[0] in ('tm_sync','autopilot','seed','path') for e in w.events)
    assert result['cleanup']['complete'] and w.actors==[w.external] and vars(w.settings)==vars(w.original)
    assert read_validated_run(tmp_path,require_supervised=False)['valid']
    distance=json.loads((tmp_path/'distance_budget.json').read_text())
    assert distance['route_length_m']==pytest.approx(h.route_length)
    assert (tmp_path/'route_preflight.json').exists() and (tmp_path/'spawn_bbox_audit.json').exists()


@pytest.mark.parametrize('fault,reason',[('hash','route_map_hash_mismatch'),('width','reviewed_waypoint_pose_mismatch'),
    ('pose','reviewed_waypoint_pose_mismatch'),('s','reviewed_waypoint_pose_mismatch'),('xml','regulatory_xml_near_route'),
    ('landmark','regulatory_landmark_near_route'),('sidewalk','static_route_obstacle'),
    ('unknown','static_route_obstacle')])
def test_new_preflight_rejects_before_settings_spawn_tick(tmp_path,monkeypatch,synthetic_policy,fault,reason):
    def setup(w):
        if fault=='hash':w.map.to_opendrive=lambda:'wrong'
        if fault in ('width','pose','s'):
            old=w.map.get_waypoint_xodr
            original_class=type(old(8,1,250))
            class ChangedPose(original_class):
                @property
                def transform(self):
                    original=super().transform
                    return Transform(original.location.x,original.location.y,heading(self.s)+.02)
            def wp(*a):
                p=old(*a)
                if fault=='width':p.lane_width=3.9
                elif fault=='s':p.s+=.05
                else:
                    p=ChangedPose(p.s)
                return p
            w.map.get_waypoint_xodr=wp
        if fault=='xml':
            # Hash mismatch would correctly reject first; shim only to reach XML semantic validation.
            text='<OpenDRIVE><road id="8"><objects><object s="100" name="STOP"/></objects></road></OpenDRIVE>'
            monkeypatch.setattr(policy,'TOWN01_MAP_SHA256',hashlib.sha256(text.encode()).hexdigest())
            w.map.to_opendrive=lambda:text
            object.__setattr__(synthetic_policy,'route_audit',dict(synthetic_policy.route_audit,opendrive_sha256=policy.TOWN01_MAP_SHA256))
        if fault=='landmark':w.map.get_all_landmarks=lambda:[NS(transform=Transform(50))]
        if fault=='sidewalk':w.get_level_bbs=lambda label:[box(170,2.15)] if label=='Sidewalks' else []
        if fault=='unknown':w.get_environment_objects=lambda label:[NS(bounding_box=box(10,1.9))]
    result,w,*_=run_town01(tmp_path,monkeypatch,synthetic_policy,setup=setup)
    assert result['status']=='safe_early_stop' and result['reason']==reason,result
    assert not any(e[0] in ('settings','spawn','tick') for e in w.events)


@pytest.mark.parametrize('fault',['peak','speed','server','getter','external','settings'])
def test_new_policy_preserves_runtime_abort_cleanup_and_no_rpc_after_failure(tmp_path,monkeypatch,synthetic_policy,fault):
    result,w,gate,h,_=run_town01(tmp_path,monkeypatch,synthetic_policy,fault=fault)
    assert result['status']!='completed' and not result['calibration_valid']
    if fault in ('server','getter'):
        assert gate.poisoned and gate.calls_after_poison==0 and not any(e[0]=='destroy' for e in w.events)
    else:
        assert result['cleanup']['complete'] and w.external in w.actors and vars(w.settings)==vars(w.original)


def test_polyline_full_width_catches_obstacle_outside_old_fixed_strip():
    route=[NS(transform=Transform(x,y),lane_width=4) for x,y in [(0,0),(50,.1),(100,.25)]]
    corridor=ribbons(route,2.42511)
    # Beyond y=2m of the initial-axis rectangle, but inside the actual shifted lane.
    obstacle=[(99,2.1),(99.1,2.1),(99.1,2.2),(99,2.2)]
    assert any(overlaps(obstacle,p) for p in corridor)
    assert not any(overlaps([(x,y+10) for x,y in obstacle],p) for p in corridor)
    rotated=box_vertices(box(99,2.15,yaw=45))
    assert any(overlaps([(p[0],p[1]) for p in rotated],r) for r in corridor)


@pytest.mark.parametrize('fault,reason',[('rear_lane','spawn_bbox_lane_occupancy'),
    ('overhead','spawn_bbox_obstacle'),('unknown_extent','spawn_bbox_unknown')])
def test_initial_actual_bbox_failure_restores_settings_without_tick(tmp_path,monkeypatch,synthetic_policy,fault,reason):
    def setup(w):
        if fault=='rear_lane':
            old=w.map.get_waypoint
            def wp(p,**kw):
                value=old(p,**kw)
                if p.x<0:value.road_id=99
                return value
            w.map.get_waypoint=wp
        old_spawn=w.try_spawn_actor
        def spawn(*args):
            a=old_spawn(*args)
            if fault=='overhead':a.bounding_box.extent.z=3
            if fault=='unknown_extent':a.bounding_box.extent.z=0
            return a
        w.try_spawn_actor=spawn
        if fault=='overhead':w.get_level_bbs=lambda label:[box(0,0,z=2.6)] if label=='Other' else []
    result,w,*_=run_town01(tmp_path,monkeypatch,synthetic_policy,setup=setup)
    assert result['status']=='safe_early_stop' and result['reason']==reason,result
    assert not any(e[0]=='tick' for e in w.events)
    assert result['cleanup']['complete'] and w.actors==[w.external] and vars(w.settings)==vars(w.original)


@pytest.mark.parametrize('kind',['trigger','stop_waypoint','missing_getter'])
def test_regulatory_actor_inventory_is_fail_closed(tmp_path,monkeypatch,synthetic_policy,kind):
    def setup(w):
        a=w.external;a.type_id='traffic.traffic_light'
        a.get_transform=lambda:Transform(10,10)
        a.trigger_volume=box(0,0,z=0)
        a.get_stop_waypoints=lambda:[]
        a.get_affected_lane_waypoints=lambda:[]
        if kind=='trigger':a.get_transform=lambda:Transform(10)
        elif kind=='stop_waypoint':a.get_stop_waypoints=lambda:[NS(transform=Transform(10))]
        else:del a.get_stop_waypoints
    result,w,*_=run_town01(tmp_path,monkeypatch,synthetic_policy,setup=setup)
    assert result['status']=='safe_early_stop' and result['reason'].startswith('regulatory_'),result
    assert not any(e[0] in ('spawn','settings','tick') for e in w.events)


def test_new_offline_cli_imports_no_carla():
    import subprocess,sys
    script=Path(__file__).parents[1]/'Run_Calibrate_Brakes.py'
    config=script.parent/'configs/brake_calibration_town01_v2.json'
    code="""import builtins,runpy,sys
from pathlib import Path
original=builtins.__import__
def guarded(name,*a,**k):
    if name=='carla':raise AssertionError('CARLA import forbidden')
    return original(name,*a,**k)
builtins.__import__=guarded
sys.path.insert(0,str(Path(sys.argv[1]).parent));sys.argv=[sys.argv[1],'--config',sys.argv[2]]
runpy.run_path(sys.argv[0],run_name='__main__')
"""
    p=subprocess.run([sys.executable,'-c',code,str(script),str(config)],capture_output=True,text=True)
    assert p.returncode==0,p.stderr
    assert json.loads(p.stdout)['status']=='validated_offline_no_connection'
