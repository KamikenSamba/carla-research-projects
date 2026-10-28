"""Synthetic prop geometry; local SDK tests never construct a Client."""
import copy
import json
import math
from types import SimpleNamespace as NS

import pytest

from idm_brake_route import box_vertices
from idm_scene_audit import AuditAbort, Evidence, Reader
from test_scene_audit import Client, audit, box, synthetic, tf


def prop(actor_id=99, x=50, y=10, z=0, mesh='Chair'):
    return NS(id=actor_id,type_id='static.prop.mesh',attributes={'mesh_path':'/Game/'+mesh},
              bounding_box=box(x=0,z=.5),get_transform=lambda:tf(x,y,z))


@pytest.mark.parametrize('mesh',['Chair','Table','UnknownAssetName'])
def test_clear_names_are_only_metadata(tmp_path,synthetic,mesh):
    client=Client();client.world.actors=[prop(mesh=mesh)]
    a,result=audit(tmp_path,synthetic,client)
    assert result['status']=='pass' and result['route_audit'] is None
    data=a.e.data['static_prop_geometry']
    assert data['endpoint_state_equal'] and data['before'][0]['status']=='CLEAR'
    assert data['after'][0]['status']=='CLEAR'
    assert json.loads((a.e.directory/'static_prop_geometry.json').read_text())['data']==data
    methods=[r['api'] for r in a.reader.ledger]
    assert methods.count('prop.get_transform')==methods.count('prop.bounding_box')==2
    assert methods.count('map.get_waypoint_xodr')==196
    assert methods.count('world.get_level_bbs')==28
    assert methods.count('world.get_environment_objects')==1
    assert len(methods)==441  # Existing 437 plus two transforms and two bbox properties.
    assert methods.index('prop.bounding_box')<methods.index('map.get_waypoint_xodr')


@pytest.mark.parametrize('x,y,z',[(50,0,0),(50,2.5,0),(50,2.5,2.7),('start',0,0),('end',0,0)])
def test_intersection_edges_caps_and_z_touch_fail(tmp_path,synthetic,x,y,z):
    cap=max(2,synthetic.c.front_extent_limit)
    if x=='start':x=-cap-.5
    if x=='end':x=synthetic.ref['measured_length_m']+cap+.5
    client=Client();p=prop(x=x,y=y,z=z)
    if z: p.bounding_box.location.z=0; p.bounding_box.extent.z=.6
    client.world.actors=[p]
    a,result=audit(tmp_path,synthetic,client)
    assert result['status']=='fail' and 'mesh bbox intersects' in result['reason']
    assert a.e.data['static_prop_geometry']['before'][0]['status']=='INTERSECT'
    assert not any(r['api']=='world.get_level_bbs' for r in a.reader.ledger)


def test_elevated_mesh_clear_by_existing_z_screen(tmp_path,synthetic):
    client=Client();client.world.actors=[prop(y=0,z=10)]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass'
    record=a.e.data['static_prop_geometry']['before'][0]
    assert record['xy_overlap'] and not record['z_overlap']


@pytest.mark.parametrize('actor_yaw,box_yaw',[(0,90),(90,0),(45,45)])
def test_rotations_cannot_be_ignored(tmp_path,synthetic,actor_yaw,box_yaw):
    client=Client();p=prop(y=4)
    transform=tf(50,4);transform.rotation.yaw=actor_yaw
    p.get_transform=lambda:transform
    p.bounding_box.extent.x=3;p.bounding_box.extent.y=.1
    p.bounding_box.rotation.yaw=box_yaw;client.world.actors=[p]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and a.e.data['static_prop_geometry']['before'][0]['status']=='INTERSECT'


@pytest.mark.parametrize('change',['bbox_missing','tf_missing','location_missing','rotation_missing',
    'extent_missing','zero','negative','nan','infinity','rotation_nan','bad_mesh','missing_mesh','bad_attribute',
    'collapsed','transform_nan','overflow'])
def test_unknown_geometry_never_filled_or_passed(tmp_path,synthetic,change):
    client=Client();p=prop()
    if change=='bbox_missing':del p.bounding_box
    if change=='tf_missing':p.get_transform=lambda:None
    if change=='location_missing':del p.bounding_box.location
    if change=='rotation_missing':del p.bounding_box.rotation.roll
    if change=='extent_missing':del p.bounding_box.extent.x
    if change=='zero':p.bounding_box.extent.x=0
    if change=='negative':p.bounding_box.extent.x=-1
    if change=='nan':p.bounding_box.location.x=math.nan
    if change=='infinity':p.bounding_box.extent.y=math.inf
    if change=='rotation_nan':p.bounding_box.rotation.pitch=math.nan
    if change=='bad_mesh':p.attributes['mesh_path']={}
    if change=='missing_mesh':p.attributes={}
    if change=='bad_attribute':p.attributes['mass']=42
    if change=='collapsed':p.get_transform=lambda:tf(1e100,1e100,1e100)
    if change=='transform_nan':p.get_transform=lambda:tf(math.nan,10,0)
    if change=='overflow':p.bounding_box.extent.x=1e308;p.bounding_box.location.x=1e308
    client.world.actors=[p]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and 'mesh geometry unavailable' in r['reason']
    record=a.e.data['static_prop_geometry']['before'][0]
    assert record['status']=='UNKNOWN' and 'reason' in record
    assert not any(r['api']=='map.get_waypoint_xodr' for r in a.reader.ledger)


def test_property_exception_poison_stops_following_getters(tmp_path,synthetic):
    class Broken:
        id=99;type_id='static.prop.mesh';attributes={'mesh_path':'/Game/Table'}
        def get_transform(self):return tf(50,10)
        @property
        def bounding_box(self):raise RuntimeError('RPC failed')
    client=Client();client.world.actors=[Broken(),prop(100)]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and a.reader.poisoned
    assert a.reader.ledger[-1]['api']=='prop.bounding_box'
    assert a.reader.ledger[-1]['status']=='failed'
    count=len(a.reader.ledger)
    with pytest.raises(AuditAbort):a.reader.attribute('prop',Broken(),'bounding_box')
    assert len(a.reader.ledger)==count


def test_transform_failure_poison_stops_property(tmp_path,synthetic):
    client=Client();p=prop()
    def fail():raise RuntimeError('failed transform')
    p.get_transform=fail;client.world.actors=[p]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and a.reader.poisoned
    assert a.reader.ledger[-1]['api']=='prop.get_transform'


@pytest.mark.parametrize('change',['move','rotate','bbox_rotate','extent','mesh','add','remove'])
def test_endpoint_change_invalidates_window(tmp_path,synthetic,change):
    client=Client();p=prop();end=copy.deepcopy(p)
    if change=='move':end.get_transform=lambda:tf(51,10)
    if change=='rotate':
        transform=tf(50,10);transform.rotation.yaw=1;end.get_transform=lambda:transform
    if change=='bbox_rotate':end.bounding_box.rotation.yaw=1
    if change=='extent':end.bounding_box.extent.x=.6
    if change=='mesh':end.attributes['mesh_path']='/Game/Other'
    calls=iter([[p],[] if change=='remove' else [end,prop(100)] if change=='add' else [end]])
    client.world.get_actors=lambda:next(calls)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'changed' in r['reason']


def test_order_changes_not_state_and_multiple_independent(tmp_path,synthetic):
    client=Client();p=prop(101);q=prop(99,x=70,mesh='Table')
    calls=iter([[p,q],[q,p]]);client.world.get_actors=lambda:next(calls)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass'
    assert [v['id'] for v in a.e.data['static_prop_geometry']['before']]==[99,101]


def test_110_props_57_traffic_full_mock_call_budget(tmp_path,synthetic):
    client=Client()
    traffic=[NS(id=i+1,type_id='traffic.traffic_light' if i<36 else 'traffic.speed_limit.30',
                attributes={},trigger_volume=box(x=0),get_transform=lambda:tf(50,50),
                get_state=lambda:'Red',get_stop_waypoints=lambda:[],get_affected_lane_waypoints=lambda:[])
             for i in range(57)]
    client.world.actors=traffic+[NS(id=58,type_id='spectator',attributes={})]+[prop(59+i) for i in range(110)]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass' and r['route_audit'] is None and r['physical_bounds'] is None
    assert len(a.e.data['actor_inventory']['before'])==168
    assert len(a.e.data['static_prop_geometry']['before'])==110
    assert len(a.reader.ledger)==1135
    assert sum(v['api']=='prop.bounding_box' for v in a.reader.ledger)==220


def test_other_unknown_actor_still_blocks_before_geometry(tmp_path,synthetic):
    client=Client();client.world.actors=[prop(),NS(id=100,type_id='static.prop.box01',attributes={})]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and 'unreviewed actor type' in r['reason']
    assert not any(v['api'].startswith('prop.') for v in a.reader.ledger)


def test_property_allowlist_rejects_before_access(tmp_path):
    class Never:
        def __getattr__(self,name):pytest.fail('unexpected property accessed')
    reader=Reader(Evidence(tmp_path,'SYNTHETIC',synthetic=True))
    with pytest.raises(AuditAbort):reader.attribute('prop',Never(),'destroy')
    assert reader.ledger==[]


def test_real_sdk_local_rotations_and_offset(monkeypatch):
    carla=pytest.importorskip('carla')
    from importlib.metadata import version
    if version('carla')!='0.9.16':pytest.skip('requires CARLA 0.9.16')
    monkeypatch.setattr(carla,'Client',lambda *a:pytest.fail('connection forbidden'))
    assert hasattr(carla.Actor,'get_transform')
    for angles in [(0,0,0),(30,20,10),(-90,45,-15),(180,-20,65)]:
        b=carla.BoundingBox(carla.Location(x=1.2,y=-.7,z=.3),carla.Vector3D(x=2,y=.4,z=.9))
        b.rotation=carla.Rotation(yaw=angles[0],pitch=angles[1],roll=angles[2])
        t=carla.Transform(carla.Location(x=40,y=15,z=2),carla.Rotation(yaw=35,pitch=-10,roll=22))
        expected=[(v.x,v.y,v.z) for v in b.get_world_vertices(t)]
        actual=box_vertices(b,t)
        assert all(min(math.dist(p,q) for q in expected)<2e-5 for p in actual)
        assert all(min(math.dist(p,q) for q in actual)<2e-5 for p in expected)
