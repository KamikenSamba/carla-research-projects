"""Signed-scale world descriptors; props retain their independent strict gate."""
import copy
import json
import math
from types import SimpleNamespace as NS

import pytest

from idm_brake_route import box_vertices
from idm_scene_bbox import raw_bbox,obstacle_vertices
from test_scene_audit import audit,box,synthetic,Client


@pytest.mark.parametrize('source',['level_bbs','environment_objects'])
@pytest.mark.parametrize('signs',[(1,1,1),(-1,1,1),(1,-1,1),(1,1,-1),(-1,-1,-1)])
def test_signed_corner_set_is_retained(source,signs):
    b=box(x=20,y=30);b.extent=NS(x=signs[0]*3,y=signs[1]*2,z=signs[2])
    b.rotation=NS(yaw=31.,pitch=4.,roll=-3.)
    before=copy.deepcopy(vars(b.extent));descriptor=raw_bbox(b)
    vertices,info=obstacle_vertices(descriptor,source)
    canonical=copy.deepcopy(b);canonical.extent=NS(x=3,y=2,z=1)
    assert vertices==box_vertices(canonical)
    assert vars(b.extent)==before and info['signed']==any(x<0 for x in signs)
    assert descriptor['fields']['extent']['fields']['x']['value']==signs[0]*3


@pytest.mark.parametrize('group',['location','extent','rotation'])
def test_missing_group_unknown(group):
    b=box();delattr(b,group);d=raw_bbox(b)
    assert d['fields'][group]['present'] is False
    with pytest.raises(ValueError,match='unknown obstacle'):obstacle_vertices(d,'level_bbs')


@pytest.mark.parametrize('group,field',[('location','x'),('extent','y'),('rotation','yaw')])
@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf,None,'bad',True])
def test_bad_attributes_are_recorded_and_refused(group,field,value):
    b=box();setattr(getattr(b,group),field,value);d=raw_bbox(b)
    encoded=json.dumps(d,allow_nan=False)
    assert encoded and d['fields'][group]['fields'][field]['finite'] is False
    with pytest.raises(ValueError,match='unknown obstacle'):obstacle_vertices(d,'environment_objects')


def test_attribute_access_exception_safe():
    class Bad:
        @property
        def extent(self):raise RuntimeError('synthetic missing descriptor')
    d=raw_bbox(Bad());json.dumps(d,allow_nan=False)
    with pytest.raises(ValueError):obstacle_vertices(d,'level_bbs')


def test_zero_extent_is_explicit():
    b=box();b.extent=NS(x=0,y=0,z=0)
    with pytest.raises(ValueError,match='all-zero'):obstacle_vertices(raw_bbox(b),'level_bbs')
    b.extent.y=2;b.extent.x=3
    vertices,_=obstacle_vertices(raw_bbox(b),'level_bbs')
    assert len(vertices)==8  # Planar bounds still participate in XY/Z gates.


def test_shared_vehicle_prop_validator_still_rejects_negative():
    b=box();b.extent.x=-1
    with pytest.raises(ValueError,match='unknown bounding box'):box_vertices(b)
    with pytest.raises(ValueError,match='world obstacle source'):obstacle_vertices(raw_bbox(b),'prop')


@pytest.mark.parametrize('intersect',[False,True])
def test_multiple_building_descriptors_do_not_skip_collision(tmp_path,synthetic,intersect):
    client=Client();clear=box(x=-50,y=0);clear.extent.x=-1
    other=box(x=50 if intersect else -60,y=0);other.extent.y=-1
    client.world.get_level_bbs=lambda label:[clear,other] if label=='Buildings' else []
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']==('fail' if intersect else 'pass')
    records=a.e.data['static_obstacles']['observed']
    assert len(records)==2 and all(x['extent_interpretation']['signed'] for x in records)
    raw=[json.loads(x) for x in (a.e.directory/'obstacle_bbox_raw.jsonl').read_text().splitlines()]
    assert len(raw)==2 and raw[1]['index']==1


def test_invalid_building_keeps_raw_and_stops_following_getters(tmp_path,synthetic):
    client=Client();b=box();b.extent.x=math.nan
    client.world.get_level_bbs=lambda label:[b] if label=='Buildings' else []
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown'
    record=a.e.data['static_obstacles']['observed'][-1]
    assert record['status']=='UNKNOWN' and record['index']==0
    assert record['descriptor']['fields']['extent']['fields']['x']['value']['special']=='NaN'
    assert a.reader.ledger[-1]['api']=='world.get_level_bbs'
    assert 'Buildings' in a.reader.ledger[-1]['args']
    assert a.e.data['static_obstacles']['counts']['Buildings']==1


def test_real_0916_signed_bbox_sdk_vertices(monkeypatch):
    carla=pytest.importorskip('carla')
    from importlib.metadata import version
    if version('carla')!='0.9.16':pytest.skip('requires CARLA0.9.16')
    monkeypatch.setattr(carla,'Client',lambda *a:pytest.fail('Client forbidden offline'))
    b=carla.BoundingBox(carla.Location(x=20,y=30,z=3),carla.Vector3D(x=-4,y=2,z=3))
    b.rotation=carla.Rotation(yaw=-89.9,pitch=2,roll=3)
    vertices,_=obstacle_vertices(raw_bbox(b),'level_bbs')
    expected=[(v.x,v.y,v.z) for v in b.get_world_vertices(carla.Transform())]
    assert max(min(math.dist(v,q) for q in expected) for v in vertices)<1e-4
