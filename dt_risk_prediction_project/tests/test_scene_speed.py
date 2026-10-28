"""Server-free regulation tests. No measured actor transform/local box is invented."""
import copy
import hashlib
import json
import math
from types import SimpleNamespace as NS

import pytest

from idm_scene_audit import speed_state
from test_scene_audit import Client, audit, box, synthetic, tf


def setup_speed(synthetic, *, overlap=True, unit='km/h', value=90.):
    client=Client();w=client.world
    # ID/type and approximate long trigger dimensions are based on saved ID52.
    # All transforms, local box fields, SDK sign association, and km/h landmark
    # below are SYNTHETIC. Real local transform/box/link were not saved; the
    # installed Town01 map declares mph, tested separately as a mismatch.
    a=NS(id=52,type_id='traffic.speed_limit.90',attributes={},get_transform=lambda:tf(-30,5),
         trigger_volume=box(x=128,y=-5 if overlap else 50,z=.42))
    a.trigger_volume.extent=NS(x=122.75,y=1.05,z=.30)
    mark=NS(id='speed1',type='274',road_id=8,s=280.,value=value,unit=unit,orientation='Positive',
            transform=tf(-30,5),get_lane_validities=lambda:[(1,3)])
    xml=(f'<OpenDRIVE><road id="8" length="400"><signals><signal id="speed1" s="280" '
         f'type="274" value="{value}" unit="{unit}" orientation="+">'
         '<validity fromLane="1" toLane="3"/></signal></signals><objects/></road></OpenDRIVE>')
    w.map.to_opendrive=lambda:xml
    synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    w.map.get_all_landmarks=lambda:[mark]
    w.get_traffic_sign=lambda m:a
    w.actors=[a]
    return client,a,mark


def test_id52_equivalent_overlap_is_regulation_not_physical_obstacle(tmp_path,synthetic):
    client,actor,mark=setup_speed(synthetic)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass' and r['route_audit'] is None and r['physical_bounds'] is None
    record=a.e.data['regulatory_elements']['observed']['traffic'][0]
    assert record['classification']=='speed_limit_application_region'
    assert record['route_xy_overlap'] and record['assessment']=='validated_speed_regulation'
    assert record['reference']['limit_kmh']==90 and record['reference']['calibration_speed_cap_mps']==1.2
    assert record['reference']['lane_validities']==[(1,3)]
    assert record['reference']['orientation']=='Positive'
    assert a.e.data['static_obstacles']['status']=='pass'  # No bypass of physical inventory.
    assert a.e.data['traffic_window']['status']=='pass'
    methods=[e['api'] for e in a.reader.ledger]
    assert methods.count('world.get_traffic_sign')==methods.count('landmark.get_lane_validities')==1
    assert methods.count('world.get_level_bbs')==28
    assert methods.count('world.get_environment_objects')==1
    data=json.loads((a.e.directory/'regulatory_elements.json').read_text())['data']
    assert data['status']=='pass' and data['observed']['traffic'][0]['reference']['limit_kmh']==90


def test_far_valid_speed_has_no_reference_lookup(tmp_path,synthetic):
    client,_,_=setup_speed(synthetic,overlap=False)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass'
    assert a.e.data['regulatory_elements']['observed']['traffic'][0]['assessment']=='outside_audit_corridor'
    assert not any(e['api']=='world.get_traffic_sign' for e in a.reader.ledger)


def test_multiple_speed_regions_share_reference_cache(tmp_path,synthetic):
    client,p,mark=setup_speed(synthetic)
    q=copy.copy(p);q.id=53
    mark2=copy.copy(mark);mark2.id='speed2'
    xml=client.world.map.to_opendrive().replace('</signals>',
        '<signal id="speed2" s="280" type="274" value="90" unit="km/h" orientation="+">'
        '<validity fromLane="1" toLane="3"/></signal></signals>')
    client.world.map.to_opendrive=lambda:xml
    synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    client.world.map.get_all_landmarks=lambda:[mark,mark2]
    client.world.get_traffic_sign=lambda m:p if m.id=='speed1' else q
    client.world.actors=[q,p]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass'
    assert sum(e['api']=='world.get_traffic_sign' for e in a.reader.ledger)==2
    assert len(a.e.data['regulatory_elements']['observed']['traffic'])==2


@pytest.mark.parametrize('change',['missing','position_nan','extent_inf','zero','negative','missing_rotation','rotation_nan'])
def test_bad_speed_trigger_is_unknown(tmp_path,synthetic,change):
    client,p,_=setup_speed(synthetic)
    if change=='missing':del p.trigger_volume
    if change=='position_nan':p.trigger_volume.location.x=math.nan
    if change=='extent_inf':p.trigger_volume.extent.x=math.inf
    if change=='zero':p.trigger_volume.extent.z=0
    if change=='negative':p.trigger_volume.extent.x=-1
    if change=='missing_rotation':del p.trigger_volume.rotation
    if change=='rotation_nan':p.trigger_volume.rotation.roll=math.nan
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown'
    assert not any(e['api']=='world.get_level_bbs' for e in a.reader.ledger)


@pytest.mark.parametrize('type_id',['traffic.speed_limit.0','traffic.speed_limit.-90','traffic.speed_limit.90.0',
    'traffic.speed_limit.nan','traffic.speed_limit.999','traffic.unknown','traffic.stop.invalid'])
def test_unknown_or_invalid_traffic_type_not_allowed(tmp_path,synthetic,type_id):
    client,p,_=setup_speed(synthetic);p.type_id=type_id
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and 'unknown traffic regulation' in r['reason']


@pytest.mark.parametrize('type_id',['traffic.stop','traffic.yield','traffic.traffic_light'])
def test_non_speed_overlapping_trigger_still_refused(tmp_path,synthetic,type_id):
    client,p,_=setup_speed(synthetic);p.type_id=type_id
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and r['reason']=='traffic trigger overlaps route'
    assert not any(e['api']=='world.get_traffic_sign' for e in a.reader.ledger)


@pytest.mark.parametrize('change',['no_match','ambiguous','wrong_lane','wrong_road','bad_ranges','bad_unit',
    'nan_value','negative_value','value_mismatch','xml_unit','xml_position','xml_orientation','xml_validity','xml_missing'])
def test_speed_reference_missing_or_inconsistent_never_passes(tmp_path,synthetic,change):
    client,p,mark=setup_speed(synthetic)
    if change=='no_match':client.world.get_traffic_sign=lambda m:None
    if change=='ambiguous':client.world.map.get_all_landmarks=lambda:[mark,mark]
    if change=='wrong_lane':mark.get_lane_validities=lambda:[(-3,-1)]
    if change=='wrong_road':mark.road_id=9
    if change=='bad_ranges':mark.get_lane_validities=lambda:[(0,3)]
    if change=='bad_unit':mark.unit='unknown'
    if change=='nan_value':mark.value=math.nan
    if change=='negative_value':mark.value=-90
    if change=='value_mismatch':mark.value=60
    replacements={'xml_unit':('unit="km/h"','unit="mph"'),'xml_position':('s="280"','s="281"'),
        'xml_orientation':('orientation="+"','orientation="-"'),
        'xml_validity':('toLane="3"','toLane="2"'),'xml_missing':('id="speed1"','id="missing"')}
    if change in replacements:
        old,new=replacements[change];xml=client.world.map.to_opendrive().replace(old,new)
        client.world.map.to_opendrive=lambda:xml
        synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    a,r=audit(tmp_path,synthetic,client)
    assert r['status'] in ('fail','unknown')
    assert not any(e['api']=='world.get_level_bbs' for e in a.reader.ledger)


def test_mph_90_is_not_silently_kmh_90(tmp_path,synthetic):
    # Real local Town01 metadata declares mph; this is not evidence of ID52 association.
    client,p,mark=setup_speed(synthetic,unit='mph')
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'unit-normalized limit mismatch' in r['reason']
    record=a.e.data['regulatory_elements']['observed']['traffic'][0]
    assert record['reference']['unit']=='mph' and record['reference']['limit_kmh']==pytest.approx(144.84096)


@pytest.mark.parametrize('name',['Speed_90','Speed_60','Unknown'])
def test_roadrunner_object_never_relabels_mph_as_kmh(tmp_path,synthetic,name):
    # Incomplete provenance stays rejected. The separate unit-contract tests
    # require the complete 0.9.16 object/parser tuple and preserve raw mph.
    client,p,mark=setup_speed(synthetic,unit='mph')
    xml=(f'<OpenDRIVE><road id="8" length="400"><signals/><objects>'
         f'<object id="speed1" name="{name}" s="280" orientation="+"/>'
         '</objects></road></OpenDRIVE>')
    client.world.map.to_opendrive=lambda:xml
    synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    a,r=audit(tmp_path,synthetic,client)
    assert r['status'] in ('fail','unknown')
    assert ('provenance' if name=='Speed_90' else 'object/landmark') in r['reason']


@pytest.mark.parametrize('field,value',[('value',None),('value','90'),('value',math.inf),
    ('s',math.inf),('orientation','Unknown'),('unit',None)])
def test_missing_or_invalid_landmark_metadata(tmp_path,synthetic,field,value):
    client,_,mark=setup_speed(synthetic);setattr(mark,field,value)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown'


def test_metres_per_second_reference_is_converted(tmp_path,synthetic):
    client,_,_=setup_speed(synthetic,unit='m/s',value=25.)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass'
    assert a.e.data['regulatory_elements']['observed']['traffic'][0]['reference']['limit_kmh']==90


def test_downstream_sign_cannot_certify_decreasing_s_route(tmp_path,synthetic):
    client,_,mark=setup_speed(synthetic)
    mark.s=10.
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and 'not upstream' in r['reason']


def test_speed_trigger_is_read_once_per_endpoint(tmp_path,synthetic):
    client,p,mark=setup_speed(synthetic)
    class Counted:
        id=p.id;type_id=p.type_id;attributes=p.attributes;reads=0
        def get_transform(self):return p.get_transform()
        @property
        def trigger_volume(self):self.reads+=1;return p.trigger_volume
    actor=Counted();client.world.actors=[actor];client.world.get_traffic_sign=lambda m:actor
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass' and actor.reads==2


def test_vehicle_speed_cap_not_adopted_from_sign(tmp_path,synthetic):
    client,_,_=setup_speed(synthetic)
    synthetic.c=NS(**{**vars(synthetic.c),'speed_limit':30.})  # Synthetic invalid future config, no control call.
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'cap exceeds' in r['reason']


@pytest.mark.parametrize('method',['lookup','validity'])
def test_reference_rpc_or_local_getter_failure_poisoned(tmp_path,synthetic,method):
    client,p,mark=setup_speed(synthetic)
    def fail(*args):raise RuntimeError('synthetic getter failure')
    if method=='lookup':client.world.get_traffic_sign=fail
    else:mark.get_lane_validities=fail
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown' and a.reader.poisoned
    assert a.reader.ledger[-1]['status']=='failed'


@pytest.mark.parametrize('change',['trigger_extent','trigger_rotation','position','actor_set','type_id'])
def test_speed_state_change_cannot_pass_window(tmp_path,synthetic,change):
    client,p,mark=setup_speed(synthetic)
    if change=='actor_set':
        calls=iter([[p],[]]);client.world.get_actors=lambda:next(calls)
    elif change=='type_id':
        q=copy.copy(p);q.type_id='traffic.speed_limit.60'
        calls=iter([[p],[q]]);client.world.get_actors=lambda:next(calls)
    else:
        n=[0]
        def transform():
            n[0]+=1
            if n[0]==2:
                if change=='trigger_extent':p.trigger_volume.extent.x+=1
                if change=='trigger_rotation':p.trigger_volume.rotation.yaw+=.1
            return tf(-30+(1 if n[0]==2 and change=='position' else 0),5)
        p.get_transform=transform
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'changed' in r['reason']


@pytest.mark.parametrize('source',['level','environment'])
def test_accepted_speed_does_not_bypass_physical_sign_or_other_obstacle(tmp_path,synthetic,source):
    client,_,_=setup_speed(synthetic)
    if source=='level':client.world.boxes=[box(x=50,y=0)]
    else:client.world.objects=[NS(id=9,type='TrafficSigns',bounding_box=box(x=50,y=0))]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'static obstacle' in r['reason']


def test_real_sdk_speed_trigger_transform_and_readonly_api(monkeypatch):
    carla=pytest.importorskip('carla')
    from importlib.metadata import version
    if version('carla')!='0.9.16':pytest.skip('requires CARLA 0.9.16')
    monkeypatch.setattr(carla,'Client',lambda *a:pytest.fail('connection forbidden'))
    assert hasattr(carla.World,'get_traffic_sign') and hasattr(carla.Landmark,'get_lane_validities')
    b=carla.BoundingBox(carla.Location(x=1,y=2,z=.3),carla.Vector3D(x=100,y=1,z=.4))
    b.rotation=carla.Rotation(yaw=20,pitch=2,roll=3)
    transform=carla.Transform(carla.Location(x=20,y=50,z=0),carla.Rotation(yaw=30,pitch=4,roll=-2))
    state=speed_state(NS(type_id='traffic.speed_limit.90',trigger_volume=b),transform)
    expected=[(v.x,v.y,v.z) for v in b.get_world_vertices(transform)]
    assert all(min(math.dist(v,p) for p in expected)<3e-5 for v in state['trigger_vertices'])
