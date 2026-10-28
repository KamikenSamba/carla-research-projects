"""Synthetic provenance tests; no assertion of real Actor52/sign binding."""
import hashlib
import json
import math

import pytest

from test_scene_audit import audit, synthetic
from test_scene_speed import setup_speed


def object_speed(synthetic):
    client,actor,mark=setup_speed(synthetic,unit='mph')
    mark.name='Speed_90';mark.sub_type='90';mark.country='OpenDRIVE';mark.is_dynamic=False
    xml=('<OpenDRIVE><road id="8" length="400"><signals/><objects>'
         '<object id="speed1" name="Speed_90" type="-1" s="280" orientation="+"/>'
         '</objects></road></OpenDRIVE>')
    set_xml(client,synthetic,xml)
    return client,actor,mark


def set_xml(client,synthetic,xml):
    client.world.map.to_opendrive=lambda:xml
    synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()


def test_object_contract_preserves_raw_values_and_physical_audit(tmp_path,synthetic):
    client,actor,mark=object_speed(synthetic)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass' and r['route_audit'] is None and r['physical_bounds'] is None
    ref=a.e.data['regulatory_elements']['observed']['traffic'][0]['reference']
    assert ref['value']==90 and ref['unit']=='mph'
    assert ref['limit_kmh']==pytest.approx(144.84096)
    assert ref['xml_source']['attributes']['name']=='Speed_90'
    assert 'unit' not in ref['xml_source']['attributes']
    internal=ref['internal_interpretation']
    assert internal['value']==90 and internal['unit']=='km/h'
    assert internal['measured_controller_value'] is None
    assert internal['actual_component_and_spawn_path']=='unknown'
    assert a.e.data['static_obstacles']['status']=='pass'
    assert a.e.data['traffic_window']['status']=='pass'
    saved=json.loads((a.e.directory/'regulatory_elements.json').read_text())['data']
    assert saved['observed']['traffic'][0]['reference']==json.loads(json.dumps(ref))


@pytest.mark.parametrize('field,value',[
    ('name','speed_90'),('sub_type','60'),('country','US'),('is_dynamic',True),
    ('is_dynamic',0),('unit','km/h'),('value',60.),('value',math.nan),
    ('value',math.inf),('value',None)])
def test_object_parser_metadata_must_match(tmp_path,synthetic,field,value):
    client,actor,mark=object_speed(synthetic);setattr(mark,field,value)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status'] in ('unknown','fail')
    assert not any(e['api']=='world.get_level_bbs' for e in a.reader.ledger)


@pytest.mark.parametrize('extra',['unit="mph"','unit="km/h"','value="90"','type="crosswalk"'])
def test_explicit_or_unknown_object_semantics_not_reinterpreted(tmp_path,synthetic,extra):
    client,actor,mark=object_speed(synthetic)
    xml=client.world.map.to_opendrive().replace('type="-1"',extra if extra.startswith('type=') else 'type="-1" '+extra)
    set_xml(client,synthetic,xml)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown'


@pytest.mark.parametrize('field',['name','sub_type','country','is_dynamic'])
def test_missing_object_provenance_is_not_defaulted(tmp_path,synthetic,field):
    client,actor,mark=object_speed(synthetic);delattr(mark,field)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='unknown'


@pytest.mark.parametrize('change',['no_match','ambiguous','wrong_type','wrong_lane'])
def test_object_semantics_never_replace_unique_actor_binding(tmp_path,synthetic,change):
    client,actor,mark=object_speed(synthetic)
    if change=='no_match':client.world.get_traffic_sign=lambda m:None
    if change=='ambiguous':client.world.map.get_all_landmarks=lambda:[mark,mark]
    if change=='wrong_type':actor.type_id='traffic.speed_limit.60'
    if change=='wrong_lane':mark.get_lane_validities=lambda:[(-3,-1)]
    a,r=audit(tmp_path,synthetic,client)
    assert r['status'] in ('unknown','fail')


@pytest.mark.parametrize('unit,value,status',[
    ('km/h',90.,'pass'),('m/s',25.,'pass'),('mph',90.,'fail'),
    ('mph',90./1.609344,'pass'),('unknown',90.,'unknown')])
def test_signal_unit_conversion_is_unchanged(tmp_path,synthetic,unit,value,status):
    client,actor,mark=setup_speed(synthetic,unit=unit,value=value)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']==status
    if status=='pass':
        ref=a.e.data['regulatory_elements']['observed']['traffic'][0]['reference']
        assert ref['internal_interpretation']['basis']=='ordinary_unit_conversion'
        assert ref['limit_kmh']==pytest.approx(90.)


def test_object_cap_uses_more_conservative_contract(tmp_path,synthetic):
    from types import SimpleNamespace
    client,actor,mark=object_speed(synthetic)
    synthetic.c=SimpleNamespace(**{**vars(synthetic.c),'speed_limit':30.})
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='fail' and 'cap exceeds' in r['reason']


@pytest.mark.parametrize('obstacle',['level','environment','stop','getter'])
def test_object_does_not_bypass_other_gates(tmp_path,synthetic,obstacle):
    from types import SimpleNamespace
    from test_scene_audit import box
    client,actor,mark=object_speed(synthetic)
    if obstacle=='level':client.world.boxes=[box(x=50,y=0)]
    if obstacle=='environment':client.world.objects=[SimpleNamespace(id=99,type='TrafficSigns',bounding_box=box(x=50,y=0))]
    if obstacle=='stop':actor.type_id='traffic.stop'
    if obstacle=='getter':
        def fail(*args):raise RuntimeError('synthetic RPC error')
        client.world.get_traffic_sign=fail
    a,r=audit(tmp_path,synthetic,client)
    assert r['status'] in ('unknown','fail')
    if obstacle=='getter':assert a.reader.poisoned and a.reader.ledger[-1]['status']=='failed'
