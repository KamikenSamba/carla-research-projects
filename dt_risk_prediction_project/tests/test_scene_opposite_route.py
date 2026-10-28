"""Fixed, independently pinned opposite-lane route. No simulator required."""
from dataclasses import asdict
import copy
import hashlib
import json
from types import SimpleNamespace as NS

import pytest

from idm_brake_config import town01_config,town01_opposite_config,BrakeConfig
from idm_brake_route import geometry_sha256,validate_review_geometry
from idm_scene_reference import GEOMETRY_SHA256,OPPOSITE_GEOMETRY_SHA256,read_policy
from test_scene_audit import audit,synthetic,Client,tf
from test_scene_speed import setup_speed


def opposite(synthetic):
    c=town01_opposite_config();synthetic.c=c
    points=[[i*c.planned_route_length/112,0.,0.] for i in range(113)]
    poses=[dict(road=8,section=0,lane=-1,s=s,yaw=0.,pitch=0.,roll=0.,width=4.) for s in c.route_s_samples]
    synthetic.ref.update(points=points,waypoints=poses,measured_length_m=c.planned_route_length)
    class Waypoint:
        road_id,section_id,lane_id,lane_width,is_junction,lane_type=8,0,-1,4.,False,'Driving'
        def __init__(self,s):self.s=s;self.transform=tf((s-135)*c.planned_route_length/112,0)
        def next(self,d):return [Waypoint(self.s+d)]
    return Waypoint


def test_policy_preserves_all_controls_and_gates(tmp_path):
    old,new=asdict(town01_config()),asdict(town01_opposite_config())
    differences={k for k in old if old[k]!=new[k]}
    assert differences=={'mode','lane','follower_s','end_s','planned_route_length'}
    c=town01_opposite_config();assert list(c.route_s_samples)==list(range(135,248))
    assert c.route_point_count==113 and list(town01_config().route_s_samples)==list(range(250,54,-1))
    p=tmp_path/'config.json';p.write_text(json.dumps(new));assert read_policy(p)==c
    with pytest.raises(ValueError,match='unaudited'):c.require_executable('brake_calibration')


@pytest.mark.parametrize('field,value',[('road',6),('lane',1),('follower_s',134),('end_s',248),
    ('map_name','Town02_Opt'),('speed_limit',1.3),('throttle_cap',.2),('dt',.1)])
def test_new_mode_does_not_open_arbitrary_routes_or_controls(field,value):
    payload=asdict(town01_opposite_config());payload[field]=value
    with pytest.raises(ValueError):BrakeConfig(**payload)


def test_independent_geometry_hash_and_spacing(synthetic):
    opposite(synthetic);c=synthetic.c
    data=dict(points=synthetic.ref['points'],waypoints=synthetic.ref['waypoints'])
    data['geometry_sha256']=geometry_sha256(data['points'],data['waypoints'])
    validate_review_geometry(c,data)
    assert GEOMETRY_SHA256!=OPPOSITE_GEOMETRY_SHA256
    data['waypoints'][0]['lane']=1
    with pytest.raises(ValueError):validate_review_geometry(c,data)


def test_mock_113_waypoint_increasing_s_full_audit(tmp_path,synthetic):
    Waypoint=opposite(synthetic);client=Client()
    client.world.map.get_waypoint_xodr=lambda road,lane,s:Waypoint(s) if (road,lane)==(8,-1) else None
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']=='pass' and r['route_audit'] is None and r['physical_bounds'] is None
    assert a.e.data['route_geometry']['point_count']==113
    apis=[x['api'] for x in a.reader.ledger]
    assert apis.count('map.get_waypoint_xodr')==113 and apis.count('waypoint.next')==112
    assert apis.count('world.get_level_bbs')==28 and apis.count('world.get_environment_objects')==1


@pytest.mark.parametrize('s,expected',[(20.,'pass'),(280.,'fail')])
def test_increasing_s_speed_reference_requires_upstream_sign(tmp_path,synthetic,s,expected):
    Waypoint=opposite(synthetic);client,actor,mark=setup_speed(synthetic)
    mark.s=s;mark.orientation='Negative';mark.get_lane_validities=lambda:[(-3,-1)]
    xml=client.world.map.to_opendrive().replace('s="280"',f's="{s}"').replace('orientation="+"','orientation="-"').replace('fromLane="1" toLane="3"','fromLane="-3" toLane="-1"')
    client.world.map.to_opendrive=lambda:xml;synthetic.ref['opendrive_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    client.world.map.get_waypoint_xodr=lambda road,lane,s:Waypoint(s)
    a,r=audit(tmp_path,synthetic,client)
    assert r['status']==expected or (expected=='fail' and r['status']=='unknown')
    if expected=='pass':assert a.e.data['regulatory_elements']['observed']['traffic'][0]['reference']['route_s_direction']=='increasing'


def test_distance_sensitivity_is_assumptions_only():
    r=town01_opposite_config().distance_report()
    assert r['required_route_m']==pytest.approx(43.34011)
    assert r['sensitivity'][-1]['required_route_m']==pytest.approx(109.00511)
    assert r['sensitivity'][-1]['margin_m']>2.9
    assert r['accepted_braking_lower_bound'] is None and r['accepted_response_upper_bound'] is None
