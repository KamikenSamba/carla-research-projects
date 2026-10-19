"""Normal-speed policy tests. All route coordinates and motion here are synthetic."""
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from idm_experiment_harness import Harness
from idm_harness_config import load_scenario
from idm_normal_stage1 import NormalStage1, NormalSafety, stopping_distance
from idm_history import read_history
from Run_IDM_Following_Experiment import main
from test_idm_harness import World, Gate, Ownership, csv_rows
from test_idm_context import vector


def reviewed_config():
    # Test fixture only: never describes Town10HD geometry or real physical validation.
    c=NormalStage1(road=10,section=0,lane=1,follower_s=120,leader_s=98,end_s=-80)
    a=dict(policy_sha256=c.policy_sha256,opendrive_sha256=hashlib.sha256(b'synthetic').hexdigest(),
           evidence_sha256='f'*64,reviewed_by='TEST FIXTURE',reviewed_at='TEST ONLY',
           evidence_reference='synthetic offline fixture',carla_version='0.9.16',map_name=c.map_name,
           physical_bounds=dict(braking_lower_bound_mps2=2.0,response_upper_bound_s=0.3,
                                evidence_reference='synthetic test assumption'),
           points=[[i,0,0] for i in range(201)])
    return replace(c,route_audit=a)


def normal_run(directory,fault=None,at=75):
    w=World();w.map.to_opendrive=lambda:'synthetic';gate=Gate();c=reviewed_config()
    original=w.tick
    def tick():
        # Piecewise kinematic fixture for contract testing, NOT a TM/physics model.
        positions={a.id:a.x for a in w.actors[1:]}
        result=original()
        for a in w.actors[1:]:
            if getattr(a,'registered',False):
                a.speed=4.5 if a.id==11 and w.frame<=61 else 4.0
                a.x=positions[a.id]+a.speed*c.dt
            a.get_acceleration=lambda:vector()
        if w.frame==at:
            f=w.actors[1]
            if fault=='peak':f.get_acceleration=lambda:vector(5.01,0,0)
            elif fault=='unknown':f.get_acceleration=lambda:None
            elif fault=='getter':
                f.get_acceleration=lambda:(_ for _ in ()).throw(RuntimeError('synthetic getter lost'))
            elif fault=='sustained':w.accel_active=True
            elif fault=='gap':w.actors[2].x=f.x+8
            elif fault=='offlane':f.y=5
            elif fault=='server':raise RuntimeError('synthetic RPC lost')
            elif fault=='interleave':w.frame+=1
            elif fault=='speed':f.speed=5.01
            elif fault=='settling':f.speed=4.0
        if getattr(w,'accel_active',False):w.actors[1].get_acceleration=lambda:vector(-3.01,0,0)
        return result
    w.tick=tick
    client=NS(get_world=lambda:w,get_client_version=lambda:'0.9.16',get_server_version=lambda:'0.9.16',
              get_trafficmanager=lambda port:w.tm)
    api=NS(CityObjectLabel=NS(**{k:k for k in ('Car','Truck','Bus','Motorcycle','Bicycle','Pedestrians')}))
    h=Harness(client,api,c,'stage1',directory,gate,Ownership(w))
    return h.run(),w,gate,h


def test_default_config_is_draft_and_refuses_connection(tmp_path,monkeypatch,capsys):
    import Run_IDM_Following_Experiment as entry
    config=Path(__file__).parents[1]/'configs/idm_normal_stage1.json'
    c=load_scenario(config)
    assert c.route_audit is None and c.road is None
    assert main(['--config',str(config)])==0
    assert json.loads(capsys.readouterr().out)['executable_route'] is False
    monkeypatch.setattr(entry,'process_alive',lambda *_:pytest.fail('OS/server checks must not be reached'))
    with pytest.raises(SystemExit):main(['--config',str(config),'--execute','--server-pid','1'])
    client=NS(get_world=lambda:pytest.fail('CARLA connection forbidden'))
    h=Harness(client,None,c,'stage1',tmp_path,Gate(),None)
    result=h.run()
    assert result['status']=='exception' and 'unaudited' in result['reason']
    assert result['cleanup']['complete']


@pytest.mark.parametrize('change',[
    dict(dt=0.1),dict(initial_spacing=12),dict(planned_route_length=199),dict(end_clearance=19),
    dict(ready_seconds=1.95),dict(ready_speed_tolerance=0.3),dict(acceleration_peak=6),
    dict(acceleration_sustained=4),dict(assumed_braking=3),dict(reaction_seconds=0.2),
    dict(lateral_limit=0.5),dict(minimum_valid_ratio=0.8),dict(minimum_ttc=0.5),
    dict(warmup_seconds=None),dict(seed=True),dict(follower_s=120),dict(road=10.0),
    dict(acceleration_peak=float('nan')),dict(follower_speed=5.0)])
def test_invalid_or_relaxed_configuration_rejected(change):
    with pytest.raises(ValueError):replace(NormalStage1(),**change)


@pytest.mark.parametrize('initialization',['single_velocity','manual_runup','unknown'])
def test_unverified_initializations_are_never_executable(initialization):
    c=replace(NormalStage1(),initialization=initialization)
    with pytest.raises(ValueError,match='initialization'):c.require_executable('stage1')


@pytest.mark.parametrize('change',[
    dict(policy_sha256='0'*64),dict(reviewed_by=''),dict(opendrive_sha256=None),
    dict(evidence_reference=''),dict(points=[[0,0,0]]),dict(physical_bounds={}),dict(physical_bounds=None),dict(points=None),
    dict(physical_bounds=dict(braking_lower_bound_mps2=1,response_upper_bound_s=0.3,evidence_reference='test')),
    dict(physical_bounds=dict(braking_lower_bound_mps2=2,response_upper_bound_s=0.5,evidence_reference='test'))])
def test_incomplete_or_mismatched_review_rejected(change):
    c=reviewed_config();a=dict(c.route_audit,**change)
    with pytest.raises(ValueError):replace(c,route_audit=a)


def test_stage2_and_reused_policy_review_are_rejected():
    c=reviewed_config()
    with pytest.raises(ValueError):c.require_executable('stage2')
    with pytest.raises(ValueError):replace(c,end_clearance=21)


def test_distance_budget_includes_acceleration_and_sensitivity():
    c=NormalStage1();r=c.distance_report()
    assert stopping_distance(5,0.3,2,5)==pytest.approx(12.2875)
    assert r['required_route_m']==pytest.approx(142.42511)
    assert c.clearance_budget==pytest.approx(77.57489)
    assert c.initial_gap==pytest.approx(17.14978)
    weak=next(v for v in r['sensitivity'] if v['delay_s']==0.3 and v['braking_mps2']==1)
    assert weak['stopping_m']==pytest.approx(22.85)
    assert not weak['gap_pass'] and not weak['endpoint_pass']
    assert stopping_distance(5,0.5,2,5)+2>c.initial_gap
    for invalid in ((5,0,0,5),(float('nan'),0.3,2,5),(5,-1,2,5)):
        with pytest.raises(ValueError):stopping_distance(*invalid)


@pytest.mark.parametrize('values,expected',[
    ([5.01],'acceleration_peak'),([-5.01],'acceleration_peak'),([5.0,0],None),
    ([3.01,3.01],'acceleration_sustained'),([-3.01,-3.01],'acceleration_sustained'),
    ([3,3],None),([4,0,4],None),([None],'acceleration_unknown'),
    ([float('nan')],'acceleration_unknown'),([True],'acceleration_unknown')])
def test_acceleration_thresholds_and_contiguity(values,expected):
    g=NormalSafety(NormalStage1());reason=None
    for frame,a in enumerate(values):
        reason,_=g.acceleration_reason([dict(actor_id=11,a_long_mps2=a)],frame)
        if reason:break
    assert reason==expected


def ready_rows():
    common=dict(a_long_mps2=0,road_id=10,section_id=0,lane_id=1,heading_error_deg=0,route_lateral_m=0)
    return [dict(common,source_actor_id=11,speed_mps=4.5,leader_id=12,gap_m=17,relative_speed_mps=0.5),
            dict(common,source_actor_id=12,speed_mps=4.0)]


@pytest.mark.parametrize('fault',['speed','acceleration','gap','leader','lane','frame','heading'])
def test_ready_requires_full_contiguous_window_and_stability(fault):
    g=NormalSafety(NormalStage1())
    for frame in range(40):assert not g.ready(ready_rows(),frame)
    rows=ready_rows()
    if fault=='speed':rows[0]['speed_mps']=4.29
    elif fault=='acceleration':rows[1]['a_long_mps2']=0.51
    elif fault=='gap':rows[0]['gap_m']=19
    elif fault=='leader':rows[0]['leader_id']=99
    elif fault=='lane':rows[0]['lane_id']=2
    elif fault=='heading':rows[0]['heading_error_deg']=6
    frame=41 if fault=='frame' else 40
    assert not g.ready(rows,frame)
    first_ready=frame+40 if fault=='frame' else frame+41
    for i in range(frame+1,first_ready):assert not g.ready(ready_rows(),i)
    assert g.ready(ready_rows(),first_ready)


def test_ready_boundary_2_seconds_not_40_snapshots():
    g=NormalSafety(NormalStage1());rows=ready_rows()
    rows[0].update(speed_mps=4.3,a_long_mps2=0.5)
    for frame in range(40):assert not g.ready(rows,frame)
    assert g.ready(rows,40)


def test_normal_e2e_frames_cleanup_and_history_compatibility(tmp_path):
    result,w,gate,h=normal_run(tmp_path)
    assert result['status']=='completed',result
    datasets=[csv_rows(tmp_path/name) for name in ('capture_states.csv','following_context.csv','control_audit.csv')]
    keys=lambda rows:[(r['logical_actor_id'],r['carla_frame']) for r in rows]
    assert all(len(rows)==600 for rows in datasets)
    assert keys(datasets[0])==keys(datasets[1])==keys(datasets[2])
    assert len(read_history(tmp_path/'capture_states.csv',tmp_path/'following_context.csv'))==600
    assert all(int(r['command_based_on_frame'])+1==int(r['control_read_at_frame'])==int(r['carla_frame']) for r in datasets[2])
    assert w.frame==361 and w.tm.cycles==361 # 20 physical settle + 41 ready + 300 capture
    assert vars(w.settings)==vars(w.original) and w.actors==[w.external]
    assert result['cleanup']['complete'] and not gate.poisoned
    raw=[json.loads(line) for line in (tmp_path/'acceleration_observations.jsonl').read_text().splitlines()]
    assert len(raw)==w.frame and all(len(v['rows'])==2 for v in raw)
    assert [v['carla_frame'] for v in raw]==list(range(1,362))


@pytest.mark.parametrize('fault,at,reason',[
    ('peak',1,'acceleration_peak'),('unknown',1,'acceleration_unknown'),
    ('peak',25,'acceleration_peak'),('sustained',30,'acceleration_sustained'),
    ('unknown',75,'acceleration_unknown'),('peak',75,'acceleration_peak'),
    ('gap',75,'dynamic_gap_stopping_margin'),('offlane',75,'off_driving_lane'),
    ('speed',75,'speed_envelope')])
def test_aborts_in_settle_warmup_and_capture_preserve_both_actors(tmp_path,fault,at,reason):
    result,w,gate,h=normal_run(tmp_path,fault,at)
    assert result['status']=='safe_early_stop' and result['reason']==reason,result
    assert result['cleanup']['complete'] and w.actors==[w.external]
    raw=json.loads((tmp_path/'last_observation.json').read_text())
    assert len(raw['rows'])==2 and raw['frame_validated']
    assert w.frame==at+(fault=='sustained')
    if fault in ('peak','unknown','sustained'):
        decision=json.loads((tmp_path/'safety_decision.json').read_text())
        assert decision['frame']==w.frame and decision['reason']==reason


@pytest.mark.parametrize('fault',['server','interleave'])
def test_normal_abnormal_end_does_not_run_cleanup_rpc(tmp_path,fault):
    result,w,gate,h=normal_run(tmp_path,fault)
    assert result['status']==('server_unavailable' if fault=='server' else 'cleanup_incomplete')
    assert not any(e[0]=='destroy' for e in w.events)
    if fault=='server':assert gate.poisoned and not gate.calls_after_poison


def test_single_bad_ready_frame_delays_capture(tmp_path):
    result,w,gate,h=normal_run(tmp_path,'settling',40)
    # The fixture only grants follower target speed through frame61, so readiness cannot recover.
    assert result['status']=='safe_early_stop' and result['reason'] in ('warmup_not_ready','warmup_distance')
    assert not (tmp_path/'following_context.csv').exists()

@pytest.mark.parametrize('fault,reason', [('hash','route_map_hash_mismatch'),('geometry','route_geometry_mismatch'),
                                         ('branch','route_branch'),('obstacle','static_road_user_obstacle')])
def test_live_readonly_route_recheck_precedes_tm_and_world_changes(tmp_path,monkeypatch,fault,reason):
    c=reviewed_config();original=Harness.prepare
    def prepare(h):
        world=h.client.get_world()
        if fault=='hash':world.map.to_opendrive=lambda:'changed map'
        elif fault=='geometry':
            a=dict(c.route_audit);a['points']=[[p[0],p[1]+0.2,p[2]] for p in a['points']]
            h.c=replace(c,route_audit=a)
        elif fault=='branch':
            original_waypoint=world.map.get_waypoint_xodr
            def waypoint(*args):
                w=original_waypoint(*args);w.next=lambda d:[];return w
            world.map.get_waypoint_xodr=waypoint
        elif fault=='obstacle':
            world.get_level_bbs=lambda label:[NS(location=vector(100,0,0),extent=vector(2,1,1),rotation=NS(yaw=0))]
        return original(h)
    monkeypatch.setattr(Harness,'prepare',prepare)
    result,w,gate,h=normal_run(tmp_path)
    assert result['reason']==reason and result['cleanup']['complete']
    assert not any(e[0] in ('port_unused','settings','spawn','tm_sync') for e in w.events)


def test_settle_rpc_failure_replaces_previous_observation_with_partial_current_frame(tmp_path):
    result,w,gate,h=normal_run(tmp_path,'getter',2)
    assert result['status']=='server_unavailable' and result['actor_id']==11
    raw=json.loads((tmp_path/'last_observation.json').read_text())
    assert raw['phase']=='settle' and raw['carla_frame']==2
    assert raw['acquisition_status']=='partial' and raw['rows']==[] and not raw['frame_validated']
    assert gate.poisoned and not gate.calls_after_poison and not result['cleanup']['attempted']
