"""Independent known-error fixtures and contract failures; no CARLA server/data."""
import builtins
import copy
import json
from pathlib import Path

import pytest

from dt_trajectory_evaluation import evaluate
from idm_fidelity import write_csv


def trajectory(origin=100,actor=1,offset=(0.,0.,0.),speed=1.,logical='one',n=10):
    return [dict(prediction_time_s=i*.1,prediction_frame=origin+i,logical_actor_id=logical,
        carla_actor_id=actor,x=i*.1*speed+offset[0],y=offset[1],z=offset[2],
        vx=speed,vy=0.,vz=0.,speed_mps=speed) for i in range(1,n+1)]


def test_known_345_offset_and_different_world_actor_ids():
    r=evaluate(trajectory(),trajectory(900,101,(3.,4.,0.)),dt=.1,horizons=(1.,),
               reference_origin_frame=100,prediction_origin_frame=900)
    s=r['per_actor'][0]
    assert s['ade_xy_m']==s['fde_xy_m']==s['ade_xyz_m']==s['fde_xyz_m']==5
    assert s['speed_mae_mps']==s['speed_rmse_mps']==0
    assert s['reference_actor_id']==1 and s['prediction_actor_id']==101
    assert r['independent_ground_truth_used'] is False and r['risk_detection_performance'] is None


def test_known_speed_error_ade_fde_and_no_time_zero_in_ade():
    r=evaluate(trajectory(),trajectory(900,101,speed=2.),dt=.1,horizons=(.5,1.))
    a,b=r['per_actor']
    assert a['ade_xy_m']==pytest.approx(.3) and a['fde_xy_m']==.5
    assert b['ade_xy_m']==pytest.approx(.55) and b['fde_xy_m']==1.
    assert b['speed_mae_mps']==b['speed_rmse_mps']==b['speed_bias_mps']==1.
    assert len(r['per_time'])==10


@pytest.mark.parametrize('fault',['duplicate','nan','off_grid','actor_switch','origin','speed_mismatch','time_zero'])
def test_malformed_alignment_rejected(fault):
    pred=trajectory(900,101)
    if fault=='duplicate':pred.append(copy.copy(pred[0]))
    if fault=='nan':pred[0]['x']='NaN'
    if fault=='off_grid':pred[0]['prediction_time_s']=.11
    if fault=='actor_switch':pred[2]['carla_actor_id']=102
    if fault=='origin':pred[2]['prediction_frame']+=1
    if fault=='speed_mismatch':pred[0]['speed_mps']=2
    if fault=='time_zero':pred[0]['prediction_time_s']=0.
    with pytest.raises(ValueError):evaluate(trajectory(),pred,dt=.1,horizons=(1.,))


def test_missing_actor_time_and_post_event_censoring_remain_visible():
    ref=trajectory()+trajectory(actor=2,logical='two')
    pred=trajectory(900,101);pred.pop(4)
    r=evaluate(ref,pred,dt=.1,horizons=(.2,1.))
    rows={(x['logical_actor_id'],x['horizon_s']):x for x in r['per_actor']}
    assert rows['one',.2]['status']=='complete'
    assert rows['one',1.]['ade_xy_m'] is None and rows['one',1.]['coverage']==.9
    assert rows['two',1.]['coverage']==0 and rows['two',1.]['fde_xy_m'] is None
    assert r['per_horizon'][0]['actor_coverage']==.5 and r['per_horizon'][1]['actor_coverage']==0
    c=evaluate(trajectory(),trajectory(900,101),dt=.1,horizons=(.2,1.),
        terminal_events={'one':dict(time_s=.3,reason='collision')})
    assert c['per_actor'][0]['status']=='complete' and c['per_actor'][1]['status']=='post_event_censored'
    assert c['per_actor'][1]['ade_xy_m'] is None and len(c['paired_samples'])==2


@pytest.mark.parametrize('kwargs',[dict(purpose='prediction_accuracy'),dict(dt=0),dict(horizons=(.15,)),dict(horizons=())])
def test_unsupported_contract_or_grid_rejected(kwargs):
    options=dict(dt=.1,horizons=(1.,));options.update(kwargs)
    with pytest.raises(ValueError):evaluate(trajectory(),trajectory(),**options)


def bundle(root,trial,origin,actor):
    from test_idm import old_row
    root.mkdir()
    snap=old_row();snap.update(carla_frame=origin,source_actor_id=actor,logical_actor_id='one',x=0.,y=0.,z=0.,
                               vx=1.,vy=0.,vz=0.,speed_mps=1.,yaw=0.)
    data={'result.json':dict(status='completed',worker_exit_code=0,supervised=True,collision_events=0,
                            cleanup=dict(complete=True)),
          'environment.json':dict(versions=['0.9.16','0.9.16'],map='synthetic',weather={},initial_actor_ids=[],
                                  original_settings={},applied_settings=dict(fixed_delta_seconds=.1)),
          'config_used.json':dict(scenario=dict(trial=trial,dt=.1)),
          'resolved_idm_parameters.json':dict(one=dict(T=1.)),
          'launch.json':dict(source_commit='synthetic'),
          'initial_snapshot.json':dict(actors=[snap]),
          'vehicle_physics.json':dict(mass=1,wheels=[]),'leader_physics.json':dict(mass=1,wheels=[])}
    for name,payload in data.items():(root/name).write_text(json.dumps(payload),encoding='utf-8')
    write_csv(root/'prediction_states.csv',trajectory(origin,actor))


def test_bundle_cli_and_refusal_of_environment_initial_state_and_mapping_changes(tmp_path,monkeypatch):
    real=builtins.__import__
    def guard(name,*a,**kw):
        if name=='carla':pytest.fail('CARLA must not be imported')
        return real(name,*a,**kw)
    monkeypatch.setattr(builtins,'__import__',guard)
    from Evaluate_DT_Trajectories import assess,main
    a,b=tmp_path/'source',tmp_path/'replay';bundle(a,'fleet',100,1);bundle(b,'replay',900,101)
    (b/'reconstruction_comparison.json').write_text(json.dumps(dict(within_initial_state_tolerance=True,
        actors=dict(one=dict(source_actor_id=1,reconstructed_actor_id=101)))))
    out=tmp_path/'metrics'
    assert main(['--source-run',str(a),'--replay-run',str(b),'--output-root',str(out),'--horizons','1'])==0
    result=json.loads((out/'evaluation.json').read_text())
    assert result['purpose']=='reconstruction_repeatability' and result['per_actor'][0]['ade_xy_m']==0
    with pytest.raises(ValueError):assess(a,b,out,(1.,))
    original=(b/'environment.json').read_text();env=json.loads(original);env['map']='wrong'
    (b/'environment.json').write_text(json.dumps(env))
    with pytest.raises(ValueError,match='environment'):assess(a,b,tmp_path/'bad',(1.,))
    (b/'environment.json').write_text(original)
    link=(b/'reconstruction_comparison.json').read_text();wrong=json.loads(link)
    wrong['actors']['one']['source_actor_id']=999
    (b/'reconstruction_comparison.json').write_text(json.dumps(wrong))
    with pytest.raises(ValueError,match='linkage'):assess(a,b,tmp_path/'bad_link',(1.,))
    (b/'reconstruction_comparison.json').write_text(link)
    snap=json.loads((b/'initial_snapshot.json').read_text());snap['actors'][0]['x']=1
    (b/'initial_snapshot.json').write_text(json.dumps(snap))
    with pytest.raises(ValueError,match='tolerance'):assess(a,b,tmp_path/'bad2',(1.,))
