"""Offline Snapshot/IDM integration. No simulator or private dataset required."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from dt_risk_common import ActorState
from idm_dt_mvp import initial_state,predict,save
from idm_model import IDMParameters,compute_idm_acceleration
from idm_t_sensitivity import gap_of
from Predict_DT_IDM import main


@pytest.fixture
def source(tmp_path):
    folder=tmp_path/'source';folder.mkdir()
    def actor(logical,physical,x):
        return asdict(ActorState('snapshot',100,5.,physical,logical,'role','vehicle.tesla.model3',
                                x,2.,.1,0.,0.,0.,15.,0.,0.,15.,False))
    rows=[actor('follower',7,0.),actor('leader',9,24.8)]
    snapshot=folder/'snapshot.json';snapshot.write_text(json.dumps({'actors':rows}))
    box=dict(location=dict(x=.02,y=0.,z=.7),extent=dict(x=2.4,y=1.,z=.7),
             rotation=dict(yaw=0.,pitch=0.,roll=0.))
    prefix=folder/'raw.jsonl'
    prefix.write_text(json.dumps(dict(carla_frame=100,sim_time_s=5.,world_time_s=55.,rows=[
        dict(actor_id=r['source_actor_id'],frame=100,sim_time_s=5.,bounding_box=box,
             transform={k:r[k] for k in ('x','y','z','yaw','pitch','roll')},
             velocity={k:r['v'+k] for k in 'xyz'},forward=dict(x=1.,y=0.,z=0.)) for r in rows]))+'\nNOT_JSON_FUTURE_MUST_NOT_BE_READ\n')
    state=folder/'state.jsonl'
    state.write_text(json.dumps(dict(frame=100,sim_time_s=5.,rows=[dict(r,logical_id=r['logical_actor_id'],
        actor_id=r['source_actor_id'],frame=100,world_time_s=55.,forward_x=1.,forward_y=0.,forward_z=0.)
        for r in rows]))+'\nNOT_JSON_FUTURE_MUST_NOT_BE_READ\n')
    return snapshot,prefix,rows


def load(source):return initial_state(source[0],source[1],'follower','leader',
    source_run_id=source[0].parent.name,state_prefix_path=source[0].parent/'state.jsonl')


def required_args(source):
    return ['--state-prefix',str(source[0].parent/'state.jsonl'),'--source-run-id',source[0].parent.name]


def test_initial_snapshot_ids_positions_and_clock_preserved(source):
    initial=load(source)
    assert initial['actors']['follower']==source[2][0]|{'desired_time_headway_s':None,'desired_speed_mps':None}
    assert initial['source_frame_f0']==100 and initial['snapshot_time_s']==5. and initial['world_time_s']==55.
    assert initial['states']['follower'].speed==15. and initial['states']['leader'].x==pytest.approx(24.8)
    assert gap_of(initial['states']['follower'],initial['states']['leader'])==pytest.approx(20.)


def test_future_never_read_and_initial_state_matches_prediction(source):
    initial=load(source);p=IDMParameters(25.)
    result=predict(initial,p,p,horizon=.2)
    assert initial['consumed_raw_lines']==1  # malformed future line was never parsed
    for row in [r for r in result['rows'] if r['prediction_step']==0]:
        original=initial['actors'][row['logical_actor_id']]
        assert [row[k] for k in ('x','y','z','yaw')]==[original[k] for k in ('x','y','z','yaw')]
        assert row['source_actor_id']==original['source_actor_id'] and row['speed_mps']==15.


def test_existing_equation_and_common_state_simultaneous_update(source):
    initial=load(source);p=IDMParameters(25.)
    result=predict(initial,p,p,dt=.05,horizon=.2)
    for run in result['results']:
        r=run['rows'][0];fp=IDMParameters(**run['parameters'])
        assert r['follower_acceleration_mps2']==compute_idm_acceleration(15.,fp,20.,15.)
        assert r['leader_acceleration_mps2']==compute_idm_acceleration(15.,p)
        assert run['rows'][1]['follower_speed_mps']==pytest.approx(15.+.05*r['follower_acceleration_mps2'])


def test_T_only_changes_follower_and_trajectories_are_finite(source):
    initial=load(source);p=IDMParameters(25.)
    result=predict(initial,p,p,dt=.1,horizon=6.)
    a,b=result['results'];pa=dict(a['parameters']);pb=dict(b['parameters'])
    pa.pop('desired_time_headway_s');pb.pop('desired_time_headway_s')
    assert pa==pb and a['leader_parameters']==b['leader_parameters']
    assert a['initial_follower']==b['initial_follower'] and a['initial_leader']==b['initial_leader']
    assert [r['leader_x_m'] for r in a['rows']]==[r['leader_x_m'] for r in b['rows']]
    assert b['follower_distance_m']<a['follower_distance_m']
    for run in result['results']:
        assert [r['time_s'] for r in run['rows']]==pytest.approx([i*.1 for i in range(61)])
    assert all(math.isfinite(r[k]) for r in result['rows'] for k in ('x','y','speed_mps','gap_m'))


@pytest.mark.parametrize('change',[
    {'logical_actor_id':'wrong'},{'source_actor_id':7},{'carla_frame':101},
    {'sim_time_s':6.},{'y':5.},{'yaw':90.},{'speed_mps':float('nan')},
    {'speed_mps':14.},{'x':-10.},
])
def test_invalid_initial_state_rejected(source,change):
    rows=source[2];rows[1].update(change);source[0].write_text(json.dumps({'actors':rows}))
    with pytest.raises((ValueError,KeyError)):load(source)


def test_future_bbox_cannot_fill_missing_F0(source):
    record=json.loads(source[1].read_text().splitlines()[0]);record['carla_frame']=101
    source[1].write_text(json.dumps(record)+'\n')
    with pytest.raises(ValueError,match='future boundary'):load(source)


def test_known_T_in_snapshot_not_used_as_inferred_trait(source):
    initial=load(source);p=IDMParameters(25.)
    expected=predict(initial,p,p,horizon=.1)
    rows=source[2]
    for row in rows:row.update(desired_time_headway_s=99.,desired_speed_mps=99.)
    source[0].write_text(json.dumps({'actors':rows}))
    assert predict(load(source),p,p,horizon=.1)['rows']==expected['rows']


def test_cli_files_and_no_source_mutation(source,tmp_path,capsys):
    before=[p.read_bytes() for p in source[:2]];output=tmp_path/'output'
    assert main(['--snapshot',str(source[0]),'--raw-prefix',str(source[1]),'--follower-id','follower',
        '--leader-id','leader','--input-kind','synthetic','--follower-v0','25','--leader-v0','25',
        '--output',str(output),'--horizon','.2',*required_args(source)])==0
    assert json.loads(capsys.readouterr().out)['CARLA_connected'] is False
    assert (output/'predicted_trajectories.csv').is_file()
    result=json.loads((output/'prediction_summary.json').read_text())
    assert result['route_audit'] is result['physical_bounds'] is None
    assert result['independent_prediction_accuracy']=='NOT_EVALUATED'
    assert [p.read_bytes() for p in source[:2]]==before
    with pytest.raises(FileExistsError):save(output,load(source),predict(load(source),IDMParameters(25.),IDMParameters(25.)),input_kind='synthetic')


def test_output_cannot_be_inside_input_or_repository(source):
    with pytest.raises(ValueError,match='input directories'):
        main(['--snapshot',str(source[0]),'--raw-prefix',str(source[1]),'--follower-id','follower',
            '--leader-id','leader','--input-kind','synthetic','--follower-v0','25','--leader-v0','25',
            '--output',str(source[0].parent/'output'),*required_args(source)])
    initial=load(source)
    with pytest.raises(ValueError,match='outside repository'):
        save(Path(__file__).parent/'forbidden_mvp_output',initial,predict(initial,IDMParameters(25.),IDMParameters(25.)),input_kind='synthetic')


def test_import_and_cli_help_never_import_CARLA():
    root=Path(__file__).resolve().parents[1]
    script='''import sys,importlib.abc
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,*args):
  if fullname=="carla":raise AssertionError("CARLA import prohibited")
sys.meta_path.insert(0,Block())
import Predict_DT_IDM
Predict_DT_IDM.main(["--help"])
'''
    result=subprocess.run([sys.executable,'-c',script],cwd=root,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr


def edit_state(source,change):
    path=source[0].parent/'state.jsonl'
    record=json.loads(path.read_text().splitlines()[0]);change(record)
    path.write_text(json.dumps(record)+'\nNOT_JSON_FUTURE_MUST_NOT_BE_READ\n')


def test_complete_F0_diagnostics_without_numeric_acceptance(source):
    initial=load(source);d=initial['f0_comparison']
    assert initial['source_run_id']=='source'
    assert d['identity_and_raw_clocks']=='CHECKED' and d['raw_required_fields']=='COMPLETE'
    assert d['snapshot_embedded_world_time']==d['snapshot_embedded_run_id']=='NOT_RECORDED'
    assert d['numerical_acceptance']=='NOT_ASSESSED_NO_APPROVED_TOLERANCE'
    assert d['bbox_gap_residual_m']==pytest.approx(0.)
    assert initial['consumed_state_lines']==1
    assert all(v==0 for r in d['residuals'].values() for v in r['pair_minus_snapshot'].values())


def test_nonzero_numerical_residual_is_recorded_not_used_as_threshold(source):
    before=load(source);p=IDMParameters(25.);baseline=predict(before,p,p,horizon=.1)['rows']
    edit_state(source,lambda r:r['rows'][1].update(x=r['rows'][1]['x']+.01,forward_y=.01))
    after=load(source);d=after['f0_comparison']
    assert d['residuals']['leader']['pair_minus_snapshot']['x']==pytest.approx(.01)
    assert d['residuals']['leader']['forward_pair_minus_snapshot_angles'][1]==.01
    assert d['numerical_acceptance']=='NOT_ASSESSED_NO_APPROVED_TOLERANCE'
    assert predict(after,p,p,horizon=.1)['rows']==baseline


@pytest.mark.parametrize('change',[
    lambda r:r.update(source_run_id='other'),
    lambda r:r.update(frame=101),
    lambda r:r['rows'][0].update(frame=99),
    lambda r:r['rows'][0].update(logical_id='wrong'),
    lambda r:r['rows'][0].update(actor_id=999),
    lambda r:r['rows'][0].update(world_time_s=56.),
    lambda r:r['rows'][0].update(sim_time_s=6.),
    lambda r:r['rows'].append(dict(r['rows'][0])),
])
def test_F0_identity_and_clock_errors_rejected(source,change):
    edit_state(source,change)
    with pytest.raises(ValueError):load(source)


@pytest.mark.parametrize('key',['x','y','z','vx','vy','vz','speed_mps','yaw','pitch','roll','forward_x','world_time_s'])
def test_missing_F0_state_field_never_succeeds(source,key):
    edit_state(source,lambda r:r['rows'][0].pop(key))
    with pytest.raises((ValueError,KeyError)):load(source)


def test_wrong_Source_directory_and_explicit_ID_rejected(source,tmp_path):
    with pytest.raises(ValueError,match='Source run ID'):
        initial_state(source[0],source[1],'follower','leader',source_run_id='wrong',
                      state_prefix_path=source[0].parent/'state.jsonl')
    other=tmp_path/'other';other.mkdir();path=other/'state.jsonl'
    path.write_bytes((source[0].parent/'state.jsonl').read_bytes())
    with pytest.raises(ValueError,match='directory mismatch'):
        initial_state(source[0],source[1],'follower','leader',source_run_id='source',state_prefix_path=path)


def test_snapshot_declared_run_and_world_time_are_checked(source):
    for extra in ({'source_run_id':'other'},{'world_time_s':56.}):
        source[0].write_text(json.dumps(dict(actors=source[2],**extra)))
        with pytest.raises(ValueError):load(source)


def test_no_F0_state_prefix_fails_closed(source):
    (source[0].parent/'state.jsonl').unlink()
    with pytest.raises(FileNotFoundError):load(source)


@pytest.mark.parametrize('key',['transform','velocity','forward','bounding_box'])
def test_missing_bbox_raw_observation_never_succeeds(source,key):
    record=json.loads(source[1].read_text().splitlines()[0]);record['rows'][0].pop(key)
    source[1].write_text(json.dumps(record)+'\nNOT_JSON_FUTURE\n')
    with pytest.raises(KeyError):load(source)


def test_bbox_and_state_world_clocks_must_match(source):
    record=json.loads(source[1].read_text().splitlines()[0]);record['world_time_s']=54.
    source[1].write_text(json.dumps(record)+'\nNOT_JSON_FUTURE\n')
    with pytest.raises(ValueError,match='world clock'):load(source)
