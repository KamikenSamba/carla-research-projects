"""Existing mock V2 -> actual receipts -> adapter -> unmodified contract validator."""
import builtins
import csv
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from dt_gt_adapter import adapt, RunBytes
from dt_gt_observation import encoded, digest
from dt_gt_manifest import validate
from test_gt_observation import collect


def hashes(root):
    return {str(p.relative_to(root)):digest(p.read_bytes()) for p in root.rglob('*') if p.is_file()}


@pytest.fixture(scope='module')
def collected(tmp_path_factory):
    root=tmp_path_factory.mktemp('gt-existing-v2')
    with pytest.MonkeyPatch.context() as patch:
        _,result,*_=collect(root/'source',patch)
        assert result['status']=='completed',result
        origin=json.loads((root/'source/gt_origin.json').read_text(encoding='utf-8'))
        parent=dict(run_id='mock-source',origin={k:origin[k] for k in ('frame','simulation_time_s')},
            snapshot_sha256=origin['view']['snapshot.json']['sha256'],prefix_sha256=origin['view']['prefix.csv']['sha256'],
            _inputs=[dict(path=str(root/'source'/ref['path']),sha256=ref['sha256'],role='model_observation',
                maximum_source_frame=origin['frame'],maximum_source_timestamp=origin['simulation_time_s'])
                for ref in origin['view'].values()])
        _,result,*_=collect(root/'prediction',patch,role='prediction',parent=parent)
        assert result['status']=='completed',result
    return root


@pytest.fixture
def runs(collected,tmp_path):
    # Copy synthetic input, preserving its original declared Source-view paths.
    root=tmp_path/'inputs';shutil.copytree(collected,root)
    return root


def edit(run,name,change):
    path=run/name;value=json.loads(path.read_text(encoding='utf-8'));change(value);path.write_bytes(encoded(value))
    inventory=run/'gt_artifact_inventory.json'
    records=json.loads(inventory.read_text(encoding='utf-8'))
    if name in records:records[name]['sha256']=digest(path.read_bytes());inventory.write_bytes(encoded(records))


def test_mock_full_e2e_reuses_real_collector_data_and_preserves_originals(collected,tmp_path,monkeypatch):
    before=hashes(collected);original=builtins.__import__
    def guard(name,*args,**kwargs):
        if name=='carla':pytest.fail('CARLA import/connection forbidden')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guard)
    out=tmp_path/'bundle';result=adapt(collected/'source',collected/'prediction',out)
    assert result['status']=='CONTRACT_VALID_PENDING_APPROVAL',result
    assert hashes(collected)==before
    manifest=json.loads((out/'manifest.json').read_text(encoding='utf-8'))
    assert manifest['evidence_kind']=='synthetic'
    assert manifest['source']['origin']['frame']==140
    assert manifest['prediction']['origin']['frame']==1040
    assert manifest['prediction']['reference_origin']==manifest['source']['origin']
    assert manifest['source']['actor_map']!=manifest['prediction']['actor_map']
    check=validate(out/'manifest.json')
    assert check['trajectory_samples_per_side']==240
    assert check['status']=='CONTRACT_VALID_PENDING_APPROVAL'
    for field in ('independent_forecast_accuracy_enabled','risk_detection_performance_enabled','runtime_execution_enabled'):
        assert check[field] is False
    assert check['route_audit'] is None and check['physical_bounds'] is None
    assert check['initial_state_assessment']=='NOT_EVALUATED_PENDING_POLICY'
    assert not check['label_contract_complete']
    for side in ('source','prediction'):
        ar=manifest[side]['artifacts']
        for ref in ar.values():assert digest((out/ref['path']).read_bytes())==ref['sha256']
        assert json.loads((out/ar['events']['path']).read_text(encoding='utf-8'))['labels']==[]
        assert ar['snapshot']['sha256']==json.loads((collected/side/'gt_origin.json').read_text(encoding='utf-8'))['view']['snapshot.json']['sha256']


@pytest.mark.parametrize('name,change,expected',[
    ('gt_run_provenance.json',lambda d:d['seed'].update(status='unknown'),'SEED_APPLICATION_UNVERIFIED'),
    ('gt_run_provenance.json',lambda d:d.update(source_dirty=None),'CODE_PROVENANCE'),
    ('gt_physics_extras.json',lambda d:next(iter(d.values()))['unknown_static'].append('wheel'), 'STATIC_PHYSICS_UNKNOWN'),
    ('gt_capture_coverage.json',lambda d:d.update(capture_status='unknown'),'EVENT_COVERAGE_UNPROVEN'),
    ('gt_capture_coverage.json',lambda d:d.update(errors=['late_callback']),'EVENT_COVERAGE_UNPROVEN'),
    ('gt_capture_coverage.json',lambda d:d['sensors'].pop(next(iter(d['sensors']))),'EVENT_SENSOR_COHORT'),
    ('gt_origin.json',lambda d:d.update(before_first_command=False),'ORIGIN_COHORT_OR_ORDER'),
    ('gt_origin.json',lambda d:d.update(raw_prefix_sha256='f'*64),'FROZEN_RAW_PREFIX_CHANGED'),
    ('gt_origin.json',lambda d:d['actor_map'].update(leader=9999),'GT raw actor'),
])
def test_incomplete_collection_never_emits_valid_manifest(runs,tmp_path,name,change,expected):
    edit(runs/'source',name,change)
    out=tmp_path/'out';result=adapt(runs/'source',runs/'prediction',out)
    assert result['status']=='INPUT_INCOMPLETE' and any(expected in e for e in result['errors']),result
    assert not (out/'manifest.json').exists()


@pytest.mark.parametrize('change,expected',[
    (lambda d:d['inputs'][0].update(maximum_source_frame=141),'FUTURE_MODEL_INPUT'),
    (lambda d:d['inputs'][0].update(maximum_source_timestamp=5000.),'FUTURE_MODEL_INPUT'),
    (lambda d:d['inputs'][0].update(contains_source_future=True),'FUTURE_MODEL_INPUT'),
    (lambda d:(d.update(parameter_policy='default'),d['inputs'][0].update(contains_source_traits=True)), 'ORACLE_INFORMATION_LEAK'),
    (lambda d:d['parent'].update(prefix_sha256='f'*64),'PARENT_FROZEN_VIEW_HASH'),
    (lambda d:d['parent'].update(run_id='wrong'),'PARENT_SOURCE_ORIGIN'),
    (lambda d:d['inputs'].clear(),'PREDICTION_FROZEN_VIEW_INPUTS_MISSING'),
    (lambda d:d.update(inventory_complete=False),'INPUT_INVENTORY_UNDECLARED'),
])
def test_future_and_parent_and_oracle_input_rejected(runs,tmp_path,change,expected):
    edit(runs/'prediction','gt_run_provenance.json',change)
    result=adapt(runs/'source',runs/'prediction',tmp_path/'out')
    assert any(expected in e for e in result['errors']),result
    assert result['status']=='INPUT_INCOMPLETE'


def test_tamper_rejected_and_old_run_not_repaired(runs,tmp_path):
    (runs/'source/calibration_raw.jsonl').write_text('{}')
    result=adapt(runs/'source',runs/'prediction',tmp_path/'hash-out')
    assert 'ADAPTER_HASH_MISMATCH' in result['errors'][0]
    old=tmp_path/'old';old.mkdir();(old/'initial_snapshot.json').write_text('{}')
    result=adapt(old,runs/'prediction',tmp_path/'old-out')
    assert result['status']=='INPUT_INCOMPLETE' and not (tmp_path/'old-out/manifest.json').exists()


def test_each_input_read_once_despite_repeated_parsers(collected,monkeypatch):
    reader=RunBytes(collected/'source');original=Path.read_bytes;counts={}
    def capture(path):counts[str(path)]=counts.get(str(path),0)+1;return original(path)
    monkeypatch.setattr(Path,'read_bytes',capture)
    reader.read('gt_origin.json');reader.json('gt_origin.json')
    assert max(counts.values())==1


def test_invalid_derived_contract_is_rejected_by_existing_validator(collected,tmp_path):
    out=tmp_path/'bundle';assert adapt(collected/'source',collected/'prediction',out)['status']=='CONTRACT_VALID_PENDING_APPROVAL'
    manifest=json.loads((out/'manifest.json').read_text(encoding='utf-8'));manifest['prediction']['parent_snapshot_sha256']='f'*64
    (out/'manifest.json').write_bytes(encoded(manifest))
    assert validate(out/'manifest.json')['status']=='INVALID'


def test_cli_is_offline_new_external_only(collected,tmp_path,monkeypatch):
    from Adapt_V2_GroundTruth import main
    args=['--source-run',str(collected/'source'),'--prediction-run',str(collected/'prediction')]
    out=tmp_path/'output'
    assert main(args+['--output-root',str(out)])==0
    with pytest.raises(SystemExit):main(args+['--output-root',str(out)])
    with pytest.raises(SystemExit):main(args+['--output-root',str(collected/'source/new')])
    with pytest.raises(SystemExit):main(args+['--output-root',str(Path(__file__).resolve().parents[1]/'must-not-create-gt')])


def test_no_risk_labels_can_be_enabled_by_zero_events(collected,tmp_path):
    out=tmp_path/'bundle';adapt(collected/'source',collected/'prediction',out)
    manifest=json.loads((out/'manifest.json').read_text(encoding='utf-8'));manifest['evaluation']['requested_metrics'].append('risk_detection')
    (out/'manifest.json').write_bytes(encoded(manifest));assessment=validate(out/'manifest.json')
    assert assessment['status']=='INVALID' and not assessment['risk_detection_performance_enabled']


def edit_lines(run,name,change):
    path=run/name;records=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    change(records);path.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in records),encoding='utf-8')
    inventory=run/'gt_artifact_inventory.json';refs=json.loads(inventory.read_text(encoding='utf-8'))
    refs[name]['sha256']=digest(path.read_bytes());inventory.write_bytes(encoded(refs))


@pytest.mark.parametrize('name,change,expected',[
    ('gt_clock.jsonl',lambda rows:rows.append(dict(rows[-1])),'CLOCK_DUPLICATE'),
    ('v2_control_diagnostics.jsonl',lambda rows:rows.pop(),'BOUNDED_IDM_WINDOW_INCOMPLETE'),
    ('v2_control_diagnostics.jsonl',lambda rows:rows[0].update(command_frame=0),'COMMAND_RESPONSE_FRAME'),
    ('v2_control_diagnostics.jsonl',lambda rows:rows[0].update(actor_id=999),'DIAGNOSTIC_ACTOR_DUPLICATE'),
    ('v2_control_diagnostics.jsonl',lambda rows:rows[0].update(response_time_s=999.),'DIAGNOSTIC_LOCAL_TIME'),
    ('v2_control_diagnostics.jsonl',lambda rows:rows[0]['observed_control'].update(throttle=.9),'OBSERVED_CONTROL_MISMATCH'),
    ('command_ledger.jsonl',lambda rows:rows[0].update(based_on_frame=0),'LEDGER_COMMAND'),
])
def test_clock_frame_id_and_ledger_corruption_rejected(runs,tmp_path,name,change,expected):
    edit_lines(runs/'source',name,change)
    result=adapt(runs/'source',runs/'prediction',tmp_path/'out')
    assert result['status']=='INPUT_INCOMPLETE' and any(expected in e for e in result['errors']),result


def test_external_collision_pair_not_mapped_to_invented_logical_id(runs,tmp_path):
    source=runs/'source';origin=json.loads((source/'gt_origin.json').read_text(encoding='utf-8'))
    row=dict(frame=origin['frame']+1,timestamp=origin['simulation_time_s']+.05,
        actor_id=origin['actor_map']['follower'],other_actor_id=9999,gt_event_id='external',normal_impulse=dict(x=1,y=0,z=0))
    path=source/'collision_events.jsonl';path.write_text(json.dumps(row)+'\n',encoding='utf-8')
    inventory=source/'gt_artifact_inventory.json';refs=json.loads(inventory.read_text(encoding='utf-8'))
    refs[path.name]=dict(path=path.name,sha256=digest(path.read_bytes()));inventory.write_bytes(encoded(refs))
    edit(source,'gt_capture_coverage.json',lambda d:d.update(event_count=1))
    result=adapt(source,runs/'prediction',tmp_path/'out')
    assert 'EXTERNAL_COLLISION_PAIR' in result['errors'][0]
    assert json.loads(path.read_text(encoding='utf-8'))['other_actor_id']==9999


def test_adapter_path_escape_is_rejected(collected):
    reader=RunBytes(collected/'source')
    with pytest.raises(ValueError,match='PATH_ESCAPE'):reader.read('../prediction/gt_origin.json')


def test_real_recording_cannot_reuse_synthetic_delivery_fence(runs,tmp_path):
    edit(runs/'source','gt_run_provenance.json',lambda d:d.update(evidence_kind='recorded_carla'))
    result=adapt(runs/'source',runs/'prediction',tmp_path/'out')
    assert result['status']=='INPUT_INCOMPLETE' and 'REAL_EVENT_DELIVERY_FENCE_UNAVAILABLE' in result['errors'][0]


def test_traits_inserted_into_kinematic_snapshot_rejected(runs,tmp_path):
    run=runs/'source';path=run/'gt_input_view/snapshot.json'
    snapshot=json.loads(path.read_text(encoding='utf-8'));snapshot['actors'][0]['desired_time_headway_s']=2.
    path.write_bytes(encoded(snapshot))
    edit(run,'gt_origin.json',lambda d:d['view']['snapshot.json'].update(sha256=digest(path.read_bytes())))
    result=adapt(run,runs/'prediction',tmp_path/'out')
    assert 'ORACLE_TRAITS_IN_KINEMATIC_VIEW' in result['errors'][0]


def test_existing_xy_near_miss_keeps_frame_and_proxy_definition(runs,tmp_path):
    from dataclasses import asdict
    from dt_risk_common import RiskEvent
    run=runs/'source';origin=json.loads((run/'gt_origin.json').read_text(encoding='utf-8'))
    event=asdict(RiskEvent('mock',0.,.05,'near_miss','follower','leader',origin['actor_map']['follower'],
        origin['actor_map']['leader'],0.,0.,0.,3.,1.1,0.,'synthetic detector event'))
    with (run/'risk_events.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(event));writer.writeheader();writer.writerow(event)
    receipt=dict(row_index=0,frame=origin['frame']+1,simulation_time_s=origin['simulation_time_s']+.05)
    (run/'gt_event_frames.jsonl').write_text(json.dumps(receipt)+'\n',encoding='utf-8')
    inventory=run/'gt_artifact_inventory.json';refs=json.loads(inventory.read_text(encoding='utf-8'))
    for name in ('risk_events.csv','gt_event_frames.jsonl'):refs[name]['sha256']=digest((run/name).read_bytes())
    inventory.write_bytes(encoded(refs))
    out=tmp_path/'out';result=adapt(run,runs/'prediction',out)
    assert result['status']=='CONTRACT_VALID_PENDING_APPROVAL',result
    events=json.loads((out/'source/events.json').read_text(encoding='utf-8'))
    assert events['raw_events'][0]['kind']=='near_miss' and events['raw_events'][0]['frame']==origin['frame']+1
    assert 'XY center' in events['raw_definitions']['near_miss'] and events['labels']==[]


def test_fresh_adapter_cli_process_forbids_carla_import(collected,tmp_path):
    import os,subprocess,sys
    script="""import builtins,sys
original=builtins.__import__
def guard(name,*a,**kw):
    if name=='carla':raise AssertionError('CARLA forbidden')
    return original(name,*a,**kw)
builtins.__import__=guard
from Adapt_V2_GroundTruth import main
raise SystemExit(main(sys.argv[1:]))
"""
    args=['--source-run',str(collected/'source'),'--prediction-run',str(collected/'prediction'),
          '--output-root',str(tmp_path/'out')]
    env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[1]))
    result=subprocess.run([sys.executable,'-X','utf8','-c',script,*args],env=env,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr


@pytest.mark.parametrize('name,policy,expected',[
    ('prediction_states.csv','reconstruction_known','FUTURE_MODEL_INPUT'),
    ('calibration_raw.jsonl','reconstruction_known','FUTURE_MODEL_INPUT'),
    ('resolved_idm_parameters.json','default','ORACLE_INFORMATION_LEAK'),
    ('initial_snapshot.json','estimated','ORACLE_INFORMATION_LEAK'),
])
def test_known_source_files_mislabelled_without_flags_are_rejected(runs,tmp_path,name,policy,expected):
    # Copy outside the Source directory: classification must use bytes, not filename alone.
    original=runs/'source'/name;disguised=tmp_path/'parameter-input.bin';disguised.write_bytes(original.read_bytes())
    def change(provenance):
        provenance['parameter_policy']=policy
        provenance['inputs'].append(dict(path=str(disguised),sha256=digest(disguised.read_bytes()),role='model_parameters'))
    edit(runs/'prediction','gt_run_provenance.json',change)
    result=adapt(runs/'source',runs/'prediction',tmp_path/'out')
    assert result['status']=='INPUT_INCOMPLETE' and expected in result['errors'][0],result
