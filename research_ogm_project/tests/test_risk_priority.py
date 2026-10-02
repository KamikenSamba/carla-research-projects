import ast
import hashlib
import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from ogm_project.grid_contract import GridContract, GridMismatchError, runtime_grid_metadata, validate_same_grid
from ogm_project.risk_priority import (OCC_TH, FREE_TH, baseline_candidate, build_priority, compute_known_masks,
    compute_priority_map, experiment_alignment, load_road_mask, probability_from_logodds, priority_summary, sha256, write_json)
from ogm_project.priority_snapshot import PrioritySnapshot


def metadata():
    return runtime_grid_metadata(map_name='Town10HD_Opt', origin_x=42., origin_y=56.,
        x_min=-30., x_max=30., y_min=-30., y_max=30., resolution=.2, nx=300, ny=300)


def risk_metadata(meta):
    return dict(meta, array_convention='risk_time[k, iy, ix]; axis 0 of spatial maps is WORLD Y',
        cell_sampling='cell_center', bounds_convention='absolute WORLD meters; min inclusive, max exclusive')


def experiment():
    return dict(scenario_id='synthetic', scenario_sha256='a'*64,
                ego_initial_pose=dict(x=5., y=70., z=.2, yaw=0.), target_id='synthetic_target',
                time_reference_id='synthetic_clock', reference_time_s=5.)


def test_same_grid_and_risk_v1_adapter():
    m = metadata()
    risk = risk_metadata(m)
    for key in ('array_axis_0', 'array_axis_1', 'cell_coordinate'):
        del risk[key]
    validate_same_grid(GridContract.from_metadata(m), GridContract.from_metadata(risk, risk_v1=True))


@pytest.mark.parametrize('key,value', [('resolution_m', .3), ('nx', 301), ('ny', 301),
    ('center_x', 43), ('origin_y', 57), ('map_name', 'Town01'), ('frame', 'EGO'),
    ('array_axis_0', 'WORLD_X'), ('array_axis_1', 'WORLD_Y'), ('schema_version', '2.0'),
    ('cell_coordinate', 'corner'), ('x_min_m', 11), ('y_max_m', 87)])
def test_mismatch(key, value):
    m = metadata(); changed = dict(m); changed[key] = value
    with pytest.raises(GridMismatchError):
        validate_same_grid(GridContract.from_metadata(m), GridContract.from_metadata(changed))


def test_same_shape_translated_grid_rejected():
    other = runtime_grid_metadata(map_name='Town10HD_Opt', origin_x=-50., origin_y=70.,
        x_min=-30., x_max=30., y_min=-30., y_max=30., resolution=.2, nx=300, ny=300)
    with pytest.raises(GridMismatchError, match='x_min_m'):
        validate_same_grid(GridContract.from_metadata(metadata()), GridContract.from_metadata(other))


def test_asymmetric_offsets_do_not_confuse_anchor_and_center():
    m = runtime_grid_metadata(map_name='test', origin_x=10., origin_y=20.,
        x_min=0., x_max=60., y_min=-20., y_max=40., resolution=.2, nx=300, ny=300)
    g = GridContract.from_metadata(m)
    assert (g.center_x, g.center_y) == (40, 30)
    assert m['origin_x'] == 10 and g.x_min_m == 10


@pytest.mark.parametrize('key', ['map_name','cell_coordinate','array_axis_0','schema_version'])
def test_missing_metadata_field(key):
    m = metadata(); del m[key]
    with pytest.raises(GridMismatchError):
        GridContract.from_metadata(m)


def test_no_unknown_risk_axis_fallback():
    m = risk_metadata(metadata()); m['array_convention'] = 'xy'
    with pytest.raises(GridMismatchError):
        GridContract.from_metadata(m, risk_v1=True)


def test_current_known_thresholds_and_boundary_behavior():
    assert (OCC_TH, FREE_TH) == (.60, .48)
    ps = np.array([[.48, .48001, .5, .59999, .60]], dtype=np.float64)
    odds = np.log(ps/(1-ps))
    unknown, known = compute_known_masks(odds, odds)
    np.testing.assert_array_equal(known, [[True, False, False, False, True]])
    np.testing.assert_array_equal(unknown, ~known)
    # Compare the exact existing expression over its whole operating range.
    values = np.linspace(-4,4,10001,dtype=np.float32)[None,:]
    p = 1/(1+np.exp(-values))
    np.testing.assert_array_equal(compute_known_masks(values,values)[1], (p>=.6)|(p<=.48))


def test_extreme_finite_sigmoid():
    np.testing.assert_array_equal(probability_from_logodds(np.array([[-1e30,0,1e30]],dtype=np.float32)), [[0,.5,1]])


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite_logodds_rejected(bad):
    with pytest.raises(ValueError):
        probability_from_logodds(np.array([[bad]],dtype=np.float32))


def test_priority_all_conditions_and_baseline():
    risk = np.array([[0,.25,1],[.8,.5,1]],dtype=np.float32)
    ones = np.ones_like(risk,dtype=bool)
    got = compute_priority_map(risk,ones,ones,ones)
    np.testing.assert_array_equal(got,risk)
    assert got.dtype == np.float32
    u = np.array([[True,False,True],[True,True,True]])
    k = np.array([[True,True,True],[False,True,True]])
    r = np.array([[True,True,True],[True,False,True]])
    expected = np.array([[0,0,1],[0,0,1]],dtype=np.float32)
    np.testing.assert_array_equal(compute_priority_map(risk,u,k,r),expected)
    np.testing.assert_array_equal(baseline_candidate(u,k,r), u & k & r)


@pytest.mark.parametrize('gate', [0,1,2])
def test_each_gate_blocks(gate):
    masks=[np.ones((2,3),dtype=bool) for _ in range(3)]; masks[gate][:]=False
    assert not compute_priority_map(np.ones((2,3),dtype=np.float32),*masks).any()


@pytest.mark.parametrize('bad', [np.nan,np.inf,-np.inf,-.1,1.1])
def test_bad_risk_values(bad):
    with pytest.raises(ValueError):
        compute_priority_map(np.array([[bad]],dtype=np.float32),*[np.ones((1,1),dtype=bool)]*3)


def test_dtype_shape_no_broadcast_or_mask_cast():
    r = np.zeros((2,3),dtype=np.float32); mask=np.ones((2,3),dtype=bool)
    with pytest.raises(ValueError): compute_priority_map(r.astype(np.float64),mask,mask,mask)
    with pytest.raises(ValueError): compute_priority_map(r,mask[:1],mask,mask)
    with pytest.raises(ValueError): compute_priority_map(r,mask.astype(float),mask,mask)


def test_zero_baseline_summary():
    risk=np.zeros((2,3),dtype=np.float32); no=np.zeros((2,3),dtype=bool)
    summary=priority_summary(risk,no,no,no,risk)
    assert summary['reduction_ratio'] is None and summary['priority_mean_positive']==0


def mask_files(tmp_path, road, *, static=False):
    path=tmp_path/'source_mask.npy'; meta_path=tmp_path/'source_mask.json'
    np.save(path,~road if static else road)
    write_json(meta_path,dict(metadata(),mask_semantics='static_non_driving_true' if static else 'road_true',
                              array_sha256=sha256(path),provenance='synthetic test fixture'))
    return path,meta_path


def test_mask_inversion_metadata_and_hash(tmp_path):
    road=np.zeros((300,300),dtype=bool); road[145:155,:]=True
    path,meta=mask_files(tmp_path,road,static=True)
    got,_=load_road_mask(path,meta,GridContract.from_metadata(metadata()))
    np.testing.assert_array_equal(got,road)
    np.save(path,road)
    with pytest.raises(GridMismatchError,match='sha256'):
        load_road_mask(path,meta,GridContract.from_metadata(metadata()))


def test_mask_missing_metadata_fails_closed(tmp_path):
    p=tmp_path/'mask.npy';np.save(p,np.ones((300,300),dtype=bool))
    with pytest.raises(GridMismatchError,match='metadata required'):
        load_road_mask(p,tmp_path/'absent.json',GridContract.from_metadata(metadata()))


def test_scenario_alignment_separate_from_grid():
    a=dict(experiment=experiment()); b=dict(experiment=experiment())
    assert experiment_alignment(a,b)['compatible'] is True
    b['experiment']['target_id']='different'
    assert experiment_alignment(a,b)['compatible'] is False
    assert experiment_alignment({},a)['compatible'] is None
    assert experiment_alignment({'experiment':{'ego_initial_pose':{}}},{})['compatible'] is None


def test_scenario_mismatch_blocks_priority_even_when_grid_matches(tmp_path):
    ogm,riskdir,*_=synthetic_snapshot(tmp_path)
    m=json.loads((riskdir/'metadata.json').read_text(encoding='utf-8'));m['experiment']['target_id']='other'
    write_json(riskdir/'metadata.json',m)
    with pytest.raises(ValueError,match='ExperimentMismatchError'):
        build_priority(ogm,riskdir,tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_missing_experiment_strict_mode_blocks(tmp_path):
    ogm,riskdir,*_=synthetic_snapshot(tmp_path)
    m=json.loads((riskdir/'metadata.json').read_text(encoding='utf-8'));del m['experiment']
    write_json(riskdir/'metadata.json',m)
    with pytest.raises(ValueError,match='ExperimentMismatchError'):
        build_priority(ogm,riskdir,tmp_path/'out',require_experiment_match=True)


def test_road_mask_wrong_origin_rejected_even_with_matching_hash(tmp_path):
    p,m=mask_files(tmp_path,np.ones((300,300),dtype=bool))
    meta=json.loads(m.read_text(encoding='utf-8'));meta['origin_x']+=1
    write_json(m,meta)
    with pytest.raises(GridMismatchError,match='origin_x'):
        load_road_mask(p,m,GridContract.from_metadata(metadata()))


def synthetic_snapshot(tmp_path):
    risk=np.zeros((300,300),dtype=np.float32);risk[140:160,140:160]=1
    unknown=np.zeros((300,300),dtype=bool);unknown[135:165,135:165]=True
    known=np.zeros((300,300),dtype=bool);known[:,150:]=True
    road=np.zeros((300,300),dtype=bool);road[145:155,:]=True
    ego=np.where(unknown,0.,2.).astype(np.float32);rsu=np.where(known,2.,0.).astype(np.float32)
    path,meta=mask_files(tmp_path,road,static=True)
    out=tmp_path/'ogm';recorder=PrioritySnapshot(out,metadata(),path,meta,experiment())
    meas=SimpleNamespace(frame=100,timestamp=5.)
    recorder.callback('ego',lambda m:None,meas);recorder.callback('rsu',lambda m:None,meas)
    assert recorder.save_if_ready(ego,rsu)
    risk_dir=tmp_path/'risk';risk_dir.mkdir()
    np.save(risk_dir/'risk_aggregate.npy',risk)
    write_json(risk_dir/'metadata.json',dict(risk_metadata(metadata()),experiment=experiment(),source_kind='synthetic'))
    return out,risk_dir,risk,unknown,known,road


def test_synthetic_cli_e2e(tmp_path):
    ogm,riskdir,risk,u,k,r=synthetic_snapshot(tmp_path)
    out=tmp_path/'priority_map'
    result=subprocess.run([sys.executable,str(ROOT/'scripts/build_communication_priority_map.py'),
        '--ogm-dir',str(ogm),'--risk-dir',str(riskdir),'--output-dir',str(out),
        '--require-experiment-match'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert {p.name for p in out.iterdir()}=={'priority_map.npy','baseline_candidate_mask.npy',
        'priority_map.png','priority_components.png','priority_components_real_carla.png',
        'priority_metadata.json','priority_summary.json'}
    got=np.load(out/'priority_map.npy');expected=np.zeros((300,300),dtype=np.float32)
    expected[145:155,150:160]=1
    np.testing.assert_array_equal(got,expected)
    assert got.dtype==np.float32 and np.isfinite(got).all()
    summary=json.loads((out/'priority_summary.json').read_text(encoding='utf-8'))
    assert summary['risk_positive_cells']==400 and summary['ego_unknown_cells']==900
    assert summary['rsu_known_cells']==45000 and summary['road_cells']==3000
    assert summary['baseline_candidate_cells']==150 and summary['priority_positive_cells']==100
    assert summary['reduction_ratio']==pytest.approx(1/3)
    assert summary['grid_compatible'] and summary['experiment_compatible']
    from matplotlib.image import imread
    for p in out.glob('*.png'):
        image=imread(p);assert image.shape[0]>100 and np.ptp(image)>0


def test_stale_saved_known_mask_rejected(tmp_path):
    ogm,riskdir,*_=synthetic_snapshot(tmp_path)
    np.save(ogm/'ego_unknown.npy',np.ones((300,300),dtype=bool))
    with pytest.raises(ValueError,match='saved mask disagrees'):
        build_priority(ogm,riskdir,tmp_path/'out')


def test_cli_grid_mismatch_no_output(tmp_path):
    ogm,riskdir,*_=synthetic_snapshot(tmp_path)
    m=json.loads((riskdir/'metadata.json').read_text(encoding='utf-8'));m['map_name']='Town01'
    write_json(riskdir/'metadata.json',m)
    result=subprocess.run([sys.executable,str(ROOT/'scripts/build_communication_priority_map.py'),
        '--ogm-dir',str(ogm),'--risk-dir',str(riskdir),'--output-dir',str(tmp_path/'out')],capture_output=True,text=True)
    assert result.returncode==2 and 'GridMismatchError' in result.stderr
    assert not (tmp_path/'out').exists()


def test_snapshot_waits_for_matching_frames_and_copies(tmp_path):
    p,m=mask_files(tmp_path,np.ones((300,300),dtype=bool))
    r=PrioritySnapshot(tmp_path/'snap',metadata(),p,m,experiment())
    e=np.zeros((300,300),dtype=np.float32);s=e.copy()
    assert not r.save_if_ready(e,s)
    r.callback('ego',lambda m:e.fill(2),SimpleNamespace(frame=1,timestamp=.05))
    r.callback('rsu',lambda m:s.fill(-2),SimpleNamespace(frame=2,timestamp=.1))
    assert not r.save_if_ready(e,s)
    r.callback('ego',lambda m:e.fill(0),SimpleNamespace(frame=2,timestamp=.1))
    assert r.save_if_ready(e,s)
    assert (r.out/'experiment_metadata.json').exists()
    e.fill(4);s.fill(4)
    assert not np.load(r.out/'ego_logodds.npy').any()
    assert (np.load(r.out/'rsu_logodds.npy')==-2).all()
    assert not r.save_if_ready(e,s)


def test_modern_experiment_alignment():
    e=dict(experiment_id='e',scenario_name='s',map_name='Town10HD_Opt',
        ego_initial_state=dict(x=0.,y=0.,z=.6,yaw=0.,vx=1.,vy=0.,vz=0.),
        target_initial_state=dict(x=1.,y=2.,z=.6,yaw=90.,vx=0.,vy=1.,vz=0.),
        rsu_pose=dict(x=3.,y=4.,z=2.,yaw=0.),grid_center_x=0.,grid_center_y=0.,
        resolution_m=.2,nx=300,ny=300,fixed_delta_seconds=.05,prediction_horizon_s=6.,
        random_seed=42,tm_seed=42,reference_frame=100,reference_simulation_time_s=5.,time_reference_id='e:100')
    result=experiment_alignment({'experiment':e},{'experiment':dict(e)})
    assert result['compatible'] is True and result['status']=='matched'
    changed=dict(e,target_initial_state=dict(e['target_initial_state'],x=2.))
    result=experiment_alignment({'experiment':e},{'experiment':changed})
    assert result['compatible'] is False and result['mismatched_fields']==['target_initial_state']


def test_failed_callback_invalidates_frame(tmp_path):
    p,m=mask_files(tmp_path,np.ones((300,300),dtype=bool))
    r=PrioritySnapshot(tmp_path/'snap',metadata(),p,m,experiment())
    meas=SimpleNamespace(frame=1,timestamp=.05)
    r.callback('ego',lambda m:None,meas);r.callback('rsu',lambda m:None,meas)
    def fails(m):raise RuntimeError('test')
    with pytest.raises(RuntimeError):r.callback('ego',fails,meas)
    assert not r.save_if_ready(np.zeros((300,300),dtype=np.float32),np.zeros((300,300),dtype=np.float32))


def test_existing_communication_and_fusion_unchanged():
    # AST fingerprints captured from actual source BEFORE this phase's edits.
    expected={
        'SimChannel':'37ee0d4ba11fc8293cc05f926f7323bcf28dcf95bb434b537ad95c95835873db',
        'encode_grid_q8':'a202ea4704cd0de10fd8d57f6cb39cb563180bdeefea9494d97d4d73add8dfbe',
        'decode_grid_q8':'9db47c21e5d0538c73fdb7e3ba34ad8070128334415ac612b20b3f4f1926eeae',
        'fuse_logodds_prefer_ego':'b83093275125d38286866b424f7c4f84bd0e705fed37e5a649fe34858f92092b',
        'update_from_points':'d405fa86da0c1c34f8416f61728a654249420f6c5eb9a1ba454f3d8ac5653ed1',
        'world_to_grid':'809a3d8d42a78ec10b0339fb2e40c1f5f426d5e294f09681dce5865af8d5f4ec'}
    tree=ast.parse((ROOT/'src/ogm_project/coop_comm_compat.py').read_text(encoding='utf-8-sig'))
    for name,digest in expected.items():
        node=next(n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name)
        assert hashlib.sha256(ast.dump(node,include_attributes=False).encode()).hexdigest()==digest


def test_default_callback_bypasses_debug_and_flag_defaults_off(monkeypatch):
    import argparse
    tree=ast.parse((ROOT/'src/ogm_project/coop_comm_compat.py').read_text(encoding='utf-8-sig'))
    parse=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='parse_args')
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    wrapper=next(n for n in ast.walk(main) if isinstance(n,ast.FunctionDef) and n.name=='safe_listen')
    ns={'argparse':argparse,'priority_debug':None}
    exec(compile(ast.Module(body=[parse,wrapper],type_ignores=[]),'actual_coop_functions','exec'),ns)
    monkeypatch.setattr(sys,'argv',['run_coop_comm.py'])
    assert ns['parse_args']().save_priority_debug is False
    received=[];ns['safe_listen'](received.append,'ego')('measurement')
    assert received==['measurement']
    monkeypatch.setattr(sys,'argv',['run_coop_comm.py','--save-priority-debug','--priority-mask-metadata','mask.json'])
    args=ns['parse_args']();assert args.save_priority_debug and args.priority_mask_metadata=='mask.json'
