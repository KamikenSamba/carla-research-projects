import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ogm_project.phase1b5_diagnostics import (clearance,SettleMonitor,transform_record,classify_query,
    variant_sensor_summary,query_hits,infer_target_hit)


@pytest.mark.parametrize('bottom,road,expected',[(.6,.0002,.5998),(.0001,.0002,-.0001),(0,0,0)])
def test_clearance_preserves_sign(bottom,road,expected):
    assert clearance(bottom,road)==pytest.approx(expected)


def test_settle_requires_consecutive_small_velocity_and_position_change():
    m=SettleMonitor(consecutive_required=3,minimum_frames=4)
    assert not m.update(.6,-1)
    assert not m.update(.5,0)
    assert not m.update(.5,0)
    assert not m.update(.5,.02)
    assert not m.update(.5,0)
    assert not m.update(.5,0)
    assert m.update(.5,0)
    assert m.metadata()['consecutive']==3


def test_settled_transform_keeps_pitch_roll_and_xyz():
    t=SimpleNamespace(location=SimpleNamespace(x=1,y=2,z=.04),rotation=SimpleNamespace(pitch=1,yaw=89.2,roll=-.3))
    assert transform_record(t)==dict(x=1.,y=2.,z=.04,pitch=1.,yaw=89.2,roll=-.3)


@pytest.mark.parametrize('available,same,target,hit,expected',[
    (True,True,True,4.,'OBB_AND_TARGET_GEOMETRY'),
    (True,True,False,10.,'OBB_ONLY'),
    (True,True,False,None,'OBB_ONLY'),
    (True,True,False,2.,'UNRESOLVED'),
    (True,True,None,4.,'UNRESOLVED'),
    (False,True,False,None,'UNRESOLVED'),
    (True,False,True,4.,'UNRESOLVED')])
def test_query_classification(available,same,target,hit,expected):
    assert classify_query(available=available,same_frame=same,first_hit_is_target=target,
        first_hit_distance=hit,target_exit_distance=6.,endpoint_distance=10.)==expected


@pytest.mark.parametrize('bad',[float('nan'),float('inf')])
def test_nonfinite_guard(bad):
    with pytest.raises(ValueError):clearance(bad,0)
    with pytest.raises(ValueError):SettleMonitor().update(0,bad)


def test_variant_comparison_schema_preserves_inputs():
    p=dict(road_unknown_to_free=123,occupied_to_unknown=2,occupied_to_free=3)
    a=dict(new_target_false_free_cells=5,cause=dict(XY_ONLY_BELOW=2,XY_ONLY_ABOVE=0,
        **{'3D_INTERSECTION':2},RASTERIZATION_OR_EDGE=1,UNRESOLVED=0))
    original=dict(p)
    row=variant_sensor_summary(p,a)
    assert row['new_false_free']==5 and row['3d_intersection']==2
    assert row['road_unknown_to_free']==123 and p==original


def test_diagnostic_numeric_functions_have_no_ogm_side_effects():
    from test_phase1a_paired import paired,legacy
    import tempfile
    with tempfile.TemporaryDirectory() as folder:
        a=paired(Path(folder)/'off');b=paired(Path(folder)/'on')
        points=np.array([[5.2,1.2,0.],[4.2,1.2,1.]])
        mask=np.array([False,True])
        production=np.zeros((6,6),np.float32)
        legacy(points[mask],(.2,1.2),production)
        for x in (a,b):x.update('ego',points,mask,(.2,1.2,3.),1.,legacy,frame=1,timestamp=.05)
        clearance(.6,0);SettleMonitor().update(.6,0)
        classify_query(available=True,same_frame=True,first_hit_is_target=False,first_hit_distance=10.,target_exit_distance=6.,endpoint_distance=10.)
        np.testing.assert_array_equal(a.phase0['ego'],b.phase0['ego'])
        np.testing.assert_array_equal(a.phase1a['ego'],b.phase1a['ego'])
        np.testing.assert_array_equal(production,b.phase0['ego'])


def test_query_api_unavailable_returns_explicit_status():
    assert query_hits(object(),np.array([0.,0.,0.]),np.array([1.,0.,0.]))['available'] is False


def test_actor_identity_is_inferred_only_for_unique_vehicle_label():
    box=dict(actor_id=42,box_matrix=np.eye(4).tolist(),extent=[1,1,1])
    assert infer_target_hit(dict(label='Car',location=[0,0,0]),box,[box]) is True
    assert infer_target_hit(dict(label='Roads',location=[0,0,0]),box,[box]) is False
    assert infer_target_hit(dict(label='Car',location=[0,0,0]),box,[box,dict(box,actor_id=43)]) is None


def test_geometry_join_handles_missing_query_and_raster_cells(tmp_path):
    import json,csv
    from summarize_phase1b5 import aggregate_geometry
    from analyze_phase1b import write_json,write_csv
    cells=[dict(ix='1',iy='2'),dict(ix='2',iy='2')]
    rays=[dict(ray_id='rsu:1:1',updated_ix='1',updated_iy='2',obb_3d_intersection='True',cell_center_in_exact_footprint='True'),
          dict(ray_id='rsu:1:2',updated_ix='2',updated_iy='2',obb_3d_intersection='True',cell_center_in_exact_footprint='False')]
    candidates=[dict(ray_id='rsu:1:1',geometry_class='OBB_ONLY',same_frame='True')]
    summary,selected,out=aggregate_geometry(cells,rays,candidates)
    assert summary['suspicious_obb_rays']==2
    assert summary['obb_only_rays']==1 and summary['unresolved_rays']==1
    assert summary['cells']=={'OBB_ONLY':1,'BOUNDARY_RASTERIZATION':1}
    write_json(tmp_path/'summary.json',summary);write_csv(tmp_path/'rays.csv',selected)
    assert json.loads((tmp_path/'summary.json').read_text())['actor_id_directly_returned'] is False
    with (tmp_path/'rays.csv').open() as f:
        saved=list(csv.DictReader(f))
    assert saved[1]['missing_query']=='True'
