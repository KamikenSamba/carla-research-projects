"""End-to-end saved-measurement replay and diagnostic CSV contract."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'scripts'))

import analyze_phase1b as analysis
from audit_phase1b_artifacts import maximum_box_interior_depth
from ogm_project.grid_contract import GridContract
from ogm_project.phase1a_paired import Phase1aPairedComparison
from ogm_project.phase1b_geometry import box_vertices


def test_obb_interior_depth_distinguishes_center_and_grazing():
    import pytest
    assert maximum_box_interior_depth([-3,0,0],[3,0,0],np.eye(4),[1,1,1],1/3,2/3)==pytest.approx(1.)
    assert maximum_box_interior_depth([-3,0,.99],[3,0,.99],np.eye(4),[1,1,1],1/3,2/3)==pytest.approx(.01)


def test_saved_capture_full_diagnostic_replay(tmp_path,monkeypatch):
    coop=analysis.coop
    for key,value in dict(ORIGIN_X=3.,ORIGIN_Y=3.,RES=1.,nx=6,ny=6,
                          X_MIN=-3.,X_MAX=3.,Y_MIN=-3.,Y_MAX=3.).items():
        monkeypatch.setattr(coop,key,value)
    monkeypatch.setattr(analysis,'render',lambda *args:None)
    capture=tmp_path/'phase1b_capture';capture.mkdir()
    saved=tmp_path/'phase1a_paired';saved.mkdir()
    grid=GridContract('1.0','TownTest','CARLA_WORLD',1.,6,6,0.,6.,0.,6.,3.,3.,'WORLD_Y','WORLD_X','center')
    road=np.ones((6,6),bool);target=np.zeros((6,6),bool);target[1,2:5]=True
    np.save(tmp_path/'road_mask_source.npy',road);np.save(capture/'priority_target_gt.npy',target)
    paired=Phase1aPairedComparison(saved,grid,road,z_min=.1,z_max=2.,lidar_range=100.,
        free_logodds=coop.L_FREE,occupied_logodds=coop.L_OCC,logodds_min=coop.L_MIN,
        logodds_max=coop.L_MAX,world_to_grid=coop.world_to_grid,in_bounds=coop.in_bounds,bresenham=coop.bresenham)
    sensor_matrix=np.eye(4);sensor_matrix[:3,3]=[.2,1.2,3.]
    box_matrix=np.eye(4);box_matrix[:3,3]=[3,1.2,1.]
    geometry=dict(box_matrix=box_matrix.tolist(),extent=[1,.6,.4],
                  vertices=box_vertices(box_matrix,[1,.6,.4]).tolist())
    points=np.array([[5.2,1.2,0.]]*100+[[4.2,2.2,1.],[4.2,1.2,4.]])
    records=[]
    for name,scale in [('ego',1.),('rsu',.15)]:
        for frame in (1,2):
            if frame==2:paired.decay(name,.05,.2,coop.decay_logodds)
            mask=(points[:,2]>.1)&(points[:,2]<2.)
            paired.update(name,points,mask,sensor_matrix[:3,3],scale,coop.update_from_points,frame=frame,timestamp=frame*.05)
            filename=f'{name}_{frame}.npz'
            np.savez(capture/filename,points_world=points,measurement_pose_points=points)
            records.append(dict(sensor=name,file=filename,frame=frame,timestamp=frame*.05,
                free_scale=scale,decay_dt=.05 if frame==2 else None,decay_rate=.2,
                used_sensor_matrix=sensor_matrix.tolist(),measurement_sensor_matrix=sensor_matrix.tolist(),
                world_snapshot_frame=frame,world_snapshot_timestamp=frame*.05,
                actors={'priority_target':geometry}))
        for tag,arr in [('phase0',paired.phase0[name]),('phase1a',paired.phase1a[name])]:
            np.save(saved/f'{name}_logodds_{tag}.npy',arr)
    manifest=dict(grid_metadata=grid.metadata(),experiment={},measurements=records,
        config=dict(grid=dict(center_x=3.,center_y=3.,resolution_m=1.,nx=6,ny=6),
                    lidar=dict(z_min_world=.1,z_max_world=2.,range_m=100.)),
        constants=dict(free=coop.L_FREE,occupied=coop.L_OCC,minimum=coop.L_MIN,maximum=coop.L_MAX))
    (capture/'capture_manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
    result=analysis.analyze(tmp_path)
    for sensor in ('ego','rsu'):
        item=result['sensors'][sensor]
        assert item['new_target_false_free_cells']>0
        assert item['phase0_replay_bit_exact'] and item['phase1a_replay_bit_exact']
        assert item['target_provenance_complete'] and item['extra_count_matches']
        assert item['trade_off']['HIGH']['extra_ray_count']==0
    for filename in ('target_false_free_cells.csv','extra_free_rays_target.csv','sensor_pose_timing.csv',
                     'occlusion_ordering.csv','phase1b_false_free_analysis.json'):
        assert (tmp_path/'phase1b_analysis'/filename).is_file()
