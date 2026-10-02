"""Check saved Phase 1b results and quantify OBB boundary/GT artifacts."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from ogm_project.phase1b_geometry import box_vertices, distribution


def maximum_box_interior_depth(start,end,matrix,extent,t0,t1):
    """Exact max of minimum distance to six box faces along intersected segment."""
    inv=np.linalg.inv(matrix)
    a=(inv@np.r_[start,1])[:3];b=(inv@np.r_[end,1])[:3]-a
    intercept=np.r_[np.asarray(extent)-a,np.asarray(extent)+a]
    slope=np.r_[-b,b]
    candidates=[t0,t1]
    for i in range(6):
        for j in range(i):
            if abs(slope[i]-slope[j])<1e-12:continue
            t=(intercept[j]-intercept[i])/(slope[i]-slope[j])
            if t0<=t<=t1:candidates.append(t)
    return max(float(np.min(intercept+t*slope)) for t in candidates)


def audit(root,previous=None):
    root=Path(root);out=root/'phase1b_analysis'
    manifest=json.loads((root/'phase1b_capture/capture_manifest.json').read_text(encoding='utf-8'))
    with (out/'extra_free_rays_target.csv').open(encoding='utf-8') as f:rows=list(csv.DictReader(f))
    result={'sensors':{}}
    target_matrices=np.array([m['actors']['priority_target']['box_matrix'] for m in manifest['measurements']])
    result['target_pose_max_change']=float(np.max(abs(target_matrices-target_matrices[0])))
    vertex_errors=[]
    for m in manifest['measurements']:
        box=m['actors']['priority_target']
        generated=box_vertices(box['box_matrix'],box['extent'])
        original=np.asarray(box['vertices'])
        vertex_errors.extend(min(np.linalg.norm(p-original,axis=1)) for p in generated)
    result['max_reconstructed_vertex_error_m']=float(max(vertex_errors))
    for sensor in ('ego','rsu'):
        subset=[r for r in rows if r['sensor']==sensor]
        rays=list({r['ray_id']:r for r in subset}.values())
        box=next(m['actors']['priority_target'] for m in manifest['measurements'] if m['sensor']==sensor)
        obb=[r for r in rays if r['obb_3d_intersection']=='True']
        entries=[];exits=[];depths=[]
        for r in obb:
            s=np.array([float(r['sensor_world_'+c]) for c in 'xyz'])
            h=np.array([float(r['endpoint_world_'+c]) for c in 'xyz'])
            t0=float(r['d_target_entry'])/float(r['d_endpoint']);t1=float(r['d_target_exit'])/float(r['d_endpoint'])
            entries.append(float((s+t0*(h-s))[2]));exits.append(float((s+t1*(h-s))[2]))
            depths.append(maximum_box_interior_depth(s,h,box['box_matrix'],box['extent'],t0,t1))
        result['sensors'][sensor]=dict(
            distinct_ray_geometries_rounded_1e5_m=len({tuple(round(float(r[p+c]),5)
                for p in ('sensor_world_','endpoint_world_') for c in 'xyz') for r in rays}),
            ray_cell_height_counts=dict(Counter(r['height_class'] for r in subset)),
            obb_entry_z=distribution(entries),obb_exit_z=distribution(exits),
            maximum_box_interior_depth_m=distribution(depths),
            endpoint_z_unique_ray_distribution=distribution([float(r['endpoint_world_z']) for r in rays]),
            new_cells_with_center_outside_exact_footprint=len({(r['updated_ix'],r['updated_iy']) for r in subset if r['cell_center_in_exact_footprint']=='False'}),
            ray_cell_rows_center_outside_exact_footprint=sum(r['cell_center_in_exact_footprint']=='False' for r in subset))
    arrays=list(out.glob('*.npy'))
    result['all_analysis_arrays_finite']=all(np.isfinite(np.load(p)).all() for p in arrays)
    result['analysis_array_count']=len(arrays)
    result['csv_rows']=dict(target_false_free_cells=sum(1 for _ in csv.DictReader((out/'target_false_free_cells.csv').open())),
                            extra_free_rays_target=len(rows))
    if previous:
        previous=Path(previous)
        result['previous_phase1a_bit_exact']={f'{sensor}_{phase}':bool(np.array_equal(
            np.load(root/f'phase1a_paired/{sensor}_logodds_{phase}.npy'),
            np.load(previous/f'phase1a_paired/{sensor}_logodds_{phase}.npy')))
            for sensor in ('ego','rsu') for phase in ('phase0','phase1a')}
        old=json.loads((previous/'experiment_metadata.json').read_text(encoding='utf-8'))
        result['previous_config_sha256_matches']=old['source_config_sha256']==manifest['experiment']['source_config_sha256']
    (out/'phase1b_artifact_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path);parser.add_argument('--previous-run',type=Path)
    args=parser.parse_args();audit(args.run_dir,args.previous_run)
