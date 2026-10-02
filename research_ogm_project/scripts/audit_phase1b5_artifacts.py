"""Fail closed if Phase 1b.5 output contracts or invariants are incomplete."""
from __future__ import annotations

import argparse,csv,json
from pathlib import Path
import numpy as np

VARIANTS=('CURRENT','PHYSICS_SETTLE','SETTLED_STATIC')


def rows(path):
    with path.open(encoding='utf-8') as stream:return list(csv.DictReader(stream))


def require_fields(path,required):
    data=rows(path)
    if not data:raise ValueError(f'{path}: data rows required')
    missing=set(required)-set(data[0])
    if missing:raise ValueError(f'{path}: missing {sorted(missing)}')
    return data


def audit(root):
    root=Path(root)
    ground=json.loads((root/'phase1b5_ground_contact_analysis.json').read_text(encoding='utf-8'))
    geometry=json.loads((root/'phase1b5_rsu_geometry_analysis.json').read_text(encoding='utf-8'))
    if set(ground)!=set(VARIANTS) or set(geometry)!=set(VARIANTS):raise ValueError('variant set mismatch')
    ground_rows=require_fields(root/'target_ground_clearance.csv',
        ['variant','frame','target_x','target_y','target_z','obb_bottom_z','road_surface_z','clearance','vertical_velocity'])
    comparison=require_fields(root/'phase1b5_false_free_by_variant.csv',
        ['variant','sensor','new_false_free','xy_only_below','3d_intersection','rasterization_edge','road_unknown_to_free'])
    queries=require_fields(root/'rsu_collision_geometry_rays.csv',
        ['ray_id','frame','sensor_x','sensor_y','sensor_z','endpoint_x','endpoint_y','endpoint_z','obb_intersection',
         'collision_query_available','first_hit_x','first_hit_y','first_hit_z','first_hit_distance','first_hit_actor_id',
         'first_hit_actor_type','first_hit_is_target','target_entry_distance','endpoint_distance','geometry_class'])
    finite_arrays=0
    for variant in VARIANTS:
        p=root/variant
        paired=json.loads((p/'phase1a_paired/phase1a_paired_comparison.json').read_text(encoding='utf-8'))
        analysis=json.loads((p/'phase1b_analysis/phase1b_false_free_analysis.json').read_text(encoding='utf-8'))
        if not all((paired['paired_same_measurements'],paired['paired_same_decay'],paired['phase0_shadow_matches_production'])):
            raise ValueError(f'{variant}: paired invariant failed')
        for sensor in ('ego','rsu'):
            if not analysis['sensors'][sensor]['phase0_replay_bit_exact'] or not analysis['sensors'][sensor]['phase1a_replay_bit_exact']:
                raise ValueError(f'{variant}/{sensor}: replay mismatch')
        for path in list((p/'phase1a_paired').glob('*.npy'))+list((p/'phase1b_analysis').glob('*.npy')):
            if not np.isfinite(np.load(path)).all():raise ValueError(f'{path}: NaN/Inf')
            finite_arrays+=1
    equality={}
    for sensor in ('ego','rsu'):
        for phase in ('phase0','phase1a'):
            key=f'{sensor}_{phase}'
            equality[key]=bool(np.array_equal(np.load(root/f'PHYSICS_SETTLE/phase1a_paired/{sensor}_logodds_{phase}.npy'),
                                               np.load(root/f'SETTLED_STATIC/phase1a_paired/{sensor}_logodds_{phase}.npy')))
    if not all(equality.values()):raise ValueError('settle/static arrays differ')
    result=dict(status='PASS',variants=list(VARIANTS),ground_csv_rows=len(ground_rows),comparison_csv_rows=len(comparison),
        collision_query_csv_rows=len(queries),finite_array_count=finite_arrays,
        physics_settle_equals_settled_static=equality,
        current_clearance_m=ground['CURRENT']['clearance'],settled_clearance_m=ground['SETTLED_STATIC']['clearance'],
        current_ego_new_false_free=ground['CURRENT']['ego_new_false_free'],
        settled_ego_new_false_free=ground['SETTLED_STATIC']['ego_new_false_free'])
    (root/'phase1b5_artifact_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_dir',type=Path)
    audit(parser.parse_args().run_dir)
