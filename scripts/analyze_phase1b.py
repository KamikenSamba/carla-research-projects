"""Replay captured Phase 1a measurements without CARLA; diagnose Target false Free.

LOW/HIGH effects are counterfactual diagnostic replays with identical valid rays,
decay, update ordering and clipping. They never feed production or Phase 1a.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'src'))
from ogm_project import coop_comm_compat as coop
from ogm_project.grid_contract import GridContract
from ogm_project.phase1a_paired import Phase1aPairedComparison, classify, LABELS
from ogm_project.height_slab_free_space import trace_height_slab_ray, ray_segment_in_height_slab
from ogm_project.phase1b_geometry import (CAUSES, aggregate_cell, distribution, endpoint_class,
    footprint, new_target_false_free, ray_geometry, segment_box_interval, segment_polygon_interval,
    vertical_ray_plane_box_section)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def write_csv(path, rows):
    fields = list(rows[0]) if rows else ['sensor','ray_id']
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def xyz_fields(prefix, xyz):
    return {prefix+'_'+axis: float(value) for axis, value in zip('xyz', xyz)}


def render(out, sensor, grid, labels0, labels1, target, new, cell_rows, ray_rows, geometry):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    colors = ['crimson','royalblue','darkorange','purple','gray']
    poly = footprint(geometry['vertices']); poly = np.vstack((poly, poly[0]))
    ys, xs = np.where(target)
    bounds = (grid.x_min_m+(xs.min()-5)*grid.resolution_m,
              grid.x_min_m+(xs.max()+6)*grid.resolution_m,
              grid.y_min_m+(ys.min()-5)*grid.resolution_m,
              grid.y_min_m+(ys.max()+6)*grid.resolution_m)
    cause_map = np.full(grid.shape, np.nan)
    for r in cell_rows: cause_map[r['iy'],r['ix']] = CAUSES.index(r['cause_class'])
    fig, axes = plt.subplots(2, 3, figsize=(15, 11), constrained_layout=True)
    panels = [(labels0,'Phase 0 classification\nFree / Unknown / Occupied',ListedColormap(['white','silver','black']),0,2),
              (labels1,'Phase 1a classification\nFree / Unknown / Occupied',ListedColormap(['white','silver','black']),0,2),
              (np.where(new,1,np.nan),'New Target false Free','Reds',0,1),
              (np.where(target,1,np.nan),'Raster GT and exact XY footprint','Greens',0,1),
              (np.where(new,1,np.nan),'Contributing ray XY (sample, raw segments)','Greys',0,1),
              (cause_map,'Primary cause per cell (most ray updates)',ListedColormap(colors),0,4)]
    for ax, (data,title,cmap,lo,hi) in zip(axes.flat, panels):
        ax.imshow(data,origin='lower',extent=grid.extent,cmap=cmap,vmin=lo,vmax=hi,interpolation='nearest')
        ax.plot(poly[:,0],poly[:,1],color='green',lw=1.5)
        ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel='WORLD X [m]',ylabel='WORLD Y [m]',title=title)
        ax.title.set_fontsize(10)
        ax.set_aspect('equal')
    unique = {r['ray_id']:r for r in ray_rows}
    for cause, color in zip(CAUSES,colors):
        candidates = [r for r in unique.values() if r['cause_class']==cause]
        for r in candidates[::max(1,len(candidates)//35)][:35]:
            axes[1,1].plot([r['sensor_world_x'],r['endpoint_world_x']],
                           [r['sensor_world_y'],r['endpoint_world_y']],color=color,alpha=.25,lw=.7)
    fig.legend(handles=[Patch(color=c,label=n) for c,n in zip(colors,CAUSES)],loc='outside lower center',ncol=3)
    fig.suptitle(sensor.upper()+' Target false Free diagnosis; green = exact OBB projection')
    fig.savefig(out/f'{sensor}_target_false_free_analysis.png',dpi=150); plt.close(fig)

    representatives = []
    for cause in CAUSES:
        candidate = next((r for r in unique.values() if r['cause_class']==cause),None)
        if candidate: representatives.append(candidate)
    if not representatives: return
    fig, axes = plt.subplots(len(representatives),1,figsize=(12,4*len(representatives)),squeeze=False,constrained_layout=True)
    for ax,r in zip(axes[:,0],representatives):
        s = np.array([r['sensor_world_'+c] for c in 'xyz'])
        h = np.array([r['endpoint_world_'+c] for c in 'xyz'])
        direction = h[:2]-s[:2]; length=np.linalg.norm(direction)
        hull=vertical_ray_plane_box_section(s,h,geometry['box_matrix'],geometry['extent'])
        ax.axhspan(r['slab_z_min'],r['slab_z_max'],color='gold',alpha=.18,label='occupancy slab')
        if len(hull)>=3:
            ax.fill(hull[:,0],hull[:,1],color='green',alpha=.25,label='Exact OBB slice in ray vertical plane')
        else:
            ax.text(.5,.9,'Ray vertical plane misses Target OBB',transform=ax.transAxes,ha='center')
        ax.plot([0,length],[s[2],h[2]],color=colors[CAUSES.index(r['cause_class'])],label='raw ray segment')
        ax.scatter([0,length],[s[2],h[2]],marker='x',color='black')
        ax.set(xlabel='Distance along ray XY [m]',ylabel='WORLD Z [m]',
               title=f"{r['ray_id']} | {r['cause_class']} | {r['endpoint_z_class']} | exact OBB={r['obb_3d_intersection']}")
        ax.legend(fontsize=8)
    fig.savefig(out/f'{sensor}_representative_ray_sections.png',dpi=150); plt.close(fig)


def analyze(root, output=None):
    root = Path(root); capture = root/'phase1b_capture'
    out = Path(output) if output else root/'phase1b_analysis'
    out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((capture/'capture_manifest.json').read_text(encoding='utf-8'))
    cfg=manifest['config']; g=cfg['grid']; lidar=cfg['lidar']; const=manifest['constants']
    grid=GridContract.from_metadata(manifest['grid_metadata'])
    coop.ORIGIN_X,coop.ORIGIN_Y=g['center_x'],g['center_y']
    coop.RES=g['resolution_m'];coop.nx=g['nx'];coop.ny=g['ny']
    coop.X_MIN=-g['nx']*g['resolution_m']/2;coop.X_MAX=-coop.X_MIN
    coop.Y_MIN=-g['ny']*g['resolution_m']/2;coop.Y_MAX=-coop.Y_MIN
    if (coop.L_FREE,coop.L_OCC,coop.L_MIN,coop.L_MAX)!=(const['free'],const['occupied'],const['minimum'],const['maximum']):
        raise ValueError('Replay code constants differ from capture')
    road=np.load(root/'road_mask_source.npy'); target=np.load(capture/'priority_target_gt.npy')
    paired=Phase1aPairedComparison(out,grid,road,z_min=lidar['z_min_world'],z_max=lidar['z_max_world'],
        lidar_range=lidar['range_m'],free_logodds=const['free'],occupied_logodds=const['occupied'],
        logodds_min=const['minimum'],logodds_max=const['maximum'],world_to_grid=coop.world_to_grid,
        in_bounds=coop.in_bounds,bresenham=coop.bresenham)
    all_cells=[];all_rays=[];timing=[];summary={}
    for sensor in ('ego','rsu'):
        log0=np.load(root/f'phase1a_paired/{sensor}_logodds_phase0.npy')
        log1=np.load(root/f'phase1a_paired/{sensor}_logodds_phase1a.npy')
        labels0,p0=classify(log0);labels1,p1=classify(log1)
        new=new_target_false_free(target,labels0,labels1)
        np.save(out/f'{sensor}_new_target_false_free.npy',new)
        shadows={k:np.zeros(grid.shape,dtype=np.float32) for k in ('LOW','HIGH')}
        counts={k:np.zeros(grid.shape,dtype=np.uint32) for k in shadows}
        ray_totals={k:0 for k in shadows};rejected_totals={k:0 for k in shadows}
        by_cell=defaultdict(list);ray_rows=[]
        records=[m for m in manifest['measurements'] if m['sensor']==sensor]
        for record_index,m in enumerate(records):
            data=np.load(capture/m['file']);points=data['points_world'];corrected=data['measurement_pose_points']
            start=np.asarray(m['used_sensor_matrix'])[:3,3]
            measured_start=np.asarray(m['measurement_sensor_matrix'])[:3,3]
            mask=(points[:,2]>paired.z_min)&(points[:,2]<paired.z_max)
            if m['decay_dt'] is not None:
                paired.decay(sensor,m['decay_dt'],m['decay_rate'],coop.decay_logodds)
                for arr in shadows.values():coop.decay_logodds(arr,m['decay_dt'],m['decay_rate'])
            paired.update(sensor,points,mask,start,m['free_scale'],coop.update_from_points,
                          frame=m['frame'],timestamp=m['timestamp'])
            if mask.any():
                for arr in shadows.values():coop.update_from_points(points[mask],start[:2],arr,free_scale=m['free_scale'])
            target_box=m['actors']['priority_target']
            timing.append(dict(sensor=sensor,measurement_frame=m['frame'],world_snapshot_frame=m['world_snapshot_frame'],
                frame_difference=m['world_snapshot_frame']-m['frame'],measurement_timestamp=m['timestamp'],
                snapshot_timestamp=m['world_snapshot_timestamp'],
                pose_translation_difference_m=float(np.linalg.norm(start-measured_start)),
                pose_matrix_max_abs_difference=float(np.max(np.abs(np.array(m['used_sensor_matrix'])-m['measurement_sensor_matrix']))),
                endpoint_pose_difference_max_m=float(np.linalg.norm(points-corrected,axis=1).max()),
                target_box_matrix=json.dumps(target_box['box_matrix'])))
            for raw_index in np.flatnonzero(~mask):
                hit=points[raw_index,:3];kind=endpoint_class(hit[2],paired.z_min,paired.z_max)
                rejected_totals[kind]+=1
                updates=trace_height_slab_ray(start,hit,paired.z_min,paired.z_max,paired.lidar_range,
                    coop.world_to_grid,coop.in_bounds,coop.bresenham)
                if not updates.free_cells:continue
                ray_totals[kind]+=1
                amount=m['free_scale']*const['free']
                relevant=[]
                for ix,iy in updates.free_cells:
                    shadows[kind][iy,ix]=np.clip(shadows[kind][iy,ix]+amount,const['minimum'],const['maximum'])
                    counts[kind][iy,ix]+=1
                    if new[iy,ix]:relevant.append((ix,iy))
                if not relevant:continue
                entry,exit=ray_segment_in_height_slab(start,hit,paired.z_min,paired.z_max)
                for ix,iy in relevant:
                    x=grid.x_min_m+(ix+.5)*grid.resolution_m;y=grid.y_min_m+(iy+.5)*grid.resolution_m
                    geom=ray_geometry(start,hit,(x,y),target_box)
                    corrected_geom=ray_geometry(measured_start,corrected[raw_index],(x,y),target_box)
                    cell_polygon=np.array([[x-grid.resolution_m/2,y-grid.resolution_m/2],
                        [x+grid.resolution_m/2,y-grid.resolution_m/2],
                        [x+grid.resolution_m/2,y+grid.resolution_m/2],
                        [x-grid.resolution_m/2,y+grid.resolution_m/2]])
                    cell_center_in_footprint=segment_polygon_interval((x,y),(x,y),footprint(target_box['vertices'])) is not None
                    row=dict(sensor=sensor,carla_frame=m['frame'],measurement_timestamp=m['timestamp'],
                        ray_id=f"{sensor}:{m['frame']}:{raw_index}",raw_return_index=int(raw_index),
                        **xyz_fields('sensor_world',start),**xyz_fields('endpoint_world',hit),
                        endpoint_distance_m=float(np.linalg.norm(hit-start)),endpoint_z_class=kind,
                        endpoint_diagnostic_label='LOW_OR_HIGH_ONLY_hit_actor_unknown',
                        **xyz_fields('slab_entry',entry),**xyz_fields('slab_exit',exit),
                        slab_z_min=paired.z_min,slab_z_max=paired.z_max,
                        free_scale=m['free_scale'],free_logodds_delta=amount,
                        updated_ix=ix,updated_iy=iy,updated_world_x=x,updated_world_y=y,
                        **geom,cell_center_in_exact_footprint=cell_center_in_footprint,
                        continuous_slab_xy_intersects_cell=segment_polygon_interval(entry,exit,cell_polygon) is not None,
                        measurement_pose_obb_intersection=corrected_geom['obb_3d_intersection'],
                        measurement_pose_cause_class=corrected_geom['cause_class'],
                        world_snapshot_frame=m['world_snapshot_frame'])
                    ray_rows.append(row);by_cell[(ix,iy)].append(row)
            if (record_index+1)%5==0:
                print(f'{sensor}: replayed {record_index+1}/{len(records)} measurements',flush=True)
        phase0_equal=bool(np.array_equal(paired.phase0[sensor],log0))
        phase1a_equal=bool(np.array_equal(paired.phase1a[sensor],log1))
        if not phase0_equal or not phase1a_equal:raise RuntimeError('Captured replay differs from live OGM')
        cells=[]
        for iy,ix in np.argwhere(new):
            rows=by_cell[(int(ix),int(iy))];agg=aggregate_cell(rows)
            cause_counts=agg.pop('cause_ray_counts')
            cells.append(dict(sensor=sensor,ix=int(ix),iy=int(iy),
                world_x=grid.x_min_m+(ix+.5)*grid.resolution_m,
                world_y=grid.y_min_m+(iy+.5)*grid.resolution_m,
                phase0_probability=float(p0[iy,ix]),phase1a_probability=float(p1[iy,ix]),
                phase0_label=LABELS[labels0[iy,ix]],phase1a_label=LABELS[labels1[iy,ix]],
                **agg,**{'cause_'+c.lower()+'_ray_count':n for c,n in cause_counts.items()},
                min_z_at_cell=min((r['z_at_cell'] for r in rows),default=None),
                max_z_at_cell=max((r['z_at_cell'] for r in rows),default=None),
                mean_z_at_cell=float(np.mean([r['z_at_cell'] for r in rows])) if rows else None))
        unique={r['ray_id']:r for r in ray_rows}
        cross={}
        for kind in ('LOW','HIGH'):
            subset=[r for r in ray_rows if r['endpoint_z_class']==kind]
            cross[kind]={}
            for cause in CAUSES:
                rr=[r for r in subset if r['cause_class']==cause]
                cross[kind][cause]=dict(rays=len({r['ray_id'] for r in rr}),
                                       cells=len({(r['updated_ix'],r['updated_iy']) for r in rr}))
        trade={}
        for kind,arr in shadows.items():
            lab,_=classify(arr)
            np.save(out/f'{sensor}_{kind.lower()}_only_logodds.npy',arr)
            np.save(out/f'{sensor}_{kind.lower()}_extra_count.npy',counts[kind])
            trade[kind]=dict(rejected_return_count=rejected_totals[kind],extra_ray_count=ray_totals[kind],
                unique_free_updated_cells=int((counts[kind]>0).sum()),
                new_target_false_free_touched_cells=int((new&(counts[kind]>0)).sum()),
                target_new_false_free_counterfactual=int(new_target_false_free(target,labels0,lab).sum()),
                road_unknown_to_free_counterfactual=int((road&(labels0==1)&(lab==0)).sum()),
                actual_road_unknown_to_free_touched=int((road&(labels0==1)&(labels1==0)&(counts[kind]>0)).sum()))
        counts_match=bool(np.array_equal(counts['LOW']+counts['HIGH'],paired.extra_count[sensor]))
        provenance_complete=all(c['extra_free_update_count']==int(paired.extra_count[sensor][c['iy'],c['ix']]) for c in cells)
        if not counts_match or not provenance_complete:raise RuntimeError('Provenance count mismatch')
        cause_counts={c:sum(r['cause_class']==c for r in cells) for c in CAUSES}
        obb=[r for r in unique.values() if r['obb_3d_intersection']]
        summary[sensor]=dict(new_target_false_free_cells=int(new.sum()),expected_previous=46 if sensor=='ego' else 60,
            gt_cells=int(target.sum()),measurements=len(records),cause=cause_counts,
            cause_percent={c:100*n/int(new.sum()) if new.any() else 0 for c,n in cause_counts.items()},
            cause_classification_rule='Exclusive primary = most ray updates; ties use listed CAUSES order. Per-class counts retained.',
            endpoint={k:dict(cells=int((new&(counts[k]>0)).sum()),rays=sum(r['endpoint_z_class']==k for r in unique.values())) for k in counts},
            rays=dict(unique_total=len(unique),ray_cell_rows=len(ray_rows),
                low_total=sum(r['endpoint_z_class']=='LOW' for r in unique.values()),
                high_total=sum(r['endpoint_z_class']=='HIGH' for r in unique.values()),
                obb_intersect_total=len(obb),endpoint_beyond_target_entry=sum(r['endpoint_beyond_target_entry'] for r in obb),
                endpoint_beyond_target_exit=sum(r['d_endpoint']>r['d_target_exit']+1e-6 for r in obb)),
            endpoint_cause_cross=cross,trade_off=trade,
            road_unknown_to_free=dict(low_contribution=trade['LOW']['road_unknown_to_free_counterfactual'],
                                      high_contribution=trade['HIGH']['road_unknown_to_free_counterfactual'],
                                      actual=int((road&(labels0==1)&(labels1==0)).sum())),
            z_at_cell_distribution=distribution([r['z_at_cell'] for r in ray_rows]),
            z_at_cell_by_cause={c:distribution([r['z_at_cell'] for r in ray_rows if r['cause_class']==c]) for c in CAUSES},
            evidence={k:distribution([r[k] for r in cells]) for k in ('extra_ray_count','extra_free_update_count','extra_logodds_total')},
            evidence_note='Sum of nominal negative updates before clipping and decay, not final logodds delta.',
            phase0_replay_bit_exact=phase0_equal,phase1a_replay_bit_exact=phase1a_equal,
            extra_count_matches=counts_match,target_provenance_complete=provenance_complete,
            measurement_pose_obb_disagreements=sum(r['obb_3d_intersection']!=r['measurement_pose_obb_intersection'] for r in unique.values()),
            raster_cell_centers_outside_exact_footprint=len({(r['updated_ix'],r['updated_iy']) for r in ray_rows if not r['cell_center_in_exact_footprint']}),
            slab_xy_misses_updated_cell_rows=sum(not r['continuous_slab_xy_intersects_cell'] for r in ray_rows),
            target_z_min=float(np.min(np.asarray(target_box['vertices'])[:,2])),
            target_z_max=float(np.max(np.asarray(target_box['vertices'])[:,2])))
        render(out,sensor,grid,labels0,labels1,target,new,cells,ray_rows,target_box)
        all_cells.extend(cells);all_rays.extend(ray_rows)
        print(f'{sensor}: {int(new.sum())} new false-Free cells, {len(unique)} contributing rays; {cause_counts}',flush=True)
    write_csv(out/'target_false_free_cells.csv',all_cells)
    write_csv(out/'extra_free_rays_target.csv',all_rays)
    write_csv(out/'sensor_pose_timing.csv',timing)
    write_csv(out/'occlusion_ordering.csv',list({r['ray_id']:r for r in all_rays if r['obb_3d_intersection']}.values()))
    result=dict(phase='1b diagnostics only',experiment=manifest['experiment'],sensors=summary,
        timing=dict(frame_mismatches=sum(r['frame_difference']!=0 for r in timing),
                    max_pose_translation_difference_m=max(r['pose_translation_difference_m'] for r in timing),
                    max_pose_matrix_difference=max(r['pose_matrix_max_abs_difference'] for r in timing)),
        limitations=['Actor bounding box is an envelope, not the LiDAR collision mesh.',
                     'Raw ray-cast returns contain no hit actor or material identity.',
                     'Cell attribution uses all historical extra rays; clipping and decay mean contributions are not additive.',
                     'Height summaries are ray-cell weighted; unique ray counts are deduplicated.',
                     'GT uses the existing raster mask unchanged; exact XY footprint is diagnosed separately.'])
    write_json(out/'phase1b_false_free_analysis.json',result)
    return result


def render_saved(root, output=None):
    """Regenerate figures from saved CSVs without repeating occupancy updates."""
    root=Path(root);out=Path(output) if output else root/'phase1b_analysis'
    manifest=json.loads((root/'phase1b_capture/capture_manifest.json').read_text(encoding='utf-8'))
    grid=GridContract.from_metadata(manifest['grid_metadata'])
    target=np.load(root/'phase1b_capture/priority_target_gt.npy')
    with (out/'target_false_free_cells.csv').open(encoding='utf-8') as f:cells=list(csv.DictReader(f))
    with (out/'extra_free_rays_target.csv').open(encoding='utf-8') as f:rays=list(csv.DictReader(f))
    for r in cells:r['ix']=int(r['ix']);r['iy']=int(r['iy'])
    for r in rays:
        for prefix in ('sensor_world','endpoint_world'):
            for c in 'xyz':r[prefix+'_'+c]=float(r[prefix+'_'+c])
        for k in ('slab_z_min','slab_z_max'):r[k]=float(r[k])
        r['obb_3d_intersection']=r['obb_3d_intersection']=='True'
    for sensor in ('ego','rsu'):
        a,_=classify(np.load(root/f'phase1a_paired/{sensor}_logodds_phase0.npy'))
        b,_=classify(np.load(root/f'phase1a_paired/{sensor}_logodds_phase1a.npy'))
        geometry=next(m['actors']['priority_target'] for m in manifest['measurements'] if m['sensor']==sensor)
        render(out,sensor,grid,a,b,target,new_target_false_free(target,a,b),
               [r for r in cells if r['sensor']==sensor],[r for r in rays if r['sensor']==sensor],geometry)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path);parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--render-only',action='store_true')
    args=parser.parse_args()
    if args.render_only:render_saved(args.run_dir,args.output_dir)
    else:analyze(args.run_dir,args.output_dir)
