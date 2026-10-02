"""Join same-frame query results with final false-Free provenance across variants."""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter,defaultdict
from pathlib import Path
import sys
import numpy as np

PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/'src'))
from ogm_project.phase1a_paired import classify
from ogm_project.phase1b_geometry import footprint,new_target_false_free
from ogm_project.phase1b5_diagnostics import variant_sensor_summary
from analyze_phase1b import write_csv,write_json

VARIANTS=('CURRENT','PHYSICS_SETTLE','SETTLED_STATIC')


def read_json(path):return json.loads(path.read_text(encoding='utf-8'))
def read_csv(path):
    with path.open(encoding='utf-8') as stream:return list(csv.DictReader(stream))


def aggregate_geometry(cells,rays,candidates):
    queries={r['ray_id']:r for r in candidates}
    suspicious={r['ray_id']:r for r in rays if r['obb_3d_intersection']=='True'}
    selected=[]
    for key,r in suspicious.items():
        q=queries.get(key)
        if q is None:
            q={k:None for k in (candidates[0] if candidates else {})}
            q.update(ray_id=key,geometry_class='UNRESOLVED',missing_query=True)
        else:q=dict(q,missing_query=False)
        selected.append(q)
    counts=Counter(q['geometry_class'] for q in selected)
    by_cell=defaultdict(list)
    for r in rays:by_cell[(int(r['updated_ix']),int(r['updated_iy']))].append(r)
    cell_rows=[]
    for c in cells:
        rr=by_cell[(int(c['ix']),int(c['iy']))]
        # Raster-expanded footprint is a separate measurable evaluation artifact.
        outside=bool(rr) and all(r['cell_center_in_exact_footprint']=='False' for r in rr)
        evidence=Counter(queries.get(r['ray_id'],{}).get('geometry_class','UNRESOLVED') for r in rr)
        cls='BOUNDARY_RASTERIZATION' if outside else (evidence.most_common(1)[0][0] if evidence else 'UNRESOLVED')
        cell_rows.append(dict(ix=int(c['ix']),iy=int(c['iy']),geometry_class=cls,
            cell_center_outside_exact_footprint=outside,
            target_geometry_ray_updates=evidence['OBB_AND_TARGET_GEOMETRY'],
            obb_only_ray_updates=evidence['OBB_ONLY'],unresolved_ray_updates=evidence['UNRESOLVED']))
    summary=dict(suspicious_obb_rays=len(suspicious),
        target_first_hit_rays=counts['OBB_AND_TARGET_GEOMETRY'],obb_only_rays=counts['OBB_ONLY'],
        target_first_hit_inferred_rays=counts['OBB_AND_TARGET_GEOMETRY'],
        obb_only_candidate_rays=counts['OBB_ONLY'],
        unresolved_rays=counts['UNRESOLVED'],
        unresolved_under_exact_standard_lidar_query=len(suspicious),
        same_frame_query_rays=sum(q.get('same_frame')=='True' for q in selected),
        query_contradiction_candidates=sum(q.get('raw_query_contradiction_candidate')=='True' for q in selected),
        cast_ray_first_target_inferred=sum(q.get('cast_first_is_target')=='True' for q in selected),
        cast_ray_first_label_counts=dict(Counter(q.get('cast_first_label','missing') for q in selected)),
        project_point_first_label_counts=dict(Counter(q.get('first_hit_label','missing') for q in selected)),
        cells=dict(Counter(r['geometry_class'] for r in cell_rows)),
        actor_id_directly_returned=False,
        target_identity_method='Car/vehicle semantic label plus unique live Target OBB containment; inferred actor identity',
        primary_query='world.project_point, channel2 default trace parameters',
        secondary_query='world.cast_ray, overlap channel3',
        lidar_query_equivalence=False)
    return summary,selected,cell_rows


def render(root,data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from ogm_project.actor_ground_truth import _convex_hull
    a,b=data['CURRENT'],data['SETTLED_STATIC']
    fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    all_verts=np.concatenate([np.array(x['capture']['actor_geometry']['vertices']) for x in (a,b)])
    for ax,variant,item in zip(axes,('CURRENT','SETTLED_STATIC'),(a,b)):
        verts=np.array(item['capture']['actor_geometry']['vertices']);poly=np.array(_convex_hull(list(map(tuple,verts[:,[1,2]]))))
        ax.fill(poly[:,0],poly[:,1],color='green',alpha=.2,label='Target OBB (Y-Z projection)')
        ground=item['ground'];ax.axhline(ground['road_surface_z'],color='brown',label='Roads-labelled query surface')
        ax.axhline(ground['road_reference_z'],color='black',ls=':',label='Waypoint road reference')
        pose=item['capture']['final_target_transform'];ax.scatter(pose['y'],pose['z'],color='black',marker='x',label='Actor origin')
        rays=[r for r in item['rays'] if r['sensor']=='ego']
        if rays:
            r=rays[0];ax.plot([float(r['sensor_world_y']),float(r['endpoint_world_y'])],
                              [float(r['sensor_world_z']),float(r['endpoint_world_z'])],color='orange',label='Actual contributing Ego ray')
        ax.set(xlim=(all_verts[:,1].min()-.5,all_verts[:,1].max()+.5),ylim=(-.2,2.3),
               xlabel='WORLD Y [m]',ylabel='WORLD Z [m]',title=f"{variant}\nOBB-road clearance = {ground['clearance']:.4f} m")
        ax.legend(fontsize=8)
    fig.savefig(root/'target_ground_contact_current_vs_settled.png',dpi=150);plt.close(fig)
    meta=read_json(root/'CURRENT/phase1b_capture/capture_manifest.json')['grid_metadata']
    extent=(meta['x_min_m'],meta['x_max_m'],meta['y_min_m'],meta['y_max_m'])
    bounds=(all_verts[:,0].min()-.6,all_verts[:,0].max()+.6,all_verts[:,1].min()-.6,all_verts[:,1].max()+.6)
    for sensor in ('ego','rsu'):
        fig,axes=plt.subplots(2,3,figsize=(12,9),constrained_layout=True)
        for row,variant in enumerate(('CURRENT','SETTLED_STATIC')):
            path=root/variant
            p0,_=classify(np.load(path/f'phase1a_paired/{sensor}_logodds_phase0.npy'))
            p1,_=classify(np.load(path/f'phase1a_paired/{sensor}_logodds_phase1a.npy'))
            target=np.load(path/'phase1b_capture/priority_target_gt.npy');new=new_target_false_free(target,p0,p1)
            poly=footprint(data[variant]['capture']['actor_geometry']['vertices']);poly=np.vstack((poly,poly[0]))
            for ax,array,title,cmap,hi in zip(axes[row],[p0,p1,np.where(new,1,np.nan)],
                    ['Phase 0','Phase 1a',f'New Target false Free ({int(new.sum())})'],
                    [ListedColormap(['white','silver','black'])]*2+['Reds'],[2,2,1]):
                ax.imshow(array,origin='lower',extent=extent,interpolation='nearest',cmap=cmap,vmin=0,vmax=hi)
                ax.plot(poly[:,0],poly[:,1],color='green',lw=1)
                ax.set(xlim=bounds[:2],ylim=bounds[2:],title=variant+'\n'+title,xlabel='WORLD X [m]',ylabel='WORLD Y [m]')
                ax.set_aspect('equal')
        fig.suptitle(sensor.upper()+' same WORLD extent; green = exact Target footprint')
        fig.savefig(root/f'{sensor}_false_free_current_vs_settled.png',dpi=150);plt.close(fig)
    examples=[]
    for variant in VARIANTS:
        for cls in ('OBB_AND_TARGET_GEOMETRY','OBB_ONLY'):
            q=next((r for r in data[variant]['selected_queries'] if r['geometry_class']==cls),None)
            if q:examples.append((variant,cls,q))
    if examples:
        from ogm_project.phase1b_geometry import vertical_ray_plane_box_section
        fig,axes=plt.subplots(len(examples),1,figsize=(12,4*len(examples)),squeeze=False,constrained_layout=True)
        for ax,(variant,cls,q) in zip(axes[:,0],examples):
            start=np.array([float(q['sensor_'+c]) for c in 'xyz']);end=np.array([float(q['endpoint_'+c]) for c in 'xyz'])
            length=np.linalg.norm(end[:2]-start[:2]);unit=(end[:2]-start[:2])/length
            box=data[variant]['capture']['actor_geometry'];poly=vertical_ray_plane_box_section(start,end,box['box_matrix'],box['extent'])
            if len(poly)>=3:ax.fill(poly[:,0],poly[:,1],color='green',alpha=.2,label='OBB exact vertical slice')
            ax.plot([0,length],[start[2],end[2]],color='blue',label='Raw LiDAR segment')
            ax.scatter([0,length],[start[2],end[2]],marker='x',color='black',label='Sensor / raw endpoint')
            if q.get('first_hit_x'):
                hit=np.array([float(q['first_hit_'+c]) for c in 'xyz']);position=(hit[:2]-start[:2])@unit
                ax.scatter(position,hit[2],color='red',marker='o',label='project_point first hit ('+q['first_hit_label']+')')
            ax.set(title=variant+' | '+cls+' | '+q['ray_id'],xlabel='Distance along ray XY [m]',ylabel='WORLD Z [m]')
            ax.legend(fontsize=8)
        fig.savefig(root/'rsu_collision_geometry_examples.png',dpi=150);plt.close(fig)


def summarize(root):
    root=Path(root);data={};ground_summary={};geometry_summary={};ground_rows=[];comparison_rows=[];all_queries=[];all_geometry_cells=[]
    for variant in VARIANTS:
        path=root/variant;capture=read_json(path/'variant_capture_summary.json')
        paired=read_json(path/'phase1a_paired/phase1a_paired_comparison.json')
        analysis=read_json(path/'phase1b_analysis/phase1b_false_free_analysis.json')
        rays=read_csv(path/'phase1b_analysis/extra_free_rays_target.csv')
        cells=read_csv(path/'phase1b_analysis/target_false_free_cells.csv')
        ground=next(r for r in reversed(capture['ground']) if r['probe']=='center')
        rows=[dict(variant=variant,sensor=s,**variant_sensor_summary(paired['sensors'][s],analysis['sensors'][s])) for s in ('ego','rsu')]
        geometry,selected,geometry_cells=aggregate_geometry([r for r in cells if r['sensor']=='rsu'],[r for r in rays if r['sensor']=='rsu'],
                                                          read_csv(path/'rsu_collision_geometry_candidates.csv'))
        geometry['collision_query_available']=read_json(root/'api_capabilities.json')['cast_ray_available']
        geometry_summary[variant]=geometry
        ground_summary[variant]=dict(target_transform_z=ground['target_z'],obb_bottom_z=ground['obb_bottom_z'],
            obb_top_z=ground['obb_top_z'],road_reference_z=ground['road_reference_z'],road_surface_z=ground['road_surface_z'],clearance=ground['clearance'],
            ego_new_false_free=rows[0]['new_false_free'],rsu_new_false_free=rows[1]['new_false_free'],
            ego_xy_only_below=rows[0]['xy_only_below'],rsu_3d_intersection=rows[1]['3d_intersection'],
            sensors={r['sensor']:r for r in rows},settle=capture['settle'],final_target_transform=capture['final_target_transform'],
            final_vertical_velocity=capture['final_vertical_velocity'],semantic=capture['semantic'],
            ray_cell_height_counts={s:dict(Counter(r['height_class'] for r in rays if r['sensor']==s)) for s in ('ego','rsu')},
            paired_same_measurements=paired['paired_same_measurements'],paired_same_decay=paired['paired_same_decay'],
            phase0_matches_production=paired['phase0_shadow_matches_production'],
            replay_bit_exact={s:analysis['sensors'][s]['phase0_replay_bit_exact'] and analysis['sensors'][s]['phase1a_replay_bit_exact'] for s in ('ego','rsu')})
        # Only the final, frame-aligned probes are used in cross-variant results.
        # Early CURRENT telemetry in the first capture preceded the first tick.
        final_probe_frame=max(r['frame'] for r in capture['ground'])
        ground_rows+=[r for r in capture['ground'] if r['frame']==final_probe_frame]
        comparison_rows+=rows;all_queries+=selected
        all_geometry_cells+=[dict(variant=variant,**r) for r in geometry_cells]
        data[variant]=dict(capture=capture,ground=ground,rays=rays,selected_queries=selected)
    write_json(root/'phase1b5_ground_contact_analysis.json',ground_summary)
    write_json(root/'phase1b5_rsu_geometry_analysis.json',geometry_summary)
    write_csv(root/'target_ground_clearance.csv',ground_rows)
    write_csv(root/'phase1b5_false_free_by_variant.csv',comparison_rows)
    write_csv(root/'rsu_collision_geometry_rays.csv',all_queries)
    write_csv(root/'rsu_false_free_geometry_cells.csv',all_geometry_cells)
    render(root,data)
    print(json.dumps(dict(ground=ground_summary,geometry=geometry_summary),indent=2))
    return ground_summary,geometry_summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_dir',type=Path)
    summarize(parser.parse_args().run_dir)
