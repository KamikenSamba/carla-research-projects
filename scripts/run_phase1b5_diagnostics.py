"""Three diagnostic-only Target grounding variants; production scenario is untouched."""
from __future__ import annotations

import argparse
import json
import queue
import sys
from pathlib import Path
import numpy as np

PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
import carla
from run_real_priority_integration import (spawn_vehicle,lidar_blueprint,take_frame,short_map,
    build_road_mask,load_json,coop,GridSpec,GridContract,runtime_grid_metadata)
from ogm_project.actor_ground_truth import vehicle_footprint_masks
from ogm_project.phase1a_paired import Phase1aPairedComparison
from ogm_project.phase1b_geometry import segment_box_interval
from ogm_project.phase1b5_diagnostics import clearance,SettleMonitor,transform_record,classify_query
from analyze_phase1b import write_json,write_csv,xyz_fields


def location(xyz):return carla.Location(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2]))
def vector(v):return [float(v.x),float(v.y),float(v.z)]


def attribute_value(attribute):
    if attribute.type==carla.ActorAttributeType.Int:return str(attribute.as_int())
    if attribute.type==carla.ActorAttributeType.Float:return str(attribute.as_float())
    if attribute.type==carla.ActorAttributeType.Bool:return str(attribute.as_bool()).lower()
    return attribute.as_str()


def actor_geometry(actor):
    tf=actor.get_transform();box=actor.bounding_box
    matrix=np.asarray(tf.get_matrix())@np.asarray(carla.Transform(box.location,box.rotation).get_matrix())
    return dict(actor_id=actor.id,actor_type=actor.type_id,actor_matrix=tf.get_matrix(),
        box_matrix=matrix.tolist(),extent=vector(box.extent),
        vertices=[vector(v) for v in box.get_world_vertices(tf)])


def query_hits(world,start,end):
    available=hasattr(world,'cast_ray')
    if not available:return dict(available=False,hits=[],project_available=False,project_hit=None)
    try:
        hits=[dict(location=vector(h.location),label=str(h.label),
                   distance=float(np.linalg.norm(np.array(vector(h.location))-start)))
              for h in world.cast_ray(location(start),location(end))]
        hits.sort(key=lambda h:h['distance'])
        length=float(np.linalg.norm(np.asarray(end)-start))
        project=None
        if hasattr(world,'project_point') and length>0:
            h=world.project_point(location(start),carla.Vector3D(*map(float,np.asarray(end)-start)),length)
            if h is not None:project=dict(location=vector(h.location),label=str(h.label),
                distance=float(np.linalg.norm(np.array(vector(h.location))-start)))
        return dict(available=True,hits=hits,project_available=hasattr(world,'project_point'),project_hit=project)
    except RuntimeError as exc:
        return dict(available=False,hits=[],project_available=False,project_hit=None,error=str(exc))


def infer_target_hit(hit,geometry,all_vehicles):
    """LabelledPoint has no actor ID. Preserve this explicitly as inference."""
    if hit is None:return False
    if hit['label'] not in ('Car','Truck','Bus','Motorcycle','Bicycle'):return False
    p=np.asarray(hit['location']);candidates=[]
    for box in all_vehicles:
        local=(np.linalg.inv(box['box_matrix'])@np.r_[p,1])[:3]
        if np.all(abs(local)<=np.array(box['extent'])+.02):candidates.append(box['actor_id'])
    if candidates==[geometry['actor_id']]:return True
    if not candidates or len(candidates)>1:return None
    return False


def ground_probe(world,target,variant):
    tf=target.get_transform();geom=actor_geometry(target)
    verts=np.array(geom['vertices']);bottom=float(verts[:,2].min());top=float(verts[:,2].max())
    ext=target.bounding_box.extent
    offsets={'center':(0,0),'front':(ext.x+.25,0),'rear':(-ext.x-.25,0),
             'left':(0,-ext.y-.25),'right':(0,ext.y+.25)}
    offsets.update({f'corner_{i}':(x,y) for i,(x,y) in enumerate(
        [(ext.x+.25,ext.y+.25),(ext.x+.25,-ext.y-.25),(-ext.x-.25,ext.y+.25),(-ext.x-.25,-ext.y-.25)])})
    rows=[]
    for name,(x,y) in offsets.items():
        p=tf.transform(carla.Location(x=x,y=y,z=0))
        start=np.array([p.x,p.y,max(top+3,5)]);end=np.array([p.x,p.y,-5.])
        query=query_hits(world,start,end);hits=query['hits']
        road=next((h for h in hits if h['label']=='Roads'),None)
        waypoint=world.get_map().get_waypoint(carla.Location(x=p.x,y=p.y,z=.5),project_to_road=True)
        ref=float(waypoint.transform.location.z) if waypoint else None
        z=road['location'][2] if road else None
        rows.append(dict(variant=variant,frame=int(world.get_snapshot().frame),probe=name,
            probe_x=float(p.x),probe_y=float(p.y),target_x=tf.location.x,target_y=tf.location.y,target_z=tf.location.z,
            obb_bottom_z=bottom,obb_top_z=top,road_reference_z=ref,road_surface_z=z,clearance=clearance(bottom,z),
            vertical_velocity=float(target.get_velocity().z),query_available=query['available'],
            first_hit_z=hits[0]['location'][2] if hits else None,first_hit_label=hits[0]['label'] if hits else None,
            first_project_hit_z=query['project_hit']['location'][2] if query['project_hit'] else None,
            road_surface_definition='Roads-labelled world.cast_ray hit; first overall hit is recorded separately',
            all_hits=json.dumps(hits)))
    return rows


def settle(world,target,dt,path):
    target.set_simulate_physics(True);target.set_enable_gravity(True)
    target.set_target_velocity(carla.Vector3D());target.set_target_angular_velocity(carla.Vector3D())
    target.apply_control(carla.VehicleControl(throttle=0.,brake=1.,hand_brake=True))
    monitor=SettleMonitor();rows=[];converged=False
    for _ in range(round(10/dt)):
        frame=world.tick();tf=target.get_transform();vel=target.get_velocity()
        converged=monitor.update(tf.location.z,vel.z)
        rows.append(dict(frame=frame,elapsed_s=monitor.frames*dt,**transform_record(tf),
                         vz=float(vel.z),consecutive_stable=monitor.consecutive))
        if converged:break
    write_csv(path/'settle_trace.csv',rows)
    result=dict(converged=converged,settle_duration=monitor.frames*dt,settle_frames=monitor.frames,
        settled_transform=transform_record(target.get_transform()),
        final_vertical_velocity=float(target.get_velocity().z),criterion=monitor.metadata())
    write_json(path/'settle_result.json',result)
    if not converged:raise RuntimeError('Target did not settle within 10 simulation seconds')
    return result


def restore_transform(record):
    return carla.Transform(carla.Location(**{k:record[k] for k in ('x','y','z')}),
                           carla.Rotation(**{k:record[k] for k in ('pitch','yaw','roll')}))


def run_variant(world,cfg,root,variant,settled_pose,api):
    path=root/variant;path.mkdir();capture=path/'phase1b_capture';capture.mkdir()
    actors=[];sensors=[];dt=cfg['fixed_delta_seconds'];lidar=cfg['lidar']
    try:
        world.tick()
        ego=spawn_vehicle(world,cfg['ego'],'priority_ego');target=spawn_vehicle(world,cfg['target'],'priority_target')
        actors.extend([ego,target]);settle_result=None
        if variant=='CURRENT':
            # Actor getters use the last snapshot; obtain one after spawning.
            world.tick()
        elif variant=='PHYSICS_SETTLE':settle_result=settle(world,target,dt,path)
        elif variant=='SETTLED_STATIC':
            target.set_transform(restore_transform(settled_pose));world.tick()
        # CURRENT receives no extra control or gravity commands.
        control=target.get_control()
        spawn_audit=dict(variant=variant,configured_target=cfg['target'],observed_transform=transform_record(target.get_transform()),
            physics_enabled_requested=variant=='PHYSICS_SETTLE',gravity_command='enabled' if variant=='PHYSICS_SETTLE' else 'not set; API default true',
            velocity=vector(target.get_velocity()),angular_velocity=vector(target.get_angular_velocity()),
            control=dict(throttle=control.throttle,brake=control.brake,hand_brake=control.hand_brake),
            fixed_method='physics OFF' if variant!='PHYSICS_SETTLE' else 'physics ON, zero initial velocity, brake + hand brake')
        # Query only our dedicated diagnostic scene. Road reference is not OGM input.
        ground=ground_probe(world,target,variant)
        bp=lidar_blueprint(world,lidar,dt)
        ego_sensor=world.spawn_actor(bp,carla.Transform(carla.Location(x=lidar['ego_relative_x'],y=lidar['ego_relative_y'],z=lidar['ego_relative_z'])),attach_to=ego)
        r=cfg['rsu'];rsu_tf=carla.Transform(carla.Location(x=r['x'],y=r['y'],z=r['z']),carla.Rotation(pitch=r['pitch'],yaw=r['yaw']))
        rsu_sensor=world.spawn_actor(bp,rsu_tf);sensors.extend([ego_sensor,rsu_sensor])
        queues={'ego':queue.Queue(),'rsu':queue.Queue()}
        ego_sensor.listen(queues['ego'].put);rsu_sensor.listen(queues['rsu'].put)
        semantic=None;semantic_queue=queue.Queue();semantic_rows=[];semantic_frames=[]
        if api['semantic_blueprint_available']:
            sbp=world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            matched={}
            for key in ('channels','upper_fov','lower_fov','horizontal_fov','range','rotation_frequency','points_per_second','sensor_tick'):
                if bp.has_attribute(key) and sbp.has_attribute(key):
                    value=attribute_value(bp.get_attribute(key));sbp.set_attribute(key,value);matched[key]=value
            api['semantic_matched_attributes']=matched
            api['semantic_unavailable_attributes']=[k for k in ('dropoff_general_rate','dropoff_intensity_limit','dropoff_zero_intensity','noise_stddev') if not sbp.has_attribute(k)]
            api['standard_lidar_attributes']={a.id:attribute_value(a) for a in bp}
            semantic=world.spawn_actor(sbp,rsu_tf);sensors.append(semantic);semantic.listen(semantic_queue.put)
        g=cfg['grid'];hx=g['nx']*g['resolution_m']/2;hy=g['ny']*g['resolution_m']/2
        coop.ORIGIN_X,coop.ORIGIN_Y=g['center_x'],g['center_y'];coop.RES=g['resolution_m'];coop.nx=g['nx'];coop.ny=g['ny']
        coop.X_MIN,coop.X_MAX=-hx,hx;coop.Y_MIN,coop.Y_MAX=-hy,hy
        meta=runtime_grid_metadata(map_name=cfg['map_name'],origin_x=g['center_x'],origin_y=g['center_y'],x_min=-hx,x_max=hx,y_min=-hy,y_max=hy,resolution=g['resolution_m'],nx=g['nx'],ny=g['ny'])
        grid=GridContract.from_metadata(meta)
        road=build_road_mask(world,grid,path/'road_mask_source.npy',path/'road_mask_metadata.json')
        paired=Phase1aPairedComparison(path/'phase1a_paired',grid,road,z_min=lidar['z_min_world'],z_max=lidar['z_max_world'],
            lidar_range=lidar['range_m'],free_logodds=coop.L_FREE,occupied_logodds=coop.L_OCC,logodds_min=coop.L_MIN,logodds_max=coop.L_MAX,
            world_to_grid=coop.world_to_grid,in_bounds=coop.in_bounds,bresenham=coop.bresenham)
        production={n:np.zeros(grid.shape,np.float32) for n in ('ego','rsu')};last={n:None for n in production}
        records=[];queries=[]
        for step in range(round(cfg['warmup_seconds']/dt)):
            frame,measurements=take_frame(world,queues)
            geometry={a.attributes['role_name']:actor_geometry(a) for a in actors};box=geometry['priority_target']
            for name,sensor,scale in [('ego',ego_sensor,1.),('rsu',rsu_sensor,lidar['rsu_free_scale'])]:
                m=measurements[name];raw=np.frombuffer(m.raw_data,np.float32).reshape(-1,4)[:,:3]
                tf=sensor.get_transform();points=coop.transform_to_world(raw,tf);start=np.array(vector(tf.location))
                passed=(points[:,2]>paired.z_min)&(points[:,2]<paired.z_max)
                decay_dt=None if last[name] is None else m.timestamp-last[name]
                rate=coop.DECAY_PER_SEC_EGO if name=='ego' else coop.DECAY_PER_SEC_RSU
                if decay_dt is not None:
                    coop.decay_logodds(production[name],decay_dt,rate);paired.decay(name,decay_dt,rate,coop.decay_logodds)
                last[name]=m.timestamp
                if passed.any():coop.update_from_points(points[passed],start[:2],production[name],free_scale=scale)
                paired.update(name,points,passed,start,scale,coop.update_from_points,frame=m.frame,timestamp=m.timestamp)
                filename=f'{name}_{m.frame}.npz'
                np.savez_compressed(capture/filename,raw_xyz=raw,points_world=points,measurement_pose_points=coop.transform_to_world(raw,m.transform))
                snapshot=world.get_snapshot()
                records.append(dict(sensor=name,frame=int(m.frame),timestamp=float(m.timestamp),world_snapshot_frame=int(snapshot.frame),
                    world_snapshot_timestamp=float(snapshot.timestamp.elapsed_seconds),used_sensor_matrix=tf.get_matrix(),measurement_sensor_matrix=m.transform.get_matrix(),
                    actors=geometry,file=filename,free_scale=scale,decay_dt=decay_dt,decay_rate=rate))
                if name!='rsu':continue
                for raw_index in np.flatnonzero(~passed):
                    hit=points[raw_index];interval=segment_box_interval(start,hit,box['box_matrix'],box['extent'])
                    if interval is None:continue
                    distance=float(np.linalg.norm(hit-start));entry,exit=np.array(interval)*distance
                    before=int(world.get_snapshot().frame);q=query_hits(world,start,hit);after=int(world.get_snapshot().frame)
                    first=q['project_hit'];inferred=infer_target_hit(first,box,list(geometry.values()))
                    same=before==after==m.frame
                    cls=classify_query(available=q['project_available'],same_frame=same,first_hit_is_target=inferred,
                        first_hit_distance=first['distance'] if first else None,target_exit_distance=exit,endpoint_distance=distance)
                    cast_first=q['hits'][0] if q['hits'] else None
                    queries.append(dict(variant=variant,ray_id=f'rsu:{m.frame}:{raw_index}',frame=int(m.frame),raw_return_index=int(raw_index),
                        measurement_timestamp=float(m.timestamp),query_snapshot_before=before,query_snapshot_after=after,same_frame=same,
                        **xyz_fields('sensor',start),**xyz_fields('endpoint',hit),obb_intersection=True,
                        collision_query_available=q['available'],primary_query_api='world.project_point (finite distance)',
                        first_hit_distance=first['distance'] if first else None,
                        first_hit_x=first['location'][0] if first else None,first_hit_y=first['location'][1] if first else None,first_hit_z=first['location'][2] if first else None,
                        first_hit_label=first['label'] if first else None,first_hit_actor_id=None,first_hit_actor_type=None,
                        first_hit_is_target=inferred,inferred_first_hit_actor_id=target.id if inferred else None,
                        actor_identity_source='inferred: vehicle label and unique live OBB containment; API does not return actor ID',
                        target_entry_distance=float(entry),target_exit_distance=float(exit),endpoint_distance=distance,
                        geometry_class=cls,raw_query_contradiction_candidate=bool(inferred and first['distance']<distance-.01),
                        cast_first_label=cast_first['label'] if cast_first else None,
                        cast_first_distance=cast_first['distance'] if cast_first else None,
                        cast_first_is_target=infer_target_hit(cast_first,box,list(geometry.values())),
                        cast_all_hits=json.dumps(q['hits']),project_query_available=q['project_available']))
            if semantic:
                found=None
                try:
                    while True:
                        item=semantic_queue.get(timeout=2)
                        if item.frame==frame:found=item;break
                        if item.frame>frame:break
                except queue.Empty:pass
                semantic_frames.append(dict(frame=frame,available=found is not None))
                if found:
                    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('cos','<f4'),('object_idx','<u4'),('object_tag','<u4')])
                    points=np.frombuffer(found.raw_data,dtype=dtype);raw=np.column_stack([points[c] for c in 'xyz'])
                    world_points=coop.transform_to_world(raw,found.transform)
                    target_xy=np.array(vector(target.get_location()))[:2]
                    selection=(np.linalg.norm(world_points[:,:2]-target_xy,axis=1)<6)|(points['object_idx']==target.id)
                    for idx in np.flatnonzero(selection):
                        semantic_rows.append(dict(variant=variant,frame=frame,object_idx=int(points['object_idx'][idx]),
                            object_tag=int(points['object_tag'][idx]),**xyz_fields('world_hit',world_points[idx]),
                            is_target=bool(points['object_idx'][idx]==target.id)))
            if (step+1)%5==0:print(f'{variant}: measurements {step+1}/20, diagnostic queries {len(queries)}',flush=True)
        _,masks=vehicle_footprint_masks(actors,grid.shape,coop.world_to_grid)
        experiment=dict(scenario_name=cfg['scenario_name'],diagnostic_variant=variant,map_name=cfg['map_name'],
            random_seed=cfg['random_seed'],tm_seed=cfg['tm_seed'],fixed_delta_seconds=dt,
            reference_frame=frame,target_final_transform=transform_record(target.get_transform()))
        paired.experiment=experiment;paired.finalize(production['ego'],production['rsu'],masks)
        for role,mask in masks.items():np.save(capture/f'{role}_gt.npy',mask)
        write_json(capture/'capture_manifest.json',dict(experiment=experiment,grid_metadata=meta,config=cfg,measurements=records,
            constants=dict(free=coop.L_FREE,occupied=coop.L_OCC,minimum=coop.L_MIN,maximum=coop.L_MAX)))
        ground+=ground_probe(world,target,variant)
        write_csv(path/'target_ground_clearance.csv',ground)
        write_csv(path/'rsu_collision_geometry_candidates.csv',queries)
        write_csv(path/'semantic_lidar_target_neighborhood.csv',semantic_rows)
        final_tf=transform_record(target.get_transform())
        summary=dict(spawn_audit=spawn_audit,ground=ground,settle=settle_result,
            final_target_transform=final_tf,final_vertical_velocity=float(target.get_velocity().z),
            semantic=dict(available=semantic is not None,matched_frames=sum(f['available'] for f in semantic_frames),
                target_hits=sum(p['is_target'] for p in semantic_rows),neighborhood_points=len(semantic_rows),
                note='Auxiliary visibility only; no pairing with standard LiDAR rays'),
            query_count=len(queries),actor_geometry=actor_geometry(target))
        write_json(path/'variant_capture_summary.json',summary)
        return final_tf,summary
    finally:
        for sensor in sensors:
            try:sensor.stop();sensor.destroy()
            except RuntimeError:pass
        for actor in actors:
            try:actor.destroy()
            except RuntimeError:pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=PROJECT/'configs/priority_crossing_v1.json')
    args=parser.parse_args();root=args.output_dir
    if root.exists() and any(root.iterdir()):raise ValueError('output directory must be empty')
    root.mkdir(parents=True,exist_ok=True);cfg=load_json(args.config)
    client=carla.Client('127.0.0.1',2000);client.set_timeout(60);world=client.get_world()
    if short_map(world)!=cfg['map_name']:world=client.load_world(cfg['map_name'])
    if list(world.get_actors().filter('vehicle.*')):raise RuntimeError('Dedicated empty CARLA world required')
    original=world.get_settings();settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=cfg['fixed_delta_seconds'];world.apply_settings(settings)
    tm=client.get_trafficmanager(8000);tm.set_synchronous_mode(True);tm.set_random_device_seed(cfg['tm_seed'])
    api=dict(server_version=client.get_server_version(),client_version=client.get_client_version(),
        cast_ray_available=hasattr(world,'cast_ray'),project_point_available=hasattr(world,'project_point'),
        labelled_point_has_actor_id=hasattr(carla.LabelledPoint,'actor_id'),
        semantic_blueprint_available=bool(world.get_blueprint_library().filter('sensor.lidar.ray_cast_semantic')),
        query_semantics='cast_ray: overlap channel3, default trace params; project_point: channel2 default; LiDAR: channel2 complex trace')
    try:
        settled=None
        for variant in ('CURRENT','PHYSICS_SETTLE','SETTLED_STATIC'):
            pose,summary=run_variant(world,cfg,root,variant,settled,api)
            if variant=='PHYSICS_SETTLE':settled=pose
            write_json(root/'api_capabilities.json',api)
            print(f'{variant} done: target z={pose["z"]:.6f}',flush=True)
    finally:
        tm.set_synchronous_mode(False);world.apply_settings(original)


if __name__=='__main__':main()
