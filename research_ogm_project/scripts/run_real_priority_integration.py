"""Run one deterministic CARLA experiment from OGM snapshot through Priority Map."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import queue
import random
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src'))
DT_PROJECT = Path(r'C:\CARLA\user_projects\dt_risk_prediction_project')
sys.path.insert(0, str(DT_PROJECT))

import carla
from dt_risk_common import (CsvWriter, PredictionState, RiskRegion, RiskEvent,
    prediction_state_fieldnames, risk_region_fieldnames, risk_event_fieldnames, to_row)
from Run_DT_Risk_V1 import detect_near_misses
from risk_grid import GridSpec
from ogm_project import coop_comm_compat as coop
from ogm_project.grid_contract import GridContract, runtime_grid_metadata
from ogm_project.actor_ground_truth import vehicle_footprint_masks
from ogm_project.phase1a_paired import Phase1aPairedComparison
from ogm_project.priority_snapshot import PrioritySnapshot
from ogm_project.risk_priority import build_priority, sha256, write_json
from ogm_project.ogm_engine import (
    OGMGridContract, OGMUpdateConfig, apply_decay, update_lidar_measurement,
    z_filter_mask,
)
from ogm_project.ogm_engine_shadow import OGMEngineShadowComparison
from ogm_project.phase1b5_diagnostics import transform_record
from ogm_project.vehicle_grounding import ground_all_vehicles


class PlannedActor:
    def __init__(self, actor, velocity):
        self.actor, self.velocity, self.id = actor, velocity, actor.id
    def get_location(self): return self.actor.get_location()
    def get_velocity(self): return self.velocity


def load_json(path):
    value = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(value, dict):
        raise ValueError(f'{path}: JSON object required')
    return value


def short_map(world):
    return world.get_map().name.replace('\\', '/').split('/')[-1]


def spawn_vehicle(world, cfg, role):
    bp = world.get_blueprint_library().find(cfg['blueprint'])
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', role)
    tf = carla.Transform(carla.Location(x=cfg['x'], y=cfg['y'], z=cfg['z']),
                         carla.Rotation(pitch=cfg.get('pitch', 0.), yaw=cfg['yaw'], roll=cfg.get('roll', 0.)))
    actor = world.try_spawn_actor(bp, tf)
    if actor is None:
        raise RuntimeError(f'failed to spawn {role} at {tf.location}')
    actor.set_simulate_physics(False)
    return actor


def lidar_blueprint(world, cfg, dt):
    bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    attrs = dict(channels=cfg['channels'], range=cfg['range_m'], rotation_frequency=cfg['rotation_hz'],
                 points_per_second=cfg['points_per_second'], sensor_tick=dt)
    for key, value in attrs.items(): bp.set_attribute(str(key), str(value))
    return bp


def take_frame(world, queues, timeout=2, attempts=30):
    # CARLA may legitimately omit a LiDAR measurement on an individual tick
    # while the sensor starts. Advance until both sensors produced the same
    # world frame; never combine adjacent frames.
    pending = {name: {} for name in queues}
    for _ in range(attempts):
        frame = world.tick()
        measurements = {}
        for name, q in queues.items():
            if frame in pending[name]:
                measurements[name] = pending[name].pop(frame)
                continue
            try:
                while True:
                    item = q.get(timeout=timeout)
                    if item.frame == frame:
                        measurements[name] = item
                        break
                    if item.frame > frame:
                        pending[name][item.frame] = item
                        break
            except queue.Empty:
                pass
        if len(measurements) == len(queues):
            return frame, measurements
    raise RuntimeError(f'no common Ego/RSU LiDAR frame after {attempts} ticks')


def build_road_mask(world, grid, path, metadata_path):
    road = np.zeros((grid.ny, grid.nx), dtype=bool)
    cmap = world.get_map()
    for iy in range(grid.ny):
        y = grid.y_min_m + (iy + .5) * grid.resolution_m
        for ix in range(grid.nx):
            x = grid.x_min_m + (ix + .5) * grid.resolution_m
            road[iy, ix] = cmap.get_waypoint(carla.Location(x=x, y=y, z=.5),
                project_to_road=False, lane_type=carla.LaneType.Driving) is not None
    np.save(path, road)
    meta = dict(runtime_grid_metadata(map_name=grid.map_name, origin_x=grid.center_x, origin_y=grid.center_y,
        x_min=grid.x_min_m-grid.center_x, x_max=grid.x_max_m-grid.center_x,
        y_min=grid.y_min_m-grid.center_y, y_max=grid.y_max_m-grid.center_y,
        resolution=grid.resolution_m, nx=grid.nx, ny=grid.ny),
        mask_semantics='road_true', array_sha256=sha256(path),
        provenance='CARLA Map.get_waypoint(project_to_road=False, lane_type=Driving) at each cell center')
    write_json(metadata_path, meta)
    return road


def actor_state(actor, logical, frame, t, velocity, yaw):
    p = actor.get_location()
    return PredictionState(frame, t, logical, actor.id, p.x, p.y, p.z, yaw,
                           velocity.x, velocity.y, velocity.z,
                           math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2), False)


def pose_dict(cfg, velocity):
    return dict(x=float(cfg['x']), y=float(cfg['y']), z=float(cfg['z']), yaw=float(cfg['yaw']),
                vx=float(velocity.x), vy=float(velocity.y), vz=float(velocity.z))


def pose_from_transform(transform, velocity):
    return dict(x=float(transform['x']), y=float(transform['y']), z=float(transform['z']),
                pitch=float(transform['pitch']), yaw=float(transform['yaw']), roll=float(transform['roll']),
                vx=float(velocity.x), vy=float(velocity.y), vz=float(velocity.z))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=PROJECT/'configs/priority_crossing_v1.json')
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--phase1a-paired-diagnostics', action='store_true',
                   help='Evaluate legacy and Phase 1a shadows on the same priority-crossing measurements')
    p.add_argument('--phase1b-ray-diagnostics', action='store_true',
                   help='Capture measurements and actor OBBs for offline Phase 1b provenance (implies paired diagnostics)')
    p.add_argument('--ogm-engine-shadow-compare', action='store_true',
                   help='Run Legacy/Common OGM Engine shadows on the same CARLA measurements')
    args = p.parse_args(argv)
    if args.phase1b_ray_diagnostics:
        args.phase1a_paired_diagnostics = True
    cfg = load_json(args.config)
    random.seed(cfg['random_seed']); np.random.seed(cfg['random_seed'])
    root = args.output_dir.resolve()
    if root.exists() and any(root.iterdir()):
        raise ValueError(f'output directory must be new or empty: {root}')
    risk_dir, ogm_dir, priority_dir, logs_dir = root/'risk', root/'ogm', root/'priority', root/'logs'
    for d in (risk_dir, ogm_dir, logs_dir): d.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port); client.set_timeout(60)
    world = client.get_world()
    if short_map(world) != cfg['map_name']:
        world = client.load_world(cfg['map_name'])
    original = world.get_settings()
    settings = world.get_settings(); settings.synchronous_mode=True; settings.fixed_delta_seconds=cfg['fixed_delta_seconds']
    world.apply_settings(settings)
    tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True); tm.set_random_device_seed(cfg['tm_seed'])
    actors=[]; sensors=[]
    log=[]
    try:
        # This dedicated server is reset to an empty deterministic scene.
        for actor in list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('sensor.*')):
            try: actor.destroy()
            except RuntimeError: pass
        world.tick()
        spawn_cfg = {role: dict(cfg[role]) for role in ('ego','target')}
        if cfg.get('grounded'):
            # CARLA may reject a direct spawn when the calibrated contact pose
            # overlaps the road collision surface.  Spawn from the recorded
            # legacy/drop pose, then obtain the configured z by physics settle.
            for role in ('ego','target'):
                legacy = cfg[role].get('grounding_provenance',{}).get('legacy_original_transform')
                if not legacy:
                    raise ValueError(f'{role}: grounded scenario requires legacy_original_transform')
                for key in ('x','y','z','pitch','yaw','roll'):
                    spawn_cfg[role][key] = legacy[key]
        ego=spawn_vehicle(world,spawn_cfg['ego'],'priority_ego'); target=spawn_vehicle(world,spawn_cfg['target'],'priority_target')
        actors += [ego,target]
        world.tick()
        configured_transforms = {
            role: {key: float(spawn_cfg[role].get(key, 0.)) for key in ('x','y','z','pitch','yaw','roll')}
            for role in ('ego','target')}
        grounding_metadata = None
        if cfg.get('grounded'):
            if not cfg.get('grounding', {}).get('enabled'):
                raise ValueError('grounded scenario requires grounding.enabled=true')
            grounding_metadata = ground_all_vehicles(
                world, {'ego': ego, 'target': target}, configured_transforms,
                cfg['fixed_delta_seconds'], root/'grounding', settings=cfg['grounding'])
            for role in ('ego','target'):
                provenance=cfg[role].get('grounding_provenance',{})
                grounding_metadata['actors'][role]['legacy_original_transform'] = provenance.get('legacy_original_transform')
                grounding_metadata['actors'][role]['calibrated_settled_transform'] = provenance.get('calibrated_settled_transform')
            grounding_metadata['calibration_artifact'] = cfg['grounding'].get('calibration_artifact')
            grounding_metadata['calibration_artifact_sha256'] = cfg['grounding'].get('calibration_artifact_sha256')
            write_json(root/'grounding_metadata.json', grounding_metadata)
        initial_transforms = {'ego': transform_record(ego.get_transform()),
                              'target': transform_record(target.get_transform())}
        lidar_cfg=cfg['lidar']; dt=cfg['fixed_delta_seconds']; bp=lidar_blueprint(world,lidar_cfg,dt)
        ego_lidar=world.spawn_actor(bp,carla.Transform(carla.Location(
            x=lidar_cfg['ego_relative_x'],y=lidar_cfg['ego_relative_y'],z=lidar_cfg['ego_relative_z'])),attach_to=ego)
        rsu_cfg=cfg['rsu']; rsu_lidar=world.spawn_actor(bp,carla.Transform(
            carla.Location(x=rsu_cfg['x'],y=rsu_cfg['y'],z=rsu_cfg['z']),
            carla.Rotation(pitch=rsu_cfg['pitch'],yaw=rsu_cfg['yaw'])))
        sensors += [ego_lidar,rsu_lidar]
        queues={'ego':queue.Queue(),'rsu':queue.Queue()}
        ego_lidar.listen(queues['ego'].put);rsu_lidar.listen(queues['rsu'].put)

        g=cfg['grid']; half_x=g['nx']*g['resolution_m']/2; half_y=g['ny']*g['resolution_m']/2
        grid=GridSpec(cfg['map_name'],'CARLA_WORLD',g['center_x'],g['center_y'],
            g['center_x']-half_x,g['center_x']+half_x,g['center_y']-half_y,g['center_y']+half_y,
            g['resolution_m'],g['nx'],g['ny'])
        # The reused standard OGM update functions read these runtime globals.
        # Set them from the same config that generated the Grid contract.
        coop.ORIGIN_X, coop.ORIGIN_Y = g['center_x'], g['center_y']
        coop.RES = g['resolution_m']; coop.nx = g['nx']; coop.ny = g['ny']
        coop.X_MIN, coop.X_MAX = -half_x, half_x
        coop.Y_MIN, coop.Y_MAX = -half_y, half_y
        write_json(root/'grid_spec.json',asdict(grid))
        mask_path=root/'road_mask_source.npy'; mask_meta=root/'road_mask_metadata.json'
        road=build_road_mask(world,grid,mask_path,mask_meta)
        logodds_ego=np.zeros(grid.shape if hasattr(grid,'shape') else (grid.ny,grid.nx),dtype=np.float32)
        logodds_rsu=np.zeros_like(logodds_ego)
        engine_grid=OGMGridContract(coop.world_to_grid,coop.in_bounds,coop.bresenham)
        def engine_config(free_scale,decay_rate):
            return OGMUpdateConfig(lidar_cfg['z_min_world'],lidar_cfg['z_max_world'],lidar_cfg['range_m'],
                coop.L_FREE,coop.L_OCC,free_scale,coop.L_MIN,coop.L_MAX,coop.FREE_TH,coop.OCC_TH,decay_rate,coop.L0)
        engine_configs={'ego':engine_config(1.,coop.DECAY_PER_SEC_EGO),
                        'rsu':engine_config(lidar_cfg['rsu_free_scale'],coop.DECAY_PER_SEC_RSU)}
        engine_shadow=None
        if args.ogm_engine_shadow_compare:
            engine_shadow=OGMEngineShadowComparison(root/'ogm_engine_shadow',logodds_ego.shape,
                engine_grid,engine_configs['ego'],engine_configs['rsu'],
                coop.legacy_update_from_points_reference,coop.legacy_decay_logodds_reference)
        contract_meta=runtime_grid_metadata(map_name=cfg['map_name'],origin_x=g['center_x'],origin_y=g['center_y'],
            x_min=-half_x,x_max=half_x,y_min=-half_y,y_max=half_y,resolution=g['resolution_m'],nx=g['nx'],ny=g['ny'])
        paired = None
        if args.phase1a_paired_diagnostics:
            paired = Phase1aPairedComparison(
                root/'phase1a_paired', GridContract.from_metadata(contract_meta), road,
                z_min=lidar_cfg['z_min_world'], z_max=lidar_cfg['z_max_world'],
                lidar_range=lidar_cfg['range_m'], free_logodds=coop.L_FREE,
                occupied_logodds=coop.L_OCC, logodds_min=coop.L_MIN, logodds_max=coop.L_MAX,
                world_to_grid=coop.world_to_grid, in_bounds=coop.in_bounds,
                bresenham=coop.bresenham)
        last={'ego':None,'rsu':None}
        captures=[]
        capture_dir=root/'phase1b_capture'
        if args.phase1b_ray_diagnostics:
            capture_dir.mkdir()
        def update(name, sensor, arr, free_scale, measurement):
            decay_dt=None
            decay_rate=coop.DECAY_PER_SEC_EGO if name=='ego' else coop.DECAY_PER_SEC_RSU
            if last[name] is not None:
                decay_dt=measurement.timestamp-last[name]
                decay_rate=coop.DECAY_PER_SEC_EGO if name=='ego' else coop.DECAY_PER_SEC_RSU
                apply_decay(arr,decay_dt,engine_configs[name])
                if paired is not None: paired.decay(name,decay_dt,decay_rate,coop.common_decay_with_runtime_rate)
                if engine_shadow is not None: engine_shadow.decay(name,decay_dt)
            last[name]=measurement.timestamp
            raw=np.frombuffer(measurement.raw_data,dtype=np.float32).reshape(-1,4)[:,:3]
            tf=sensor.get_transform(); points=coop.transform_to_world(raw,tf)
            if engine_shadow is not None:
                engine_shadow.update(name,points,(tf.location.x,tf.location.y,tf.location.z))
            z_mask=z_filter_mask(points,engine_configs[name])
            valid=points[z_mask]
            update_lidar_measurement(arr,points,(tf.location.x,tf.location.y,tf.location.z),
                                     engine_grid,engine_configs[name])
            if paired is not None:
                paired.update(name,points,z_mask,(tf.location.x,tf.location.y,tf.location.z),free_scale,
                    coop.update_from_points,frame=measurement.frame,timestamp=measurement.timestamp)
            if args.phase1b_ray_diagnostics:
                snapshot=world.get_snapshot()
                actor_geometry={}
                for actor in actors:
                    actor_tf=actor.get_transform()
                    box=actor.bounding_box
                    box_tf=carla.Transform(box.location,box.rotation)
                    matrix=np.asarray(actor_tf.get_matrix()) @ np.asarray(box_tf.get_matrix())
                    vertices=np.array([[v.x,v.y,v.z] for v in box.get_world_vertices(actor_tf)])
                    actor_geometry[actor.attributes['role_name']]=dict(
                        actor_id=actor.id,actor_matrix=actor_tf.get_matrix(),box_matrix=matrix.tolist(),
                        extent=[box.extent.x,box.extent.y,box.extent.z],vertices=vertices.tolist())
                measurement_tf=measurement.transform
                filename=f'{name}_{measurement.frame}.npz'
                np.savez_compressed(capture_dir/filename,raw_xyz=raw,points_world=points,
                    measurement_pose_points=coop.transform_to_world(raw,measurement_tf))
                captures.append(dict(sensor=name,frame=int(measurement.frame),
                    timestamp=float(measurement.timestamp),world_snapshot_frame=int(snapshot.frame),
                    world_snapshot_timestamp=float(snapshot.timestamp.elapsed_seconds),
                    used_sensor_matrix=tf.get_matrix(),measurement_sensor_matrix=measurement_tf.get_matrix(),
                    actors=actor_geometry,file=filename,free_scale=float(free_scale),
                    decay_dt=decay_dt,decay_rate=float(decay_rate)))

        warmup_frames=max(1,round(cfg['warmup_seconds']/dt))
        final_measurements=None
        for index in range(warmup_frames):
            frame, measurements=take_frame(world,queues)
            if index+1 < warmup_frames:
                update('ego',ego_lidar,logodds_ego,1.,measurements['ego'])
                update('rsu',rsu_lidar,logodds_rsu,lidar_cfg['rsu_free_scale'],measurements['rsu'])
            else: final_measurements=measurements
        reference_frame=int(frame); reference_time=float(final_measurements['ego'].timestamp)
        if final_measurements['rsu'].timestamp != reference_time:
            raise RuntimeError('Ego/RSU t0 sensor timestamps differ')
        ego_v=carla.Vector3D(x=cfg['ego']['speed_mps'],y=0,z=0)
        target_speed=(cfg['target']['conflict_y']-initial_transforms['target']['y'])/cfg['target']['arrival_time_s']
        target_v=carla.Vector3D(x=0,y=target_speed,z=0)
        experiment=dict(schema_version='2.0' if cfg.get('grounded') else '1.0',
            experiment_id=cfg['experiment_id'],scenario_name=cfg['scenario_name'],
            scenario_version=cfg.get('scenario_version','1.0'),grounded=bool(cfg.get('grounded',False)),
            map_name=cfg['map_name'],ego_initial_state=pose_from_transform(initial_transforms['ego'],ego_v),
            target_initial_state=pose_from_transform(initial_transforms['target'],target_v),rsu_pose=dict(
                x=rsu_cfg['x'],y=rsu_cfg['y'],z=rsu_cfg['z'],yaw=rsu_cfg['yaw']),
            grid_center_x=g['center_x'],grid_center_y=g['center_y'],resolution_m=g['resolution_m'],nx=g['nx'],ny=g['ny'],
            fixed_delta_seconds=dt,prediction_horizon_s=cfg['prediction_horizon_s'],random_seed=cfg['random_seed'],tm_seed=cfg['tm_seed'],
            reference_frame=reference_frame,reference_simulation_time_s=reference_time,
            time_reference_id=f"{cfg['experiment_id']}:{reference_frame}",
            actor_initialization=(grounding_metadata['actors'] if grounding_metadata else {
                role:{'blueprint':cfg[role]['blueprint'],'settled_transform':initial_transforms[role],
                      'grounding_method':'legacy fixed transform; no physics settle'} for role in ('ego','target')}),
            grounding_method=(cfg.get('grounding',{}).get('method') if cfg.get('grounded') else 'legacy_fixed_transform'),
            control_plan='deterministic kinematic crossing: Ego east then south; Target north',
            source_config=str(args.config.resolve()),source_config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest())
        write_json(root/'experiment_metadata.json',experiment)
        recorder=PrioritySnapshot(ogm_dir/'priority_debug',contract_meta,mask_path,mask_meta,experiment)
        recorder.callback('ego',lambda m:update('ego',ego_lidar,logodds_ego,1.,m),final_measurements['ego'])
        recorder.callback('rsu',lambda m:update('rsu',rsu_lidar,logodds_rsu,lidar_cfg['rsu_free_scale'],m),final_measurements['rsu'])
        if not recorder.save_if_ready(logodds_ego,logodds_rsu): raise RuntimeError('t0 OGM snapshot was not saved')
        if paired is not None:
            paired.experiment=experiment
            _,gt_masks=vehicle_footprint_masks(actors,(grid.ny,grid.nx),coop.world_to_grid)
            paired_result=paired.finalize(logodds_ego,logodds_rsu,gt_masks)
            if cfg.get('grounded'):
                for sensor_name in ('ego','rsu'):
                    np.save(root/f'{sensor_name}_phase0.npy',paired.phase0[sensor_name])
                    np.save(root/f'{sensor_name}_phase1a.npy',paired.phase1a[sensor_name])
                write_json(root/'paired_comparison.json',paired_result)
                false_free_summary={sensor_name: paired_result['sensors'][sensor_name]['false_free']
                                    for sensor_name in ('ego','rsu')}
                write_json(root/'false_free_summary.json',false_free_summary)
            if args.phase1b_ray_diagnostics:
                for role,mask in gt_masks.items(): np.save(capture_dir/f'{role}_gt.npy',mask)
                write_json(capture_dir/'capture_manifest.json',dict(
                    experiment=experiment,grid_metadata=contract_meta,config=cfg,measurements=captures,
                    constants=dict(free=coop.L_FREE,occupied=coop.L_OCC,
                                   minimum=coop.L_MIN,maximum=coop.L_MAX)))
        for sensor in sensors: sensor.stop(); sensor.destroy()
        sensors.clear()

        risk_cfg=cfg['risk']; detector_args=SimpleNamespace(
            near_miss_distance_m=risk_cfg['near_miss_distance_m'],min_closing_speed_mps=risk_cfg['min_closing_speed_mps'],
            near_miss_cooldown_s=risk_cfg['near_miss_cooldown_s'],near_miss_region_radius_m=risk_cfg['near_miss_region_radius_m'])
        events=[];regions=[];cooldown={}; rows=[]
        turn_t=(cfg['ego']['turn_x']-initial_transforms['ego']['x'])/cfg['ego']['speed_mps']
        steps=round(cfg['prediction_horizon_s']/dt)
        for step in range(1,steps+1):
            t=step*dt
            if t <= turn_t:
                ex=initial_transforms['ego']['x']+cfg['ego']['speed_mps']*t;ey=initial_transforms['ego']['y'];eyaw=initial_transforms['ego']['yaw'];ev=ego_v
            else:
                ex=cfg['ego']['turn_x'];ey=initial_transforms['ego']['y']-cfg['ego']['speed_mps']*(t-turn_t);eyaw=cfg['ego']['turn_yaw'];ev=carla.Vector3D(x=0,y=-cfg['ego']['speed_mps'],z=0)
            tx=initial_transforms['target']['x'];ty=initial_transforms['target']['y']+target_speed*t;tyaw=initial_transforms['target']['yaw'];tv=target_v
            ego.set_transform(carla.Transform(carla.Location(x=ex,y=ey,z=initial_transforms['ego']['z']),carla.Rotation(yaw=eyaw)))
            target.set_transform(carla.Transform(carla.Location(x=tx,y=ty,z=initial_transforms['target']['z']),carla.Rotation(yaw=tyaw)))
            current_frame=world.tick(); snap=world.get_snapshot()
            detect_near_misses({'ego':PlannedActor(ego,ev),'target':PlannedActor(target,tv)},events,regions,
                reference_time,t,detector_args,cooldown)
            rows.extend([to_row(actor_state(ego,'ego',current_frame,t,ev,eyaw)),to_row(actor_state(target,'target',current_frame,t,tv,tyaw))])
        with CsvWriter(risk_dir/'prediction_states.csv',prediction_state_fieldnames()) as w:w.writerows(rows)
        with CsvWriter(risk_dir/'risk_events.csv',risk_event_fieldnames()) as w:w.writerows(to_row(e) for e in events)
        with CsvWriter(risk_dir/'risk_regions.csv',risk_region_fieldnames()) as w:w.writerows(to_row(r) for r in regions)
        write_json(risk_dir/'experiment_metadata.json',experiment)
        build_cmd=[sys.executable,str(DT_PROJECT/'build_future_risk_heatmap.py'),
            '--risk-regions',str(risk_dir/'risk_regions.csv'),'--prediction-states',str(risk_dir/'prediction_states.csv'),
            '--grid-spec',str(root/'grid_spec.json'),'--prediction-horizon-s',str(cfg['prediction_horizon_s']),
            '--prediction-dt-s',str(dt),'--sigma-m',str(risk_cfg['sigma_m']),
            '--experiment-metadata',str(root/'experiment_metadata.json'),'--output-dir',str(risk_dir/'risk_map')]
        result=subprocess.run(build_cmd,capture_output=True,text=True)
        (logs_dir/'heatmap_stdout.log').write_text(result.stdout,encoding='utf-8')
        (logs_dir/'heatmap_stderr.log').write_text(result.stderr,encoding='utf-8')
        if result.returncode: raise RuntimeError('Heatmap build failed: '+result.stderr)
        summary=build_priority(ogm_dir/'priority_debug',risk_dir/'risk_map',priority_dir,require_experiment_match=True)

        risk=np.load(risk_dir/'risk_map/risk_aggregate.npy');unknown=np.load(ogm_dir/'priority_debug/ego_unknown.npy')
        known=np.load(ogm_dir/'priority_debug/rsu_known.npy');road=np.load(ogm_dir/'priority_debug/road_mask.npy')
        priority=np.load(priority_dir/'priority_map.npy')
        if cfg.get('grounded'):
            np.save(root/'future_risk.npy',risk)
            np.save(root/'priority.npy',priority)
        if engine_shadow is not None:
            shadow_result=engine_shadow.finalize(encode_grid=coop.encode_grid_q8,
                fuse=coop.fuse_logodds_prefer_ego,road_mask=road,risk=risk,
                production_ego=logodds_ego,production_rsu=logodds_rsu)
            if not shadow_result['all_pass']:
                raise RuntimeError('Common OGM Engine shadow comparison failed')
        checks=[]
        def add_check(kind, risk_id, risk_type, t, actor_a, actor_b, ix, iy):
            x,y=grid.grid_to_world(ix,iy)
            checks.append(dict(check_kind=kind,risk_id=risk_id,risk_type=risk_type,prediction_time_s=t,
                logical_actor_a=actor_a,logical_actor_b=actor_b,world_x=x,world_y=y,grid_ix=ix,grid_iy=iy,
                risk_score=float(risk[iy,ix]),ego_unknown=bool(unknown[iy,ix]),rsu_known=bool(known[iy,ix]),
                road=bool(road[iy,ix]),priority=float(priority[iy,ix])))
        for region in regions:
            cell=grid.world_to_grid(region.center_x,region.center_y)
            if cell is None: continue
            ix,iy=cell
            add_check('risk_region_center',region.risk_id,region.risk_type,region.start_time_s,
                      region.logical_actor_a,region.logical_actor_b,ix,iy)
            candidates=np.argwhere((risk>0)&unknown&known&road)
            if len(candidates):
                distances=(candidates[:,0]-iy)**2+(candidates[:,1]-ix)**2
                cy,cx=map(int,candidates[np.argmin(distances)])
                add_check('nearest_priority_positive',region.risk_id,region.risk_type,region.start_time_s,
                          region.logical_actor_a,region.logical_actor_b,cx,cy)
        target_cell=grid.world_to_grid(cfg['target']['x'],cfg['target']['y'])
        if target_cell:
            add_check('target_at_t0','','',0.,'', 'target',*target_cell)
        columns=list(checks[0]) if checks else ['check_kind','risk_id','risk_type','prediction_time_s','logical_actor_a','logical_actor_b','world_x','world_y','grid_ix','grid_iy','risk_score','ego_unknown','rsu_known','road','priority']
        with (priority_dir/'target_priority_check.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(checks)
        first=min((r.start_time_s for r in regions),default=None);last_event=max((r.start_time_s for r in regions),default=None)
        integration=dict(experiment=experiment,reference_frame=reference_frame,reference_simulation_time_s=reference_time,
            prediction_frame_start=rows[0]['prediction_frame'] if rows else None,prediction_frame_end=rows[-1]['prediction_frame'] if rows else None,
            risk_event_count=len(events),near_miss_count=sum(e.risk_type=='near_miss' for e in events),collision_count=0,
            first_risk_time_s=first,last_risk_time_s=last_event,risk_max=float(risk.max()),priority=summary,
            target_t0=next((c for c in checks if c['check_kind']=='target_at_t0'),None),
            representative_positive=next((c for c in checks if c['risk_score']>0 and c['ego_unknown'] and c['rsu_known'] and c['road'] and c['priority']>0),None))
        write_json(root/'integration_summary.json',integration)
        if not regions: raise RuntimeError('no RiskRegion was generated')
        if not integration['target_t0'] or not integration['target_t0']['ego_unknown'] or not integration['target_t0']['rsu_known'] or not integration['target_t0']['road']:
            raise RuntimeError('Target at t0 does not satisfy Ego Unknown, RSU Known and Road')
        if integration['representative_positive'] is None:
            raise RuntimeError('no representative Risk cell passed all four Priority conditions')
        print(json.dumps(integration,ensure_ascii=False,indent=2))
    finally:
        for sensor in sensors:
            try:sensor.stop();sensor.destroy()
            except RuntimeError:pass
        for actor in actors:
            try:actor.destroy()
            except RuntimeError:pass
        try:tm.set_synchronous_mode(False)
        except RuntimeError:pass
        world.apply_settings(original)
    return 0


if __name__=='__main__':raise SystemExit(main())
