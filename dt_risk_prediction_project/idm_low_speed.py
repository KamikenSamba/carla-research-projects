"""Independent exploratory protocol; never certifies the strict scene audit.

Reuse the brake worker's PI, observations, safety gates, ticks and owned cleanup.
No Traffic Manager, target velocity, teleport or manufactured IDM trajectory.
"""
import csv
from dataclasses import dataclass, asdict
import hashlib
import json
import math
import threading
import time

from idm_brake_config import BrakeConfig, finite
from idm_brake_harness import BrakeHarness, physics_record
from idm_brake_metrics import FIELDS, analyze
from idm_experiment_harness import Harness, EarlyStop, save, settings_dict, raw_actor_observation
from idm_brake_route import geometry_sha256, box_vertices
from idm_scene_audit import weather_scalars
from idm_reverse_motion import ReverseMotion


@dataclass(frozen=True)
class ExploratoryConfig(BrakeConfig):
    mode: str = 'exploratory_model3_v6'
    map_name: str = 'Town01_Opt'
    road: int = 8
    lane: int = -1
    follower_s: int = 170
    end_s: int = 230
    planned_route_length: float = 60
    spawn_height: float = .1
    throttle_slew_per_second: float = .02
    trial: str = 'trial1'
    target_speed: float = .5
    speed_limit: float = .7
    drive_distance: float = 3
    drive_seconds: float = 12
    maximum_frames: int = 400

    def __post_init__(self):
        if self.trial not in ('trial1','trial2','calibration'):
            raise ValueError('unknown exploratory trial')
        expected=asdict(BrakeConfig())
        expected.update(mode='exploratory_model3_v6',map_name='Town01_Opt',road=8,spawn_height=.1,
            throttle_slew_per_second=.02,
            lane=-1,follower_s=170,end_s=230,planned_route_length=60,
            trial=self.trial,target_speed=.5 if self.trial=='trial1' else 1.,
            speed_limit=.7 if self.trial=='trial1' else 1.2,
            drive_distance=3 if self.trial=='trial1' else 10,drive_seconds=12 if self.trial=='trial1' else 25,
            maximum_frames=400 if self.trial=='trial1' else 700)
        if asdict(self)!=expected:
            raise ValueError('outside fixed exploratory design; no automatic threshold changes')

    @property
    def extended_route_audit(self):return False

    def require_executable(self,stage):
        if stage!='exploratory_low_speed':raise ValueError('not the strict calibration protocol')


def config_for(trial):
    if trial=='trial1':return ExploratoryConfig()
    return ExploratoryConfig(trial=trial,target_speed=1.,speed_limit=1.2,
        drive_distance=10,drive_seconds=25,maximum_frames=700)


def local_pose_ok(row):
    return (row['speed_planar_mps']<.05 and abs(row['vz'])<.05
        and abs(row['pitch'])<5 and abs(row['roll'])<5
        and abs(row['z'])<.8 and abs(row['a_long_mps2'])<=.5)


class ExploratoryHarness(BrakeHarness):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.stage='exploratory_low_speed';self.result['stage']=self.stage
        self.collisions=[];self.collision_lock=threading.Lock();self.sensor=None
        self.result.update(strict_scene_audit='BLOCKED',route_audit=None,physical_bounds=None,
            exploratory_protocol=True,trial=self.c.trial)

    def collision(self,event):
        impulse=event.normal_impulse
        record=dict(frame=event.frame,other_actor_id=event.other_actor.id,
            other_type=event.other_actor.type_id,received_wall_ns=time.time_ns(),
            normal_impulse=dict(x=impulse.x,y=impulse.y,z=impulse.z))
        with self.collision_lock:
            self.collisions.append(record)
            with (self.directory/'collision_events.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(record,allow_nan=False)+'\n');f.flush()

    def collision_gate(self):
        with self.collision_lock:self.fail_if(bool(self.collisions),'collision_event',self.actor.id)
        self.fail_if(self.sensor is None or not self.sensor.is_listening,'collision_sensor_not_listening')

    def prepare(self):
        c=self.c;c.require_executable(self.stage)
        versions=[self.call(self.client.get_client_version),self.call(self.client.get_server_version)]
        self.fail_if(versions!=['0.9.16','0.9.16'],'carla_version_mismatch')
        self.world=self.call(self.client.get_world);self.map=self.call(self.world.get_map)
        self.fail_if(self.map.name!='Carla/Maps/Town01_Opt','map_mismatch')
        self.original=self.call(self.world.get_settings)
        self.fail_if(self.original.synchronous_mode,'existing_tick_owner_or_sync_session')
        actors=list(self.call(self.world.get_actors));self.initial_ids={a.id for a in actors}
        self.fail_if(any(a.type_id.startswith(('vehicle.','walker.','sensor.','controller.')) for a in actors),
                     'existing_dynamic_actor_or_sensor')
        xml=self.call(self.map.to_opendrive)
        self.fail_if(hashlib.sha256(xml.encode()).hexdigest()!=
            '97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709','opendrive_mismatch')
        self.route=[self.call(self.map.get_waypoint_xodr,c.road,c.lane,float(s)) for s in c.route_s_samples]
        self.fail_if(any(w is None or w.is_junction or w.lane_width<3.9 or
            (w.road_id,w.section_id,w.lane_id)!=(8,0,-1) for w in self.route),'local_route_invalid')
        self.path=[(w.transform.location.x,w.transform.location.y,w.transform.location.z) for w in self.route]
        self.route_length=sum(math.dist(p,q) for p,q in zip(self.path,self.path[1:]))
        direction=tuple((self.path[-1][i]-self.path[0][i])/self.route_length for i in range(3))
        self.reverse_monitor=ReverseMotion(direction,c.dt)
        self.fail_if(abs(self.route_length-60)>.02,'local_route_length')
        self.fail_if(any(abs((w.transform.rotation.yaw-self.route[0].transform.rotation.yaw+180)%360-180)>.5
            or abs(w.transform.location.z-self.route[0].transform.location.z)>.02 for w in self.route),
            'local_route_heading_or_height')
        points=[list(p) for p in self.path]
        metadata=[dict(s=w.s,yaw=w.transform.rotation.yaw,pitch=w.transform.rotation.pitch,
            roll=w.transform.rotation.roll,width=w.lane_width,road=w.road_id,section=w.section_id,lane=w.lane_id) for w in self.route]
        save(self.directory/'local_route.json',dict(points=points,waypoints=metadata,
            geometry_sha256=geometry_sha256(points,metadata),route_audit=None,
            strict_scene_status='BLOCKED',local_observation_only=True))
        self.initial_clearance(self.route[0].transform)
        # No TM or autopilot. Own all subsequent ticks and restore the original settings.
        settings=self.call(self.world.get_settings);settings.synchronous_mode=True;settings.fixed_delta_seconds=c.dt
        settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10
        self.settings_touched=True;self.command('world_sync',self.world.apply_settings,settings)
        snap=self.call(self.world.get_snapshot);self.last_frame=int(snap.frame);self.last_time=float(snap.timestamp.elapsed_seconds)
        self.expected_settings=settings_dict(self.call(self.world.get_settings))
        self.fail_if(not self.expected_settings['synchronous_mode'] or self.expected_settings['fixed_delta_seconds']!=c.dt,'world_sync_not_applied')
        library=self.call(self.world.get_blueprint_library);bp=library.find(c.blueprint)
        bp.set_attribute('role_name','exploratory_low_speed:vehicle')
        tf=self.route[0].transform;tf.location.z+=c.spawn_height
        def spawn():
            a=self.world.try_spawn_actor(bp,tf)
            if a is not None:self.owned['vehicle']=a
            return a
        self.actor=self.command('spawn_exploratory_model3',spawn)
        self.fail_if(self.actor is None,'spawn_failed');self.active_actor=self.actor.id
        box=self.actor.bounding_box
        self.fail_if(box.extent.x+abs(box.location.x)>c.front_extent_limit or box.extent.y+abs(box.location.y)>1.25,
                     'actual_bbox_outside_initial_clearance',self.actor.id)
        self.fail_if(min(box.extent.x,box.extent.y,box.extent.z)<=0,'unknown_actual_bbox')
        physics=physics_record(self.call(self.actor.get_physics_control));save(self.directory/'vehicle_physics.json',physics)
        self.fail_if(not finite(physics['mass']) or physics['mass']<=0 or len(physics['wheels'])!=4 or
            any(not finite(w[k]) or w[k]<=0 for w in physics['wheels'] for k in
                ('radius','max_brake_torque','tire_friction')),'physics_unknown')
        sensor_bp=library.find('sensor.other.collision')
        def spawn_sensor():
            s=self.world.spawn_actor(sensor_bp,self.api.Transform(),attach_to=self.actor)
            self.owned={'collision':s,**self.owned};return s
        self.sensor=self.command('spawn_collision_sensor',spawn_sensor)
        self.command('listen_collision',self.sensor.listen,self.collision)
        self.collision_gate()
        self.initialize_drive_gear()
        save(self.directory/'environment.json',dict(versions=versions,map=self.map.name,
            initial_actor_ids=sorted(self.initial_ids),owned_actor_id=self.actor.id,collision_sensor_id=self.sensor.id,
            original_settings=settings_dict(self.original),applied_settings=self.expected_settings,
            initial_sync_frame=self.last_frame,weather=weather_scalars(self.call(self.world.get_weather)),
            initialization='bounded_manual_PI',autopilot_registration='not_requested',tm_created=False,
            physical_bounds=None,route_audit=None,strict_scene_audit='BLOCKED',exclusive_control_basis=
            'operator research session attestation; no observed other dynamic actors/clients; frame and actor interleaving gates'))

    def initialize_drive_gear(self):
        # CARLA DefaultMovementComponent sets target gear only on reverse transition
        # or manual selection. Low throttle can otherwise remain in neutral.
        control=self.api.VehicleControl(throttle=0.,brake=.1,steer=0.,hand_brake=False,
            reverse=False,manual_gear_shift=True,gear=1)
        self.command('initial_drive_gear_select',self.actor.apply_control,control)
        self.t+=self.c.dt;self.tick();time.sleep(.02);self.collision_gate()
        tf=self.call(self.actor.get_transform);v=self.call(self.actor.get_velocity)
        observed=self.call(self.actor.get_control);a=self.call(self.actor.get_acceleration)
        raw=raw_actor_observation(self.actor,'vehicle',tf,v,observed,a,self.last_frame,self.t)
        save(self.directory/'drive_gear_initialization.json',dict(observation=raw,
            excluded_from_braking_and_estimation=True,requested_gear=1,one_time_selection=True))
        self.fail_if(observed.gear!=1 or not observed.manual_gear_shift or observed.throttle>1e-5
            or observed.brake<.099,'initial_drive_gear_not_applied')
        self.fail_if(math.hypot(v.x,v.y)>.05 or abs(v.z)>2 or abs(tf.rotation.pitch)>5 or abs(tf.rotation.roll)>5,
            'initial_drive_gear_motion_or_pose')
        automatic=self.api.VehicleControl(throttle=0.,brake=0.,steer=0.,hand_brake=False,
            reverse=False,manual_gear_shift=False,gear=1)
        self.command('initial_drive_gear_auto_handoff',self.actor.apply_control,automatic)

    def initial_clearance(self,start):
        # Local rays establish only sampled initial clearance, not whole-scene safety.
        f=start.get_forward_vector();right=start.get_right_vector()
        def loc(x,y,z):return self.api.Location(x=start.location.x+f.x*x+right.x*y,
            y=start.location.y+f.y*x+right.y*y,z=z)
        ground=[]
        for x,y in ((0,0),(-2.5,-1.2),(-2.5,1.2),(2.5,-1.2),(2.5,1.2)):
            p=self.call(self.world.ground_projection,loc(x,y,2),3.)
            self.fail_if(p is None or p.label!=self.api.CityObjectLabel.Roads,'initial_ground_not_road')
            self.fail_if(abs(p.location.z)>.2,'initial_ground_height')
            ground.append(dict(x=p.location.x,y=p.location.y,z=p.location.z,label=str(p.label)))
        self.fail_if(max(p['z'] for p in ground)-min(p['z'] for p in ground)>.1,'initial_ground_nonplanar')
        rays=[]
        for z in (.35,.85,1.35):
            for p,q in (((-2.6,-1.25),(2.6,1.25)),((-2.6,1.25),(2.6,-1.25)),
                        ((-2.6,-1.25),(2.6,-1.25)),((-2.6,1.25),(2.6,1.25))):
                hits=list(self.call(self.world.cast_ray,loc(*p,z),loc(*q,z)))
                begin,end=loc(*p,z),loc(*q,z)
                direction=self.api.Vector3D(x=end.x-begin.x,y=end.y-begin.y,z=end.z-begin.z)
                blocking=self.call(self.world.project_point,begin,direction,
                    math.dist((begin.x,begin.y,begin.z),(end.x,end.y,end.z)))
                rays.append(dict(height=z,overlap_channel_hits=[dict(label=str(h.label),x=h.location.x,y=h.location.y,z=h.location.z) for h in hits],
                    sensor_trace_blocking_hit=(dict(label=str(blocking.label),x=blocking.location.x,
                        y=blocking.location.y,z=blocking.location.z) if blocking else None),
                    untagged_overlap_collision_status='UNKNOWN' if hits else None))
                save(self.directory/'initial_clearance.json',dict(ground=ground,rays=rays,
                    limitations='finite SensorTrace and OverlapChannel rays are not the vehicle collision channel; untagged overlap remains UNKNOWN',route_audit=None))
                self.fail_if(blocking is not None,'initial_clearance_blocking_trace_hit')
                self.fail_if(any(h.label!=getattr(self.api.CityObjectLabel,'NONE',None) for h in hits),
                    'initial_clearance_tagged_obstacle')

    def reverse_motion_gate(self,r):
        snap=self.frame_unchanged();state=snap.find(self.actor.id)
        self.fail_if(state is None,'actor_snapshot_missing',self.actor.id)
        tf=state.get_transform();v=state.get_velocity();a=state.get_acceleration()
        snapshot=dict(frame=int(snap.frame),world_time_s=float(snap.timestamp.elapsed_seconds),
            position={k:getattr(tf.location,k) for k in ('x','y','z')},
            velocity={k:getattr(v,k) for k in ('x','y','z')},
            acceleration={k:getattr(a,k) for k in ('x','y','z')})
        record=self.reverse_monitor.update(r)
        record.update(actor_snapshot=snapshot,command_based_on_frame=r['command_based_on_frame'],
            control_read_at_frame=r['control_read_at_frame'],throttle=r['throttle'],brake=r['brake'],gear=r['gear'])
        with (self.directory/'longitudinal_diagnostics.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(record,allow_nan=False)+'\n');f.flush()
        self.fail_if(any(abs(snapshot['position'][k]-r[k])>1e-6 or
            abs(snapshot['velocity'][k]-r['v'+k])>1e-6 or
            abs(snapshot['acceleration'][k]-r['a'+k])>1e-5 for k in ('x','y','z')),
            'actor_snapshot_api_state_mismatch',self.actor.id)
        self.fail_if(record['reason'] is not None,record['reason'],self.actor.id)

    def step(self,*args,**kwargs):
        self.collision_gate();self.fail_if(len(self.rows)>=self.c.maximum_frames,'frame_limit')
        row=super().step(*args,**kwargs)
        time.sleep(.02) # allow asynchronous event delivery; absence is not collision proof
        self.collision_gate()
        self.fail_if(row['z']<-.1 or row['z']>1 or abs(row['vz'])>2,'abnormal_contact_or_vertical_motion')
        self.fail_if(row['phase']=='launch' and (row['gear'] is None or row['gear']<1),'drive_gear_not_engaged')
        return row

    def capture(self):
        if self.c.trial=='calibration':
            return super().capture()
        with (self.directory/'calibration_observations.csv').open('x',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,FIELDS).writeheader()
        row=None
        for _ in range(40):row=self.step('settle',0,0,row)
        self.fail_if(not all(local_pose_ok(r) for r in self.rows[-10:]),'initial_vehicle_not_stable')
        save(self.directory/'initial_vehicle_state.json',dict(actor_id=self.actor.id,
            settled_rows=self.rows[-10:],wheel_contact_directly_measured=False,friction_measured=False))
        drive_origin=tuple(row[k] for k in ('x','y','z'))
        save(self.directory/'drive_target_reference.json',dict(frame=row['frame'],position=drive_origin,
            road_direction=self.reverse_monitor.road,required_signed_displacement_m=self.c.drive_distance))
        for _ in range(round(self.c.drive_seconds/self.c.dt)):
            row=self.step('launch',self.controller.step(row['speed_planar_mps']),0,row)
            displacement=sum((row[k]-drive_origin[i])*self.reverse_monitor.road[i] for i,k in enumerate(('x','y','z')))
            if displacement>=self.c.drive_distance:break
        else:raise EarlyStop('short_trial_distance_not_reached',self.actor.id)
        self.brake_request=dict(brake_request_frame=self.last_frame,brake_request_time_s=self.t,
            brake_request_progress_m=row['route_progress_m'],brake_request_speed_mps=row['speed_planar_mps'])
        stopped=[]
        for _ in range(round(self.c.brake_seconds/self.c.dt)):
            row=self.step('braking',0,.1,row)
            if self.t-self.brake_request['brake_request_time_s']>=self.c.response_timeout:
                self.fail_if(row['brake']<.099 or row['throttle']>1e-5,'exploratory_brake_control_timeout')
                self.fail_if(row['speed_planar_mps']>self.brake_request['brake_request_speed_mps']-.02,
                    'exploratory_deceleration_timeout')
            good=(row['speed_planar_mps']<.05 and abs(row['vz'])<.05 and abs(row['a_long_mps2'])<=.5 and row['brake']>=.099)
            stopped=stopped+[row] if good else []
            if stopped and (row['frame']-stopped[0]['frame'])*self.c.dt+1e-9>=.5:return
        raise EarlyStop('exploratory_stop_timeout',self.actor.id)

    def cleanup(self):
        if self.sensor is not None and not self.gate.poisoned:
            try:
                self.frame_unchanged();self.call(self.sensor.stop)
            except BaseException as exc:self.result['sensor_stop_error']=repr(exc)
        if self.result.get('status')!='completed' and self.owned.get('vehicle') is not None and not self.gate.poisoned:
            try:
                self.frame_unchanged();self.issue('abort',0,self.c.brake,0)
            except BaseException as exc:self.result['abort_control_error']=repr(exc)
        if self.c.trial=='calibration':super().cleanup()
        else:Harness.cleanup(self,manage_tm=False)
        with self.collision_lock:events=list(self.collisions)
        self.result.update(collision_events=len(events),physical_bounds=None,route_audit=None,
            motion_samples=len(self.rows),route_progress_m=(self.rows[-1]['route_progress_m'] if self.rows else None),
            distance_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(self.rows,self.rows[1:])),
            controlled_distance_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(self.rows,self.rows[1:])
                if b['phase'] in ('launch','braking','stop_confirm')),
            maximum_speed_mps=max((r['speed_planar_mps'] for r in self.rows),default=None))
        if events:
            self.result.update(status='early_stop',reason='collision_event',exit_code=2,calibration_valid=False)
        save(self.directory/'collision_summary.json',dict(events=events,absence_is_not_collision_geometry_proof=True))
