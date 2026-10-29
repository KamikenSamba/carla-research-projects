"""Single-vehicle calibration worker. CARLA is injected; no TM or velocity forcing."""
import csv
from dataclasses import asdict
import json
import math
import time

from idm_brake_config import finite
from idm_brake_metrics import FIELDS, analyze, brake_matched, stop_ready
from idm_experiment_harness import Harness, EarlyStop, raw_actor_observation, save, settings_dict
from idm_leader import path_projection, sample_vehicle
from types import SimpleNamespace as NS


class LaunchController:
    """Bounded PI throttle with slew limit and anti-windup; gains remain uncalibrated."""
    def __init__(self,c):self.c=c;self.integral=0.;self.throttle=0.
    def step(self,speed):
        c=self.c;error=c.target_speed-speed
        proposed=max(0.,min(c.throttle_cap,self.integral+c.speed_ki*error*c.dt))
        output=c.speed_kp*error+proposed
        if not (output>c.throttle_cap and error>0):self.integral=proposed
        target=max(0.,min(c.throttle_cap,c.speed_kp*error+self.integral))
        slew=c.throttle_slew_per_second*c.dt
        self.throttle=max(0.,min(c.throttle_cap,self.throttle+max(-slew,min(slew,target-self.throttle))))
        return self.throttle


def physics_record(physics):
    def vector(v):return {k:getattr(v,k,None) for k in ('x','y','z')}
    names=('mass','drag_coefficient','moi','max_rpm','use_gear_autobox','gear_switch_time','clutch_strength','final_ratio',
           'damping_rate_full_throttle','damping_rate_zero_throttle_clutch_engaged','damping_rate_zero_throttle_clutch_disengaged')
    record={k:getattr(physics,k,None) for k in names}
    record['center_of_mass']=vector(getattr(physics,'center_of_mass',None))
    record['wheels']=[dict({k:getattr(w,k,None) for k in
        ('tire_friction','radius','max_brake_torque','max_handbrake_torque','damping_rate','max_steer_angle')},
        position=vector(getattr(w,'position',None))) for w in getattr(physics,'wheels',[])]
    record['torque_curve']=[dict(x=p.x,y=p.y) for p in getattr(physics,'torque_curve',[])]
    record['steering_curve']=[dict(x=p.x,y=p.y) for p in getattr(physics,'steering_curve',[])]
    record['forward_gears']=[{k:getattr(g,k,None) for k in ('ratio','down_ratio','up_ratio')} for g in getattr(physics,'forward_gears',[])]
    record['wheel_rpm']=None # no independent wheel/RPM getter is assumed
    record['unknown_fields']=[k for k,v in record.items() if v is None]
    return record


class BrakeHarness(Harness):
    def __init__(self,client,api,config,directory,gate):
        super().__init__(client,api,config,'brake_calibration',directory,gate,None)
        self.rows=[];self.metrics={};self.previous_velocity=None;self.accel_count=0
        self.brake_request=None;self.controller=LaunchController(config)
        self.result.update(autopilot_registration='not_requested',tm_created=False,
                           calibration_valid=False,physical_bounds_qualified=False)

    def prepare(self):
        c=self.c;c.require_executable(self.stage)
        if c.extended_route_audit:
            from idm_brake_route import inspect_town01
            self.inspect_world_route(strict=True,obstacle_audit=inspect_town01)
        else:
            self.inspect_world_route(strict=True)
        save(self.directory/'distance_budget.json',c.distance_report(
            **({'route_length_m':self.route_length} if c.extended_route_audit else {})))
        settings=self.call(self.world.get_settings)
        settings.synchronous_mode=True;settings.fixed_delta_seconds=c.dt
        settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10
        self.settings_touched=True
        self.command('world_sync',self.world.apply_settings,settings)
        snap=self.call(self.world.get_snapshot)
        self.last_frame,self.last_time=int(snap.frame),float(snap.timestamp.elapsed_seconds)
        self.started_world_time=self.last_time
        self.expected_settings=settings_dict(self.call(self.world.get_settings))
        self.fail_if(not self.expected_settings['synchronous_mode'] or
                     self.expected_settings['fixed_delta_seconds']!=c.dt,'world_sync_not_applied')
        bp=self.call(self.world.get_blueprint_library).find(c.blueprint)
        bp.set_attribute('role_name','brake_calibration:vehicle')
        tf=self.route[0].transform;tf.location.z+=c.spawn_height
        def spawn_owned():
            actor=self.world.try_spawn_actor(bp,tf)
            if actor is not None:self.owned['vehicle']=actor
            return actor
        actor=self.command('spawn_calibration_vehicle',spawn_owned)
        self.fail_if(actor is None,'spawn_failed')
        self.actor=actor;self.active_actor=actor.id
        self.fail_if(actor.type_id!=c.blueprint,'spawn_blueprint_mismatch',actor.id)
        self.ledger('autopilot_policy',actor_id=actor.id,registration_requested=False,
                    evidence='owned fresh actor; no autopilot/TM API calls',registration_verified=None)
        self.fail_if(actor.bounding_box.extent.x+abs(actor.bounding_box.location.x)>c.front_extent_limit,
                     'vehicle_extent_exceeds_budget',actor.id)
        if c.extended_route_audit:
            from idm_brake_route import inspect_spawn_bbox
            inspect_spawn_bbox(self,actor)
        self.fail_if(not hasattr(actor,'get_physics_control'),'physics_parameters_unknown',actor.id)
        physics=physics_record(self.call(actor.get_physics_control))
        save(self.directory/'vehicle_physics.json',physics)
        self.fail_if(not finite(physics['mass']) or physics['mass']<=0 or len(physics['wheels'])!=4 or
            any(not finite(w[k]) or w[k]<=0 for w in physics['wheels'] for k in
                ('radius','max_brake_torque','tire_friction')),'physics_parameters_unknown',actor.id)
        weather=settings_dict(self.call(self.world.get_weather)) if hasattr(self.world,'get_weather') else None
        save(self.directory/'environment.json',dict(map=self.map.name,versions=['0.9.16','0.9.16'],
            initial_actor_ids=sorted(self.initial_ids),owned_actor_id=actor.id,actual_blueprint=actor.type_id,route_length_m=self.route_length,
            original_settings=settings_dict(self.original),applied_settings=self.expected_settings,
            initial_sync_frame=self.last_frame,weather=weather,surface_material=None,tm_created=False,tm_seed=None,
            autopilot_registration='not_requested',physics_bounds=None,initialization='bounded_manual_PI'))
        self.active_actor=None

    def issue(self,phase,throttle,brake,steer):
        values=dict(throttle=throttle,brake=brake,steer=steer,hand_brake=False,reverse=False,manual_gear_shift=False)
        seq=self.ledger('manual_control',outcome='requested',actor_id=self.actor.id,phase=phase,control=values)
        control=self.api.VehicleControl(**values)
        try:returned=self.call(self.actor.apply_control,control)
        except BaseException as exc:
            self.ledger('manual_control',outcome='failed',request_sequence=seq,error=str(exc));raise
        self.ledger('manual_control',outcome='call_returned',request_sequence=seq,
                    returned=returned if isinstance(returned,bool) else None)
        self.fail_if(returned is False,'control_command_failed',self.actor.id)
        return dict(sequence=seq,based_on_frame=self.last_frame,control=values)

    def observe(self,phase,cmd):
        self.active_actor=self.actor.id
        raw=dict(schema_version=1,phase=phase,carla_frame=self.last_frame,sim_time_s=self.t,
                 world_time_s=self.last_time,command=cmd,brake_request=self.brake_request,
                 expected_actor_ids=[self.actor.id],rows=[],acquired={},acquisition_status='partial',frame_validated=False)
        save(self.directory/'last_observation.json',raw)
        self.frame_unchanged()
        ids={a.id for a in self.call(self.world.get_actors)}
        self.fail_if(self.actor.id not in ids,'actor_disappeared',self.actor.id)
        self.fail_if(not self.initial_ids.issubset(ids),'external_actor_disappeared')
        self.fail_if(ids!=self.initial_ids|{a.id for a in self.owned.values()},'external_actor_appeared')
        tf=self.call(self.actor.get_transform)
        raw['acquired']['position']={k:getattr(tf.location,k) if math.isfinite(getattr(tf.location,k)) else str(getattr(tf.location,k)) for k in ('x','y','z')}
        save(self.directory/'last_observation.json',raw)
        velocity=self.call(self.actor.get_velocity)
        raw['acquired']['velocity']={k:getattr(velocity,k) if math.isfinite(getattr(velocity,k)) else str(getattr(velocity,k)) for k in ('x','y','z')}
        save(self.directory/'last_observation.json',raw)
        control=self.call(self.actor.get_control)
        acceleration=self.call(self.actor.get_acceleration) if hasattr(self.actor,'get_acceleration') else None
        observation=raw_actor_observation(self.actor,'vehicle',tf,velocity,control,acceleration,self.last_frame,self.t)
        observation['logical_actor_id']='role:brake_calibration:vehicle'
        raw['rows']=[observation];save(self.directory/'last_observation.json',raw)
        self.frame_unchanged();raw.update(acquisition_status='complete',frame_validated=True)
        save(self.directory/'last_observation.json',raw)
        with (self.directory/'calibration_raw.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(raw,allow_nan=False)+'\n')
        # Raw is retained before semantic rejection, including unknown acceleration.
        row=dict.fromkeys(FIELDS,None)
        row.update(phase=phase,frame=self.last_frame,sim_time_s=self.t,world_time_s=self.last_time,actor_id=self.actor.id,
                   control_read_at_frame=self.last_frame,command_sequence=cmd['sequence'],
                   command_based_on_frame=cmd['based_on_frame'])
        row.update(observation['transform'])
        for prefix,source in (('v',observation['velocity']),('a',observation['acceleration']),('forward_',observation['forward'])):
            for k,v in source.items():row[prefix+k]=v
        row.update(observation['control']);row['a_long_mps2']=observation['a_long_mps2']
        row.update({'command_'+k:cmd['control'][k] for k in ('throttle','brake','steer')})
        if self.brake_request:row.update(self.brake_request)
        # Unknown fields remain in raw even when no geometrically valid flat row can be produced.
        self.fail_if(any(not finite(row[k]) for k in ('x','y','z','vx','vy','vz','yaw','pitch','roll',
                    'forward_x','forward_y','forward_z')),'nonfinite_state',self.actor.id)
        forward=[row['forward_'+k] for k in ('x','y','z')]
        self.fail_if(abs(sum(v*v for v in forward)-1)>1e-3,'forward_unknown',self.actor.id)
        row['speed_mps']=math.sqrt(sum(row['v'+k]**2 for k in ('x','y','z')))
        row['speed_planar_mps']=math.hypot(row['vx'],row['vy'])
        row['v_long_mps']=sum(row['v'+k]*row['forward_'+k] for k in ('x','y','z'))
        if self.previous_velocity is not None:
            row['a_fd_long_mps2']=sum((row['v'+k]-self.previous_velocity[k])/self.c.dt*row['forward_'+k] for k in ('x','y','z'))
            if finite(row['a_long_mps2']):row['a_api_minus_fd_mps2']=row['a_long_mps2']-row['a_fd_long_mps2']
        self.previous_velocity={k:row['v'+k] for k in ('x','y','z')}
        vehicle,wp=sample_vehicle(NS(id=self.actor.id,attributes=self.actor.attributes,type_id=self.actor.type_id,
            bounding_box=self.actor.bounding_box,get_transform=lambda:tf,get_velocity=lambda:velocity),self.map)
        self.fail_if(vehicle is None,'off_driving_lane',self.actor.id)
        projection=path_projection(vehicle.x,vehicle.y,self.path)
        self.fail_if(projection is None,'route_projection_failed',self.actor.id)
        lateral,progress=projection[:2]
        heading=(tf.rotation.yaw-wp.transform.rotation.yaw+180)%360-180
        start,end=self.path[0],self.path[-1];length=math.dist(start,end)
        signed=(-(row['x']-start[0])*(end[1]-start[1])+(row['y']-start[1])*(end[0]-start[0]))/length
        angle=math.radians(heading)
        half_length=abs(math.cos(angle))*self.actor.bounding_box.extent.x+abs(math.sin(angle))*self.actor.bounding_box.extent.y
        row.update(route_progress_m=progress,route_lateral_m=lateral,signed_lateral_m=signed,
            heading_error_deg=heading,front_clearance_m=self.route_length-progress-half_length,
            road_id=wp.road_id,section_id=wp.section_id,lane_id=wp.lane_id,
            at_traffic_light=self.call(self.actor.is_at_traffic_light))
        self.frame_unchanged()
        self.rows.append(row)
        with (self.directory/'calibration_observations.csv').open('a',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,FIELDS).writerow(row);f.flush()
        self.safety(row,wp)
        self.active_actor=None
        return row

    def safety(self,r,wp):
        c=self.c;who=self.actor.id
        self.fail_if(not finite(r['a_long_mps2']),'acceleration_unknown',who)
        self.accel_count=self.accel_count+1 if abs(r['a_long_mps2'])>c.acceleration_sustained else 0
        self.fail_if(abs(r['a_long_mps2'])>c.acceleration_peak,'acceleration_peak',who)
        self.fail_if(self.accel_count>=2,'acceleration_sustained',who)
        self.fail_if(max(r['speed_planar_mps'],abs(r['v_long_mps']))>c.speed_limit,'speed_envelope',who)
        self.reverse_motion_gate(r)
        self.fail_if((wp.road_id,wp.section_id,wp.lane_id)!=(c.road,c.section,c.lane) or wp.is_junction,'lane_or_junction_departure',who)
        self.fail_if(r['route_lateral_m']>c.lateral_limit or abs(r['heading_error_deg'])>c.heading_limit or
                     abs((r['roll']+180)%360-180)>5 or abs((r['pitch']+180)%360-180)>5,'route_deviation',who)
        half_width=abs(math.cos(math.radians(r['heading_error_deg'])))*self.actor.bounding_box.extent.y+abs(math.sin(math.radians(r['heading_error_deg'])))*self.actor.bounding_box.extent.x
        self.fail_if(r['route_lateral_m']+half_width>=wp.lane_width/2,'bbox_outside_lane',who)
        self.fail_if(r['front_clearance_m']<c.end_clearance,'route_endpoint_margin',who)
        self.fail_if(r['at_traffic_light'] is not False,'signal_or_unknown_signal',who)
        self.fail_if(any(type(r[k]) is not bool for k in ('hand_brake','reverse','manual_gear_shift')) or
                     any(not finite(r[k]) for k in ('throttle','brake','steer')),'control_unknown',who)
        self.fail_if(r['hand_brake'] or r['reverse'] or r['manual_gear_shift'],'hand_brake_reverse_or_manual_gear',who)
        self.fail_if(not 0<=r['throttle']<=c.throttle_cap+1e-5 or not 0<=r['brake']<=1 or abs(r['steer'])>c.steer_cap+1e-5,
                     'observed_control_envelope',who)
        self.fail_if(r['phase'] in ('settle','launch') and r['brake']>1e-5,'unexpected_brake_before_test',who)
        self.fail_if(settings_dict(self.call(self.world.get_settings))!=self.expected_settings,'world_settings_changed')

    def reverse_motion_gate(self,r):
        self.fail_if(r['v_long_mps']<-.02,'backward_motion',self.actor.id)

    def step(self,phase,throttle,brake,row=None):
        steer=0 if row is None else max(-self.c.steer_cap,min(self.c.steer_cap,
            -self.c.heading_kp*row['heading_error_deg']-self.c.lateral_kp*row['signed_lateral_m']))
        cmd=self.issue(phase,throttle,brake,steer)
        self.t+=self.c.dt;self.tick()
        return self.observe(phase,cmd)

    def capture(self):
        with (self.directory/'calibration_observations.csv').open('x',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,FIELDS).writeheader()
        row=None
        for _ in range(round(self.c.settle_seconds/self.c.dt)):row=self.step('settle',0,0,row)
        window=[];start=self.t
        for _ in range(round(self.c.launch_seconds/self.c.dt)):
            throttle=self.controller.step(row['speed_planar_mps'])
            row=self.step('launch',throttle,0,row)
            self.fail_if(row['route_progress_m']>self.c.launch_distance,'launch_distance',self.actor.id)
            good=(abs(row['speed_planar_mps']-self.c.target_speed)<=self.c.ready_speed_tolerance+1e-9
                  and abs(row['a_long_mps2'])<=self.c.ready_acceleration and abs(row['vz'])<.05)
            window=window+[row] if good else []
            if window and (row['frame']-window[0]['frame'])*self.c.dt+1e-9>=self.c.ready_seconds:break
        else:raise EarlyStop('launch_not_ready',self.actor.id)
        self.brake_request=dict(brake_request_frame=self.last_frame,brake_request_time_s=self.t,
            brake_request_progress_m=row['route_progress_m'],brake_request_speed_mps=row['speed_planar_mps'])
        self.ledger('brake_transition',actor_id=self.actor.id,reference=self.brake_request,launch_seconds=self.t-start)
        first_control=False;candidate=None;confirmed=False;stopped=[]
        for _ in range(round(self.c.brake_seconds/self.c.dt)):
            phase='stop_confirm' if stopped else 'braking'
            row=self.step(phase,0,self.c.brake,row)
            matched=brake_matched(row,self.c)
            self.fail_if(first_control and not matched,'brake_not_held',self.actor.id)
            first_control=first_control or matched
            decreasing=(row['speed_planar_mps']<=self.brake_request['brake_request_speed_mps']-self.c.deceleration_speed_drop
                        and row['a_long_mps2']<=-self.c.deceleration_threshold)
            if decreasing and candidate is not None and row['frame']==candidate+1:confirmed=True
            candidate=row['frame'] if decreasing else None
            if self.t-self.brake_request['brake_request_time_s']+1e-9>=self.c.response_timeout:
                self.fail_if(not first_control,'control_response_timeout',self.actor.id)
                self.fail_if(not confirmed,'deceleration_response_timeout',self.actor.id)
            stopped=stopped+[row] if stop_ready(row,self.c) else []
            if stopped and (row['frame']-stopped[0]['frame'])*self.c.dt+1e-9>=self.c.stopped_seconds:
                metrics=analyze(self.rows,self.c,completed=True)
                self.fail_if(not metrics['valid'],'invalid_calibration',self.actor.id)
                return
        raise EarlyStop('sustained_stop_timeout',self.actor.id)

    def cleanup(self):
        super().cleanup(manage_tm=False)
        complete=self.result.get('status')=='completed' and self.result['cleanup'].get('complete')
        try:self.metrics=analyze(self.rows,self.c,completed=complete,abort_reason=self.result.get('reason'))
        except (ValueError,KeyError,TypeError) as exc:
            self.metrics=dict(valid=False,censored=True,invalid_reasons=[self.result.get('reason') or str(exc),str(exc)],physical_bounds_qualified=False,
                              accepted_braking_lower_bound_mps2=None,accepted_response_upper_bound_s=None)
        self.result['calibration_valid']=self.metrics['valid'] and complete
        save(self.directory/'calibration_metrics.json',self.metrics)
