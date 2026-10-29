"""Bounded real-CARLA validation of the existing V2 IDMFleetController."""
import csv
import json
from dataclasses import asdict, dataclass
import logging
import math
import time
from types import SimpleNamespace as NS

from idm_controller import IDMFleetController, ControlConfig, LOG_FIELDS
from idm_model import IDMParameters
from idm_low_speed import ExploratoryConfig, ExploratoryHarness, config_for, local_pose_ok
from idm_brake_metrics import FIELDS
from idm_experiment_harness import EarlyStop, save
from idm_staged_stop import StagedStop, stopped
from Run_DT_Risk_V2_IDM import save_snapshot
from Run_DT_Risk_V1 import detect_near_misses
from dt_risk_common import PredictionState, prediction_state_fieldnames, risk_event_fieldnames, to_row
from idm_pair_exploration import PairHarness
from idm_v2_reconstruction import SnapshotRestorer, physics_settings


@dataclass(frozen=True)
class ValidationConfig(ExploratoryConfig):
    mode: str = 'existing_v2_idm_carla_v1'
    trial: str = 'single'
    drive_seconds: float = 6.

    def __post_init__(self):
        expected=asdict(config_for('trial1'))
        expected.update(mode='existing_v2_idm_carla_v1',trial=self.trial,drive_seconds=6.)
        if self.trial not in ('single','fleet','replay') or asdict(self)!=expected:
            raise ValueError('outside fixed V2 IDM validation design')


def validation_parameters(logical):
    # Known assigned parameters, not fitted traits. All use the existing IDM type.
    return IDMParameters(desired_speed_mps=.6 if logical=='follower' else .5,
        desired_time_headway_s=2.0 if logical=='follower' else 1.0,
        minimum_gap_m=2.,max_accel_mps2=.3,comfortable_decel_mps2=1.5,max_decel_mps2=2.)


class GuardedActor:
    def __init__(self,h,actor):self.h,self.raw=h,actor
    def __getattr__(self,name):
        if name in ('get_transform','get_velocity','get_location'):
            return lambda *a,**kw:self.h.call(getattr(self.raw,name),*a,**kw)
        if name in ('id','type_id','attributes','bounding_box'):
            return self.h.call(lambda:getattr(self.raw,name))
        raise AttributeError('not an allowed V2 validation actor operation: '+name)


class GuardedWaypoint:
    def __init__(self,h,wp):self.h,self.raw=h,wp
    def next(self,distance):
        return [GuardedWaypoint(self.h,w) for w in self.h.call(self.raw.next,distance)]
    def __getattr__(self,name):
        if name not in ('transform','is_junction','road_id','section_id','lane_id','lane_width'):
            raise AttributeError(name)
        return self.h.call(lambda:getattr(self.raw,name))


class GuardedMap:
    def __init__(self,h):self.h=h
    def get_waypoint(self,*a,**kw):
        wp=self.h.call(self.h.map.get_waypoint,*a,**kw)
        return None if wp is None else GuardedWaypoint(self.h,wp)


class GuardedWorld:
    def __init__(self,h):self.h=h
    def get_map(self):return GuardedMap(self.h)
    def get_actors(self):
        actors=list(self.h.call(self.h.world.get_actors))
        def selected(pattern):
            if pattern!='vehicle.*':raise ValueError('unexpected V2 actor filter')
            return [GuardedActor(self.h,a) for a in actors if a.type_id.startswith('vehicle.')]
        return NS(filter=selected)


class BoundedV2Actuator:
    """Transparent experimental scaling/slew; original V2 requests are kept intact."""
    def __init__(self,c):self.c=c;self.throttle=0.;self.brake=0.;self.last_diagnostics=None
    def step(self,request,speed,reason):
        if reason:raise EarlyStop('v2_fallback:'+reason)
        values=[request.throttle,request.brake,request.steer,speed]
        if any(not math.isfinite(x) for x in values):raise EarlyStop('v2_nonfinite_command')
        if request.brake>0 and speed>.1:raise EarlyStop('v2_brake_transition_not_qualified')
        desired=min(self.c.throttle_cap,max(0.,request.throttle*.10))
        slew=self.c.throttle_slew_per_second*self.c.dt
        previous_throttle,previous_brake=self.throttle,self.brake
        self.throttle+=max(-slew,min(slew,desired-self.throttle))
        self.brake=min(.02,max(0.,request.brake*.10),self.brake+.10*self.c.dt)
        if request.brake>0:self.throttle=0.
        self.last_diagnostics=dict(positive_scale=.10,throttle_cap=self.c.throttle_cap,
            throttle_scaled=request.throttle*.10,throttle_after_cap=desired,
            previous_throttle=previous_throttle,throttle_step_limit=slew,
            throttle_slew_limited=abs(desired-previous_throttle)>slew,
            previous_brake=previous_brake,brake_scaled=request.brake*.10,brake_cap=.02,
            brake_step_limit=.10*self.c.dt,final_throttle=self.throttle,final_brake=self.brake)
        return self.throttle,self.brake,max(-self.c.steer_cap,min(self.c.steer_cap,request.steer))


V2_FIELDS=['observation_frame','command_based_on_frame','observation_time_s','logical_actor_id',
    'actor_id','x','y','z','speed_planar_mps','a_long_mps2','a_fd_long_mps2',
    'actual_throttle','actual_brake','actual_steer',
    *['v2_'+k for k in LOG_FIELDS],
    'T_s','v0_mps','s0_m','a_mps2','b_mps2','maximum_deceleration_parameter_mps2']


class V2SingleHarness(ExploratoryHarness):
    def initialize_fleet(self,actors,last):
        self.parameters=(self.restorer.parameters if getattr(self,'restorer',None) else
                         {k:validation_parameters(k) for k in actors})
        self.last_observed=last;self.sent={};self.actuators={k:BoundedV2Actuator(self.c) for k in actors}
        self.logical_actors=actors
        def sink(logical,actor,request,reason):
            self.frame_unchanged()
            self.ledger('v2_requested_control',logical_actor_id=logical,
                requested=dict(throttle=request.throttle,brake=request.brake,steer=request.steer),
                fallback_reason=reason,measured_speed_mps=last[logical]['speed_planar_mps'])
            if hasattr(self,'select') and logical in getattr(self,'slots',{}):self.select(logical)
            throttle,brake,steer=self.actuators[logical].step(request,last[logical]['speed_planar_mps'],reason)
            self.sent[logical]=self.issue('launch',throttle,brake,steer)
        control_config=ControlConfig(maximum_steer=self.c.steer_cap,leader_search_m=30.)
        self.fleet=IDMFleetController(GuardedWorld(self),{k:GuardedActor(self,a) for k,a in actors.items()},
            self.parameters,self.c.dt,control_config,logging.getLogger('v2-validation'),
            control_sink=sink,disable_autopilot=False)
        save(self.directory/'resolved_idm_parameters.json',{k:asdict(p) for k,p in self.parameters.items()})
        save(self.directory/'resolved_control_config.json',asdict(control_config))
        snapshot=[]
        for logical,r in last.items():
            actor=actors[logical];p=self.parameters[logical]
            snapshot.append(dict(phase='snapshot',carla_frame=r['frame'],sim_time_s=r['sim_time_s'],
                source_actor_id=actor.id,logical_actor_id=logical,role_name=actor.attributes.get('role_name',''),
                blueprint_id=self.c.blueprint,autopilot_enabled=False,
                **{k:r[k] for k in ('x','y','z','roll','pitch','yaw','vx','vy','vz','speed_mps')},
                desired_time_headway_s=p.desired_time_headway_s,desired_speed_mps=p.desired_speed_mps))
        save_snapshot(self.directory/'initial_snapshot.csv',snapshot)
        if getattr(self,'restorer',None):self.restorer.compare(snapshot)
        self.prediction_origin=self.t;self.near_events=[];self.near_regions=[];self.near_cooldown={}
        with (self.directory/'prediction_states.csv').open('x',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,prediction_state_fieldnames()).writeheader()
        with (self.directory/'risk_events.csv').open('x',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,risk_event_fieldnames()).writeheader()
        with (self.directory/'v2_validation.csv').open('x',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,V2_FIELDS).writeheader()
        (self.directory/'v2_control_diagnostics.jsonl').open('x',encoding='utf-8').close()

    def log_idm(self,requests,observed):
        new_events=len(self.near_events)
        args=NS(near_miss_distance_m=4.,min_closing_speed_mps=1.,near_miss_cooldown_s=1.,near_miss_region_radius_m=4.)
        self.frame_unchanged()
        detect_near_misses({k:GuardedActor(self,a) for k,a in self.logical_actors.items()},
            self.near_events,self.near_regions,0.,self.t-self.prediction_origin,args,self.near_cooldown)
        self.frame_unchanged()
        for event in self.near_events[new_events:]:
            with (self.directory/'risk_events.csv').open('a',encoding='utf-8',newline='') as f:
                csv.DictWriter(f,risk_event_fieldnames()).writerow(to_row(event))
        for request in requests:
            logical=request['logical_actor_id'];r=observed[logical];p=self.parameters[logical]
            diagnostic=dict(schema_version=1,provenance='recorded_during_control',
                logical_actor_id=logical,actor_id=r['actor_id'],
                state_frame=r['command_based_on_frame'],command_frame=r['command_based_on_frame'],
                response_frame=r['frame'],state_time_s=request['simulation_time'],
                response_time_s=r['sim_time_s'],
                controller=self.fleet.controllers[logical].last_diagnostics,
                actuator=self.actuators[logical].last_diagnostics,
                observed_control={k:r[k] for k in ('throttle','brake','steer')},
                api_longitudinal_acceleration_mps2=r['a_long_mps2'],
                vector_fd_longitudinal_acceleration_mps2=r['a_fd_long_mps2'])
            with (self.directory/'v2_control_diagnostics.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(diagnostic,allow_nan=False)+'\n');f.flush()
            row=dict(observation_frame=r['frame'],command_based_on_frame=r['command_based_on_frame'],
                observation_time_s=r['sim_time_s'],logical_actor_id=logical,actor_id=r['actor_id'],
                **{k:r[k] for k in ('x','y','z','speed_planar_mps','a_long_mps2','a_fd_long_mps2')},
                actual_throttle=r['throttle'],actual_brake=r['brake'],actual_steer=r['steer'],
                **{'v2_'+k:v for k,v in request.items()},T_s=p.desired_time_headway_s,v0_mps=p.desired_speed_mps,
                s0_m=p.minimum_gap_m,a_mps2=p.max_accel_mps2,b_mps2=p.comfortable_decel_mps2,
                maximum_deceleration_parameter_mps2=p.max_decel_mps2)
            with (self.directory/'v2_validation.csv').open('a',encoding='utf-8',newline='') as f:
                csv.DictWriter(f,V2_FIELDS).writerow(row);f.flush()
            prediction=PredictionState(prediction_frame=r['frame'],prediction_time_s=self.t-self.prediction_origin,
                logical_actor_id=logical,carla_actor_id=r['actor_id'],autopilot_enabled=False,
                **{k:r[k] for k in ('x','y','z','yaw','vx','vy','vz','speed_mps')})
            with (self.directory/'prediction_states.csv').open('a',encoding='utf-8',newline='') as f:
                csv.DictWriter(f,prediction_state_fieldnames()).writerow(to_row(prediction));f.flush()

    def capture(self):
        with (self.directory/'calibration_observations.csv').open('x',encoding='utf-8',newline='') as f:
            csv.DictWriter(f,FIELDS).writeheader()
        row=None
        for _ in range(40):row=self.step('settle',0,0,row)
        self.fail_if(not all(local_pose_ok(r) for r in self.rows[-10:]),'initial_vehicle_not_stable')
        origin=row['route_progress_m'];last={'solo':row};self.initialize_fleet({'solo':self.actor},last)
        for _ in range(round(self.c.drive_seconds/self.c.dt)):
            self.collision_gate();self.frame_unchanged()
            requests=self.fleet.step(self.t);self.frame_unchanged()
            self.t+=self.c.dt;self.tick();row=self.observe('launch',self.sent['solo'])
            self.fail_if(row['z']<-.1 or row['z']>1 or abs(row['vz'])>2,'abnormal_contact_or_vertical_motion')
            self.fail_if(row['gear'] is None or row['gear']<1,'drive_gear_not_engaged')
            self.fail_if(row['route_progress_m']-origin>3.,'v2_single_distance_limit')
            for k in ('throttle','brake','steer'):
                self.fail_if(abs(row[k]-self.sent['solo']['control'][k])>1e-5,'v2_control_not_reflected')
            last['solo']=row;self.log_idm(requests,last)
            time.sleep(.02);self.collision_gate()
        self.fail_if(row['route_progress_m']-origin<.10,'v2_forward_progress_insufficient')
        self.controlled_stop(row)

    def controlled_stop(self,row):
        reference=dict(frame=row['frame'],sim_time_s=row['sim_time_s'],progress_m=row['route_progress_m'],speed_mps=row['speed_planar_mps'])
        ctl=StagedStop(row['throttle'],self.c.dt);window=[]
        for _ in range(round(self.c.brake_seconds/self.c.dt)):
            command=ctl.step(row['speed_planar_mps']);self.ledger('v2_validation_stop',**command)
            row=self.step('braking',command['throttle'],command['brake'],row)
            self.fail_if(any(abs(row[k]-command[k])>1e-5 for k in ('throttle','brake')),'stop_control_not_reflected')
            window=window+[row] if stopped(row) else []
            if len(window)>=11:
                save(self.directory/'v2_completion.json',dict(idm_vehicle_count=1,idm_frames=120,
                    stop_request=reference,stop_onset_frame=window[0]['frame'],final_frame=row['frame'],
                    stop_distance_m=window[0]['route_progress_m']-reference['progress_m'],
                    stop_time_s=window[0]['sim_time_s']-reference['sim_time_s'],continuous_stop_s=.5,
                    strict_scene_audit='BLOCKED',route_audit=None,physical_bounds=None,
                    lower_controller='V2 PID/feed-forward with explicit 0.10 effort scaling and safety envelope',
                    end_stop='separate staged stop; no mandatory zero-input coasting'))
                return
        raise EarlyStop('v2_controlled_stop_timeout',self.actor.id)


class V2FleetHarness(PairHarness):
    initialize_fleet=V2SingleHarness.initialize_fleet
    log_idm=V2SingleHarness.log_idm

    def __init__(self,*args,single_stop_m,source_run=None,**kwargs):
        super().__init__(*args,single_stop_m=single_stop_m,**kwargs)
        self.source_run=source_run
        self.restorer=(SnapshotRestorer(self,source_run/'initial_snapshot.json',source_run/'resolved_idm_parameters.json')
                       if source_run is not None else None)

    def prepare(self):
        super().prepare()
        if self.source_run is not None:
            source=json.loads((self.source_run/'environment.json').read_text(encoding='utf-8'))
            actual=json.loads((self.directory/'environment.json').read_text(encoding='utf-8'))
            keys=('versions','map','initial_actor_ids','original_settings','applied_settings','weather')
            comparison={k:source[k]==actual[k] for k in keys}
            wheel_positions={}
            for name in ('vehicle_physics.json','leader_physics.json'):
                old=json.loads((self.source_run/name).read_text(encoding='utf-8'))
                new=json.loads((self.directory/name).read_text(encoding='utf-8'))
                comparison[name]=physics_settings(old)==physics_settings(new)
                wheel_positions[name]=dict(source=[w['position'] for w in old.get('wheels',[])],
                    actual=[w['position'] for w in new.get('wheels',[])],compared_as_static_settings=False)
            save(self.directory/'physics_wheel_position_observations.json',wheel_positions)
            save(self.directory/'source_environment_comparison.json',comparison)
            self.fail_if(not all(comparison.values()),'snapshot_source_environment_mismatch')

    def spawn_exploratory_vehicle(self,bp,tf):
        return self.restorer.spawn(bp,tf) if self.restorer else super().spawn_exploratory_vehicle(bp,tf)

    def capture(self):
        from idm_pair_exploration import PAIR_FIELDS
        for name,fields in (('calibration_observations.csv',FIELDS),('pair_observations.csv',PAIR_FIELDS)):
            with (self.directory/name).open('x',encoding='utf-8',newline='') as f:csv.DictWriter(f,fields).writeheader()
        last={}
        for _ in range(40):last=self.pair_step('settle',{k:(0.,0.) for k in self.slots},last)
        self.fail_if(not all(local_pose_ok(r) for r in self.rows[-20:]),'pair_initial_not_stable')
        origins={k:r['route_progress_m'] for k,r in last.items()}
        self.initialize_fleet({k:s['actor'] for k,s in self.slots.items()},last)
        for _ in range(120):
            self.frame_unchanged();requests=self.fleet.step(self.t);self.frame_unchanged()
            observed=self.pair_step('launch',None,last,issued_commands=dict(self.sent))
            for k,r in observed.items():
                self.fail_if(r['route_progress_m']-origins[k]>3.,'v2_fleet_distance_limit',r['actor_id'])
            last.update(observed);self.log_idm(requests,last)
        self.fail_if(any(r['route_progress_m']-origins[k]<.1 for k,r in last.items()),'v2_forward_progress_insufficient')
        reference={k:dict(frame=r['frame'],sim_time_s=r['sim_time_s'],progress_m=r['route_progress_m'],speed_mps=r['speed_planar_mps']) for k,r in last.items()}
        controllers={k:StagedStop(r['throttle'],self.c.dt) for k,r in last.items()};window=[]
        for _ in range(100):
            controls={k:ctl.step(last[k]['speed_planar_mps']) for k,ctl in controllers.items()}
            last.update(self.pair_step('braking',{k:(x['throttle'],x['brake']) for k,x in controls.items()},last))
            window=window+[dict(last)] if all(stopped(r) for r in last.values()) else []
            if len(window)>=11:
                save(self.directory/'v2_completion.json',dict(idm_vehicle_count=2,idm_frames=120,
                    stop_request=reference,stop_onset_frame=window[0]['leader']['frame'],final_frame=self.last_frame,
                    stop_distance_m={k:window[0][k]['route_progress_m']-ref['progress_m'] for k,ref in reference.items()},
                    stop_time_s=window[0]['leader']['sim_time_s']-reference['leader']['sim_time_s'],continuous_stop_s=.5,
                    near_miss_events=len(self.near_events),strict_scene_audit='BLOCKED',route_audit=None,physical_bounds=None,
                    control='existing V2 IDM/PID/feed-forward plus bounded actuator',
                    snapshot_reconstructed=self.restorer is not None))
                return
        raise EarlyStop('v2_fleet_stop_timeout')
