"""Single tick owner, two manual controllers. Entry requires completed single evidence."""
import csv
import json
import math
import time

from idm_exploratory_following import ApproachPI, ready, following_gap_gate
from idm_low_speed import ExploratoryHarness, local_pose_ok
from idm_brake_harness import physics_record
from idm_brake_metrics import FIELDS
from idm_experiment_harness import Harness, EarlyStop, save
from idm_reverse_motion import ReverseMotion
from idm_staged_stop import StagedStop, stopped

PAIR_FIELDS = ['logical_id', *FIELDS, 'leader_id', 'gap_m', 'relative_speed_mps']


def pair_budget(single_stop_m, half_lengths):
    if not math.isfinite(single_stop_m) or single_stop_m <= 0 or len(half_lengths) != 2:
        raise ValueError('unknown single stop or actual vehicle dimensions')
    if any(not math.isfinite(x) or not 0 < x <= 2.42511 for x in half_lengths):
        raise ValueError('actual dimensions exceed budget')
    gap = 20. - sum(half_lengths)
    reserve = 2. * single_stop_m + .3 * 1.2
    total = 20. + 8. + 2. * 1.2 + reserve + max(half_lengths) + 20.
    return dict(initial_center_spacing_m=20., initial_gap_m=gap,
        empirical_stop_reserve_m=reserve, required_route_m=total, route_m=60.,
        admissible=total <= 60. and gap >= 8. + reserve,
        accepted_physical_bounds=None, interpretation='twofold empirical profile plus operational delay; not a guarantee')


def single_admission(result, stability, stop):
    return (result.get('status') == 'completed' and result.get('supervised') is True
        and result.get('worker_exit_code') == 0 and result.get('collision_events') == 0
        and result.get('cleanup', {}).get('complete') is True
        and stability.get('continuous_ready_s') == 2.
        and stop.get('continuous_stop_s') == .5
        and stop.get('physical_bounds') is None)


class PairHarness(ExploratoryHarness):
    def __init__(self, *args, single_stop_m, **kwargs):
        super().__init__(*args, **kwargs)
        self.single_stop_m = single_stop_m
        self.slots = {}
        self.pair_rows = []
        self.pair_frames = 0

    def collision(self, event):
        self.record_collision(event, getattr(self, 'follower_actor', self.actor).id)

    def record_collision(self, event, owner):
        record=dict(frame=event.frame, actor_id=owner, other_actor_id=event.other_actor.id,
            other_type=event.other_actor.type_id, received_wall_ns=time.time_ns(),
            normal_impulse={k:getattr(event.normal_impulse,k) for k in ('x','y','z')})
        with self.collision_lock:
            self.collisions.append(record)
            with (self.directory/'collision_events.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(record,allow_nan=False)+'\n');f.flush()

    def collision_gate(self):
        super().collision_gate()
        for slot in self.slots.values():
            self.fail_if(not slot['sensor'].is_listening,'collision_sensor_not_listening',slot['actor'].id)

    def prepare(self):
        super().prepare()
        self.follower_actor=self.actor
        def slot(actor,sensor):
            return dict(actor=actor,sensor=sensor,previous_velocity=None,accel_count=0,
                reverse_monitor=ReverseMotion(self.reverse_monitor.road,self.c.dt),controller=ApproachPI(self.c))
        self.slots['follower']=slot(self.actor,self.sensor)
        start=self.route[20].transform
        (self.directory/'follower_initial_clearance.json').write_bytes((self.directory/'initial_clearance.json').read_bytes())
        self.initial_clearance(start)
        (self.directory/'leader_initial_clearance.json').write_bytes((self.directory/'initial_clearance.json').read_bytes())
        bp=self.call(self.world.get_blueprint_library).find(self.c.blueprint)
        bp.set_attribute('role_name','exploratory_pair:leader')
        start.location.z+=self.c.spawn_height
        def spawn():
            actor=self.spawn_exploratory_vehicle(bp,start)
            if actor is not None:self.owned['leader']=actor
            return actor
        self.actor=self.command('spawn_pair_leader',spawn)
        self.fail_if(self.actor is None,'leader_spawn_failed')
        physics=physics_record(self.call(self.actor.get_physics_control))
        self.fail_if(not isinstance(physics['mass'],(int,float)) or not math.isfinite(physics['mass'])
            or physics['mass']<=0 or len(physics['wheels'])!=4 or any(
            not isinstance(w[k],(int,float)) or not math.isfinite(w[k]) or w[k]<=0
            for w in physics['wheels'] for k in ('radius','max_brake_torque','tire_friction')),'leader_physics_unknown')
        save(self.directory/'leader_physics.json',physics)
        sensor_bp=self.call(self.world.get_blueprint_library).find('sensor.other.collision')
        leader=self.actor
        def spawn_sensor():
            sensor=self.world.spawn_actor(sensor_bp,self.api.Transform(),attach_to=leader)
            self.owned={'leader_collision':sensor,**self.owned}
            return sensor
        self.sensor=self.command('spawn_leader_collision_sensor',spawn_sensor)
        self.command('listen_leader_collision',self.sensor.listen,lambda e:self.record_collision(e,leader.id))
        self.initialize_drive_gear()
        self.slots['leader']=slot(self.actor,self.sensor)
        lengths=[abs(s['actor'].bounding_box.location.x)+s['actor'].bounding_box.extent.x for s in self.slots.values()]
        budget=pair_budget(self.single_stop_m,lengths)
        save(self.directory/'pair_distance_budget.json',budget)
        self.fail_if(not budget['admissible'],'pair_distance_or_gap_budget')
        save(self.directory/'pair_environment.json',dict(roles={k:s['actor'].id for k,s in self.slots.items()},
             manual_controllers=True,tm_created=False,autopilot_requested=False,
             strict_scene_audit='BLOCKED',route_audit=None,physical_bounds=None))

    def select(self, role):
        s=self.slots[role]
        self.actor,self.sensor=s['actor'],s['sensor']
        self.previous_velocity,self.accel_count=s['previous_velocity'],s['accel_count']
        self.reverse_monitor=s['reverse_monitor']

    def pair_step(self, phase, controls, last, *, issued_commands=None):
        self.collision_gate()
        self.fail_if(self.pair_frames>=self.c.maximum_frames,'frame_limit')
        commands={} if issued_commands is None else issued_commands
        if issued_commands is None:
            for role in ('leader','follower'):
                self.select(role);r=last.get(role)
                steer=0. if r is None else max(-self.c.steer_cap,min(self.c.steer_cap,
                    -self.c.heading_kp*r['heading_error_deg']-self.c.lateral_kp*r['signed_lateral_m']))
                commands[role]=self.issue(phase,*controls[role],steer)
        else:
            self.fail_if(set(commands)!={'leader','follower'} or any(
                cmd['based_on_frame']!=self.last_frame for cmd in commands.values()),'issued_pair_command_frame_mismatch')
        self.t+=self.c.dt;self.tick();self.pair_frames+=1
        observed={}
        for role in ('leader','follower'):
            self.select(role)
            r=self.observe(phase,commands[role]);observed[role]=r
            self.fail_if(r['z']<-.1 or r['z']>1 or abs(r['vz'])>2,'abnormal_contact_or_vertical_motion',self.actor.id)
            self.fail_if(r['gear'] is None or r['gear']<1,'drive_gear_not_engaged',self.actor.id)
            self.fail_if(any(abs(r[k]-commands[role]['control'][k])>1e-5 for k in ('throttle','brake','steer')),
                         'pair_control_not_reflected',self.actor.id)
            self.slots[role].update(previous_velocity=self.previous_velocity,accel_count=self.accel_count)
        # Conservatively include bbox offset, rotation and lateral extent in gap.
        bounds={}
        for role,r in observed.items():
            box=self.slots[role]['actor'].bounding_box
            angle=math.radians(r['heading_error_deg']+box.rotation.yaw)
            half=abs(math.cos(angle))*box.extent.x+abs(math.sin(angle))*box.extent.y
            offset=abs(box.location.x)+abs(box.location.y)
            bounds[role]=half+offset
        gap=observed['leader']['route_progress_m']-observed['follower']['route_progress_m']-sum(bounds.values())
        relative=observed['follower']['v_long_mps']-observed['leader']['v_long_mps']
        reason=following_gap_gate(gap,relative)
        for role,r in observed.items():
            row=dict(logical_id=role,**r,leader_id=self.slots['leader']['actor'].id if role=='follower' else None,
                     gap_m=gap if role=='follower' else None,relative_speed_mps=relative if role=='follower' else None)
            self.pair_rows.append(row)
            with (self.directory/'pair_observations.csv').open('a',encoding='utf-8',newline='') as f:
                csv.DictWriter(f,PAIR_FIELDS).writerow(row);f.flush()
        with (self.directory/'pair_raw.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(dict(frame=self.last_frame,sim_time_s=self.t,rows=self.pair_rows[-2:]),allow_nan=False)+'\n');f.flush()
        self.fail_if(reason is not None,reason,self.slots['follower']['actor'].id)
        time.sleep(.02);self.collision_gate()
        return observed

    def capture(self):
        for name,fields in (('calibration_observations.csv',FIELDS),('pair_observations.csv',PAIR_FIELDS)):
            with (self.directory/name).open('x',encoding='utf-8',newline='') as f:csv.DictWriter(f,fields).writeheader()
        last={}
        for _ in range(40):last=self.pair_step('settle',{k:(0.,0.) for k in self.slots},last)
        self.fail_if(not all(local_pose_ok(r) for r in self.rows[-20:]),'pair_initial_not_stable')
        origin=last['leader']['route_progress_m'];window=[]
        for _ in range(500):
            controls={k:(s['controller'].step(last[k]['speed_planar_mps']),0.) for k,s in self.slots.items()}
            last=self.pair_step('launch',controls,last)
            self.fail_if(last['leader']['route_progress_m']-origin>8.,'pair_warmup_distance')
            window=window+[last] if all(ready(r) for r in last.values()) else []
            if len(window)>=41:
                gaps=[r['leader']['route_progress_m']-r['follower']['route_progress_m'] for r in window[-41:]]
                if max(gaps)-min(gaps)<=.25:break
        else:raise EarlyStop('pair_not_continuously_ready')
        desired_gap=self.pair_rows[-1]['gap_m']
        for _ in range(40):
            follower_target=max(.8,min(1.,last['leader']['speed_planar_mps']+.1*(self.pair_rows[-1]['gap_m']-desired_gap)))
            controls={k:(s['controller'].step(last[k]['speed_planar_mps'],1. if k=='leader' else follower_target),0.)
                      for k,s in self.slots.items()}
            last=self.pair_step('launch',controls,last)
        stop_ctl={k:StagedStop(r['throttle'],self.c.dt) for k,r in last.items()};window=[]
        for _ in range(100):
            commands={k:ctl.step(last[k]['speed_planar_mps']) for k,ctl in stop_ctl.items()}
            last=self.pair_step('braking',{k:(x['throttle'],x['brake']) for k,x in commands.items()},last)
            window=window+[last] if all(stopped(r) for r in last.values()) else []
            if len(window)>=11:
                save(self.directory/'pair_completion.json',dict(continuous_stop_s=.5,
                    stop_onset_frame=window[0]['leader']['frame'],final_frame=self.last_frame,
                    pair_frames=self.pair_frames,following_capture_frames=40,collision_events=len(self.collisions),
                    control='manual PI with gap feedback, no TM/IDM-generated trajectory',
                    calibration_valid=False,route_audit=None,physical_bounds=None))
                return
        raise EarlyStop('pair_stop_timeout')

    def cleanup(self):
        if not self.slots:
            return super().cleanup()
        if not self.gate.poisoned:
            for s in self.slots.values():
                try:self.frame_unchanged();self.call(s['sensor'].stop)
                except BaseException as exc:self.result['sensor_stop_error']=repr(exc)
        Harness.cleanup(self,manage_tm=False)
        self.result.update(collision_events=len(self.collisions),motion_samples=len(self.rows),
            pair_frames=self.pair_frames,route_audit=None,physical_bounds=None)
        save(self.directory/'collision_summary.json',dict(events=self.collisions,absence_is_not_geometry_proof=True))
        if self.collisions:self.result.update(status='early_stop',reason='collision_event',exit_code=2)
