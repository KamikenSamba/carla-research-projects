"""Bound the existing V1/V2 snapshot spawner; never teleport a live actor."""
from dataclasses import asdict
import json
import math
from types import SimpleNamespace as NS

from idm_snapshot import read_snapshot, resolve_parameters
from idm_experiment_harness import EarlyStop, save
from Run_DT_Risk_V1 import spawn_from_snapshot


def physics_settings(record):
    # CARLA 0.9.16 GetVehiclePhysicsControl reads wheel Position from the
    # current Wheels[i]->Location. Preserve it in evidence, not as a static setting.
    result={k:v for k,v in record.items() if k not in ('wheels','wheel_rpm','unknown_fields')}
    result['wheels']=[{k:v for k,v in w.items() if k!='position'} for w in record.get('wheels',[])]
    return result


class SnapshotRestorer:
    def __init__(self,h,path,expected_parameters):
        self.h=h;self.rows={r['logical_actor_id']:r for r in read_snapshot(path)}
        if set(self.rows)!={'leader','follower'}:raise ValueError('fixed two-vehicle snapshot required')
        args=NS(default_idm_headway=1.,idm_desired_speed_mps=.5,idm_min_desired_speed_mps=.5,
            idm_minimum_gap_m=2.,idm_max_accel_mps2=.3,idm_comfortable_decel_mps2=1.5,
            idm_acceleration_exponent=4.,idm_max_decel_mps2=2.)
        self.parameters={k:resolve_parameters(r,args,{}) for k,r in self.rows.items()}
        source=json.loads(expected_parameters.read_text(encoding='utf-8'))
        if {k:asdict(p) for k,p in self.parameters.items()}!=source:
            raise ValueError('snapshot T/v0 and original resolved parameters differ')
        for r in self.rows.values():
            if (r['blueprint_id']!='vehicle.tesla.model3' or r['autopilot_enabled']
                or math.sqrt(sum(r[k]**2 for k in ('vx','vy','vz')))>.01
                or not -.01<=r['z']<=.05):
                raise ValueError('only observed stationary Model3 snapshot supported')
        self.spawned=[];self.velocity_requests=set()

    def spawn(self,bp,expected_transform):
        h=self.h;logical='follower' if not self.spawned else 'leader'
        if logical in self.spawned:raise EarlyStop('snapshot_duplicate_spawn')
        row=self.rows[logical]
        if (math.hypot(row['x']-expected_transform.location.x,row['y']-expected_transform.location.y)>.05
            or abs((row['yaw']-expected_transform.rotation.yaw+180)%360-180)>.2):
            raise EarlyStop('snapshot_outside_checked_initial_volume')
        restorer=self
        class Spawned:
            def __init__(self,raw):self.raw=raw
            def __getattr__(self,k):return getattr(self.raw,k)
            def set_target_velocity(self,velocity):
                if logical in restorer.velocity_requests:raise EarlyStop('repeated_snapshot_velocity_request')
                restorer.velocity_requests.add(logical)
                h.command('snapshot_initial_velocity_once',self.raw.set_target_velocity,velocity,actor=self.raw.id)
        class World:
            def get_blueprint_library(self):return h.call(h.world.get_blueprint_library)
            def try_spawn_actor(self,blueprint,transform):
                # The legacy helper's first pose is used, but its height retries are disabled.
                if transform.location.z>.25:raise EarlyStop('snapshot_spawn_height_retry_forbidden')
                h.frame_unchanged()
                actor=h.call(h.world.try_spawn_actor,blueprint,transform)
                if actor is None:raise EarlyStop('snapshot_spawn_failed_no_retry')
                h.owned['vehicle' if logical=='follower' else 'leader']=actor
                if actor.type_id!='vehicle.tesla.model3':raise EarlyStop('snapshot_blueprint_mismatch')
                return Spawned(actor)
        created=NS(add_actor=lambda a:None)
        actors=spawn_from_snapshot(World(),[row],created,__import__('logging').getLogger('v2-reconstruction'))
        # The legacy helper logs velocity-setting exceptions; a poisoned gate must
        # still prevent the caller from inspecting or making another server call.
        h.call(lambda:None)
        self.spawned.append(logical)
        return actors[logical].raw

    def compare(self,actual):
        comparisons={}
        for row in actual:
            logical=row['logical_actor_id'];source=self.rows[logical]
            comparisons[logical]=dict(position_error_m=math.dist([row[k] for k in ('x','y','z')],
                [source[k] for k in ('x','y','z')]),velocity_error_mps=math.dist([row[k] for k in ('vx','vy','vz')],
                [source[k] for k in ('vx','vy','vz')]),yaw_error_deg=abs((row['yaw']-source['yaw']+180)%360-180),
                source_actor_id=source['source_actor_id'],reconstructed_actor_id=row['source_actor_id'])
        valid=(set(comparisons)==set(self.rows) and all(x['position_error_m']<=.02 and
               x['velocity_error_mps']<=.01 and x['yaw_error_deg']<=.2 for x in comparisons.values()))
        save(self.h.directory/'reconstruction_comparison.json',dict(within_initial_state_tolerance=valid,
            actors=comparisons,physics_internal_state_restored=False,
            limitations='snapshot does not encode engine RPM, suspension state or all physics internal state; no exact physical reset claim'))
        if not valid:raise EarlyStop('reconstructed_initial_state_mismatch')
