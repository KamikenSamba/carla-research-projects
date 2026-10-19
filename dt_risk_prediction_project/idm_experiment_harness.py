"""Single-owner experiment worker. Importing this module never connects to CARLA."""
from contextlib import ExitStack
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace as NS

from dt_risk_common import CsvWriter, actor_state_fieldnames, actor_state_from_actor, to_row
from idm_context import FollowingContextCollector
from idm_harness_host import ServerUnavailable
from idm_leader import find_leader, path_projection, sample_vehicle
from Run_DT_Risk_V2_IDM import save_snapshot


class EarlyStop(Exception):
    def __init__(self, reason, actor=None):
        self.reason, self.actor = reason, actor
        super().__init__(reason)


def settings_dict(settings):
    return {key: getattr(settings, key) for key in dir(settings)
            if not key.startswith("_") and not callable(getattr(settings, key))}


def save(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


AUDIT_FIELDS = ["phase", "carla_frame", "sim_time_s", "world_time_s", "source_actor_id", "logical_actor_id",
    "x", "y", "z", "vx", "vy", "vz", "speed_mps", "yaw", "ax", "ay", "az", "a_long_mps2",
    "throttle", "brake", "hand_brake", "steer", "reverse", "gear", "manual_gear_shift",
    "autopilot_call_result", "autopilot_registered_verified", "tm_port", "seed_requested", "seed_applied",
    "world_sync", "tm_sync_command_succeeded", "command_sequence", "command_based_on_frame",
    "leader_desired_speed_mps", "follower_desired_speed_mps", "control_read_at_frame",
    "leader_id", "gap_m", "pair_gap_m", "relative_speed_mps", "road_id", "section_id", "lane_id",
    "route_progress_m", "route_lateral_m", "heading_error_deg", "front_clearance_m",
    "signal_stop", "lane_change", "is_junction", "fallback_reason", "at_traffic_light",
    "traffic_light_state", "tm_next_action", "observation_error", "collision_event"]


class Harness:
    def __init__(self, client, api, config, stage, directory, gate, ownership):
        self.client, self.api, self.c = client, api, config
        self.stage, self.directory = stage, Path(directory)
        self.gate, self.ownership = gate, ownership
        self.call = gate.call
        self.world = self.tm = self.original = None
        self.owned, self.registered = {}, set()
        self.tm_owned = self.settings_touched = False
        self.last_frame = self.last_time = None
        self.t = 0.0
        self.active_actor = None
        self.sequence = 0
        self.seed_applied = False
        self.holds = {}
        self.metrics = {"moving_frames": 0, "eligible": 0, "valid": 0,
                        "past_eligible": 0, "past_valid": 0, "speed": [], "gap": []}
        self.result = dict(status="running", stage=stage, pid=os.getpid(), config_sha256=config.fingerprint,
                           real_physics_verified=False, final_frame=None, sim_time_s=0, cleanup={})

    def ledger(self, operation, **data):
        self.sequence += 1
        row = dict(sequence=self.sequence, operation=operation, wall_time_ns=time.time_ns(),
                   based_on_frame=self.last_frame, sim_time_s=self.t, **data)
        with (self.directory / "command_ledger.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
        return self.sequence

    def command(self, operation, function, *args, actor=None, **kwargs):
        def describe(value):
            if value is None or isinstance(value,(bool,int,float,str)):
                return value
            if isinstance(value,(list,tuple)):
                return [describe(v) for v in value]
            if hasattr(value,'id'):
                return dict(actor_id=value.id)
            if all(hasattr(value,k) for k in ('x','y','z')):
                return dict(x=value.x,y=value.y,z=value.z)
            return dict(type=type(value).__name__)
        seq = self.ledger(operation, actor_id=actor, outcome="requested",
                          arguments=[describe(a) for a in args],keyword_arguments=kwargs)
        try:
            returned = self.call(function, *args, **kwargs)
        except BaseException as exc:
            self.ledger(operation, actor_id=actor, outcome="failed", error=str(exc), request_sequence=seq)
            raise
        self.ledger(operation, actor_id=actor, outcome="call_returned",
                    returned=returned if isinstance(returned, (bool, int, str)) else None,
                    request_sequence=seq, registration_verified=None)
        return returned

    def fail_if(self, condition, reason, actor=None):
        if condition:
            raise EarlyStop(reason, actor)

    def hold(self, key, condition, duration, actor=None):
        self.holds[key] = self.holds.get(key, 0.0) + self.c.dt if condition else 0.0
        self.fail_if(self.holds[key] + 1e-9 >= duration, key, actor)

    def frame_unchanged(self):
        snap = self.call(self.world.get_snapshot)
        self.fail_if(int(snap.frame) != self.last_frame, "unexpected_frame_progression")
        return snap

    def tick(self):
        self.frame_unchanged()
        frame = int(self.call(self.world.tick))
        snap = self.call(self.world.get_snapshot)
        self.fail_if(frame != self.last_frame + 1 or int(snap.frame) != frame,
                     "unexpected_frame_progression")
        now = float(snap.timestamp.elapsed_seconds)
        self.fail_if(abs(now - self.last_time - self.c.dt) > 1e-5, "unexpected_time_progression")
        self.last_frame, self.last_time = frame, now
        self.result.update(final_frame=frame, sim_time_s=self.t)
        return snap

    def prepare(self):
        c = self.c
        self.world = self.call(self.client.get_world)
        versions = [self.call(self.client.get_client_version), self.call(self.client.get_server_version)]
        self.fail_if(versions != ["0.9.16", "0.9.16"], "version_mismatch")
        self.map = self.call(self.world.get_map)
        self.fail_if(self.map.name.split("/")[-1] != c.map_name, "map_mismatch")
        self.original = self.call(self.world.get_settings)
        self.fail_if(self.original.synchronous_mode, "world_already_synchronous")
        actors = list(self.call(self.world.get_actors))
        self.initial_ids = {a.id for a in actors}
        self.fail_if(any(a.type_id.startswith(("vehicle.", "walker.", "sensor.", "controller."))
                         for a in actors), "external_dynamic_actors_present")
        self.route = []
        for s in range(int(c.follower_s), int(c.end_s) - 1, -1):
            wp = self.map.get_waypoint_xodr(c.road, c.lane, s)
            self.fail_if(wp is None or (wp.road_id, wp.section_id, wp.lane_id) != (c.road, c.section, c.lane)
                         or wp.is_junction or wp.lane_width < 3.4, "route_preflight_failed")
            if self.route:
                delta = (wp.transform.rotation.yaw-self.route[0].transform.rotation.yaw+180)%360-180
                self.fail_if(abs(delta) > 0.5, "route_not_straight")
                following = self.route[-1].next(1.0)
                self.fail_if(len(following) != 1 or following[0].is_junction or
                             (following[0].road_id, following[0].section_id, following[0].lane_id)
                             != (c.road, c.section, c.lane), "route_branch")
            self.route.append(wp)
        self.fail_if(bool(self.route[0].get_landmarks(95, stop_at_junction=False)), "regulatory_landmarks")
        self.path = [(w.transform.location.x, w.transform.location.y) for w in self.route]
        self.route_length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
        # Parked/static road users may not be vehicle actors. Query their level boxes too.
        labels = ('Car','Truck','Bus','Motorcycle','Bicycle','Pedestrians')
        self.fail_if(not hasattr(self.world,'get_level_bbs') or not hasattr(self.api,'CityObjectLabel'),
                     'obstacle_inventory_unavailable')
        angle = math.radians(self.route[0].transform.rotation.yaw)
        ux,uy=math.cos(angle),math.sin(angle)
        for label in labels:
            boxes=self.call(self.world.get_level_bbs,getattr(self.api.CityObjectLabel,label))
            for box in boxes:
                dx,dy=box.location.x-self.path[0][0],box.location.y-self.path[0][1]
                along,lateral=dx*ux+dy*uy,abs(-dx*uy+dy*ux)
                relative=math.radians(box.rotation.yaw)-angle
                length=abs(math.cos(relative))*box.extent.x+abs(math.sin(relative))*box.extent.y
                width=abs(math.cos(relative))*box.extent.y+abs(math.sin(relative))*box.extent.x
                self.fail_if(along+length>=0 and along-length<=self.route_length and lateral-width<=1.75,
                             'static_road_user_obstacle')
        self.ownership.require_unused()
        self.tm = self.call(self.client.get_trafficmanager, c.tm_port)
        # A fresh worker plus unused->self listener proof excludes an existing Remote TM.
        # A racing external claimant is rejected before sync/seed/actor mutations.
        self.ownership.require_self()
        self.tm_owned = True
        self.ledger("tm_ownership", port=c.tm_port, owner_pid=os.getpid(), outcome="os_listener_verified")
        settings = self.call(self.world.get_settings)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = c.dt
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        self.settings_touched = True
        self.command("world_sync", self.world.apply_settings, settings)
        snap = self.call(self.world.get_snapshot)
        self.last_frame, self.last_time = int(snap.frame), float(snap.timestamp.elapsed_seconds)
        returned = self.command("tm_sync", self.tm.set_synchronous_mode, True)
        self.fail_if(returned is False,"tm_sync_failed")
        actual = self.call(self.world.get_settings)
        self.fail_if(not actual.synchronous_mode or abs(actual.fixed_delta_seconds-c.dt)>1e-9,
                     "world_sync_not_applied")
        lights = [a.id for a in actors if a.type_id.startswith("traffic.traffic_light")]
        if lights:
            self.ledger("seed", requested=c.seed, applied=False, reason="setter_resets_external_traffic_lights")
        else:
            self.command("seed", self.tm.set_random_device_seed, c.seed)
            self.seed_applied = True
        self.command("hybrid_physics", self.tm.set_hybrid_physics_mode, False)
        self.command("respawn", self.tm.set_respawn_dormant_vehicles, False)
        snap = self.call(self.world.get_snapshot)
        self.last_frame, self.last_time = int(snap.frame), float(snap.timestamp.elapsed_seconds)
        for role, offset in (("follower", 0), ("leader", int(c.follower_s-c.leader_s))):
            bp = self.call(self.world.get_blueprint_library).find(c.blueprint)
            bp.set_attribute("role_name", "idm_harness:" + role)
            tf = self.route[offset].transform
            tf.location.z += c.spawn_height
            def spawn_owned(blueprint, transform):
                spawned = self.world.try_spawn_actor(blueprint, transform)
                if spawned is not None:
                    # Establish ownership before any post-spawn logging can fail.
                    self.owned[role] = spawned
                return spawned
            actor = self.command("spawn_"+role, spawn_owned, bp, tf)
            self.fail_if(actor is None, "spawn_failed", role)
            self.owned[role] = actor
            self.ledger("owned_actor", role=role, actor_id=actor.id, blueprint=actor.type_id,
                        spawn=dict(x=tf.location.x, y=tf.location.y, z=tf.location.z,
                                   yaw=tf.rotation.yaw, road=c.road, section=c.section, lane=c.lane))
        # Settle under physics, no forced target velocity. All ticks go through tick().
        for _ in range(20):
            self.tick()
            for role,actor in self.owned.items():
                tf=self.call(actor.get_transform)
                velocity=self.call(actor.get_velocity)
                control=self.call(actor.get_control)
                self.ledger('settle_observation',actor_id=actor.id,role=role,
                            frame=self.last_frame,world_time_s=self.last_time,
                            x=tf.location.x,y=tf.location.y,z=tf.location.z,
                            velocity={k:getattr(velocity,k) for k in ('x','y','z')},
                            control={k:getattr(control,k,None) for k in
                                     ('throttle','brake','hand_brake','steer','reverse','gear')},
                            autopilot_requested=False,autopilot_registered_verified=None)
            self.frame_unchanged()
        for role, actor in self.owned.items():
            self.registered.add(actor.id)  # cleanup also covers a successful call with failed logging
            returned = self.command("autopilot_register", actor.set_autopilot, True, c.tm_port, actor=actor.id)
            self.fail_if(returned is False,"autopilot_register_failed",actor.id)
            self.command("lane_change", self.tm.auto_lane_change, actor, False, actor=actor.id)
            self.command("following_distance", self.tm.distance_to_leading_vehicle,
                         actor, c.following_distance, actor=actor.id)
            offset = 0 if role == "follower" else int(c.follower_s-c.leader_s)
            path = [w.transform.location for w in self.route[offset+1:]]
            self.command("path", self.tm.set_path, actor, path, actor=actor.id)
            self.ledger("path_points", actor_id=actor.id,
                        points=[[p.x,p.y,p.z] for p in path])
        save(self.directory / "environment.json", dict(versions=versions, map=self.map.name,
             original_settings=settings_dict(self.original), initial_actor_ids=sorted(self.initial_ids),
             tm_port=c.tm_port, tm_owner_pid=os.getpid(), seed_requested=c.seed,
             seed_applied=self.seed_applied, registration_verified=None,
             route_length_m=self.route_length, route=[[x,y] for x,y in self.path]))

    def speed_command(self, speed):
        based_on = self.last_frame
        self.command("leader_speed_kmh", self.tm.set_desired_speed, self.owned["leader"], speed*3.6,
                     actor=self.owned["leader"].id)
        self.command("follower_speed_kmh", self.tm.set_desired_speed, self.owned["follower"],
                     self.c.follower_speed*3.6, actor=self.owned["follower"].id)
        return dict(sequence=self.sequence, based_on=based_on, speed=speed)

    def observe(self, phase, sim_time, command, collector=None):
        snap = self.frame_unchanged()
        current = list(self.call(self.world.get_actors))
        current_ids = {a.id for a in current}
        self.fail_if(not self.initial_ids.issubset(current_ids), "external_actor_disappeared")
        self.fail_if(any(a.id not in self.initial_ids | {v.id for v in self.owned.values()} for a in current),
                     "external_actor_appeared")
        self.fail_if(any(a.id not in current_ids for a in self.owned.values()), "actor_disappeared")
        views, capture, audits, samples, raw = [], [], [], [], []
        for role, a in self.owned.items():
            self.active_actor = a.id
            tf, velocity = self.call(a.get_transform), self.call(a.get_velocity)
            control = self.call(a.get_control)
            accel = self.call(a.get_acceleration) if hasattr(a, "get_acceleration") else None
            # Persist the triggering frame even if a safety check below rejects it.
            def scalar(value):
                return value if not isinstance(value,float) or math.isfinite(value) else str(value)
            raw.append(dict(actor_id=a.id,role=role,frame=self.last_frame,sim_time_s=sim_time,
                       transform={k:scalar(v) for k,v in dict(x=tf.location.x,y=tf.location.y,
                                                            z=tf.location.z,yaw=tf.rotation.yaw).items()},
                       velocity={k:scalar(getattr(velocity,k)) for k in ('x','y','z')},
                       control={k:scalar(getattr(control,k,None)) for k in
                                ('throttle','brake','hand_brake','steer','reverse','gear','manual_gear_shift')}))
            save(self.directory/'last_observation.json',dict(phase=phase,rows=raw))
            at_light = self.call(a.is_at_traffic_light)
            light_state = str(self.call(a.get_traffic_light_state)) if hasattr(a, "get_traffic_light_state") else ""
            view = NS(id=a.id, attributes=a.attributes, type_id=a.type_id, bounding_box=a.bounding_box,
                      get_transform=lambda tf=tf: tf, get_velocity=lambda v=velocity: v,
                      is_at_traffic_light=lambda value=at_light: value)
            views.append(view)
            state = to_row(actor_state_from_actor(view, phase="capture", frame=self.last_frame,
                           sim_time_s=sim_time, logical_actor_id="role:idm_harness:"+role))
            for key in ("x", "y", "z", "vx", "vy", "vz", "yaw", "speed_mps"):
                self.fail_if(not math.isfinite(state[key]), "nonfinite_state", a.id)
            capture.append(state)
            vehicle, wp = sample_vehicle(view, self.map)
            self.fail_if(vehicle is None, "off_driving_lane", a.id)
            self.fail_if((wp.road_id,wp.section_id,wp.lane_id)!=(self.c.road,self.c.section,self.c.lane)
                         or wp.is_junction, "lane_or_junction_departure", a.id)
            projection = path_projection(vehicle.x, vehicle.y, self.path)
            heading = (tf.rotation.yaw-wp.transform.rotation.yaw+180)%360-180
            self.fail_if(projection is None, "route_projection_failed", a.id)
            lateral, progress = projection[0], projection[1]
            angle = math.radians(heading)
            half_length = abs(math.cos(angle))*a.bounding_box.extent.x + abs(math.sin(angle))*a.bounding_box.extent.y
            half_width = abs(math.cos(angle))*a.bounding_box.extent.y + abs(math.sin(angle))*a.bounding_box.extent.x
            front_clearance = self.route_length-progress-half_length
            self.fail_if(front_clearance < self.c.end_clearance, "route_endpoint_margin", a.id)
            self.fail_if(lateral+half_width >= wp.lane_width/2, "bbox_outside_lane", a.id)
            self.hold("route_deviation_"+role, lateral>self.c.lateral_limit or abs(heading)>self.c.heading_limit,
                      self.c.deviation_seconds, a.id)
            self.fail_if(type(at_light) is not bool or at_light, "signal_or_unknown_signal", a.id)
            required = ("throttle", "brake", "hand_brake", "steer", "reverse")
            self.fail_if(any(not hasattr(control, k) for k in required), "control_unknown", a.id)
            self.fail_if(type(control.hand_brake) is not bool or type(control.reverse) is not bool
                         or any(not isinstance(getattr(control,k),(int,float))
                                for k in ('throttle','brake','steer')), "control_unknown", a.id)
            self.fail_if(any(not math.isfinite(getattr(control,k)) for k in ("throttle","brake","steer")),
                         "nonfinite_control", a.id)
            self.fail_if(not 0<=control.throttle<=1 or not 0<=control.brake<=1 or not -1<=control.steer<=1,
                         "control_out_of_range", a.id)
            self.fail_if(control.hand_brake or control.reverse, "hand_brake_or_reverse", a.id)
            self.fail_if(state["speed_mps"] > (self.c.leader_speed_limit if role=="leader" else self.c.follower_speed_limit),
                         "speed_envelope", a.id)
            row = dict.fromkeys(AUDIT_FIELDS, "")
            row.update({k:v for k,v in state.items() if k in row})
            row.update(phase=phase, world_time_s=float(snap.timestamp.elapsed_seconds),
                tm_port=self.c.tm_port, seed_requested=self.c.seed, seed_applied=self.seed_applied,
                world_sync=True, tm_sync_command_succeeded=True, autopilot_call_result="call_returned",
                command_sequence=command["sequence"], command_based_on_frame=command["based_on"],
                leader_desired_speed_mps=command["speed"], follower_desired_speed_mps=self.c.follower_speed,
                control_read_at_frame=self.last_frame, route_progress_m=progress, route_lateral_m=lateral,
                heading_error_deg=heading, front_clearance_m=front_clearance,
                road_id=wp.road_id,section_id=wp.section_id,lane_id=wp.lane_id,
                is_junction=wp.is_junction, at_traffic_light=at_light, traffic_light_state=light_state)
            for key in ("throttle","brake","hand_brake","steer","reverse","gear","manual_gear_shift"):
                row[key] = getattr(control,key,"")
            if accel is not None:
                values = [getattr(accel,k,None) for k in ("x","y","z")]
                if all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
                    row.update(zip(("ax","ay","az"),values))
                    forward = tf.get_forward_vector()
                    row["a_long_mps2"] = sum(v*getattr(forward,k) for v,k in zip(values,("x","y","z")))
                else:
                    row["observation_error"] = "acceleration_unknown"
            if hasattr(self.tm, "get_next_action"):
                action = self.call(self.tm.get_next_action, a)
                row["tm_next_action"] = str(action[0]) if action else ""
            samples.append(vehicle)
            audits.append(row)
        leader = find_leader(samples[0], samples, self.path, 3.5*0.45)
        audits[0]['pair_gap_m'] = (audits[1]['route_progress_m']-audits[0]['route_progress_m']
                                  -samples[0].half_length_m-samples[1].half_length_m)
        if leader:
            audits[0].update(leader_id=leader.vehicle.actor_id, gap_m=leader.gap_m,
                             relative_speed_mps=samples[0].speed_mps-leader.vehicle.speed_mps)
        context = []
        if collector is not None:
            observed_world = NS(get_snapshot=lambda: self.call(self.world.get_snapshot), get_map=lambda: self.map)
            context = collector(observed_world, snap, views, capture)
            self.fail_if([(r['logical_actor_id'],r['carla_frame']) for r in capture] !=
                         [(r['logical_actor_id'],r['carla_frame']) for r in context], "capture_context_mismatch")
            for row, ctx in zip(audits, context):
                row.update({k:ctx[k] for k in ("leader_id","gap_m","signal_stop","lane_change","fallback_reason")})
        self.frame_unchanged()
        self.active_actor = None
        return capture, audits, context

    def audit_safety(self, phase, audits, context):
        f,l = audits
        self.fail_if(float(f["pair_gap_m"]) <= self.c.minimum_gap,
                     "contact_gap", f["source_actor_id"])
        if f["gap_m"] != "" and f["relative_speed_mps"] != "" and float(f["relative_speed_mps"])>0:
            self.fail_if(float(f["gap_m"])/float(f["relative_speed_mps"]) < self.c.minimum_ttc,
                         "contact_ttc", f["source_actor_id"])
        self.hold("leader_missing", f["leader_id"] != l["source_actor_id"],
                  self.c.leader_missing_seconds, f["source_actor_id"])
        if phase == "capture":
            self.hold("follower_stopped", f["speed_mps"]<self.c.minimum_speed,
                      self.c.stop_seconds, f["source_actor_id"])
            self.metrics['speed'].append(f['speed_mps'])
            if f['gap_m'] != "":
                self.metrics['gap'].append(float(f['gap_m']))
            self.metrics['moving_frames'] += f['speed_mps'] >= self.c.minimum_speed
            if self.t > 2.0 + 1e-9:
                valid = (context[0]['fallback_reason']=='' and context[0]['signal_stop'] is False
                         and context[0]['lane_change'] is False and context[0]['leader_id']==l['source_actor_id'])
                self.hold("invalid_context", not valid, self.c.invalid_seconds, f['source_actor_id'])
                self.metrics['eligible'] += 1
                self.metrics['valid'] += valid
                if self.t <= self.c.snapshot_at + 1e-9:
                    self.metrics['past_eligible'] += 1
                    self.metrics['past_valid'] += valid

    def capture(self):
        with ExitStack() as stack:
            warm = stack.enter_context(CsvWriter(self.directory/'warmup_control_audit.csv', AUDIT_FIELDS))
            audit = stack.enter_context(CsvWriter(self.directory/'control_audit.csv', AUDIT_FIELDS))
            writer = stack.enter_context(CsvWriter(self.directory/'capture_states.csv',actor_state_fieldnames()))
            ready = 0
            start = self.last_time
            for step in range(round(self.c.warmup_seconds/self.c.dt)):
                self.t = (step+1)*self.c.dt
                cmd = self.speed_command(self.c.warmup_speed)
                self.tick()
                _, rows, _ = self.observe("warmup", self.t, cmd)
                warm.writerows(rows); warm.flush()
                self.audit_safety("warmup",rows,[])
                self.fail_if(rows[1]['route_progress_m']-(self.c.follower_s-self.c.leader_s)>self.c.warmup_distance,
                             "warmup_distance",rows[1]['source_actor_id'])
                self.hold("warmup_not_moving", self.t>=3 and rows[0]['speed_mps']<0.1, 1.0,
                          rows[0]['source_actor_id'])
                good = (all(r['speed_mps']>=self.c.minimum_speed for r in rows)
                        and rows[0]['gap_m']!='' and 4<=float(rows[0]['gap_m'])<=12)
                ready = ready+self.c.dt if good else 0
                if ready+1e-9>=self.c.ready_seconds:
                    break
            else:
                raise EarlyStop("warmup_not_ready")
            self.ledger("capture_start", warmup_time_s=self.last_time-start)
            self.holds.clear()
            collector = stack.enter_context(FollowingContextCollector(self.directory,self.c.dt))
            snapshot_saved = False
            for step in range(round(self.c.duration/self.c.dt)):
                # The command precedes tick; TM's computed control is for a subsequent physics step.
                cmd = self.speed_command(self.c.speed_at(step*self.c.dt,self.stage))
                self.t = (step+1)*self.c.dt
                self.tick()
                capture, rows, context = self.observe("capture",self.t,cmd,collector)
                writer.writerows(capture); writer.flush()
                audit.writerows(rows); audit.flush()
                self.audit_safety("capture",rows,context)
                if not snapshot_saved and self.t+1e-9>=self.c.snapshot_at:
                    enriched = [dict(r,phase='snapshot',desired_time_headway_s=1.5,
                                     desired_speed_mps=self.c.follower_speed) for r in capture]
                    save_snapshot(self.directory/'snapshot_states.csv',enriched)
                    snapshot_saved = True
            m = self.metrics
            self.fail_if(m['moving_frames']*self.c.dt+1e-9 < self.c.minimum_moving_seconds,
                         "insufficient_moving_time")
            for prefix in ('','past_'):
                self.fail_if(not m[prefix+'eligible'] or m[prefix+'valid']/m[prefix+'eligible']<self.c.minimum_valid_ratio,
                             "insufficient_"+prefix+"valid_context")
            if self.stage=='stage2':
                self.fail_if(max(m['speed'])-min(m['speed'])<0.3 and
                             (not m['gap'] or max(m['gap'])-min(m['gap'])<1.0),
                             'insufficient_following_variation')
            self.result['metrics'] = m

    def cleanup(self):
        audit = dict(attempted=False, destroyed={}, unregistered=[], settings_restored=False,
                     remaining_owned=None, external_actors_preserved=None)
        self.result['cleanup'] = audit
        if self.world is None or not self.settings_touched:
            audit.update(required=False, complete=True)
            return
        try:
            self.gate.check()
            self.ownership.require_self()
            # Do not mutate external objects or tick after observed interleaving.
            self.frame_unchanged()
            audit['attempted'] = True
            live_before = {a.id for a in self.call(self.world.get_actors)}
            for actor in self.owned.values():
                if actor.id in self.registered and actor.id in live_before:
                    self.command("autopilot_unregister",actor.set_autopilot,False,self.c.tm_port,actor=actor.id)
                    audit['unregistered'].append(actor.id)
            for actor in self.owned.values():
                audit['destroyed'][str(actor.id)] = (self.command("destroy_owned",actor.destroy,actor=actor.id)
                                                    if actor.id in live_before else True)
            if self.tm_owned:
                self.command("tm_sync_release",self.tm.set_synchronous_mode,False)
            self.command("restore_world_settings",self.world.apply_settings,self.original)
            audit['settings_restored'] = settings_dict(self.call(self.world.get_settings))==settings_dict(self.original)
            live = {a.id for a in self.call(self.world.get_actors)}
            audit['remaining_owned'] = [a.id for a in self.owned.values() if a.id in live]
            audit['external_actors_preserved'] = self.initial_ids.issubset(live)
            audit['complete'] = (audit['settings_restored'] and not audit['remaining_owned']
                                 and audit['external_actors_preserved'] and all(audit['destroyed'].values()))
        except BaseException as exc:
            audit.update(complete=False,error=str(exc),rpc_suppressed=self.gate.poisoned)

    def run(self):
        if any((self.directory/name).exists() for name in
               ('config_used.json','capture_states.csv','worker_result.json','command_ledger.jsonl')):
            raise FileExistsError('harness requires a new empty run directory; evidence is never overwritten')
        save(self.directory/'config_used.json',dict(scenario=asdict(self.c),stage=self.stage,
                                                  config_sha256=self.c.fingerprint))
        try:
            self.prepare()
            self.capture()
            self.result.update(status="completed", exit_code=0)
        except EarlyStop as exc:
            self.result.update(status="early_stop",exit_code=2,reason=exc.reason,actor_id=exc.actor)
        except ServerUnavailable as exc:
            self.result.update(status="server_unavailable",exit_code=4,reason=str(exc),actor_id=self.active_actor)
        except BaseException as exc:
            self.result.update(status="exception",exit_code=1,reason=repr(exc))
        finally:
            self.cleanup()
            if not self.result['cleanup'].get('complete') and self.result['status']!='server_unavailable':
                self.result.update(prior_status=self.result['status'],status="cleanup_incomplete",exit_code=5)
            elif self.result['status']=='early_stop':
                self.result['status']='safe_early_stop'
            self.result.update(final_frame=self.last_frame,sim_time_s=self.t,wall_time_ns=time.time_ns(),
                               metrics=self.metrics)
            save(self.directory/'worker_result.json',self.result)
        return self.result
