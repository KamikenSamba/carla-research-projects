"""Passive bounded-V2 receipts. No CARLA import, tick or vehicle command API."""
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import threading
import time

from dt_gt_manifest import load_json, integer, num
from idm_snapshot import read_snapshot_rows


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)+'\n').encode('utf-8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def put(path, data):
    with Path(path).open('xb') as stream:
        stream.write(data)


def csv_bytes(rows):
    stream=io.StringIO(newline='')
    writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    return stream.getvalue().encode('utf-8')


def options(payload):
    """Metadata travels through the existing config; scenario stays untouched."""
    value=payload.get('ground_truth_observation')
    if value is None:return None
    if not isinstance(value,dict) or value.get('enabled') is not True:
        raise ValueError('GT observation requires explicit enabled=true metadata')
    if value.get('role') not in ('source','prediction'):
        raise ValueError('GT role required')
    if value.get('evidence_kind') not in ('synthetic','recorded_carla'):
        raise ValueError('GT evidence kind required')
    for key in ('run_id','protocol','parameter_policy'):
        if not isinstance(value.get(key),str) or not value[key]:raise ValueError('GT metadata missing: '+key)
    if not isinstance(value.get('inputs'),list) or value.get('inventory_complete') is not True:
        raise ValueError('GT explicit input inventory declaration required')
    return load_json(encoded(value))


def raw_rows(raw, actor_map):
    """Normalize existing raw state with its *measured* world clock, not loop t."""
    inverse={v:k for k,v in actor_map.items()};result=[];seen=set()
    for envelope in raw:
        if not envelope.get('frame_validated') or envelope.get('acquisition_status')!='complete':
            raise ValueError('GT incomplete raw acquisition')
        frame=integer(envelope['carla_frame']);world_time=num(envelope['world_time_s'])
        for row in envelope['rows']:
            actor=integer(row['actor_id'])
            if actor not in inverse:raise ValueError('GT raw actor not in origin map')
            key=(actor,frame)
            if key in seen:raise ValueError('GT duplicate raw state')
            seen.add(key)
            if row['frame']!=frame or row['control_read_at_frame']!=frame:
                raise ValueError('GT raw observation frame')
            velocity=row['velocity'];logical=inverse[actor]
            state=dict(phase=envelope['phase'],carla_frame=frame,sim_time_s=world_time,
                source_actor_id=actor,logical_actor_id=logical,role_name=logical,
                blueprint_id='vehicle.tesla.model3',autopilot_enabled=False,
                **row['transform'],**{'v'+k:velocity[k] for k in 'xyz'},
                speed_mps=math.sqrt(sum(velocity[k]**2 for k in 'xyz')))
            state=read_snapshot_rows([state])[0]
            result.append((state,row,envelope))
    return result


class GTObserver:
    """One run, append-only clock and exclusive receipts. Declarations are not attestations."""
    def __init__(self, directory, provenance):
        self.directory=Path(directory);self.provenance=load_json(encoded(provenance))
        self.errors=[];self.clocks={};self.physics_records={};self.sensors={}
        self.lock=threading.Lock();self.event_count=0;self.synthetic_fence=False
        self.external=[];self.origin=None
        put(self.directory/'gt_run_provenance.json',encoded(self.provenance))
        put(self.directory/'gt_clock.jsonl',b'')
        put(self.directory/'gt_event_frames.jsonl',b'')

    def check(self):
        if self.errors:raise OSError('GT observation incomplete: '+str(self.errors))

    def clock(self, snapshot):
        frame=integer(snapshot.frame);stamp=snapshot.timestamp
        now=float(stamp.elapsed_seconds)
        if not math.isfinite(now) or frame in self.clocks:raise ValueError('GT invalid/duplicate clock')
        row=dict(frame=frame,simulation_time_s=now,source='existing_WorldSnapshot')
        for key in ('delta_seconds','platform_timestamp'):
            value=getattr(stamp,key,None)
            row[key]=value if isinstance(value,(int,float)) and math.isfinite(value) else None
        with (self.directory/'gt_clock.jsonl').open('ab') as stream:
            stream.write(json.dumps(row,allow_nan=False).encode('utf-8')+b'\n');stream.flush()
        self.clocks[frame]=row

    def opendrive(self, xml, actors, frame=None, world_time=None):
        put(self.directory/'gt_opendrive.xodr',xml.encode('utf-8'))
        self.external=[dict(actor_id=a.id,type_id=a.type_id) for a in actors]
        self.opendrive_context=dict(frame=frame,simulation_time_s=world_time,
            basis='cached snapshot context; unknown when no validated snapshot exists')

    def physics(self, actor_id, physics, filename, frame=None, world_time=None):
        extras={};unknown=[]
        def value(obj,key,path,boolean=False):
            try:v=getattr(obj,key)
            except (AttributeError,RuntimeError,TypeError):v=None
            valid=isinstance(v,bool) if boolean else type(v) in (int,float) and math.isfinite(v)
            if not valid:unknown.append(path);return None
            return v
        extras['use_sweep_wheel_collision']=value(physics,'use_sweep_wheel_collision','use_sweep_wheel_collision',True)
        extras['wheels']=[{key:value(w,key,f'wheels/{i}/{key}') for key in
            ('lat_stiff_max_load','lat_stiff_value','long_stiff_value')} for i,w in enumerate(physics.wheels)]
        self.physics_records[str(actor_id)]=dict(legacy_file=filename,extra_static=extras,unknown_static=unknown,
            dynamic_state='unknown',acquisition='existing_prepare_get_physics_control_return',
            acquisition_frame=frame,acquisition_time_s=world_time,
            frame_basis='cached snapshot context; not an atomic physics snapshot')

    def sensor_started(self, sensor_id, owner, frame):
        self.sensors[str(sensor_id)]=dict(owner=owner,start_frame=frame,stop_returned=False)

    def sensor_stopped(self, sensor_id):
        if str(sensor_id) in self.sensors:self.sensors[str(sensor_id)]['stop_returned']=True

    def collision_metadata(self, event, owner):
        # Only the existing callback payload is inspected; never query a world/actor.
        with self.lock:
            self.event_count+=1
            stamp=getattr(event,'timestamp',None)
            if type(stamp) not in (float,int) or not math.isfinite(stamp):
                stamp=None;self.errors.append('collision_timestamp_unknown')
            if self.synthetic_fence:self.errors.append('callback_after_delivery_fence')
            return dict(actor_id=owner,timestamp=stamp,gt_event_id=f'callback-{self.event_count}',
                received_monotonic_ns=time.monotonic_ns())

    def callback_failure(self, error):
        with self.lock:self.errors.append('callback_io:'+repr(error))

    def near_event(self, index, frame, world_time):
        with (self.directory/'gt_event_frames.jsonl').open('ab') as stream:
            stream.write(json.dumps(dict(row_index=index,frame=frame,simulation_time_s=world_time),allow_nan=False).encode('utf-8')+b'\n')

    def environment(self, frame=None, world_time=None):
        refs={}
        for name in ('environment.json','local_route.json','gt_opendrive.xodr'):
            refs[name]=dict(path=name,sha256=digest((self.directory/name).read_bytes()))
        put(self.directory/'gt_environment_receipt.json',encoded(dict(files=refs,
            external_actor_inventory=sorted(self.external,key=lambda r:r['actor_id']),
            acquisition='prepare; not an origin-frame environment measurement',
            after_prepare_frame=frame,after_prepare_time_s=world_time,
            opendrive_context=getattr(self,'opendrive_context',None))))

    def freeze(self, harness, last):
        """Called once after fleet initialization, before its first step/command."""
        self.check()
        amap={k:a.id for k,a in harness.logical_actors.items()}
        frame=harness.last_frame
        if any(row['frame']!=frame or row['world_time_s']!=harness.last_time for row in last.values()):
            raise ValueError('GT origin state clock mismatch')
        clock=self.clocks.get(frame)
        if clock is None or clock['simulation_time_s']!=harness.last_time:raise ValueError('GT missing origin snapshot clock')
        raw_data=(self.directory/'calibration_raw.jsonl').read_bytes()
        raw=[load_json(line) for line in raw_data.decode('utf-8').splitlines()]
        if any(r['carla_frame']>frame for r in raw):raise ValueError('GT freeze contains future raw')
        normalized=raw_rows(raw,amap);rows=[state for state,_,_ in normalized]
        origin_rows={s['logical_actor_id']:(s,r) for s,r,_ in normalized if s['carla_frame']==frame}
        if set(origin_rows)!=set(amap):raise ValueError('GT incomplete origin actors')
        # Compare original Snapshot kinematics, retaining its original local clock/archive.
        original=(self.directory/'initial_snapshot.json').read_bytes()
        original_rows=read_snapshot_rows(load_json(original)['actors'])
        for old in original_rows:
            actual=origin_rows[old['logical_actor_id']][0]
            if old['source_actor_id']!=amap[old['logical_actor_id']] or old['carla_frame']!=frame:
                raise ValueError('GT Snapshot actor/frame mismatch')
            if any(old[k]!=actual[k] for k in ('x','y','z','roll','pitch','yaw','vx','vy','vz','speed_mps')):
                raise ValueError('GT Snapshot/raw kinematics mismatch')
        view=self.directory/'gt_input_view';view.mkdir(exist_ok=False)
        snapshots=[dict(s,phase='snapshot') for s,_ in origin_rows.values()]
        # Optional assigned traits are absent/null in this kinematic observation view.
        put(view/'snapshot.json',encoded(dict(actors=snapshots)))
        put(view/'prefix.csv',csv_bytes(rows))
        refs={name:dict(path='gt_input_view/'+name,sha256=digest((view/name).read_bytes()))
              for name in ('snapshot.json','prefix.csv')}
        internal={k:dict(speed_pid=dict(integral=c.speed_pid.integral,previous=c.speed_pid.previous),
            acceleration_filter=c.acceleration,actuator=dict(throttle=harness.actuators[k].throttle,
            brake=harness.actuators[k].brake),completeness='subset_unknown') for k,c in harness.fleet.controllers.items()}
        put(self.directory/'gt_internal_state.json',encoded(dict(frame=frame,simulation_time_s=harness.last_time,
            actors=internal,completeness='subset_unknown')))
        put(self.directory/'gt_physics_extras.json',encoded(self.physics_records))
        self.origin=dict(frame=frame,simulation_time_s=harness.last_time,actor_map=amap,
            before_first_command=True,view=refs,raw_prefix_sha256=digest(raw_data),raw_prefix_size=len(raw_data),
            legacy_snapshot_sha256=digest(original),prefix_start_frame=min(s['carla_frame'] for s in rows),
            initial_control={k:r['control'] for k,(_,r) in origin_rows.items()},
            bbox={k:r['bounding_box'] for k,(_,r) in origin_rows.items()})
        put(self.directory/'gt_origin.json',encoded(self.origin))

    def confirm_synthetic_delivery(self):
        """Mock-only fence; never a CARLA callback delivery guarantee."""
        if self.provenance['evidence_kind']!='synthetic':raise ValueError('real delivery fence unavailable')
        self.synthetic_fence=True

    def finish(self, result, final_frame):
        with self.lock:
            coverage=dict(sensors=self.sensors,final_frame=final_frame,event_count=self.event_count,
                errors=list(self.errors),delivery_fence='synthetic_explicit' if self.synthetic_fence else 'unknown',
                capture_status='complete' if self.synthetic_fence and not self.errors and self.sensors and
                all(s['stop_returned'] for s in self.sensors.values()) else 'unknown',
                cleanup=result.get('cleanup'),run_status=result.get('status'),
                limitations='sensor stop/empty events do not prove physical delivery completeness or negative labels')
        put(self.directory/'gt_capture_coverage.json',encoded(coverage))
        # Hash receipts only after all original writers/cleanup have returned.
        originals={'environment.json','local_route.json','initial_snapshot.json','calibration_raw.jsonl',
            'calibration_observations.csv','command_ledger.jsonl','prediction_states.csv','risk_events.csv',
            'v2_validation.csv','v2_control_diagnostics.jsonl','resolved_idm_parameters.json',
            'resolved_control_config.json','vehicle_physics.json','leader_physics.json','collision_events.jsonl',
            'pair_environment.json','pair_raw.jsonl','pair_observations.csv','v2_completion.json','config_used.json'}
        files=[p for p in self.directory.iterdir() if p.is_file() and
               (p.name in originals or p.name.startswith('gt_') and p.suffix in ('.json','.jsonl','.xodr'))
               and p.name!='gt_artifact_inventory.json']
        put(self.directory/'gt_artifact_inventory.json',encoded({p.name:dict(path=p.name,sha256=digest(p.read_bytes()))
                                                              for p in sorted(files)}))


class GTObservationMixin:
    """Bounded harness hooks; normal commands/gates are inherited unchanged."""
    def frame_unchanged(self):
        observer=getattr(self,'gt_observer',None)
        if observer and not getattr(self,'_gt_cleaning',False):observer.check()
        return super().frame_unchanged()

    def tick(self):
        snapshot=super().tick()
        observer=getattr(self,'gt_observer',None)
        if observer:observer.clock(snapshot)
        return snapshot

    def prepare(self):
        super().prepare()
        observer=getattr(self,'gt_observer',None)
        if observer:observer.environment(self.last_frame,self.last_time)

    def cleanup(self):
        self._gt_cleaning=True
        try:return super().cleanup()
        finally:
            observer=getattr(self,'gt_observer',None)
            if observer:
                try:observer.finish(self.result,self.last_frame)
                except BaseException as error:
                    self.result['gt_observation_error']=repr(error)
                    if self.result.get('status')=='completed':
                        self.result.update(status='exception',exit_code=1,reason='GT finalize failed')
