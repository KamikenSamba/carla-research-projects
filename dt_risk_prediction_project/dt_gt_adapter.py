"""Offline bounded-V2 adapter. Missing evidence never becomes recorded ground truth."""
from dataclasses import asdict
from pathlib import Path

from dt_gt_manifest import check, integer, num, sha, commit, load_json, finite_tree, validate
from dt_gt_observation import encoded, digest, csv_bytes, put, raw_rows
from idm_fidelity import csv_rows_from_bytes
from idm_snapshot import read_snapshot_rows
from idm_v2_reconstruction import physics_settings
from dt_risk_common import PredictionState
from Validate_DT_GroundTruth import guarded_output


class RunBytes:
    """Read each path once; every parser consumes the bytes whose hash was checked."""
    def __init__(self, root):
        self.root=Path(root).resolve();self.cache={};self.hashes={}
        self.inventory=self.json('gt_artifact_inventory.json',listed=False)

    def read(self, name, listed=True, expected=None):
        p=Path(name)
        check(not p.is_absolute(),'ADAPTER_ABSOLUTE_ARTIFACT')
        p=(self.root/p).resolve();check(p.is_relative_to(self.root),'ADAPTER_PATH_ESCAPE')
        if p not in self.cache:
            check(p.stat().st_size<=16*1024*1024,'ADAPTER_FILE_TOO_LARGE')
            data=p.read_bytes();check(len(data)<=16*1024*1024,'ADAPTER_FILE_TOO_LARGE')
            self.cache[p]=data;self.hashes[str(p)]=digest(data)
        data=self.cache[p]
        if listed:
            ref=self.inventory[name];check(ref['path']==name,'ADAPTER_INVENTORY_PATH')
            check(digest(data)==sha(ref['sha256']),'ADAPTER_HASH_MISMATCH:'+name)
        if expected is not None:check(digest(data)==sha(expected),'ADAPTER_PARENT_HASH:'+name)
        return data

    def json(self, name, **kwargs):
        result=load_json(self.read(name,**kwargs).decode('utf-8-sig'));finite_tree(result);return result

    def lines(self, name):
        rows=[load_json(line) for line in self.read(name).decode('utf-8').splitlines()]
        finite_tree(rows);return rows

    def csv(self, name):return csv_rows_from_bytes(self.read(name))


def require_seed(provenance):
    seed=provenance.get('seed',{})
    check(seed.get('status')=='applied' and bool(seed.get('components')) and bool(seed.get('evidence')),
          'SEED_APPLICATION_UNVERIFIED')
    return integer(seed['value'])


def static_physics(run, origin):
    supplemental=run.json('gt_physics_extras.json');result={}
    scalar_fields={'mass','drag_coefficient','moi','max_rpm','gear_switch_time','clutch_strength','final_ratio',
        'damping_rate_full_throttle','damping_rate_zero_throttle_clutch_engaged',
        'damping_rate_zero_throttle_clutch_disengaged'}
    wheel_fields={'tire_friction','radius','max_brake_torque','max_handbrake_torque','damping_rate',
        'max_steer_angle','lat_stiff_max_load','lat_stiff_value','long_stiff_value'}
    for logical,actor in origin['actor_map'].items():
        receipt=supplemental[str(actor)];check(receipt['unknown_static']==[],'STATIC_PHYSICS_UNKNOWN')
        legacy=run.json(receipt['legacy_file']);record=physics_settings(legacy)
        extras=receipt['extra_static'];record['use_sweep_wheel_collision']=extras['use_sweep_wheel_collision']
        check(type(record['use_sweep_wheel_collision']) is bool and type(record['use_gear_autobox']) is bool,'PHYSICS_BOOLEAN_UNKNOWN')
        for key in scalar_fields:num(record[key])
        check(num(record['mass'])>0 and len(record['wheels'])==len(extras['wheels'])==4,'STATIC_PHYSICS_WHEELS')
        for wheel,extra in zip(record['wheels'],extras['wheels']):
            wheel.update(extra)
            for key in wheel_fields:num(wheel[key])
        for axis in 'xyz':num(record['center_of_mass'][axis])
        for key in ('torque_curve','steering_curve'):
            check(bool(record[key]),'PHYSICS_CURVE_UNKNOWN')
            for point in record[key]:num(point['x']);num(point['y'])
        check(bool(record['forward_gears']),'PHYSICS_GEARS_UNKNOWN')
        for gear in record['forward_gears']:
            for key in ('ratio','down_ratio','up_ratio'):num(gear[key])
        result[logical]=record
    return dict(measurement_status='complete_static',unknown_fields=[],actors=result)


def canonical_environment(run):
    receipt=run.json('gt_environment_receipt.json')
    for ref in receipt['files'].values():run.read(ref['path'],expected=ref['sha256'])
    legacy=run.json('environment.json')
    return dict(client_version=legacy['versions'][0],server_version=legacy['versions'][1],map=legacy['map'],
        weather=legacy['weather'],world_settings=legacy['applied_settings'],
        opendrive_sha256=digest(run.read('gt_opendrive.xodr')),route_sha256=digest(run.read('local_route.json')),
        external_actor_inventory=receipt['external_actor_inventory'],known_exogenous_information=[])


def events(run, origin, end, clock, result):
    coverage=run.json('gt_capture_coverage.json')
    check(coverage['capture_status']=='complete' and not coverage['errors'],'EVENT_COVERAGE_UNPROVEN')
    check(coverage['run_status']==result['status'] and coverage['cleanup']==result['cleanup'] and
        coverage['final_frame']==result['final_frame'],'COVERAGE_COMPLETION_MISMATCH')
    # Only a mock currently supplies a delivery fence; real sensors are not certified.
    check(coverage['delivery_fence']=='synthetic_explicit' and
          run.json('gt_run_provenance.json')['evidence_kind']=='synthetic','REAL_EVENT_DELIVERY_FENCE_UNAVAILABLE')
    amap=origin['actor_map'];inverse={actor:logical for logical,actor in amap.items()}
    check({s['owner'] for s in coverage['sensors'].values()}==set(inverse),'EVENT_SENSOR_COHORT')
    check(all(s['start_frame'] is not None and s['start_frame']<=origin['frame'] and s['stop_returned']
              for s in coverage['sensors'].values()) and coverage['final_frame']>=end,'EVENT_SENSOR_INTERVAL')
    raw=run.lines('collision_events.jsonl') if 'collision_events.jsonl' in run.inventory else []
    check(len(raw)==coverage['event_count'],'EVENT_RAW_COUNT')
    output=[];seen=set()
    for record in raw:
        # Preserve external collisions as incompatibility; never invent a cohort ID.
        check(record['actor_id'] in inverse and record['other_actor_id'] in inverse,'EXTERNAL_COLLISION_PAIR')
        eid=record['gt_event_id'];check(eid not in seen,'DUPLICATE_CALLBACK');seen.add(eid)
        frame=integer(record['frame']);stamp=num(record['timestamp'])
        if origin['frame']<=frame<=end:
            check(frame in clock and abs(stamp-clock[frame])<=1e-6,'COLLISION_CLOCK')
            output.append(dict(event_id=eid,kind='collision',frame=frame,simulation_time_s=stamp,
                pair=[inverse[record['actor_id']],inverse[record['other_actor_id']]],
                provenance='existing_collision_callback',raw_payload=record))
    check(result['collision_events']==coverage['event_count']==0,'BOUNDED_COMPLETION_COLLISION_COUNT')
    near=run.csv('risk_events.csv');receipts=run.lines('gt_event_frames.jsonl')
    check(len(near)==len(receipts),'NEAR_EVENT_FRAME_COVERAGE')
    for i,(record,receipt) in enumerate(zip(near,receipts)):
        check(receipt['row_index']==i,'NEAR_EVENT_INDEX')
        frame=integer(receipt['frame']);stamp=num(receipt['simulation_time_s'])
        check(origin['frame']<=frame<=end and frame in clock and abs(stamp-clock[frame])<=1e-6,'NEAR_EVENT_CLOCK')
        check(abs(num(record['prediction_time_s'])-(stamp-origin['simulation_time_s']))<=1e-6,'NEAR_EVENT_LOCAL_TIME')
        pair=[record['logical_actor_a'],record['logical_actor_b']]
        check(set(pair)<=set(amap) and len(set(pair))==2,'NEAR_EVENT_PAIR')
        check(record['risk_type']=='near_miss','UNSUPPORTED_RISK_EVENT')
        check([integer(record['carla_actor_a']),integer(record['carla_actor_b'])]==[amap[k] for k in pair],'NEAR_EVENT_ACTORS')
        output.append(dict(event_id='near-'+str(i),kind='near_miss',frame=frame,simulation_time_s=stamp,
            pair=pair,provenance='existing_XY_center_detector',raw_payload=record))
    return dict(capture_status='complete',frame_start=origin['frame'],frame_end=end,
        covered_logical_ids=list(amap),raw_definitions=dict(collision='existing collision sensor raw callback',
        near_miss='existing XY center distance <4m, closing speed >=1m/s, cooldown 1s; not bumper gap/TTC'),
        raw_events=output,label_status='missing',label_protocol=None,labels=[])


def input_policy(source, prediction, source_origin):
    """Check declared inputs, never claim a sandbox or complete runtime read audit."""
    parent=prediction['parent']
    check(parent['run_id']==source['run_id'] and parent['origin']=={
        'frame':source_origin['frame'],'simulation_time_s':source_origin['simulation_time_s']},'PARENT_SOURCE_ORIGIN')
    for field,name in [('snapshot_sha256','snapshot.json'),('prefix_sha256','prefix.csv')]:
        check(parent[field]==source_origin['view'][name]['sha256'],'PARENT_FROZEN_VIEW_HASH')
    verified_inputs={}
    for provenance in (source,prediction):
        check(provenance.get('inventory_complete') is True,'INPUT_INVENTORY_UNDECLARED')
        for entry in provenance['inputs']:
            check(entry['role'] in ('model_observation','model_parameters','safety_admission','evaluation_only','configuration'),'INPUT_ROLE')
            if entry['role']=='model_observation':
                check(integer(entry['maximum_source_frame'])<=source_origin['frame'] and
                    num(entry['maximum_source_timestamp'])<=source_origin['simulation_time_s']+1e-6,'FUTURE_MODEL_INPUT')
                check(entry['sha256'] in {ref['sha256'] for ref in source_origin['view'].values()},'MODEL_INPUT_NOT_FROZEN_VIEW')
            if entry.get('contains_source_traits'):
                check(provenance['parameter_policy'] in ('oracle','reconstruction_known') and entry['role']=='model_parameters','ORACLE_INFORMATION_LEAK')
            if entry.get('contains_source_future'):check(entry['role'] in ('evaluation_only','safety_admission'),'FUTURE_MODEL_INPUT')
            # Hash inventory is checked by value against the actual declared file.
            path=Path(entry['path']).resolve()
            if path not in verified_inputs:
                check(path.stat().st_size<=16*1024*1024,'DECLARED_INPUT_TOO_LARGE')
                data=path.read_bytes();check(len(data)<=16*1024*1024,'DECLARED_INPUT_TOO_LARGE')
                verified_inputs[path]=data
            check(digest(verified_inputs[path])==sha(entry['sha256']),'DECLARED_INPUT_HASH')
    required={ref['sha256'] for ref in source_origin['view'].values()}
    supplied={entry['sha256'] for entry in prediction['inputs'] if entry['role']=='model_observation'}
    check(supplied==required,'PREDICTION_FROZEN_VIEW_INPUTS_MISSING')
    return {str(path):digest(data) for path,data in verified_inputs.items()}


def transform_run(run):
    provenance=run.json('gt_run_provenance.json');origin=run.json('gt_origin.json')
    seed=require_seed(provenance);commit(provenance['code_commit'])
    check(provenance.get('source_dirty') is False,'CODE_PROVENANCE_DIRTY_OR_UNKNOWN')
    result=run.json('worker_result.json',listed=False)
    check(result['status']=='completed' and result['cleanup']['complete'] and
        result.get('route_audit') is None and result.get('physical_bounds') is None,'RUN_INCOMPLETE')
    check(origin['before_first_command'] is True and len(origin['actor_map'])>=2,'ORIGIN_COHORT_OR_ORDER')
    amap=origin['actor_map'];f0=integer(origin['frame']);t0=num(origin['simulation_time_s'])
    for name,ref in origin['view'].items():run.read(ref['path'],listed=False,expected=ref['sha256'])
    snapshot=run.json(origin['view']['snapshot.json']['path'],listed=False)
    check(all(r.get('desired_time_headway_s') is None and r.get('desired_speed_mps') is None
              for r in snapshot['actors']),'ORACLE_TRAITS_IN_KINEMATIC_VIEW')
    raw_data=run.read('calibration_raw.jsonl');prefix_size=integer(origin['raw_prefix_size'])
    check(digest(raw_data[:prefix_size])==origin['raw_prefix_sha256'],'FROZEN_RAW_PREFIX_CHANGED')
    normalized=raw_rows(run.lines('calibration_raw.jsonl'),amap)
    index={(r['logical_actor_id'],r['carla_frame']):(r,raw,env) for r,raw,env in normalized}
    prefix=[r for r,_,_ in normalized if r['carla_frame']<=f0]
    frozen=csv_rows_from_bytes(run.read(origin['view']['prefix.csv']['path'],listed=False))
    check([read_snapshot_rows([row])[0] for row in frozen]==prefix,'FROZEN_PREFIX_RAW_MISMATCH')
    clock={}
    for row in run.lines('gt_clock.jsonl'):
        frame=integer(row['frame']);check(frame not in clock,'CLOCK_DUPLICATE');clock[frame]=num(row['simulation_time_s'])
    diagnostics=run.lines('v2_control_diagnostics.jsonl')
    check(len(diagnostics)==120*len(amap),'BOUNDED_IDM_WINDOW_INCOMPLETE')
    seen=set();future=[];end=f0+120
    ledger=run.lines('command_ledger.jsonl');requests={r['sequence']:r for r in ledger if r['operation']=='manual_control' and r.get('outcome')=='requested'}
    returned={r['request_sequence'] for r in ledger if r['operation']=='manual_control' and r.get('outcome')=='call_returned'}
    check(len(requests)==sum(r['operation']=='manual_control' and r.get('outcome')=='requested' for r in ledger),'LEDGER_DUPLICATE_SEQUENCE')
    for state,raw,envelope in normalized:
        command=envelope['command'];seq=command['sequence'];requested=requests[seq]
        check(seq in returned and command['based_on_frame']==requested['based_on_frame']==state['carla_frame']-1 and
            requested['actor_id']==state['source_actor_id'] and requested['control']==command['control'],'LEDGER_COMMAND')
        check(state['carla_frame'] in clock and abs(state['sim_time_s']-clock[state['carla_frame']])<=1e-6 and
            abs(num(requested['sim_time_s'])+.05-num(envelope['sim_time_s']))<=1e-6,'RAW_LEDGER_CLOCK')
        for field in ('throttle','brake','steer'):
            check(abs(num(raw['control'][field])-num(command['control'][field]))<=1e-5,'RAW_CONTROL_ECHO')
    old_future=run.csv('prediction_states.csv');old_index={(r['logical_actor_id'],integer(r['prediction_frame'])):r for r in old_future}
    check(len(old_index)==len(old_future)==len(diagnostics),'FUTURE_CSV_KEYS')
    for d in diagnostics:
        logical=d['logical_actor_id'];frame=integer(d['response_frame']);key=(logical,frame)
        check(key not in seen and logical in amap and d['actor_id']==amap[logical],'DIAGNOSTIC_ACTOR_DUPLICATE')
        seen.add(key)
        check(d['state_frame']==d['command_frame']==frame-1 and f0<frame<=end,'COMMAND_RESPONSE_FRAME')
        state,raw,envelope=index[key];_,_,before=index[(logical,frame-1)]
        check(frame in clock and frame-1 in clock and abs(state['sim_time_s']-clock[frame])<=1e-6 and
            abs(before['world_time_s']-clock[frame-1])<=1e-6,'RAW_CLOCK')
        check(abs(num(d['response_time_s'])-num(envelope['sim_time_s']))<=1e-6 and
            abs(num(d['state_time_s'])-num(before['sim_time_s']))<=1e-6,'DIAGNOSTIC_LOCAL_TIME')
        command=envelope['command'];seq=command['sequence'];requested=requests[seq]
        check(seq in returned and command['based_on_frame']==requested['based_on_frame']==frame-1 and
            requested['actor_id']==amap[logical] and requested['control']==command['control'],'LEDGER_COMMAND')
        for control in ('throttle','brake','steer'):
            check(abs(num(raw['control'][control])-num(command['control'][control]))<=1e-5 and
                num(d['observed_control'][control])==num(raw['control'][control]),'OBSERVED_CONTROL_MISMATCH')
        old=old_index[key]
        for field in ('x','y','z','yaw','vx','vy','vz','speed_mps'):check(num(old[field])==state[field],'CSV_RAW_STATE')
        check(integer(old['carla_actor_id'])==amap[logical] and abs(num(old['prediction_time_s'])-(clock[frame]-t0))<=1e-6,'CSV_RAW_TIME_ID')
        future.append(asdict(PredictionState(frame,clock[frame]-t0,logical,amap[logical],
            **{k:state[k] for k in ('x','y','z','yaw','vx','vy','vz','speed_mps')},autopilot_enabled=False)))
    check(seen=={(logical,frame) for logical in amap for frame in range(f0+1,end+1)},'FUTURE_COHORT_FRAMES')
    check(all(frame in clock for frame in range(f0,end+1)) and abs(clock[f0]-t0)<=1e-6,'ORIGIN_CLOCK')
    params=run.json('resolved_idm_parameters.json');config=run.json('resolved_control_config.json')
    control={logical:dict(longitudinal='existing_V2_IDM_bounded',steering='existing_V2_route_PID',
        parameters=dict(idm=params[logical],lower_controller=config,T_s=params[logical]['desired_time_headway_s']),
        state_capture='unknown') for logical in amap}
    artifacts=dict(snapshot=snapshot,prefix=prefix,future=future,
        clock=[dict(frame=f,simulation_time_s=clock[f]) for f in range(f0,end+1)],
        environment=canonical_environment(run),physics=static_physics(run,origin),bbox=origin['bbox'],
        initial_control=dict(frame=f0,simulation_time_s=t0,actors=origin['initial_control']),
        events=events(run,origin,end,clock,result))
    block=dict(run_id=provenance['run_id'],code_commit=provenance['code_commit'],seed=seed,dt_s=.05,
        horizon_steps=120,origin=dict(frame=f0,simulation_time_s=t0),actor_map=amap,control=control)
    return provenance,origin,block,artifacts


def adapt(source_dir, prediction_dir, output):
    """Exclusive external bundle or INPUT_INCOMPLETE report; never repair inputs."""
    output=Path(output);roots=[Path(source_dir),Path(prediction_dir)]
    for root in roots:guarded_output(root/'gt_origin.json',output)
    output.mkdir(exist_ok=False)
    report=dict(status='INPUT_INCOMPLETE',errors=[],input_hashes={},manifest=None,
        route_audit=None,physical_bounds=None,experiment_status='BLOCKED',
        independent_forecast_accuracy_enabled=False,risk_detection_performance_enabled=False,runtime_execution_enabled=False)
    readers=[]
    try:
        for root in roots:readers.append(RunBytes(root))
        converted=[transform_run(run) for run in readers]
        (s,so,sb,sa),(p,po,pb,pa)=converted
        check(s['role']=='source' and p['role']=='prediction' and s['run_id']!=p['run_id'],'RUN_PARENT_ROLES')
        check(s['evidence_kind']==p['evidence_kind'] and s['evidence_kind'] in ('synthetic','recorded_carla'),'EVIDENCE_KIND')
        report['declared_input_hashes']=input_policy(s,p,so)
        # This adapter connects the existing replay protocol, not a new predictor.
        check(p['parameter_policy']=='reconstruction_known','ONLY_EXPLICIT_A_RECONSTRUCTION_SUPPORTED')
        def artifact(name,data,csv=False):
            path=output/name;path.parent.mkdir(parents=True,exist_ok=True)
            content=csv_bytes(data) if csv else encoded(data);put(path,content)
            return dict(path=name,sha256=digest(content))
        for label,block,artifacts in [('source',sb,sa),('prediction',pb,pa)]:
            block['artifacts']={k:artifact(label+'/'+k+('.csv' if k in ('prefix','future','clock') else '.json'),
                value,k in ('prefix','future','clock')) for k,value in artifacts.items() if label=='source' or k!='prefix'}
        sb.update(protocol=s['protocol'],prefix_start_frame=so['prefix_start_frame'])
        pb.update(method=p['protocol'],parent_source_id=sb['run_id'],reference_origin=sb['origin'],
            parent_snapshot_sha256=sb['artifacts']['snapshot']['sha256'],parent_prefix_sha256=sb['artifacts']['prefix']['sha256'],
            initialization=dict(stage='after_initialization_before_first_prediction_tick',
                first_command_based_on_frame=po['frame'],first_response_frame=po['frame']+1,
                ticks_after_initial_measurement=0,internal_physics_state='unknown'),
            inputs=dict(prefix=sb['artifacts']['prefix'],maximum_source_frame=so['frame'],
                maximum_source_timestamp=so['simulation_time_s'],inventory_complete=True),
            model_data=p['model_data'])
        source_manifest=artifact('source_data_manifest.json',dict(source=sb,producer=s,raw_hashes=readers[0].hashes))
        check(s['split']=='test','SOURCE_SPLIT_REQUIRED')
        registry=artifact('registry.json',dict(sources=[dict(source_run_id=sb['run_id'],split=s['split'],
            protocol=sb['protocol'],code_commit=sb['code_commit'],data_manifest_sha256=source_manifest['sha256'])]))
        manifest=dict(schema_version='dt_source_future_contract_v1',evidence_kind=s['evidence_kind'],source=sb,
            prediction=pb,registry=registry,evaluation=dict(purpose='A_reconstruction',requested_metrics=['trajectory'],
            initial_policy=dict(status='PROPOSED_UNAPPROVED',thresholds=None,approval_reference=None),information_policy='causal_prefix_only'),
            safety=dict(experiment_status='BLOCKED',route_audit=None,physical_bounds=None))
        artifact('manifest.json',manifest)
        assessment=validate(output/'manifest.json');artifact('validation.json',assessment)
        report.update(status=assessment['status'],errors=assessment['errors'],manifest='manifest.json')
    except (OSError,ValueError,KeyError,TypeError,AttributeError,IndexError,OverflowError,RecursionError) as error:
        report['errors'].append(str(error))
    finally:
        for run in readers:report['input_hashes'].update(run.hashes)
        put(output/'adapter_report.json',encoded(report))
    return report
