"""Offline Source/future contract checks; never scores or authorizes execution."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import re

CONTRACT=json.loads(Path(__file__).with_name('dt_source_future_contract_v1.json').read_text(encoding='utf-8'))

def fail(message):raise ValueError(message)
def check(value,message):
    if not value:fail(message)
def keys(obj,kind):check(isinstance(obj,dict) and set(obj)==set(CONTRACT['required'][kind]),'FIELDS:'+kind)
def num(value):
    check(not isinstance(value,bool),'BOOLEAN_MEASUREMENT')
    value=float(value);check(math.isfinite(value),'NONFINITE');return value
def integer(value):
    n=num(value);check(n.is_integer() and n>=0,'INTEGER_REQUIRED');return int(n)
def text(value):check(isinstance(value,str) and bool(value.strip()),'TEXT_REQUIRED');return value
def finite_tree(obj):
    if isinstance(obj,float):num(obj)
    elif isinstance(obj,dict):
        for v in obj.values():finite_tree(v)
    elif isinstance(obj,list):
        for v in obj:finite_tree(v)
def sha(value):check(isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None,'SHA256_REQUIRED');return value
def commit(value):check(isinstance(value,str) and re.fullmatch('[0-9a-f]{40}|[0-9a-f]{64}',value) is not None,'COMMIT_REQUIRED');return value
def unique_pairs(pairs):
    obj={}
    for key,value in pairs:
        check(key not in obj,'DUPLICATE_JSON_KEY:'+key);obj[key]=value
    return obj
def load_json(data):
    return json.loads(data,parse_constant=lambda x:fail('NONFINITE_JSON:'+x),object_pairs_hook=unique_pairs)

def value_differences(source,prediction,path=''):
    """Diagnostic values only: no tolerance or equality gate is applied."""
    result=[]
    if isinstance(source,dict) and isinstance(prediction,dict):
        for key in sorted(set(source)|set(prediction)):
            result.extend(value_differences(source.get(key),prediction.get(key),path+'/'+key))
    elif isinstance(source,list) and isinstance(prediction,list):
        for i in range(max(len(source),len(prediction))):
            result.extend(value_differences(source[i] if i<len(source) else None,
                prediction[i] if i<len(prediction) else None,path+'/'+str(i)))
    elif source!=prediction:
        delta=None
        if all(isinstance(v,(int,float)) and not isinstance(v,bool) for v in (source,prediction)):
            candidate=prediction-source
            if math.isfinite(candidate):delta=candidate
        result.append(dict(path=path,source=source,prediction=prediction,delta=delta))
    return result

def validate(manifest_path):
    """Read-only; every successful structure result remains pending approval."""
    result=dict(status='INVALID',errors=[],warnings=[],initial_residuals={},input_hashes={},
        initial_state_assessment='NOT_EVALUATED_PENDING_POLICY',
        independent_forecast_accuracy_enabled=False,risk_detection_performance_enabled=False,
        runtime_execution_enabled=False,route_audit=None,physical_bounds=None)
    try:
        root=Path(manifest_path).resolve().parent
        raw=Path(manifest_path).read_bytes();result['manifest_sha256']=hashlib.sha256(raw).hexdigest()
        check(len(raw)<=1024*1024,'MANIFEST_TOO_LARGE');m=load_json(raw.decode('utf-8-sig'))
        keys(m,'manifest');check(m['schema_version']==CONTRACT['contract_id'],'SCHEMA_VERSION')
        check(m['evidence_kind'] in ('synthetic','recorded_carla'),'EVIDENCE_KIND')
        keys(m['safety'],'safety');check(m['safety']==dict(experiment_status='BLOCKED',route_audit=None,physical_bounds=None),'SAFETY_SCOPE')
        keys(m['evaluation'],'evaluation');ev=m['evaluation'];check(ev['purpose'] in CONTRACT['purposes'],'PURPOSE')
        check(ev['requested_metrics'] and set(ev['requested_metrics'])<=set(('trajectory','risk_detection')),'METRICS')
        check(ev['information_policy']=='causal_prefix_only','INFORMATION_POLICY')
        policy=ev['initial_policy'];keys(policy,'initial_policy')
        check(policy==dict(status='PROPOSED_UNAPPROVED',thresholds=None,approval_reference=None),'UNSUPPORTED_POLICY_APPROVAL')
        # Reuse existing Snapshot/CSV/frame validators, not evaluator accuracy paths.
        from idm_snapshot import read_snapshot
        from idm_fidelity import csv_rows
        from idm_v2_reconstruction import physics_settings
        from dt_trajectory_evaluation import _index
        def asset(ref):
            keys(ref,'artifact');sha(ref['sha256']);p=Path(text(ref['path']))
            check(not p.is_absolute(),'ARTIFACT_ABSOLUTE_PATH');p=(root/p).resolve()
            check(p.is_relative_to(root) and p.is_file(),'ARTIFACT_PATH')
            check(p.stat().st_size<=16*1024*1024,'ARTIFACT_TOO_LARGE')
            b=p.read_bytes();h=hashlib.sha256(b).hexdigest();result['input_hashes'][str(p)]=h
            check(h==ref['sha256'],'HASH_MISMATCH:'+ref['path']);return p
        def document(ref):
            obj=load_json(asset(ref).read_text(encoding='utf-8-sig'));finite_tree(obj);return obj
        source=m['source'];pred=m['prediction'];keys(source,'source');keys(pred,'prediction')
        text(source['run_id']);text(source['protocol']);commit(source['code_commit']);integer(source['seed'])
        text(pred['run_id']);text(pred['method']);commit(pred['code_commit']);integer(pred['seed'])
        check(source['run_id']!=pred['run_id'],'RUN_ID_COLLISION')
        check(pred['parent_source_id']==source['run_id'],'PARENT_SOURCE')
        check(pred['parent_snapshot_sha256']==source['artifacts']['snapshot']['sha256'],'PARENT_SNAPSHOT_HASH')
        check(pred['parent_prefix_sha256']==source['artifacts']['prefix']['sha256'],'PARENT_PREFIX_HASH')
        check(pred['reference_origin']==source['origin'],'REFERENCE_T0')
        dt=num(source['dt_s']);check(dt>0 and num(pred['dt_s'])==dt,'DT_MISMATCH')
        n=integer(source['horizon_steps']);check(0<n<=100000 and integer(pred['horizon_steps'])==n,'HORIZON')
        ids=set(source['actor_map']);check(len(ids)>=2 and ids==set(pred['actor_map']),'LOGICAL_ID_SET')
        check(n*len(ids)<=100000,'PROTOTYPE_ROW_LIMIT')
        for side in (source,pred):
            check(all(isinstance(x,str) and x for x in side['actor_map']),'LOGICAL_ID')
            actor_ids=[integer(v) for v in side['actor_map'].values()]
            check(min(actor_ids)>0 and len(set(actor_ids))==len(ids),'ACTOR_MAPPING')
        # Global Source and Prediction clock values may differ; reference_t0 may not.
        for side in (source,pred):keys(side['origin'],'origin');integer(side['origin']['frame']);num(side['origin']['simulation_time_s'])
        registry=document(m['registry']);check(set(registry)=={'sources'},'REGISTRY_FIELDS')
        registry_ids={};registry_hashes=set()
        for entry in registry['sources']:
            check(set(entry)=={'source_run_id','split','protocol','code_commit','data_manifest_sha256'},'REGISTRY_ENTRY')
            sid=text(entry['source_run_id']);check(sid not in registry_ids,'SOURCE_SPLIT_DUPLICATE')
            check(entry['split'] in ('train','tune','test'),'SPLIT');commit(entry['code_commit']);text(entry['protocol']);sha(entry['data_manifest_sha256'])
            check(entry['data_manifest_sha256'] not in registry_hashes,'SOURCE_HASH_ALIAS');registry_hashes.add(entry['data_manifest_sha256'])
            registry_ids[sid]=entry
        check(source['run_id'] in registry_ids and registry_ids[source['run_id']]['split']=='test','EVALUATION_SOURCE_SPLIT')
        check(registry_ids[source['run_id']]['protocol']==source['protocol'] and registry_ids[source['run_id']]['code_commit']==source['code_commit'],'SOURCE_REGISTRY_PROVENANCE')
        model=pred['model_data'];keys(model,'model_data')
        check(model['approach'] in ('prescribed','learned'),'MODEL_APPROACH')
        for field,split in [('training_source_ids','train'),('tuning_source_ids','tune'),('evaluation_source_ids','test')]:
            group=model[field];check(isinstance(group,list) and len(group)==len(set(group)),'MODEL_SOURCE_IDS')
            check(all(x in registry_ids and registry_ids[x]['split']==split for x in group),'SOURCE_SPLIT_LEAKAGE')
        check(model['evaluation_source_ids']==[source['run_id']],'MODEL_EVALUATION_PARENT')
        check(model['approach']!='learned' or bool(model['training_source_ids']),'LEARNED_MODEL_TRAINING_PROVENANCE')
        init=pred['initialization'];keys(init,'initialization');g0=integer(pred['origin']['frame'])
        check(init['stage']=='after_initialization_before_first_prediction_tick' and integer(init['ticks_after_initial_measurement'])==0,'INITIALIZATION_STAGE')
        check(integer(init['first_command_based_on_frame'])==g0 and integer(init['first_response_frame'])==g0+1,'FIRST_COMMAND_RESPONSE')
        check(init['internal_physics_state'] in ('unknown','recorded'),'INTERNAL_PHYSICS_STATUS')
        if init['internal_physics_state']=='unknown':result['warnings'].append('INTERNAL_PHYSICS_NOT_RESTORED')
        state_sets=[];environments=[];physics=[];boxes=[];initial_controls=[];event_contracts=[]
        for label,side in [('source',source),('prediction',pred)]:
            keys(side['artifacts'],label+'_artifacts');ar=side['artifacts'];f0=integer(side['origin']['frame']);t0=num(side['origin']['simulation_time_s'])
            sp=asset(ar['snapshot']);rawsnap=load_json(sp.read_text(encoding='utf-8-sig'))
            for r in rawsnap['actors']:integer(r['carla_frame']);integer(r['source_actor_id'])
            snap={r['logical_actor_id']:r for r in read_snapshot(sp)}
            check(set(snap)==ids,'SNAPSHOT_LOGICAL_IDS')
            for logical,r in snap.items():
                check(r['source_actor_id']==side['actor_map'][logical] and r['carla_frame']==f0,'SNAPSHOT_ID_FRAME')
                check(abs(r['sim_time_s']-t0)<=1e-6,'SNAPSHOT_T0')
                check(abs(math.hypot(r['vx'],r['vy'],r['vz'])-r['speed_mps'])<=1e-6,'SNAPSHOT_SPEED_DEFINITION')
            state_sets.append(snap)
            future=csv_rows(asset(ar['future']));index,actors,_=_index(future,dt,f0)
            for r in future:num(r['yaw'])
            check(actors==side['actor_map'],'FUTURE_ACTOR_IDS')
            check(set(index)=={(logical,k) for logical in ids for k in range(1,n+1)},'FUTURE_MISSING_OR_EXTRA_FRAMES')
            clock=csv_rows(asset(ar['clock']));clock_frames=set()
            for clockrow in clock:
                cf=integer(clockrow['frame']);ct=num(clockrow['simulation_time_s'])
                check(cf not in clock_frames,'CLOCK_DUPLICATE');clock_frames.add(cf)
                check(f0<=cf<=f0+n and abs(ct-(t0+(cf-f0)*dt))<=1e-6,'CLOCK_FRAME_TIME')
            check(clock_frames==set(range(f0,f0+n+1)),'CLOCK_MISSING')
            env=document(ar['environment']);keys(env,'environment')
            for k in ('client_version','server_version','map'):text(env[k])
            check(env['client_version']==env['server_version'],'CARLA_VERSION_MISMATCH')
            sha(env['opendrive_sha256']);sha(env['route_sha256'])
            check(isinstance(env['weather'],dict) and bool(env['weather']),'WEATHER_MISSING')
            check(env['world_settings'].get('synchronous_mode') is True and env['world_settings'].get('fixed_delta_seconds')==dt,'WORLD_DT')
            check(env['known_exogenous_information']==[],'EXOGENOUS_INFORMATION_REQUIRES_SEPARATE_REVIEW')
            environments.append(env)
            ph=document(ar['physics']);check(set(ph)=={'measurement_status','unknown_fields','actors'},'PHYSICS_FIELDS')
            check(ph['measurement_status']=='complete_static' and ph['unknown_fields']==[] and set(ph['actors'])==ids,'PHYSICS_INCOMPLETE')
            for record in ph['actors'].values():check(num(record['mass'])>0 and bool(record['wheels']),'PHYSICS_REQUIRED_FIELDS')
            physics.append({k:physics_settings(v) for k,v in ph['actors'].items()})
            bb=document(ar['bbox']);check(set(bb)==ids,'BBOX_IDS')
            for b in bb.values():
                check(set(b)=={'location','extent','rotation'},'BBOX_FIELDS')
                for k in ('location','extent'):
                    check(set(b[k])==set('xyz'),'BBOX_XYZ')
                    for v in b[k].values():check(num(v)>0 if k=='extent' else math.isfinite(num(v)),'BBOX_DIMENSIONS')
                check(set(b['rotation'])=={'roll','pitch','yaw'},'BBOX_ROTATION')
                for v in b['rotation'].values():num(v)
            boxes.append(bb)
            initial_control=document(ar['initial_control'])
            check(set(initial_control)=={'frame','simulation_time_s','actors'},'INITIAL_CONTROL_FIELDS')
            check(integer(initial_control['frame'])==f0 and abs(num(initial_control['simulation_time_s'])-t0)<=1e-6,'INITIAL_CONTROL_TIME')
            check(set(initial_control['actors'])==ids,'INITIAL_CONTROL_IDS')
            for ctl in initial_control['actors'].values():
                check(set(ctl)=={'throttle','brake','steer','hand_brake','reverse','gear','manual_gear_shift'},'INITIAL_CONTROL_VALUES')
                check(0<=num(ctl['throttle'])<=1 and 0<=num(ctl['brake'])<=1 and -1<=num(ctl['steer'])<=1,'CONTROL_RANGE')
                check(isinstance(ctl['gear'],int) and not isinstance(ctl['gear'],bool),'GEAR')
                check(all(isinstance(ctl[k],bool) for k in ('hand_brake','reverse','manual_gear_shift')),'CONTROL_BOOLEAN')
            initial_controls.append(initial_control['actors'])
            events=document(ar['events'])
            check(set(events)=={'capture_status','frame_start','frame_end','covered_logical_ids','raw_definitions','raw_events','label_status','label_protocol','labels'},'EVENT_FIELDS')
            check(set(events['raw_definitions'])=={'collision','near_miss'},'EVENT_DEFINITIONS')
            for definition in events['raw_definitions'].values():text(definition)
            check(events['capture_status']=='complete' and events['frame_start']==f0 and events['frame_end']==f0+n and set(events['covered_logical_ids'])==ids,'RAW_EVENT_COVERAGE')
            event_ids=set()
            for e in events['raw_events']:
                check(set(e)=={'event_id','kind','frame','simulation_time_s','pair','provenance','raw_payload'},'RAW_EVENT_FIELDS')
                check(isinstance(e['raw_payload'],dict) and bool(e['raw_payload']),'RAW_EVENT_PAYLOAD')
                text(e['event_id']);text(e['provenance']);check(e['kind'] in ('collision','near_miss'),'RAW_EVENT_KIND')
                check(e['event_id'] not in event_ids,'DUPLICATE_RAW_EVENT_ID');event_ids.add(e['event_id'])
                ef=integer(e['frame']);check(f0<=ef<=f0+n and abs(num(e['simulation_time_s'])-(t0+(ef-f0)*dt))<=1e-6,'RAW_EVENT_TIME')
                check(len(e['pair'])==2 and len(set(e['pair']))==2 and set(e['pair'])<=ids,'EVENT_PAIR')
            labels_complete=events['label_status']=='complete_independent'
            if labels_complete:
                text(events['label_protocol']);coverage=set()
                for e in events['labels']:
                    check(set(e)=={'pair','start_frame','end_frame','class','provenance'},'LABEL_FIELDS')
                    pair=tuple(sorted(e['pair']));check(len(pair)==2 and len(set(pair))==2 and set(pair)<=ids,'LABEL_PAIR')
                    start,end=integer(e['start_frame']),integer(e['end_frame']);check(f0+1<=start<=end<=f0+n,'LABEL_TIME')
                    check(e['class'] in ('collision','near_miss','negative'),'LABEL_CLASS');text(e['provenance'])
                    for frame in range(start,end+1):
                        key=(pair,frame);check(key not in coverage,'LABEL_OVERLAP');coverage.add(key)
                expected={(pair,k) for pair in itertools.combinations(sorted(ids),2) for k in range(f0+1,f0+n+1)}
                check(coverage==expected,'LABEL_COVERAGE_INCOMPLETE')
                for e in events['raw_events']:
                    if e['kind']=='collision' and e['frame']>f0:
                        relevant=[label for label in events['labels'] if set(label['pair'])==set(e['pair']) and label['start_frame']<=e['frame']<=label['end_frame']]
                        check(relevant and relevant[0]['class']=='collision','COLLISION_LABEL_CONTRADICTION')
            else:
                check(events['label_status'] in ('missing','partial','proxy'),'LABEL_STATUS')
                result['warnings'].append(label.upper()+'_INDEPENDENT_LABELS_INCOMPLETE')
            event_contracts.append(labels_complete)
        check(environments[0]==environments[1],'ENVIRONMENT_MISMATCH')
        result['initial_configuration_differences']=dict(
            static_physics=value_differences(*physics),bbox=value_differences(*boxes),
            actual_control=value_differences(*initial_controls))
        for logical,a in state_sets[0].items():
            b=state_sets[1][logical];delta=dict(position_xyz_m=math.dist([a[k] for k in 'xyz'],[b[k] for k in 'xyz']),
                velocity_xyz_mps=math.dist([a['v'+k] for k in 'xyz'],[b['v'+k] for k in 'xyz']),
                heading_deg=abs((b['yaw']-a['yaw']+180)%360-180),
                roll_deg=abs((b['roll']-a['roll']+180)%360-180),pitch_deg=abs((b['pitch']-a['pitch']+180)%360-180))
            for value in delta.values():num(value)
            result['initial_residuals'][logical]=delta
        check(all(state_sets[0][i]['blueprint_id']==state_sets[1][i]['blueprint_id'] for i in ids),'BLUEPRINT_MISMATCH')
        prefix=csv_rows(asset(source['artifacts']['prefix']));f0=integer(source['origin']['frame']);t0=num(source['origin']['simulation_time_s']);start=integer(source['prefix_start_frame'])
        check(start<=f0,'PREFIX_RANGE');seen=set()
        check((f0-start+1)*len(ids)<=100000,'PROTOTYPE_PREFIX_ROW_LIMIT')
        for r in prefix:
            f=integer(r['carla_frame']);logical=r['logical_actor_id'];tm=num(r['sim_time_s'])
            check(f<=f0 and tm<=t0+1e-6,'FUTURE_INFORMATION_IN_PREFIX')
            check(logical in ids and integer(r['source_actor_id'])==source['actor_map'][logical],'PREFIX_ACTOR')
            check(start<=f and abs(tm-(t0+(f-f0)*dt))<=1e-6,'PREFIX_TIME')
            key=(logical,f);check(key not in seen,'PREFIX_DUPLICATE');seen.add(key)
            for name in ('x','y','z','roll','pitch','yaw','vx','vy','vz','speed_mps'):num(r[name])
            check(num(r['speed_mps'])>=0 and abs(math.hypot(*(num(r['v'+k]) for k in 'xyz'))-num(r['speed_mps']))<=1e-6,'PREFIX_SPEED_DEFINITION')
            if f==f0:check(all(num(r[k])==state_sets[0][logical][k] for k in ('x','y','z','roll','pitch','yaw','vx','vy','vz','speed_mps')),'PREFIX_SNAPSHOT_END_STATE')
        check(seen=={(i,f) for i in ids for f in range(start,f0+1)},'PREFIX_MISSING')
        inputs=pred['inputs'];keys(inputs,'inputs');check(inputs['inventory_complete'] is True,'UNDECLARED_INPUTS')
        check(inputs['prefix']==source['artifacts']['prefix'],'OBSERVATION_PARENT_HASH')
        check(integer(inputs['maximum_source_frame'])==max(integer(r['carla_frame']) for r in prefix),'INPUT_MAX_FRAME')
        check(abs(num(inputs['maximum_source_timestamp'])-max(num(r['sim_time_s']) for r in prefix))<=1e-6,'INPUT_MAX_TIME')
        for side in (source,pred):
            check(set(side['control'])==ids,'CONTROL_IDS')
            for ctl in side['control'].values():
                keys(ctl,'control');text(ctl['longitudinal']);text(ctl['steering']);finite_tree(ctl['parameters'])
                check(isinstance(ctl['parameters'],dict) and bool(ctl['parameters']),'PARAMETERS_MISSING')
                check(ctl['state_capture'] in ('recorded','unknown'),'CONTROL_STATE_STATUS')
                if ctl['state_capture']=='unknown':result['warnings'].append('CONTROLLER_INTERNAL_STATE_UNKNOWN:'+side['run_id'])
        if ev['purpose']=='A_reconstruction':check(source['control']==pred['control'],'A_CONTROL_MISMATCH')
        if ev['purpose']=='C_T_counterfactual':
            changed=[]
            for logical in ids:
                a,b=source['control'][logical],pred['control'][logical]
                check({k:v for k,v in a.items() if k!='parameters'}=={k:v for k,v in b.items() if k!='parameters'},'C_CONTROL_MISMATCH')
                check(set(a['parameters'])==set(b['parameters']) and 'T_s' in a['parameters'],'C_PARAMETER_SET')
                check(num(a['parameters']['T_s'])>0 and num(b['parameters']['T_s'])>0,'C_T_VALUE')
                check({k:v for k,v in a['parameters'].items() if k!='T_s'}=={k:v for k,v in b['parameters'].items() if k!='T_s'},'C_NON_T_CHANGE')
                changed.append(a['parameters']['T_s']!=b['parameters']['T_s'])
            check(any(changed),'C_NO_T_CHANGE')
            result['warnings'].append('COUNTERFACTUAL_NOT_OBSERVED_OTHER_T_FUTURE')
        if 'risk_detection' in ev['requested_metrics']:check(all(event_contracts),'INCOMPLETE_INDEPENDENT_EVENT_LABELS')
        result.update(status='CONTRACT_VALID_PENDING_APPROVAL',purpose=ev['purpose'],
            evidence_kind=m['evidence_kind'],label_contract_complete=all(event_contracts),
            trajectory_samples_per_side=n*len(ids),causal_prefix_verified=True)
        result['warnings'].append('INITIAL_THRESHOLDS_AND_FORECAST_SCORING_NOT_APPROVED')
    except (ValueError,KeyError,TypeError,OSError,IndexError,OverflowError,AttributeError,RecursionError) as error:
        result['errors'].append(str(error))
    return result
