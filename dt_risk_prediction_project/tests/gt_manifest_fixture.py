"""Synthetic two-actor bundle, never represents measured CARLA data."""
from pathlib import Path
import hashlib,json
from dataclasses import asdict

def build(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=False)
    from dt_risk_common import ActorState,PredictionState
    from idm_fidelity import write_csv
    def put(name,data):
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')
        return dict(path=name,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    def putcsv(name,data):
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);write_csv(p,data)
        return dict(path=name,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    controls={logical:dict(longitudinal='existing_IDM',steering='existing_route_controller',
        parameters=dict(T_s=1.,v0_mps=.5,s0_m=2.,a_mps2=.3,b_mps2=1.5),state_capture='unknown') for logical in ('leader','follower')}
    runs=[]
    for name,frame,t0,actor0 in [('source',100,10.,1),('prediction',1000,100.,101)]:
        amap=dict(leader=actor0,follower=actor0+1);snaps=[];future=[];prefix=[]
        for logical,actor in amap.items():
            x=20. if logical=='leader' else 0.
            state=asdict(ActorState('snapshot',frame,t0,actor,logical,logical,'vehicle.tesla.model3',
                x,0.,0.,0.,0.,0.,.5,0.,0.,.5,False))
            snaps.append(state)
            for k in (-2,-1,0):
                row=dict(state);row.update(carla_frame=frame+k,sim_time_s=t0+k*.05,x=x+k*.05*.5);prefix.append(row)
            for k in range(1,101):future.append(asdict(PredictionState(frame+k,k*.05,logical,actor,
                x+k*.05*.5,0.,0.,0.,.5,0.,0.,.5,False)))
        origin=dict(frame=frame,simulation_time_s=t0)
        env=dict(client_version='0.9.16',server_version='0.9.16',map='SYNTHETIC_MAP',opendrive_sha256='a'*64,
            weather=dict(cloudiness=0.),world_settings=dict(synchronous_mode=True,fixed_delta_seconds=.05),
            route_sha256='b'*64,external_actor_inventory=[],known_exogenous_information=[])
        ar=dict(snapshot=put(name+'/snapshot.json',dict(actors=snaps)),future=putcsv(name+'/future.csv',future),
            clock=putcsv(name+'/clock.csv',[dict(frame=frame+k,simulation_time_s=t0+k*.05) for k in range(101)]),
            environment=put(name+'/environment.json',env),
            physics=put(name+'/physics.json',dict(measurement_status='complete_static',unknown_fields=[],actors={
                i:dict(mass=1000.,wheels=[dict(radius=30.,position=dict(x=0,y=0,z=0))]) for i in amap})),
            bbox=put(name+'/bbox.json',{i:dict(location=dict(x=0,y=0,z=0),extent=dict(x=2,y=1,z=1),
                                           rotation=dict(roll=0,pitch=0,yaw=0)) for i in amap}),
            initial_control=put(name+'/initial_control.json',dict(frame=frame,simulation_time_s=t0,actors={
                i:dict(throttle=0.,brake=0.,steer=0.,hand_brake=False,reverse=False,gear=1,manual_gear_shift=False) for i in amap})),
            events=put(name+'/events.json',dict(capture_status='complete',frame_start=frame,frame_end=frame+100,
                covered_logical_ids=list(amap),raw_definitions=dict(collision='synthetic sensor-event fixture',near_miss='synthetic independently specified event fixture'),raw_events=[],label_status='complete_independent',
                label_protocol='synthetic-only known negative fixture',labels=[dict(pair=['follower','leader'],start_frame=frame+1,
                    end_frame=frame+100,**{'class':'negative'},provenance='synthetic fixture')])) )
        if name=='source':ar['prefix']=putcsv(name+'/prefix.csv',prefix)
        runs.append(dict(origin=origin,actor_map=amap,artifacts=ar,control=controls))
    s,p=runs
    s.update(run_id='synthetic_source',protocol='synthetic_fixture_v1',code_commit='c'*40,seed=42,dt_s=.05,prefix_start_frame=98,horizon_steps=100)
    p.update(run_id='synthetic_prediction',code_commit='c'*40,seed=42,method='synthetic_existing_IDM_replay',dt_s=.05,horizon_steps=100,
        parent_source_id=s['run_id'],parent_snapshot_sha256=s['artifacts']['snapshot']['sha256'],
        parent_prefix_sha256=s['artifacts']['prefix']['sha256'],reference_origin=dict(s['origin']),
        initialization=dict(stage='after_initialization_before_first_prediction_tick',first_command_based_on_frame=1000,
            first_response_frame=1001,ticks_after_initial_measurement=0,internal_physics_state='unknown'),
        inputs=dict(prefix=dict(s['artifacts']['prefix']),maximum_source_frame=100,maximum_source_timestamp=10.,inventory_complete=True),
        model_data=dict(training_source_ids=[],tuning_source_ids=[],evaluation_source_ids=[s['run_id']],approach='prescribed'))
    registry=put('registry.json',dict(sources=[dict(source_run_id=s['run_id'],split='test',protocol=s['protocol'],
        code_commit=s['code_commit'],data_manifest_sha256='d'*64)]))
    m=dict(schema_version='dt_source_future_contract_v1',evidence_kind='synthetic',source=s,prediction=p,registry=registry,
        evaluation=dict(purpose='A_reconstruction',requested_metrics=['trajectory'],information_policy='causal_prefix_only',
            initial_policy=dict(status='PROPOSED_UNAPPROVED',thresholds=None,approval_reference=None)),
        safety=dict(experiment_status='BLOCKED',route_audit=None,physical_bounds=None))
    put('manifest.json',m);return root/'manifest.json'
