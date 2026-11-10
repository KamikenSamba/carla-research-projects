"""Snapshot-initialized offline DT MVP, reusing the existing two-vehicle IDM rollout."""
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

from idm_model import IDMParameters, finite
from idm_snapshot import read_snapshot_rows
from idm_t_sensitivity import State1D, gap_of, rollout
from idm_fidelity import write_csv


def _f0_records(path,frame,run_id,ids,*,paired):
    """Read only through complete F0; identity/clock mismatches are not tolerances."""
    found={};digest=hashlib.sha256();count=0
    with Path(path).open('rb') as stream:
        for line in stream:
            record=json.loads(line.decode('utf-8-sig'))
            if record.get('source_run_id',run_id)!=run_id:raise ValueError('raw Source run ID mismatch')
            observed=record.get('frame') if paired else record.get('carla_frame')
            if type(observed) is not int:raise ValueError('raw frame missing/invalid')
            if observed>frame:raise ValueError('missing F0 observations before future boundary')
            digest.update(line);count+=1
            if observed!=frame:continue
            for row in record['rows']:
                if row['actor_id'] not in ids:continue
                if row['actor_id'] in found:raise ValueError('duplicate F0 actor observation')
                if row['frame']!=frame:raise ValueError('raw actor frame differs from F0')
                if row.get('sim_time_s')!=record.get('sim_time_s'):raise ValueError('raw frame/run clock mismatch')
                clock=row['world_time_s'] if paired else record['world_time_s']
                found[row['actor_id']]=dict(row=row,world_time_s=finite(clock,'F0 world time',0.))
            if paired and set(found)!=ids:raise ValueError('missing required F0 logical/Actor identities')
            if set(found)==ids:break
    if set(found)!=ids:raise ValueError('both complete F0 observations are required')
    return found,digest.hexdigest(),count


def initial_state(snapshot_path, raw_prefix_path, follower_id, leader_id,*,source_run_id,state_prefix_path):
    """Consume only through the two F0 bbox observations; never read the future.

    Snapshot sim_time_s in old bounded runs is run-relative, not necessarily the
    world clock. Preserve it separately from the raw F0 world_time_s.
    """
    paths=[Path(p).resolve() for p in (snapshot_path,raw_prefix_path,state_prefix_path)]
    if (not source_run_id or any(p.parent!=paths[0].parent for p in paths)
        or paths[0].parent.name!=source_run_id):raise ValueError('Source run ID/directory mismatch')
    raw=paths[0].read_bytes();payload=json.loads(raw.decode('utf-8-sig'))
    declared=payload.get('source_run_id') if isinstance(payload,dict) else None
    if declared is not None and declared!=source_run_id:raise ValueError('Snapshot Source run ID mismatch')
    rows=read_snapshot_rows(payload['actors'] if isinstance(payload,dict) else payload)
    actors={r['logical_actor_id']:r for r in rows}
    if follower_id==leader_id or set(actors)!={follower_id,leader_id}:
        raise ValueError('exactly the two distinct selected logical IDs are required')
    f,l=actors[follower_id],actors[leader_id]
    if f['source_actor_id']==l['source_actor_id']:raise ValueError('duplicate physical actor ID')
    if (f['carla_frame'],f['sim_time_s'])!=(l['carla_frame'],l['sim_time_s']):
        raise ValueError('initial states do not share F0/t0')
    frame=f['carla_frame'];ids={f['source_actor_id'],l['source_actor_id']}
    bbox_rows,bbox_hash,line_count=_f0_records(paths[1],frame,source_run_id,ids,paired=False)
    state_rows,state_hash,state_count=_f0_records(paths[2],frame,source_run_id,ids,paired=True)
    clocks=[r['world_time_s'] for r in [*bbox_rows.values(),*state_rows.values()]]
    if len(set(clocks))!=1:raise ValueError('inconsistent F0 world clock')
    clock=clocks[0];declared_world=payload.get('world_time_s') if isinstance(payload,dict) else None
    if declared_world is not None and finite(declared_world,'declared world time')!=clock:
        raise ValueError('declared Snapshot F0 world clock mismatch')
    boxes={actor:r['row']['bounding_box'] for actor,r in bbox_rows.items()}
    residuals={}
    for logical,snapshot in actors.items():
        actor=snapshot['source_actor_id'];pair=state_rows[actor]['row'];box_row=bbox_rows[actor]['row']
        if pair['logical_id']!=logical:raise ValueError('F0 logical/Actor ID mismatch')
        if pair['sim_time_s']!=snapshot['sim_time_s'] or box_row['sim_time_s']!=snapshot['sim_time_s']:
            raise ValueError('Snapshot/raw F0 run clock mismatch')
        fields=('x','y','z','vx','vy','vz','speed_mps','yaw','pitch','roll')
        deltas={k:finite(pair[k],'raw '+k)-snapshot[k] for k in fields}
        bstate=dict(box_row['transform'],**{'v'+k:box_row['velocity'][k] for k in 'xyz'})
        bstate['speed_mps']=math.hypot(*(box_row['velocity'][k] for k in 'xyz'))
        bdeltas={k:finite(bstate[k],'bbox raw '+k)-snapshot[k] for k in fields}
        yaw,pitch=math.radians(snapshot['yaw']),math.radians(snapshot['pitch'])
        expected=(math.cos(yaw)*math.cos(pitch),math.sin(yaw)*math.cos(pitch),math.sin(pitch))
        residuals[logical]=dict(pair_minus_snapshot=deltas,bbox_raw_minus_snapshot=bdeltas,
            forward_pair_minus_snapshot_angles=[finite(pair['forward_'+k],'raw forward')-v for k,v in zip('xyz',expected)],
            forward_bbox_raw_minus_snapshot_angles=[finite(box_row['forward'][k],'raw forward')-v for k,v in zip('xyz',expected)],
            raw_speed_norm_internal_residual_mps=pair['speed_mps']-math.hypot(*(pair['v'+k] for k in 'xyz')))
    yaw=math.radians(f['yaw']);axis=(math.cos(yaw),math.sin(yaw))
    lateral=(-(l['x']-f['x'])*axis[1]+(l['y']-f['y'])*axis[0])
    if abs(lateral)>.25:raise ValueError('not a shared straight lane: lateral offset > 0.25m')
    states={};centers={}
    for logical,row in actors.items():
        angle=math.radians(row['yaw']);delta=angle-yaw
        if abs((row['yaw']-f['yaw']+180)%360-180)>5 or abs(row['pitch'])>1e-8 or abs(row['roll'])>1e-8:
            raise ValueError('flat, aligned vehicles required by the ideal 1D model')
        box=boxes[row['source_actor_id']]
        if any(finite(box['rotation'][k],k)!=0 for k in ('yaw','pitch','roll')):
            raise ValueError('rotated local bbox unsupported')
        ext={k:finite(box['extent'][k],'bbox extent',0.) for k in ('x','y','z')}
        loc={k:finite(box['location'][k],'bbox location') for k in ('x','y','z')}
        if min(ext.values())<=0:raise ValueError('positive bbox extents required')
        center=(row['x']+math.cos(angle)*loc['x']-math.sin(angle)*loc['y'],
                row['y']+math.sin(angle)*loc['x']+math.cos(angle)*loc['y'])
        length=2*(abs(math.cos(delta))*ext['x']+abs(math.sin(delta))*ext['y'])
        speed=math.hypot(row['vx'],row['vy'],row['vz'])
        if not math.isclose(speed,row['speed_mps'],rel_tol=1e-6,abs_tol=1e-12):
            raise ValueError('snapshot speed differs from recorded velocity magnitude')
        centers[logical]=center;states[logical]=State1D(sum(center[i]*axis[i] for i in range(2)),speed,length)
    origin=states[follower_id].x
    states={key:replace(s,x=s.x-origin) for key,s in states.items()}
    if gap_of(states[follower_id],states[leader_id])<=0:raise ValueError('leader is behind or initial bumper overlap')
    raw_centers={};raw_lengths={}
    for logical,snapshot in actors.items():
        row=state_rows[snapshot['source_actor_id']]['row'];box=boxes[snapshot['source_actor_id']]
        angle=math.radians(row['yaw']);delta=angle-yaw
        center=(row['x']+math.cos(angle)*box['location']['x']-math.sin(angle)*box['location']['y'],
                row['y']+math.sin(angle)*box['location']['x']+math.cos(angle)*box['location']['y'])
        raw_centers[logical]=sum(center[i]*axis[i] for i in range(2))
        raw_lengths[logical]=2*(abs(math.cos(delta))*box['extent']['x']+abs(math.sin(delta))*box['extent']['y'])
    raw_gap=raw_centers[leader_id]-raw_centers[follower_id]-(raw_lengths[leader_id]+raw_lengths[follower_id])/2
    initial_gap=gap_of(states[follower_id],states[leader_id])
    return dict(actors=actors,states=states,axis=axis,follower_id=follower_id,leader_id=leader_id,
        source_run_id=source_run_id,source_run_id_basis='same resolved Source directory + explicit ID; not authenticity proof',
        source_frame_f0=frame,snapshot_time_s=f['sim_time_s'],world_time_s=clock,
        source_snapshot_sha256=hashlib.sha256(raw).hexdigest(),consumed_raw_prefix_sha256=bbox_hash,
        consumed_state_prefix_sha256=state_hash,consumed_state_lines=state_count,
        f0_comparison=dict(identity_and_raw_clocks='CHECKED',raw_required_fields='COMPLETE',
            snapshot_embedded_run_id='NOT_RECORDED' if declared is None else 'CHECKED',
            snapshot_embedded_world_time='NOT_RECORDED' if declared_world is None else 'CHECKED',
            numerical_acceptance='NOT_ASSESSED_NO_APPROVED_TOLERANCE',residuals=residuals,
            snapshot_bbox_gap_m=initial_gap,raw_bbox_gap_on_snapshot_axis_m=raw_gap,
            bbox_gap_residual_m=raw_gap-initial_gap,
            raw_pair_reported_gap_m=state_rows[f['source_actor_id']]['row'].get('gap_m'),
            pair_gap_definition='legacy PairHarness conservative footprint; not the V2 bbox gap'),
        consumed_raw_lines=line_count,boxes=boxes,
        model_assumptions='flat straight lane; projected bbox centers; V2 3D speed magnitude as 1D speed; '
                          'fixed lateral position, z and heading; no CARLA physics/PID/steering')


def predict(initial, follower_parameters, leader_parameters, *, headways=(1.,2.),dt=.05,horizon=6.):
    """Known T intervention only; leader uses IDM free road from its t0 state."""
    ts=tuple(finite(t,'T',0.) for t in headways)
    if not ts or len(set(ts))!=len(ts):raise ValueError('distinct headway values required')
    results=[];rows=[];f_id,l_id=initial['follower_id'],initial['leader_id']
    for t in ts:
        p=replace(follower_parameters,desired_time_headway_s=t)
        result=rollout(initial['states'][f_id],initial['states'][l_id],p,leader_parameters,dt=dt,horizon=horizon)
        results.append(result)
        for step,sample in enumerate(result['rows']):
            for role,logical in [('follower',f_id),('leader',l_id)]:
                original=initial['actors'][logical];state=initial['states'][logical]
                displacement=sample[role+'_x_m']-state.x
                rows.append(dict(T_s=t,prediction_step=step,time_s=sample['time_s'],
                    logical_actor_id=logical,source_actor_id=original['source_actor_id'],role=role,
                    source_frame_f0=initial['source_frame_f0'],source_snapshot_time_s=initial['snapshot_time_s'],
                    source_world_time_s=initial['world_time_s'],
                    x=original['x']+initial['axis'][0]*displacement,
                    y=original['y']+initial['axis'][1]*displacement,z=original['z'],yaw=original['yaw'],
                    along_bbox_center_m=sample[role+'_x_m'],speed_mps=sample[role+'_speed_mps'],
                    idm_acceleration_mps2=sample[role+'_acceleration_mps2'],
                    gap_m=sample['gap_m'],closing_speed_mps=sample['closing_speed_mps'],ttc_s=sample['ttc_s']))
    first=results[0]
    comparisons=[dict(T_s=r['parameters']['desired_time_headway_s'],status=r['status'],
        final_follower_distance_m=r['follower_distance_m'],final_follower_speed_mps=r['final_follower_speed_mps'],
        final_gap_m=r['final_gap_m'],minimum_gap_m=r['minimum_continuous_gap_m'],
        final_position_difference_from_first_T_m=r['follower_distance_m']-first['follower_distance_m']) for r in results]
    return dict(results=results,rows=rows,comparison=comparisons,
        leader_model='existing IDM free road, no measured future/exogenous future profile',
        route_audit=None,physical_bounds=None,real_CARLA_trial='BLOCKED',
        independent_prediction_accuracy='NOT_EVALUATED',interpretation=initial['model_assumptions'])


def save(output,initial,prediction,*,input_kind,plots=False):
    if input_kind not in ('recorded_snapshot','synthetic'):raise ValueError('explicit input kind required')
    output=Path(output).resolve();repo=Path(__file__).resolve().parent.parent
    if output.is_relative_to(repo):raise ValueError('outputs must be outside repository')
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'predicted_trajectories.csv',prediction['rows'])
    write_csv(output/'T_comparison.csv',prediction['comparison'])
    metadata={k:v for k,v in initial.items() if k!='states'}
    metadata['states']={k:asdict(v) for k,v in initial['states'].items()}
    summary={k:v for k,v in prediction.items() if k not in ('rows','results')}
    summary.update(input_kind=input_kind,initial=metadata,
        results=[{k:v for k,v in r.items() if k!='rows'} for r in prediction['results']])
    with (output/'prediction_summary.json').open('x',encoding='utf8') as stream:
        json.dump(summary,stream,indent=2,allow_nan=False)
    if plots:plot(output,prediction)
    return summary


def plot(output,prediction):
    """Reuse the project's matplotlib/Agg conventions; plot no measured future."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True,layout='constrained')
    for result in prediction['results']:
        T=result['parameters']['desired_time_headway_s'];rows=result['rows'];times=[r['time_s'] for r in rows]
        for role in ('leader','follower'):
            style='--' if role=='leader' else '-'
            for ax,key in [(axes[0],role+'_x_m'),(axes[1],role+'_speed_mps'),(axes[3],role+'_acceleration_mps2')]:
                ax.plot(times,[r[key] for r in rows],style,label=f'{role}, T_f={T:g}s')
        axes[2].plot(times,[r['gap_m'] for r in rows],label=f'T_f={T:g}s')
    for ax,label in zip(axes,('Along-lane bbox center [m]','Speed [m/s]','Bumper gap [m]','IDM acceleration [m/s2]')):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Prediction time [s]; no measured future used')
    fig.suptitle('Snapshot-initialized DT: ideal 1D IDM, not CARLA physics')
    fig.savefig(Path(output)/'prediction_comparison.png',dpi=140);plt.close(fig)
