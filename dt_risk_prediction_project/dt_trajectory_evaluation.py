"""Strict, server-free numerical comparison of already aligned trajectories.

Current supported evidence is synthetic validation or reconstruction repeatability,
not independent forecasting accuracy. Never infers GT or collision labels.
"""
import math
from statistics import mean

from idm_fidelity import number


def _index(rows,dt,origin_frame=None):
    result={};actors={};offset=None
    for row in rows:
        logical=str(row['logical_actor_id'])
        if not logical:raise ValueError('empty logical ID')
        t=number(row['prediction_time_s']);step=round(t/dt)
        if step<1 or abs(t-step*dt)>1e-6:raise ValueError('future time is not on the declared positive grid')
        actor=number(row['carla_actor_id']);frame=number(row['prediction_frame'])
        if not actor.is_integer() or actor<=0 or not frame.is_integer():raise ValueError('invalid actor/frame')
        actor=int(actor);frame=int(frame)
        if logical in actors and actors[logical]!=actor:raise ValueError('actor identity changed within a trajectory')
        actors[logical]=actor
        observed_offset=int(frame)-step
        if offset is None:offset=observed_offset
        if observed_offset!=offset or (origin_frame is not None and observed_offset!=origin_frame):
            raise ValueError('frame/time origin inconsistency')
        values={k:number(row[k]) for k in ('x','y','z','vx','vy','vz','speed_mps')}
        if values['speed_mps']<0 or abs(math.hypot(values['vx'],values['vy'],values['vz'])-values['speed_mps'])>1e-6:
            raise ValueError('inconsistent 3D speed definition')
        key=(logical,step)
        if key in result:raise ValueError('duplicate logical ID/time key')
        result[key]=values
    if not result:raise ValueError('empty trajectory')
    return result,actors,offset


def evaluate(reference,prediction,*,dt=.05,horizons=(1.,2.,3.,5.),purpose='synthetic_validation',
             reference_origin_frame=None,prediction_origin_frame=None,terminal_events=None):
    """No interpolation, best-fit alignment, extrapolation or silent key intersection.

    Missing/censored actor-horizons have null scores and remain in denominators.
    Global world frame numbers may differ; their declared relative origins may not.
    """
    dt=number(dt)
    if dt<=0:raise ValueError('dt must be positive')
    if purpose not in ('synthetic_validation','reconstruction_repeatability'):
        raise ValueError('independent forecast/GT contract is not established by this evaluator')
    hs=sorted(set(number(h) for h in horizons))
    if not hs or any(h<=0 or abs(h/dt-round(h/dt))>1e-6 for h in hs):raise ValueError('invalid horizons')
    if round(hs[-1]/dt)>100000:raise ValueError('evaluation grid too large')
    ref,ra,rf=_index(reference,dt,reference_origin_frame)
    pred,pa,pf=_index(prediction,dt,prediction_origin_frame)
    ids=sorted(set(ra)|set(pa));events=terminal_events or {}
    if any(k not in ids or number(v['time_s'])<0 or v['reason'] not in ('collision','disappearance') for k,v in events.items()):
        raise ValueError('invalid terminal event declaration')
    samples=[];scores=[]
    for logical in ids:
        for i in range(1,round(hs[-1]/dt)+1):
            key=(logical,i)
            if key not in ref or key not in pred:continue
            if logical in events and i*dt>=number(events[logical]['time_s'])-1e-9:continue
            a,b=ref[key],pred[key]
            samples.append(dict(logical_actor_id=logical,time_s=i*dt,
                displacement_xy_m=math.hypot(b['x']-a['x'],b['y']-a['y']),
                displacement_xyz_m=math.dist([b[k] for k in 'xyz'],[a[k] for k in 'xyz']),
                speed_error_mps=b['speed_mps']-a['speed_mps']))
        actor_samples={round(s['time_s']/dt):s for s in samples if s['logical_actor_id']==logical}
        for h in hs:
            n=round(h/dt);available=[actor_samples[i] for i in range(1,n+1) if i in actor_samples]
            censored=logical in events and number(events[logical]['time_s'])<=h+1e-9
            valid=len(available)==n and not censored
            row=dict(logical_actor_id=logical,horizon_s=h,status='complete' if valid else 'post_event_censored' if censored else 'missing_samples',
                expected_samples=n,paired_samples=len(available),coverage=len(available)/n,
                reference_actor_id=ra.get(logical),prediction_actor_id=pa.get(logical),
                ade_xy_m=None,fde_xy_m=None,ade_xyz_m=None,fde_xyz_m=None,
                speed_mae_mps=None,speed_rmse_mps=None,speed_bias_mps=None)
            if valid:
                e=[s['speed_error_mps'] for s in available]
                row.update(ade_xy_m=mean(s['displacement_xy_m'] for s in available),fde_xy_m=available[-1]['displacement_xy_m'],
                    ade_xyz_m=mean(s['displacement_xyz_m'] for s in available),fde_xyz_m=available[-1]['displacement_xyz_m'],
                    speed_mae_mps=mean(abs(x) for x in e),speed_rmse_mps=math.sqrt(mean(x*x for x in e)),speed_bias_mps=mean(e))
            scores.append(row)
    aggregate=[]
    for h in hs:
        subset=[r for r in scores if r['horizon_s']==h];complete=[r for r in subset if r['status']=='complete']
        aggregate.append(dict(horizon_s=h,total_actors=len(ids),scorable_actors=len(complete),
            actor_coverage=len(complete)/len(ids),ade_xy_macro_m=mean(r['ade_xy_m'] for r in complete) if complete else None,
            fde_xy_macro_m=mean(r['fde_xy_m'] for r in complete) if complete else None))
    time_errors=[]
    for step in range(1,round(hs[-1]/dt)+1):
        group=[r for r in samples if round(r['time_s']/dt)==step]
        time_errors.append(dict(time_s=step*dt,total_actors=len(ids),paired_actors=len(group),
            mean_displacement_xy_m=mean(r['displacement_xy_m'] for r in group) if group else None))
    return dict(purpose=purpose,independent_ground_truth_used=False,dt_s=dt,
        reference_origin_frame=rf,prediction_origin_frame=pf,actors=ids,per_actor=scores,
        per_horizon=aggregate,per_time=time_errors,paired_samples=samples,
        risk_detection_performance=None,route_audit=None,physical_bounds=None)
