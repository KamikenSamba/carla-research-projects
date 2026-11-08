"""Offline derivative and ideal 1D two-vehicle motion, using the existing IDM.

No PID, CARLA physics, traffic rules, uncertainty model or collision sensor.
Positions are bbox centers along one straight lane; gap is bumper to bumper.
"""
from dataclasses import asdict, dataclass, replace
import math

from idm_model import IDMParameters, compute_desired_gap, compute_idm_acceleration, finite

T_VALUES = (.5, 1., 1.5, 2., 3.)


def t_sensitivity(speed, leader_speed, gap, parameters, h=1e-5):
    """Analytic one-sided derivatives of the *clamped* existing equation.

    Boundary classification tolerance is 1e-10 (analysis only). At kinks the
    two-sided analytic derivative is None; a central FD is not a derivative there.
    """
    v=finite(speed,'speed',0.);vl=finite(leader_speed,'leader speed',0.)
    h=finite(h,'difference step',0.)
    if h==0:raise ValueError('difference step must be positive')
    p=parameters;T=p.desired_time_headway_s
    gap=None if gap is None else finite(gap,'gap')
    acc=compute_idm_acceleration(v,p,gap,vl)
    def f(t):return compute_idm_acceleration(v,replace(p,desired_time_headway_s=t),gap,vl)
    fd_left=(acc-f(T-h))/h if T>=h else None
    fd_right=(f(T+h)-acc)/h
    left=right=0.;kind='free_road' if gap is None else 'nonpositive_gap'
    dynamic=raw=None
    if gap is not None and finite(gap,'gap')>0:
        g=max(.001,gap)
        dynamic=v*T+v*(v-vl)/(2*math.sqrt(p.max_accel_mps2)*math.sqrt(p.comfortable_decel_mps2))
        desired=compute_desired_gap(v,v-vl,p)
        raw=p.max_accel_mps2*(1-(v/p.desired_speed_mps)**p.acceleration_exponent-(desired/g)**2)
        slope=-2*p.max_accel_mps2*desired*v/g**2
        if abs(dynamic)<=1e-10:
            left,right=0.,slope;kind='max_boundary'
        elif dynamic<0:
            kind='inactive_dynamic_gap'
        else:
            left=right=slope;kind='active_dynamic_gap'
        if raw < -p.max_decel_mps2-1e-10:
            left=right=0.;kind='lower_clamp'
        elif abs(raw+p.max_decel_mps2)<=1e-10:
            right=0.;kind='lower_clamp_boundary'
        elif raw>p.max_accel_mps2+1e-10:
            left=right=0.;kind='upper_clamp'
        elif abs(raw-p.max_accel_mps2)<=1e-10:
            left=0.;kind='upper_clamp_boundary'
    return dict(T_s=T,speed_mps=v,leader_speed_mps=vl,gap_m=gap,
        acceleration_mps2=acc,dynamic_gap_term_m=dynamic,unclamped_acceleration_mps2=raw,
        regime=kind,analytic_left=left if T>0 else None,analytic_right=right,
        analytic_derivative=right if T==0 or left==right else None,
        fd_left=fd_left,fd_right=fd_right,
        fd_central=(fd_left+fd_right)/2 if fd_left is not None else None,fd_step_s=h)


@dataclass(frozen=True)
class State1D:
    x: float
    speed: float
    length: float=4.8

    def __post_init__(self):
        for key in ('x','speed','length'):
            object.__setattr__(self,key,finite(getattr(self,key),key,None if key=='x' else 0.))
        if self.length<=0:raise ValueError('vehicle length must be positive')


def advance(state, acceleration, seconds):
    """Constant-acceleration ballistic step, stopping exactly at v=0 then holding."""
    a=finite(acceleration,'acceleration');dt=finite(seconds,'duration',0.)
    moving=min(dt,state.speed/-a) if a<0 else dt
    return State1D(state.x+state.speed*moving+.5*a*moving*moving,
                   max(0.,state.speed+a*moving),state.length)


def gap_of(follower,leader):
    return leader.x-follower.x-(follower.length+leader.length)/2


def step_geometry(follower,leader,af,al,dt):
    """Locate first contact and continuous minimum gap within a ballistic step.

    Split at each exact stop and at extrema of relative position, so an overlap
    followed by separation within the same step cannot be missed by endpoints.
    """
    gap=lambda t:gap_of(advance(follower,af,t),advance(leader,al,t))
    if gap(0)<=0:return 0.,gap(0)
    breaks=sorted({0.,dt,*[s.speed/-a for s,a in ((follower,af),(leader,al))
                           if a<0 and 0<s.speed/-a<dt]})
    points=set(breaks)
    for lo,hi in zip(breaks,breaks[1:]):
        derivative=lambda t:advance(leader,al,t).speed-advance(follower,af,t).speed
        dl,dr=derivative(lo),derivative(hi)
        if dl*dr<0:points.add(lo-dl*(hi-lo)/(dr-dl))
    points=sorted(points);minimum=min(gap(t) for t in points)
    for lo,hi in zip(points,points[1:]):
        if gap(hi)<=0:
            for _ in range(55):
                mid=(lo+hi)/2
                if gap(mid)<=0:hi=mid
                else:lo=mid
            return hi,0.
    return None,minimum


def rollout(follower,leader,parameters,leader_parameters,*,dt=.05,horizon=6.,leader_acceleration=None):
    """Simultaneous explicit IDM evaluation + ballistic update; halt at contact.

    Only follower T is varied by callers. Leader defaults to the same existing
    IDM free-road function, or a declared exogenous acceleration(time) profile.
    """
    dt=finite(dt,'dt',0.);horizon=finite(horizon,'horizon',0.)
    if dt==0 or horizon==0 or math.ceil(horizon/dt)>100000:
        raise ValueError('positive bounded integration design required')
    f,l=follower,leader;t=0.;rows=[];minimum=gap_of(f,l);status='completed';contact_bracket=None
    def observe(af=None,al=None):
        gap=gap_of(f,l);closing=f.speed-l.speed
        rows.append(dict(time_s=t,T_s=parameters.desired_time_headway_s,
            follower_x_m=f.x,leader_x_m=l.x,follower_speed_mps=f.speed,leader_speed_mps=l.speed,
            gap_m=gap,closing_speed_mps=closing,ttc_s=gap/closing if gap>0 and closing>0 else
                            0. if gap<=0 and closing>0 else None,
            follower_acceleration_mps2=af,leader_acceleration_mps2=al))
    if minimum<=0:
        observe();status='initial_overlap'
    else:
        for i in range(math.ceil(horizon/dt)):
            h=min(dt,horizon-t)
            af=compute_idm_acceleration(f.speed,parameters,gap_of(f,l),l.speed)
            al=compute_idm_acceleration(l.speed,leader_parameters) if leader_acceleration is None else finite(leader_acceleration(t),'leader acceleration')
            observe(af,al)
            contact,min_step=step_geometry(f,l,af,al,h)
            minimum=min(minimum,min_step)
            used=h if contact is None else contact
            f,l=advance(f,af,used),advance(l,al,used)
            if contact is not None:
                contact_bracket=[t,t+h];t+=used;status='contact';break
            t=min((i+1)*dt,horizon)
        observe()
    ttcs=[r['ttc_s'] for r in rows if r['ttc_s'] is not None]
    return dict(status=status,dt_s=dt,requested_horizon_s=horizon,duration_s=t,
        initial_follower=asdict(follower),initial_leader=asdict(leader),parameters=asdict(parameters),
        leader_parameters=asdict(leader_parameters),contact_step_bracket_s=contact_bracket,
        contact_time_s=t if status in ('contact','initial_overlap') else None,
        minimum_continuous_gap_m=minimum,minimum_sampled_ttc_s=min(ttcs) if ttcs else None,
        follower_distance_m=f.x-follower.x,final_follower_speed_mps=f.speed,final_gap_m=gap_of(f,l),
        interpretation='ideal 1D numerical IDM model, not CARLA motion or accident-risk accuracy',
        rows=rows)
