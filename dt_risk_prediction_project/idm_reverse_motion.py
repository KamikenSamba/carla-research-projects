"""Signed displacement/endpoint velocity checks for exploratory physics only.

Limits are provisional operational guards, not certified physical bounds.
Legacy calibration retains its original single-sample reverse gate.
"""
import math


def unit(v):
    if len(v)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in v):
        raise ValueError('unknown direction')
    norm=math.sqrt(sum(x*x for x in v))
    if norm<.9 or norm>1.1:raise ValueError('invalid direction norm')
    return tuple(x/norm for x in v)


class ReverseMotion:
    def __init__(self,road_direction,dt=.05):
        self.road=unit(road_direction);self.dt=dt
        self.previous=None;self.origin=None;self.maximum_progress=0.;self.negative_count=0

    def update(self,r):
        forward=unit([r['forward_'+k] for k in ('x','y','z')])
        p=tuple(r[k] for k in ('x','y','z'));v=tuple(r['v'+k] for k in ('x','y','z'))
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in p+v+(r['world_time_s'],)):
            raise ValueError('unknown longitudinal observation')
        dot=lambda a,b:sum(x*y for x,y in zip(a,b))
        body_v,road_v=dot(v,forward),dot(v,self.road)
        if self.origin is None:self.origin=p
        progress=dot(tuple(x-y for x,y in zip(p,self.origin)),self.road)
        self.maximum_progress=max(self.maximum_progress,progress)
        retreat=self.maximum_progress-progress
        ds=body_ds=fd=elapsed=None
        if self.previous:
            previous=self.previous
            elapsed=r['world_time_s']-previous['time']
            if r['frame']!=previous['frame']+1 or abs(elapsed-self.dt)>1e-5:
                raise ValueError('reverse monitor frame/time gap')
            delta=tuple(x-y for x,y in zip(p,previous['position']))
            ds=dot(delta,self.road);body_ds=dot(delta,forward);fd=ds/elapsed
        negative=min(body_v,road_v)<-.02
        self.negative_count=self.negative_count+1 if negative else 0
        reason=None
        if min(body_v,road_v)<-.10:reason='reverse_api_severe'
        elif ds is not None and (ds<-.001 or body_ds<-.001):reason='reverse_signed_displacement'
        elif retreat>.002:reason='reverse_cumulative_retreat'
        elif self.negative_count>=3:reason='reverse_velocity_persistent_or_unexplained'
        record=dict(frame=r['frame'],world_time_s=r['world_time_s'],road_direction=self.road,
            body_direction=forward,api_body_velocity_mps=body_v,api_road_velocity_mps=road_v,
            signed_road_displacement_m=ds,signed_body_displacement_m=body_ds,
            position_interval_velocity_mps=fd,observation_interval_s=elapsed,
            signed_progress_from_first_observation_m=progress,retreat_from_maximum_m=retreat,
            consecutive_negative_api_frames=self.negative_count,reason=reason,
            classification=('abort' if reason else 'transient_disagreement_under_observation' if negative else 'no_reverse_evidence'),
            limits=dict(api_warning_mps=-.02,api_immediate_mps=-.10,one_frame_negative_displacement_m=.001,
                cumulative_retreat_m=.002,negative_api_frame_limit=3,maximum_negative_api_observation_s=.15))
        self.previous=dict(frame=r['frame'],time=r['world_time_s'],position=p)
        return record
