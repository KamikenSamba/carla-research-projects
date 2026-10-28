"""Fixed first brake calibration policy. No CARLA import, no empirical physical bounds."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

TOWN01_MODE = 'single_brake_town01_v2'
TOWN01_MAP_SHA256 = '97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709'
TOWN01_LENGTH_M = 195.00316911583323
TOWN01_OPPOSITE_MODE = 'single_brake_town01_opposite_v3'
TOWN01_OPPOSITE_LENGTH_M = 111.99716727741058


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


@dataclass(frozen=True)
class BrakeConfig:
    mode: str = 'single_brake_v1'
    map_name: str = 'Town10HD_Opt'
    blueprint: str = 'vehicle.tesla.model3'
    dt: float = .05
    road: int = 10
    section: int = 0
    lane: int = 1
    follower_s: int = 120
    end_s: int = 64
    planned_route_length: float = 56
    spawn_height: float = .4
    front_extent_limit: float = 2.42511
    settle_seconds: float = 1
    target_speed: float = 1
    speed_limit: float = 1.2
    throttle_cap: float = .1
    throttle_slew_per_second: float = .1
    speed_kp: float = .08
    speed_ki: float = .03
    steer_cap: float = .1
    heading_kp: float = .05
    lateral_kp: float = .2
    launch_seconds: float = 5
    launch_distance: float = 6
    ready_seconds: float = 2
    ready_speed_tolerance: float = .1
    ready_acceleration: float = .5
    brake: float = .7
    response_timeout: float = .3
    deceleration_speed_drop: float = .02
    deceleration_threshold: float = .2
    brake_seconds: float = 5
    stopped_speed: float = .05
    stopped_seconds: float = .5
    acceleration_peak: float = 5
    acceleration_sustained: float = 3
    lateral_limit: float = .3
    heading_limit: float = 5
    end_clearance: float = 20
    route_audit: dict | None = None

    def __post_init__(self):
        # First trial only. Changing speeds, control gains, or gates requires a separately reviewed version.
        defaults={name:f.default for name,f in self.__dataclass_fields__.items() if name!='route_audit'}
        if self.mode == TOWN01_MODE:
            defaults.update(mode=TOWN01_MODE, map_name='Town01_Opt', road=8,
                            follower_s=250, end_s=55, planned_route_length=TOWN01_LENGTH_M)
            if any(type(getattr(self,k)) is not int for k in ('road','section','lane','follower_s','end_s')):
                raise ValueError('Town01 route identifiers and s samples must be integers')
        if self.mode == TOWN01_OPPOSITE_MODE:
            defaults.update(mode=TOWN01_OPPOSITE_MODE,map_name='Town01_Opt',road=8,lane=-1,
                            follower_s=135,end_s=247,planned_route_length=TOWN01_OPPOSITE_LENGTH_M)
            if any(type(getattr(self,k)) is not int for k in ('road','section','lane','follower_s','end_s')):
                raise ValueError('Town01 route identifiers and s samples must be integers')
        for name,value in defaults.items():
            actual=getattr(self,name)
            if isinstance(value,(int,float)) and (not finite(actual) or actual!=value):
                raise ValueError('outside first-calibration design: '+name)
            if isinstance(value,str) and actual!=value:
                raise ValueError('unsupported calibration mode/map/vehicle')
        if self.route_audit is not None:self.require_executable('brake_calibration')

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True).encode()).hexdigest()

    @property
    def policy_sha256(self):
        values=asdict(self);values.pop('route_audit')
        return hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest()

    @property
    def extended_route_audit(self):
        return self.mode in (TOWN01_MODE,TOWN01_OPPOSITE_MODE)

    @property
    def route_point_count(self):
        return abs(self.follower_s-self.end_s)+1

    @property
    def route_s_samples(self):
        step=1 if self.end_s>self.follower_s else -1
        return range(self.follower_s,self.end_s+step,step)

    def distance_report(self, *, route_length_m=None):
        # Exploratory envelope, not a claimed physical lower bound. Stop if the observed envelope fails.
        v,tau,a,b=self.speed_limit,.5,self.acceleration_peak,.5
        distance=v*tau+a*tau*tau/2+(v+a*tau)**2/(2*b)
        required=self.launch_distance+self.front_extent_limit+distance+self.end_clearance
        length=self.planned_route_length if route_length_m is None else route_length_m
        if not finite(length) or length<=0:raise ValueError('invalid measured route length')
        report=dict(assumptions_only=True,v_mps=v,tau_s=tau,a_mps2=a,b_mps2=b,
                    stopping_distance_m=distance,required_route_m=required,
                    margin_m=length-required,accepted_braking_lower_bound=None,
                    accepted_response_upper_bound=None)
        if self.extended_route_audit:
            report.update(route_length_m=length,route_length_source=(
                'live_waypoint_coordinates' if route_length_m is not None else
                'reviewed_coordinates' if self.route_audit is not None else 'offline_reference_not_scene_approved'),
                sensitivity=[])
            if route_length_m is None and self.route_audit is not None:
                length=sum(math.dist(p,q) for p,q in zip(self.route_audit['points'],self.route_audit['points'][1:]))
                report.update(route_length_m=length,margin_m=length-required)
            for tau,b in ((.5,.5),(.5,.25),(1.,.25)):
                stop=v*tau+a*tau*tau/2+(v+a*tau)**2/(2*b)
                need=self.launch_distance+self.front_extent_limit+stop+self.end_clearance
                report['sensitivity'].append(dict(tau_s=tau,a_mps2=a,b_mps2=b,
                    stopping_distance_m=stop,required_route_m=need,margin_m=length-need))
        return report

    def require_executable(self,stage):
        if stage!='brake_calibration':raise ValueError('not a following Stage 1/2 scenario')
        a=self.route_audit
        if not isinstance(a,dict):raise ValueError('calibration route unaudited; refuse before connection')
        def sha(v):return isinstance(v,str) and len(v)==64 and all(c in '0123456789abcdef' for c in v)
        if (a.get('policy_sha256')!=self.policy_sha256 or not sha(a.get('opendrive_sha256'))
                or not sha(a.get('evidence_sha256')) or a.get('map_name')!=self.map_name
                or a.get('carla_version')!='0.9.16' or any(not isinstance(a.get(k),str) or not a[k].strip()
                    for k in ('reviewed_by','reviewed_at','evidence_reference','scene_review_reference','dedicated_session_reference'))):
            raise ValueError('incomplete/mismatched calibration route/session review')
        if a.get('physical_bounds') is not None:
            raise ValueError('calibration must not prefill unmeasured physical bounds')
        points=a.get('points')
        if (not isinstance(points,list) or len(points)!=self.route_point_count or any(not isinstance(p,list) or len(p)!=3
                or not all(finite(v) for v in p) for p in points)):
            raise ValueError(f'review requires all {self.route_point_count} one-metre geometry points')
        if any(not .95<=math.dist(p,q)<=1.05 or abs(p[2]-q[2])>.02 for p,q in zip(points,points[1:])):
            raise ValueError('invalid reviewed route spacing/grade')
        if self.extended_route_audit:
            if a['opendrive_sha256']!=TOWN01_MAP_SHA256:
                raise ValueError('Town01 policy OpenDRIVE hash mismatch')
            from idm_brake_route import validate_review_geometry
            validate_review_geometry(self,a)


def town01_config():
    """Only map/route/distance defaults differ; all control and safety values stay fixed."""
    return BrakeConfig(mode=TOWN01_MODE,map_name='Town01_Opt',road=8,
                       follower_s=250,end_s=55,planned_route_length=TOWN01_LENGTH_M)


def town01_opposite_config():
    """Versioned fixed opposite-lane route; all controls and safety limits unchanged."""
    return BrakeConfig(mode=TOWN01_OPPOSITE_MODE,map_name='Town01_Opt',road=8,lane=-1,
                       follower_s=135,end_s=247,planned_route_length=TOWN01_OPPOSITE_LENGTH_M)


def load_config(path):
    payload=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(payload,dict):raise ValueError('config must be an object')
    try:return BrakeConfig(**payload)
    except TypeError as exc:raise ValueError('unknown/invalid calibration config fields') from exc
