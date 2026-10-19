"""Fixed first brake calibration policy. No CARLA import, no empirical physical bounds."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path


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

    def distance_report(self):
        # Exploratory envelope, not a claimed physical lower bound. Stop if the observed envelope fails.
        v,tau,a,b=self.speed_limit,.5,self.acceleration_peak,.5
        distance=v*tau+a*tau*tau/2+(v+a*tau)**2/(2*b)
        required=self.launch_distance+self.front_extent_limit+distance+self.end_clearance
        return dict(assumptions_only=True,v_mps=v,tau_s=tau,a_mps2=a,b_mps2=b,
                    stopping_distance_m=distance,required_route_m=required,
                    margin_m=self.planned_route_length-required,accepted_braking_lower_bound=None,
                    accepted_response_upper_bound=None)

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
        if (not isinstance(points,list) or len(points)!=57 or any(not isinstance(p,list) or len(p)!=3
                or not all(finite(v) for v in p) for p in points)):
            raise ValueError('review requires all 57 one-metre geometry points')
        if any(not .95<=math.dist(p,q)<=1.05 or abs(p[2]-q[2])>.02 for p,q in zip(points,points[1:])):
            raise ValueError('invalid reviewed route spacing/grade')


def load_config(path):
    payload=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(payload,dict):raise ValueError('config must be an object')
    try:return BrakeConfig(**payload)
    except TypeError as exc:raise ValueError('unknown/invalid calibration config fields') from exc
