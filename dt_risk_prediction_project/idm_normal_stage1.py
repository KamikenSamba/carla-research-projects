"""Normal-speed Stage 1 policy, independent of CARLA and the IDM estimator."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

from idm_harness_config import Scenario


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def stopping_distance(speed, delay=0.3, braking=2.0, acceleration=5.0):
    """Acceleration during reaction, then constant assumed braking; not a guarantee."""
    if not all(finite(v) for v in (speed, delay, braking, acceleration)) or min(speed, delay, acceleration) < 0 or braking <= 0:
        raise ValueError('invalid stopping assumptions')
    return speed*delay + acceleration*delay**2/2 + (speed+acceleration*delay)**2/(2*braking)


@dataclass(frozen=True)
class NormalStage1(Scenario):
    scenario_kind: str = 'normal_stage1'
    # No real route coordinates are shipped. A new, independently reviewed route is required.
    road: int | None = None
    section: int | None = None
    lane: int | None = None
    follower_s: float | None = None
    leader_s: float | None = None
    end_s: float | None = None
    route_audit: dict | None = None
    initialization: str = 'tm_from_rest'
    follower_speed: float = 4.5
    warmup_speed: float = 4.0
    following_distance: float = 12.0
    warmup_seconds: float = 8.0
    warmup_distance: float = 25.0
    ready_seconds: float = 2.0
    leader_speed_limit: float = 4.5
    follower_speed_limit: float = 5.0
    end_clearance: float = 20.0
    initial_spacing: float = 22.0
    planned_route_length: float = 200.0
    acceleration_peak: float = 5.0
    acceleration_sustained: float = 3.0
    ready_speed_tolerance: float = 0.2
    ready_acceleration: float = 0.5
    ready_gap_range: float = 1.2
    ready_relative_speed: float = 0.6
    reaction_seconds: float = 0.3
    assumed_braking: float = 2.0
    profile: tuple = ((0, 4.0), (15, 4.0))

    def __post_init__(self):
        # Validate all inherited thresholds through the original policy, without its 56m coordinates.
        Scenario(tm_port=self.tm_port, seed=self.seed, spawn_height=self.spawn_height,
                 minimum_speed=self.minimum_speed, stop_seconds=self.stop_seconds,
                 minimum_gap=self.minimum_gap, minimum_ttc=self.minimum_ttc,
                 nominal_front_extent=self.nominal_front_extent, lateral_limit=self.lateral_limit,
                 heading_limit=self.heading_limit, deviation_seconds=self.deviation_seconds,
                 leader_missing_seconds=self.leader_missing_seconds, invalid_seconds=self.invalid_seconds,
                 minimum_valid_ratio=self.minimum_valid_ratio, minimum_moving_seconds=self.minimum_moving_seconds)
        numeric_exceptions={'map_name','blueprint','scenario_kind','initialization','profile','route_audit',
                            'road','section','lane','follower_s','leader_s','end_s'}
        for key, value in asdict(self).items():
            if key not in numeric_exceptions and not finite(value):
                raise ValueError('invalid numeric setting: '+key)
            if isinstance(value, (int, float)) and not finite(value):
                raise ValueError('invalid numeric setting: '+key)
        fixed = dict(scenario_kind='normal_stage1', map_name='Town10HD_Opt', blueprint='vehicle.tesla.model3',
                     dt=0.05, duration=15.0, snapshot_at=10.0, follower_speed=4.5, warmup_speed=4.0,
                     warmup_seconds=8.0, warmup_distance=25.0, ready_seconds=2.0,
                     leader_speed_limit=4.5, follower_speed_limit=5.0, initial_spacing=22.0,
                     acceleration_peak=5.0, acceleration_sustained=3.0, ready_speed_tolerance=0.2,
                     ready_acceleration=0.5, ready_gap_range=1.2, ready_relative_speed=0.6,
                     reaction_seconds=0.3, assumed_braking=2.0, following_distance=12.0)
        if any(getattr(self,k) != v for k,v in fixed.items()) or self.profile != ((0,4.0),(15,4.0)):
            raise ValueError('outside reviewed normal Stage 1 policy; redesign requires review')
        baseline=Scenario()
        for key in ('lateral_limit','heading_limit','deviation_seconds','leader_missing_seconds',
                    'invalid_seconds','stop_seconds'):
            if getattr(self,key)>getattr(baseline,key):
                raise ValueError('existing safety threshold must not be relaxed: '+key)
        if (self.minimum_valid_ratio<baseline.minimum_valid_ratio or self.minimum_ttc<baseline.minimum_ttc
                or self.minimum_moving_seconds<baseline.minimum_moving_seconds):
            raise ValueError('existing quality/TTC threshold must not be relaxed')
        if self.nominal_front_extent<2.42511:
            raise ValueError('vehicle extent budget cannot be reduced')
        if self.end_clearance < 20 or self.planned_route_length < 200:
            raise ValueError('insufficient planned road/end margin')
        coordinates = (self.road,self.section,self.lane,self.follower_s,self.leader_s,self.end_s)
        if any(v is not None for v in coordinates):
            if not all(finite(v) for v in coordinates) or any(type(v) is not int for v in coordinates[:3]):
                raise ValueError('partial/invalid route coordinates')
            if any(v != int(v) for v in coordinates[3:]):
                raise ValueError('route s must be integer metre samples')
            if self.follower_s-self.leader_s != self.initial_spacing or self.follower_s-self.end_s < self.planned_route_length:
                raise ValueError('route spacing/length mismatch; only decreasing-s routes supported')
        if self.clearance_budget < self.end_clearance or self.initial_gap <= self.stop_requirement+self.minimum_gap:
            raise ValueError('insufficient distance budget')
        if self.route_audit is not None:
            self.require_executable('stage1')

    @property
    def initial_gap(self):
        return self.initial_spacing-2*self.nominal_front_extent

    @property
    def stop_requirement(self):
        return stopping_distance(self.follower_speed_limit,self.reaction_seconds,self.assumed_braking,self.acceleration_peak)

    @property
    def clearance_budget(self):
        leader=self.initial_spacing+self.warmup_distance+self.leader_speed_limit*self.duration
        follower=self.follower_speed_limit*(20*self.dt+self.warmup_seconds+self.duration)
        return self.planned_route_length-max(leader,follower)-self.nominal_front_extent

    @property
    def policy_sha256(self):
        values=asdict(self); values.pop('route_audit')
        return hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest()

    def distance_report(self):
        return dict(initial_bumper_gap_m=self.initial_gap, remaining_front_clearance_m=self.clearance_budget,
                    leader_travel_from_follower_start_m=self.initial_spacing+self.warmup_distance+self.leader_speed_limit*self.duration,
                    follower_travel_bound_m=self.follower_speed_limit*(20*self.dt+self.warmup_seconds+self.duration),
                    required_route_m=self.planned_route_length-self.clearance_budget+
                                     max(self.end_clearance,self.stop_requirement),
                    sensitivity=[dict(delay_s=t,braking_mps2=b,stopping_m=stopping_distance(5,t,b,5),
                                      gap_pass=stopping_distance(5,t,b,5)+self.minimum_gap<self.initial_gap,
                                      endpoint_pass=stopping_distance(5,t,b,5)<self.end_clearance)
                                 for t in (0.2,0.3,0.5) for b in (1.0,2.0,3.0)])

    def require_executable(self,stage):
        if stage != 'stage1' or self.initialization != 'tm_from_rest':
            raise ValueError('unverified stage/initialization is not executable')
        a=self.route_audit
        if not isinstance(a,dict) or self.follower_s is None:
            raise ValueError('normal Stage 1 route is unaudited; execution refused before connection')
        def sha(value):
            return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value)
        if (a.get('policy_sha256')!=self.policy_sha256 or not sha(a.get('opendrive_sha256'))
                or not sha(a.get('evidence_sha256')) or any(not isinstance(a.get(k),str) or not a[k].strip()
                       for k in ('reviewed_by','reviewed_at','evidence_reference')) or a.get('carla_version')!='0.9.16'
                or a.get('map_name')!=self.map_name):
            raise ValueError('missing/mismatched route audit provenance')
        # Human-reviewed physical bounds are prerequisites, not inferred from target speed or PID.
        physical=a.get('physical_bounds',{})
        if (not isinstance(physical,dict) or not finite(physical.get('braking_lower_bound_mps2')) or physical['braking_lower_bound_mps2']<self.assumed_braking
                or not finite(physical.get('response_upper_bound_s')) or physical['response_upper_bound_s']>self.reaction_seconds
                or physical['response_upper_bound_s']<0 or not physical.get('evidence_reference')):
            raise ValueError('unverified braking/response assumptions')
        points=a.get('points',[])
        if not isinstance(points,list) or len(points)!=int(self.follower_s-self.end_s)+1 or any(
                not isinstance(p,list) or len(p)!=3 or not all(finite(v) for v in p) for p in points):
            raise ValueError('audit requires every one-metre route point (x,y,z)')
        if any(not 0.95<=math.dist(p,q)<=1.05 or abs(p[2]-q[2])>0.02 for p,q in zip(points,points[1:])):
            raise ValueError('audit route point spacing')

    def speed_at(self,time,stage):
        if stage!='stage1':
            raise ValueError('normal speed Stage 2 is not implemented')
        return self.warmup_speed


class NormalSafety:
    """Read-only gates. No tick, CARLA RPC, or control mutation."""
    def __init__(self,config):
        self.c=config; self.counts={}; self.window=[]; self.last_frames={}

    def acceleration_reason(self,rows,frame):
        reason=None; actor=None
        for r in rows:
            a=r.get('a_long_mps2'); who=r['actor_id']
            if self.last_frames.get(who)==frame:
                continue
            contiguous=self.last_frames.get(who)==frame-1
            self.last_frames[who]=frame
            self.counts[who]=(self.counts.get(who,0) if contiguous else 0)
            if not finite(a):
                reason,actor='acceleration_unknown',who
            else:
                self.counts[who]=self.counts[who]+1 if abs(a)>self.c.acceleration_sustained else 0
                if abs(a)>self.c.acceleration_peak:
                    reason,actor='acceleration_peak',who
                elif self.counts[who]>=2 and reason is None:
                    reason,actor='acceleration_sustained',who
        return reason,actor

    def ready(self,rows,frame):
        f,l=rows; c=self.c
        good=(all(finite(r.get('a_long_mps2')) and abs(r['a_long_mps2'])<=c.ready_acceleration and
                  abs(r['speed_mps']-target)<=c.ready_speed_tolerance+1e-9
                  for r,target in zip(rows,(c.follower_speed,c.warmup_speed))) and
              f['leader_id']==l['source_actor_id'] and finite(f['gap_m']) and
              f['gap_m']>c.stop_requirement+c.minimum_gap and
              abs(f['relative_speed_mps'])<=c.ready_relative_speed and
              all(abs(r['heading_error_deg'])<=c.heading_limit and
                  r['route_lateral_m']<=c.lateral_limit for r in rows))
        identity=tuple((r['source_actor_id'],r['road_id'],r['section_id'],r['lane_id']) for r in rows)
        if (not good or (self.window and (frame!=self.window[-1][0]+1 or identity!=self.window[-1][2]))):
            self.window=[]
        if not good:
            return False
        self.window.append((frame,float(f['gap_m']),identity))
        # 41 snapshots span 2s at 0.05s. Any failed observation resets the entire window.
        while self.window and (frame-self.window[0][0])*c.dt>c.ready_seconds+1e-9:
            self.window.pop(0)
        if max(p[1] for p in self.window)-min(p[1] for p in self.window)>c.ready_gap_range+1e-9:
            self.window=[self.window[-1]]
        return (frame-self.window[0][0])*c.dt+1e-9>=c.ready_seconds
