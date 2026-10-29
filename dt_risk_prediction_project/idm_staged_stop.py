"""Exploratory low-speed stop commands; no physics guarantees or relaxed gates."""
import math


class StagedStop:
    def __init__(self,initial_throttle,dt=.05):
        if not math.isfinite(initial_throttle) or not 0<=initial_throttle<=.100001 or dt!=.05:
            raise ValueError('invalid stop initialization')
        self.throttle=initial_throttle;self.brake=0.;self.dt=dt;self.stage='throttle_taper'

    def step(self,speed):
        if not math.isfinite(speed) or speed<0:raise ValueError('unknown stop speed')
        if self.stage=='throttle_taper' and speed<=.10:
            self.stage='brake_ramp';self.throttle=0.
        if self.stage=='throttle_taper':
            # Do not combine an abrupt throttle-to-zero transition and braking at
            # cruise speed. Tiny residual throttle is removed only below 0.10m/s.
            self.throttle=min(self.throttle,max(.001,self.throttle-.02*self.dt))
            return dict(stage=self.stage,throttle=self.throttle,brake=0.)
        self.throttle=0.;self.brake=min(.02,self.brake+.10*self.dt)
        if self.brake>=.02:self.stage='stop_hold'
        return dict(stage=self.stage,throttle=0.,brake=self.brake)


def stopped(row):
    return (row['speed_planar_mps']<.05 and abs(row['v_long_mps'])<.05 and abs(row['vz'])<.05
        and abs(row['a_long_mps2'])<=.5 and abs(row['throttle'])<=1e-5 and row['brake']>=.01999
        and row['hand_brake'] is False)
