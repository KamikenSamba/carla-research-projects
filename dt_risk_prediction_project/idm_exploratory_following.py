"""Exploratory manual-control protocol. No physical guarantees or synthetic IDM data."""
import csv
from dataclasses import asdict, dataclass
import math

from idm_low_speed import ExploratoryConfig, ExploratoryHarness, config_for, local_pose_ok
from idm_brake_harness import LaunchController
from idm_brake_metrics import FIELDS
from idm_experiment_harness import EarlyStop, save
from idm_staged_stop import StagedStop, stopped


@dataclass(frozen=True)
class ResearchConfig(ExploratoryConfig):
    mode: str = 'exploratory_manual_following_v1'
    trial: str = 'stability'
    target_speed: float = 1.
    speed_limit: float = 1.2
    drive_distance: float = 18.
    drive_seconds: float = 25.
    maximum_frames: int = 700

    def __post_init__(self):
        expected = asdict(config_for('trial2'))
        expected.update(mode='exploratory_manual_following_v1', trial=self.trial,
                        drive_distance=18., drive_seconds=25., maximum_frames=700)
        if self.trial not in ('stability', 'following') or asdict(self) != expected:
            raise ValueError('outside fixed exploratory following design')


class ApproachPI:
    """Original launch PI, then a bumpless lower-gain PI near the target.

    Only controller feedback changes. Raw observations and acceleration gates are
    never filtered. Small gains address the observed frame-alternating inputs;
    this is a testable hypothesis, not a fitted CARLA physics model.
    """
    def __init__(self, config):
        self.c = config
        self.launch = LaunchController(config)
        self.fine = False
        self.throttle = 0.
        self.integral = 0.

    def step(self, speed, target=None):
        target = self.c.target_speed if target is None else target
        if not all(math.isfinite(x) for x in (speed, target)) or not 0 <= speed <= self.c.speed_limit:
            raise ValueError('invalid controller feedback')
        error = target - speed
        if not self.fine and speed < .7 * self.c.target_speed:
            self.throttle = self.launch.step(speed)
            return self.throttle
        kp, ki = .002, .01
        if not self.fine:
            self.integral = max(0., min(self.c.throttle_cap, self.throttle - kp * error))
            self.fine = True
        proposed = max(0., min(self.c.throttle_cap, self.integral + ki * error * self.c.dt))
        if not (kp * error + proposed > self.c.throttle_cap and error > 0):
            self.integral = proposed
        desired = max(0., min(self.c.throttle_cap, kp * error + self.integral))
        slew = self.c.throttle_slew_per_second * self.c.dt
        self.throttle += max(-slew, min(slew, desired - self.throttle))
        return self.throttle


def ready(row, target=1.):
    return (abs(row['speed_planar_mps'] - target) <= .1 + 1e-9
            and abs(row['a_long_mps2']) <= .5 and abs(row['vz']) < .05)


class StabilityHarness(ExploratoryHarness):
    def capture(self):
        self.controller = ApproachPI(self.c)
        with (self.directory / 'calibration_observations.csv').open('x', encoding='utf-8', newline='') as f:
            csv.DictWriter(f, FIELDS).writeheader()
        row = None
        for _ in range(40):
            row = self.step('settle', 0, 0, row)
        self.fail_if(not all(local_pose_ok(r) for r in self.rows[-10:]), 'initial_vehicle_not_stable')
        window = []
        for _ in range(round(self.c.drive_seconds / self.c.dt)):
            row = self.step('launch', self.controller.step(row['speed_planar_mps']), 0, row)
            self.fail_if(row['route_progress_m'] > self.c.drive_distance, 'stability_distance_limit')
            window = window + [row] if ready(row) else []
            if window and (row['frame'] - window[0]['frame']) * self.c.dt >= 2. - 1e-9:
                break
        else:
            raise EarlyStop('stability_not_ready', self.actor.id)
        save(self.directory / 'stability_result.json', dict(continuous_ready_s=2.,
             first_frame=window[0]['frame'], final_frame=row['frame'], rows=window,
             controller='manual original launch PI; near-target kp=.002 ki=.01 bumpless',
             raw_acceleration_gate_unchanged=True))
        self.stop_separated(row)

    def stop_separated(self, row):
        request = dict(frame=row['frame'], sim_time_s=row['sim_time_s'],
                       position=[row[k] for k in ('x', 'y', 'z')], speed=row['speed_planar_mps'])
        ctl = StagedStop(row['throttle'], self.c.dt)
        window, coast_frames, first_brake = [], 0, None
        for _ in range(round(self.c.brake_seconds / self.c.dt)):
            if (row['speed_planar_mps'] <= .1 or coast_frames) and coast_frames < 4:
                command = dict(throttle=0., brake=0., stage='zero_throttle_coast')
                coast_frames += 1
            else:
                if coast_frames:
                    self.fail_if(row['speed_planar_mps'] > .1, 'coast_speed_rebound')
                command = ctl.step(row['speed_planar_mps'])
            if command['brake'] > 0 and first_brake is None:
                first_brake = dict(frame=row['frame'], sim_time_s=row['sim_time_s'],
                    speed=row['speed_planar_mps'], position=[row[k] for k in ('x', 'y', 'z')])
            self.ledger('separated_stop_command', stage=command['stage'], control=command)
            row = self.step('braking', command['throttle'], command['brake'], row)
            self.fail_if(abs(row['throttle'] - command['throttle']) > 1e-5 or
                         abs(row['brake'] - command['brake']) > 1e-5, 'stop_control_not_reflected')
            window = window + [row] if stopped(row) else []
            if window and (row['frame'] - window[0]['frame']) * self.c.dt >= .5 - 1e-9:
                save(self.directory / 'separated_stop_result.json', dict(
                    stop_request=request, first_brake_request=first_brake,
                    zero_throttle_coast_frames=coast_frames,
                    stop_onset_frame=window[0]['frame'], stop_confirmed_frame=row['frame'],
                    stop_time_s=window[0]['sim_time_s']-request['sim_time_s'],
                    stop_distance_m=sum((window[0][k]-request['position'][i])*self.reverse_monitor.road[i]
                                        for i,k in enumerate(('x','y','z'))),
                    continuous_stop_s=.5, physical_bounds=None,
                    brake_entry_moving=bool(first_brake and first_brake['speed'] >= .05),
                    additional_brake_deceleration_identified=None,
                    calibration_valid=False))
                return
        raise EarlyStop('separated_stop_timeout', self.actor.id)


def following_gap_gate(gap, closing_speed):
    if not all(math.isfinite(x) for x in (gap, closing_speed)):
        return 'gap_unknown'
    if gap < 8.:
        return 'following_gap_below_8m'
    if closing_speed > 0 and gap / closing_speed < 5.:
        return 'following_ttc_below_5s'
    return None
