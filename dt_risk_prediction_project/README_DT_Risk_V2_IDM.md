# DT Risk Prediction V2: IDM / TM

V1 (`Run_DT_Risk_V1.py`, `dt_risk_common.py`) is unchanged. V2 adds a custom
IDM prediction controller and delegates `--prediction-controller tm` to V1's
prediction function. Both modes reconstruct the same snapshot, use the same
fixed delta and horizon, and reuse V1's collision callbacks and near-miss
detector. No new runtime dependencies are required.

## Architecture and current baseline

V1 captures position, rotation, blueprint, velocity, speed, source/logical IDs,
frame/time, role and an inferred autopilot flag in **CSV**, not snapshot JSON.
`config_used.json` and `summary.json` describe a run. Prediction spawns actors
with initial target velocity, ticks once to settle reconstruction, attaches
collision sensors, enables TM autopilot, then ticks for
`max(1, round(prediction_seconds / fixed_delta_seconds))` steps. Collision uses
CARLA collision sensors; near miss uses pairwise XY center distance, closing
speed and cooldown. The defaults remain 0.05 s, 8 s, 4 m and 1 m/s.

V2 prediction follows the same sequence. In IDM mode, autopilot is explicitly
disabled for each reconstructed vehicle; custom controls are computed from all
vehicle states **before** each tick. It never switches to TM when a leader is
absent. IDM mode does not change Traffic Manager settings. Owned sensors and
actors are destroyed before restoring the previous world settings, including
on exceptions or Ctrl+C. Sensors are tracked before `listen()` to cover callback
setup failures. World physics substeps are configured at 0.01 s and restored.
Use a dedicated prediction world for fair comparisons; without a world reload,
existing traffic remains present and can affect both risk and leader detection.
V1 capture and TM mode retain their original world/TM lifecycle behavior.

## Snapshot and trait inputs

`IDMActorState` extends V1's dataclass without changing V1's CSV columns:

| Optional field | Meaning | Unit |
|---|---|---|
| `desired_time_headway_s` | Desired following time headway T | s |
| `desired_speed_mps` | Desired free-flow speed v0 | m/s |

Old CSV snapshots and JSON actor objects without these fields load with `None`.
V2 accepts CSV, an array of actor objects, or
`{"schema_version": 2, "actors": [...]}`. JSON actor fields match the CSV fields.
V2 capture writes both `snapshot_states.csv` and `snapshot_states.json` with
resolved traits. Missing/duplicate logical IDs and nonfinite states are rejected.

Headway precedence: actor-traits JSON > snapshot T > `--default-idm-headway`
(default 1.5 s). Desired-speed precedence: actor-traits JSON > snapshot v0 >
`--idm-desired-speed-mps` > `max(snapshot speed, --idm-min-desired-speed-mps)`
(floor default 1 m/s). This last policy preserves observed speed as a conservative
proxy; it does not infer true free-flow speed. A stopped snapshot has no evidence
for v0, so select an explicit value for a study. Neither the 18 km/h follower nor
the 12 km/h leader prototype speed is imposed on prediction actors.

Actor-traits JSON keys are exact `logical_actor_id` values from the snapshot.
Unknown IDs cause an error before prediction connects to CARLA. For example:

```json
{
  "follower_short": {"desired_time_headway_s": 0.8, "desired_speed_mps": 5.0},
  "follower_normal": {"desired_time_headway_s": 1.5, "desired_speed_mps": 5.0},
  "follower_long": {"desired_time_headway_s": 2.5, "desired_speed_mps": 5.0}
}
```

Replace these IDs with the real logical IDs. Future offline estimates can be
written directly to `desired_time_headway_s` or this same sidecar JSON. Explicit
overrides and resolved per-actor parameters are retained in the run output.

## IDM and vehicle control

All internal calculations use meters and seconds:

```text
delta_v = v - v_leader
s_star = s0 + max(0, v*T + v*delta_v / (2*sqrt(a_max*b)))
a = a_max * [1 - (v/v0)^delta - (s_star/s)^2]
```

The interaction term is zero when there is no leader. `s` is bumper gap along
the lane path, accounting for vehicle bounding-box center offsets and lengths.
Overlap forces maximum braking; positive gaps are floored at 0.001 m in the
denominator. Acceleration is bounded to `[-max_decel, a_max]` and filtered with
`alpha*a + (1-alpha)*previous`. The bounded acceleration is filtered so a tiny
gap cannot leave an unbounded filter state. Overlap and route fallbacks bypass
the filter and command full brake. IDM bounds are requested accelerations,
not guarantees about the physical deceleration achieved by a CARLA vehicle.

| Parameter | CLI option | Default | Unit |
|---|---|---|---|
| T | `--default-idm-headway` | 1.5 | s |
| v0 | `--idm-desired-speed-mps` | snapshot policy above | m/s |
| s0 | `--idm-minimum-gap-m` | 2.0 | m |
| a_max | `--idm-max-accel-mps2` | 1.5 | m/s² |
| b (comfortable deceleration) | `--idm-comfortable-decel-mps2` | 2.0 | m/s² |
| delta | `--idm-acceleration-exponent` | 4.0 | dimensionless |
| maximum deceleration | `--idm-max-decel-mps2` | 4.0 | m/s² |

Target speed is `max(0, measured_speed + filtered_acceleration * dt)`.
Speed PID uses Kp=3.0, Ki=0.35, Kd=0.05 with an integral limit of 3.
Acceleration feed-forward divides positive acceleration by a_max and negative
acceleration by max_decel. The signed command becomes either throttle or brake,
bounded in [0,1], never both. Steering uses a lane route lookahead with a heading
error PID (Kp=1.25, Ki=0.02, Kd=0.12, integral limit 1, steering limit 0.75).

`--control-config control.json` accepts any `ControlConfig` field, for example:

```json
{
  "acceleration_filter_alpha": 0.30,
  "route_lookahead_m": 6.0,
  "leader_search_m": 80.0,
  "speed_kp": 3.0,
  "speed_ki": 0.35,
  "speed_kd": 0.05,
  "steering_kp": 1.25,
  "steering_ki": 0.02,
  "steering_kd": 0.12
}
```

## Leader detection and limitations

The adapter reads CARLA driving-lane waypoints without snapping off-lane actors
to another lane. A follower path traces unique `next(2 m)` waypoints up to the
configured search distance, stopping at junctions, forks, lane-key changes or
route ends. Pure geometry projects candidate bounding-box centers onto this
polyline, using arc progress rather than just Euclidean distance. Candidates
must have the same `(road_id, section_id, lane_id)`, lie ahead within the traced
path and within 45% of lane width, and face along the local path (dot product
>=0.5). The smallest bumper gap wins, with actor ID breaking ties. External
vehicles are considered but never controlled or destroyed.

This implementation intentionally covers unambiguous nonjunction lanes. Off-lane
poses, follower heading mismatch, junction occupancy and overlapping leaders
command full brake. An approaching route boundary commands brake once the
remaining path is within lookahead or the nominal stopping distance
`v²/(2*max_decel) + s0 + v*dt`. Every affected tick records a fallback reason;
`run.log` also records actor ID, simulation time and changes of reason.
Full junction traversal, lane changes, cross-road/section leader continuity,
traffic-light/sign compliance and saved turning-route intent are not modeled.
Unusual bounding-box rotations and very sharp curves can affect the approximate
bumper gap. Steering and deceleration need CARLA calibration before research
claims about collision avoidance. A server-free test does not validate physics.

## CLI examples

Run from `dt_risk_prediction_project/` (both modes use the same snapshot):

```powershell
python Run_DT_Risk_V2_IDM.py --mode predict --prediction-controller tm --snapshot snapshot_states.csv --prediction-seconds 8 --fixed-delta-seconds 0.05 --reload-world
python Run_DT_Risk_V2_IDM.py --mode predict --prediction-controller idm --snapshot snapshot_states.csv --prediction-seconds 8 --fixed-delta-seconds 0.05 --reload-world --default-idm-headway 1.5 --idm-desired-speed-mps 5 --actor-traits actor_traits.json
python Run_DT_Risk_V2_IDM.py --mode capture --duration 10 --snapshot-at 8
```

`--reload-world` reloads the prediction world; use it only on a dedicated server.
Do not use `--mode all` as a paired comparison: capture and prediction share the
same server. Capture once and predict twice from the saved snapshot with the
same reset world, seed, map, delta, horizon and risk thresholds.

## Outputs and verification

Runs use the same V1 output-root layout. In addition to baseline risk/state files:

- `input_snapshot.csv/.json`: normalized source input, preserving optional traits.
- `resolved_idm_parameters.json`: effective parameters for each logical actor.
- `idm_states.csv`: pre-tick time, actor/logical/leader IDs, measured speeds,
  bumper gap, relative speed, T, desired gap, IDM and filtered acceleration,
  target speed, throttle/brake/steer, road/section/lane, fallback reason.
- `reconstructed_states.csv`: includes resolved T and desired speed in IDM mode.
- `config_used.json`: controller, CLI and resolved control/trait configuration.

Absent leader measurements are empty CSV fields. `prediction_states.csv` is
post-tick; its IDM autopilot flag is explicitly false. Times in `idm_states.csv`
describe the state used to compute the control for the *next* tick. V1's inferred
autopilot flag is only a role-name heuristic and remains unchanged in TM mode.

```powershell
python -m compileall -q research_ogm_project dt_risk_prediction_project
python -m pytest -q dt_risk_prediction_project/tests
cd research_ogm_project
python -m pytest -q
```

The DT suite requires only pytest and no CARLA wheel or server. It tests IDM
limits, invalid inputs, headway-dependent following equilibrium, old snapshots,
trait precedence, curved lane geometry, exclusion of wrong leaders, controller
commands, fallback logs, TM delegation and mocked tick/cleanup failures. CI runs
this suite separately from the existing OGM suite. Real CARLA smoke validation
was not executed during initial implementation because no server was listening.

## Prototype investigation and next step

All available Git branches/history were searched; the historical headway branch
contains a progress report, not a tracked IDM implementation. Local excluded
reference scripts and the 20260718 experiment configuration confirmed the IDM
parameters, filter, PID gains and acceleration feed-forward. The prototype was
limited to a saved leader/follower route and braked when no leader was detected.
V2 implements independent author-maintained modules for the monorepo, retaining
the confirmed mathematical/control approach while adding free-road behavior and
general same-lane leader selection. No excluded scripts or datasets are imported,
copied into the repository or required at runtime.

For offline T estimation, first fix s0, a_max, b, delta and v0. Collect roughly
10–20 seconds of clean following observations `v(t), v_leader(t), gap(t)`;
exclude route fallbacks, signals, overlaps, saturated commands and very low
speeds. Start with equilibrium windows or a bounded T grid (e.g. 0.3–4 s),
then compare measured acceleration with IDM acceleration residuals. Differentiate
speed with explicit smoothing and account for the controller/filter: commanded
IDM acceleration differs from observed physical acceleration. Report fit error
and identifiability before persisting an estimate to the snapshot trait. This
is an offline design proposal, not an implemented estimator.

Reference: M. Treiber, A. Hennecke, and D. Helbing, “Congested Traffic States in
Empirical Observations and Microscopic Simulations,” *Physical Review E*,
Vol. 62, No. 2, pp. 1805–1824, 2000.
[DOI: 10.1103/PhysRevE.62.1805](https://doi.org/10.1103/PhysRevE.62.1805).
