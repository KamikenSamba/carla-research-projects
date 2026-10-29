# Model 3 exploratory low-speed protocol v6

This protocol does not certify or replace the strict Town01 scene audit. The
Pole/Sidewalks results remain BLOCKED; route_audit and physical_bounds stay null.
It requires an explicitly authorized, operator-attested research session and a
supervised single tick owner. Default CLI execution is offline only.

Fixed location: Town01_Opt road8/section0/lane-1, increasing s170 to230 (about60m).
Spawn height0.10m: the prior0.40m initialization produced ordinary free fall
az=-9.8m/s² and vz=-2.45m/s before first ground contact. Lower initial placement
retains the |vz|2m/s gate; it does not change or teleport an existing actor.
Recorded actual wheel center/radius and road projections justify this clearance.
Initialization frames are excluded from controlled driving distance and calibration.
Select first gear once under zero throttle and brake0.10, observe it for one
initialization frame, then return to automatic gears. This responds to observed
gear0 with throttle0.10 and no longitudinal motion. Initialization is separately
logged; all subsequent measured frames retain the original automatic-gear gate.
Maximum measured frames exclude this one initialization frame.
Before spawning: exact native server identity/ports and OS resources, no observed
other clients, no vehicle/walker/sensor/controller actors, map/XML identity,
local waypoint continuity, five ground projections and twelve initial-volume
rays. OverlapChannel hits are not vehicle collision events. Untagged overlap hits
remain UNKNOWN and require empty corresponding SensorTrace blocking projections;
tagged obstacles or blocking projections stop the run. Finite rays and absence of
collision events are not complete geometry proof. This is the explicitly authorized
exploratory protocol, not a change to any strict scene gate.

Trial1: target0.5m/s, cap0.7m/s, nominal3m drive, maximum12s drive.
Trial2: only after successful Trial1 and complete cleanup; target1.0m/s,
cap1.2m/s, nominal10m drive, maximum25s drive. Manual PI throttle cap0.10,
slew0.02/s (stricter than the existing0.10/s cap), original gains. This bounded
change responds to the first launch frame reporting longitudinal velocity-0.0252m/s
at throttle0.005. Backward-motion and acceleration gates are unchanged; the
underlying transient is not claimed diagnosed. These exploratory trials use brake0.10, five-second
stop deadline, continuous stop0.5s. No TM, autopilot, velocity forcing or teleport.

Exploratory reverse monitoring uses signed displacement, not nonnegative path
length. An API longitudinal value below-0.02m/s starts a bounded observation:
three consecutive samples (0.15s) abort even if position progresses forward.
API velocity below-0.10m/s aborts immediately. A signed backward displacement
over1mm in one frame, or retreat over2mm from the greatest signed progress, aborts
independently of API sign. These are provisional operational values, not CARLA
physical guarantees. At the observed world coordinates float position resolution
is tens of micrometers; the displacement checks reject meaningful reverse motion
without summing quantization noise as travel. No speed/acceleration/collision/lane
or endpoint limit is relaxed. Legacy brake calibration keeps its original gate.

Each measured frame additionally records the same-frame ActorSnapshot transform,
velocity and acceleration, road/body directions, signed position differences,
actual dt, control/command frame and reverse-monitor decision. Mismatching snapshot
and actor state, missing snapshot or noncontiguous time/frame aborts. Reading the
snapshot does not generate another tick. A single saved negative endpoint velocity
and positive interval-average velocity do not establish the cause of the transient.

Calibration: only after Trial2 succeeds. Reuse the existing calibration capture,
two-second continuous readiness, brake0.70, response0.3s, continuous stop0.5s,
and all original acceleration/lane/endpoint/control gates. Label results as
calibration under an exploratory route, not a strict route-approved calibration.
Retain incomplete/aborted results; they are not valid calibration inputs.

All modes: collision sensor attached before any tick, immediately stop on an
observed collision; original peak5m/s² and sustained3m/s² two-frame gates;
lateral0.3m, heading5deg, vehicle bbox within lane, endpoint20m. Trial1 maximum400
frames; other modes700. RPC timeout5s; supervisor120s with bounded shutdown grace.
External OS monitor writes halt_rpc on server/monitor/resource failure. A poisoned
HealthGate suppresses cleanup RPC. Native abnormal exits may leave actors/settings
unknown and require separately assessed recovery, never blind retry.

Initialization/contact observations are recorded separately from braking. API
wheel-contact flags and measured friction are UNKNOWN. Log command-frame,
get_control frame, API/finite-difference acceleration, collision events and cleanup.
An event-free short trial does not prove high-speed or two-car safety.

CLI: Run_Exploratory_Model3.py --config <external JSON> is offline. The JSON contains
trial, exact scenario from config_for(trial), and a new session attestation.
Explicit --execute --server-pid <pid> --port <port> --output-root <new directory>
starts the existing bounded supervisor. Never reuse evidence directories.

Experiments and session-specific configuration belong outside Git. Keep the
source commit and per-file hashes with each run. Stop the sequence after any
abnormal trial, incomplete cleanup, missing observation or identity mismatch.
