# Model 3 exploratory low-speed protocol v1

This protocol does not certify or replace the strict Town01 scene audit. The
Pole/Sidewalks results remain BLOCKED; route_audit and physical_bounds stay null.
It requires an explicitly authorized, operator-attested research session and a
supervised single tick owner. Default CLI execution is offline only.

Fixed location: Town01_Opt road8/section0/lane-1, increasing s170 to230 (about60m).
Before spawning: exact native server identity/ports and OS resources, no observed
other clients, no vehicle/walker/sensor/controller actors, map/XML identity,
local waypoint continuity, five ground projections and twelve initial-volume
rays. Finite rays and absence of collision events are not complete geometry proof.

Trial1: target0.5m/s, cap0.7m/s, nominal3m drive, maximum12s drive.
Trial2: only after successful Trial1 and complete cleanup; target1.0m/s,
cap1.2m/s, nominal10m drive, maximum25s drive. Manual PI throttle cap0.10,
slew0.10/s, original gains. These exploratory trials use brake0.10, five-second
stop deadline, continuous stop0.5s. No TM, autopilot, velocity forcing or teleport.

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
