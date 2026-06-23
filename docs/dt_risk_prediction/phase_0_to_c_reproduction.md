# Town10HD_Opt Digital Twin Risk Prediction Reproduction: Phase 0-C

## 1. Purpose

This note records a minimal reproduction of the previous research pipeline on CARLA's standard Town10HD_Opt map:

```text
vehicle trajectory capture
-> UDP replay into CARLA
-> future simulation with Autopilot from a selected payload frame
-> collision log output
```

The goal was to verify the execution foundation of the digital-twin-style accident prediction workflow using existing code, not to add new prediction logic.

## 2. Environment

| Item | Value |
|---|---|
| CARLA | 0.9.16 |
| CARLA Python API | 0.9.16 |
| Map | Town10HD_Opt |
| Traffic Manager port | 8000 |
| Fixed delta | 0.1 s |
| Execution style | Offline reproduction with one CARLA server |

The original traffic actors were recorded and cleaned up first. The recorded trajectory CSV was then replayed into the same map, and future simulation was run from the selected switch frame.

## 3. Existing Programs Used

| Process | Script |
|---|---|
| Traffic generation | `exp_future/denger_traffic.py` |
| Vehicle state recording | `scripts/vehicle_state_stream.py` |
| CSV conversion | `scripts/convert_vehicle_state_csv.py` |
| UDP sending | `send_data/send_udp_frames_from_csv.py` |
| Replay, Autopilot future simulation, collision detection | `scripts/udp_replay/replay_from_udp_future_exp.py` |

## 4. Execution Flow

```text
Generate 8 traffic vehicles on Town10HD_Opt
-> Record vehicle states to CSV
-> Clean up the original traffic actors
-> Convert to replay CSV
-> Send vehicle states over UDP
-> Reconstruct 8 actors in CARLA
-> Switch to Autopilot at payload frame 29904
-> Run 5 seconds of future simulation
-> Save collision log
-> Clean up replay actors and sensors
```

## 5. Phase Results

### Phase 0

- CARLA connection confirmed.
- Town10HD_Opt confirmed.
- Initial actor count: vehicle / walker / sensor = `0 / 0 / 0`.

### Phase A

- Traffic vehicles: `8`
- Walkers: `0`
- State CSV rows: `29,568`
- Frame range: `29804` to `33515`
- Unique vehicles: `8`
- All recorded frames contained 8 vehicles.
- Cleanup after recording succeeded.

### Phase B

- Replay ID source: source CSV `id` column.
- Replay IDs: `1` to `8`
- `START_FRAME`: `29854`
- `SWITCH_FRAME`: `29904`
- `SENDER_END_FRAME`: `29904`
- `future-duration-sec`: `5`
- `object_id` was empty in the source CSV, but the stable `id` column made replay possible.
- The replay CSV `type` value was normalized to `vehicle`; the replay script resolves this to CARLA vehicle blueprints.

### Phase C

- UDP frames sent: `51`
- Replayed vehicles: `8`
- Vehicles switched to Autopilot: `8`
- Future mode: `autopilot`
- Future simulation duration: `5.0` seconds
- Collision events: `0`
- Collision log was generated as a CSV with a header.
- Final actor count after cleanup: vehicle / walker / sensor = `0 / 0 / 0`.

## 6. Main Generated Logs

Only lightweight evidence files are included in this repository under `artifacts/`.

| File | Content |
|---|---|
| `actor.csv` | Vehicle states during tracking and Autopilot phases. Not committed because it is larger run output. |
| `id_map.csv` | Mapping between input vehicle IDs and CARLA actor IDs. Included as `artifacts/phase_c_id_map.csv`. |
| `collisions.csv` | Collision events. Included as `artifacts/phase_c_collisions.csv`. |
| `meta.json` | Experiment settings, switch frame, and future mode. Included as `artifacts/phase_c_meta.json`. |
| `control_state.json` | Final control state per actor. Not committed because it is a run artifact. |

## 7. Interpretation

The transition from tracking replay to Autopilot future simulation worked as intended. Collision count was `0`, which does not indicate a pipeline failure. This run used ordinary traffic and did not intentionally create a dangerous scenario.

This result confirms the execution foundation of the accident prediction workflow:

- Recorded trajectories can be replayed into CARLA.
- Actors can be reconstructed from UDP payloads.
- Control can switch to Autopilot at a selected payload frame.
- Future simulation and collision logging can run to completion.

Evaluating prediction performance still requires scenarios that produce near misses or collisions, plus comparisons across multiple switch timings and conditions.

## 8. Current Status and Next Work

Reached:

- Standard Town map reproduction using existing senior research code.
- Vehicle trajectory input, UDP replay, Autopilot future simulation, and collision log output confirmed.

Next:

- Run Phase D near miss post-processing.
- Design a scenario that produces dangerous proximity or collision events.
- Evaluate early detection across multiple switch timings.
- Later, connect predicted risk regions to OGM priority sharing logic.

## 9. Notes

- This is a method validation on CARLA Town10HD_Opt, not a complete PLATEAU-based urban digital twin reproduction.
- Raw CSVs, execution logs, model weights, CARLA binaries, archives, and virtual environments are intentionally excluded from the repository.
- Use and publication of senior research code should follow the lab's research-sharing rules.

## Evidence Artifacts

The included artifacts are small files copied from the Phase C run:

- `artifacts/phase_c_meta.json`
- `artifacts/phase_c_id_map.csv`
- `artifacts/phase_c_collisions.csv`
