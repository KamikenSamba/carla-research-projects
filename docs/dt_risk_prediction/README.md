# DT Risk Prediction Reproduction Notes

This directory records the progress of reproducing the core accident prediction pipeline from previous research code on CARLA Town10HD_Opt.

The reproduction uses the existing senior research scripts without adding new prediction logic. It covers:

- Recording vehicle states
- Converting recorded states to replay CSV
- Sending vehicle states over UDP
- Reconstructing CARLA actors from UDP payloads
- Running future simulation with CARLA Autopilot
- Writing collision logs

Out of scope for this run:

- PLATEAU custom maps
- Real LiDAR input
- Real-time synchronization across multiple CARLA servers
- LSTM comparison
- Near miss post-processing
- Connection to OGM communication control

See [phase_0_to_c_reproduction.md](phase_0_to_c_reproduction.md) for the detailed procedure, results, and limitations.
