# AegisLink

A simulated UAV-to-ground video and telemetry link that keeps working under electronic warfare:
bursty packet loss, bandwidth down to tens of kbps, total blackouts and GPS spoofing.

Three processes on one machine, connected over UDP:

- `uav_node` (Python): renders the onboard camera, detects and tracks targets, and sends a
  foveated stream (low-rate grayscale background, color target patches).
- `ew_chaos_proxy` (C++20): sits between the two and degrades the link on command.
- `gcs_node` (Python, PyQt6): the ground station. It rebuilds the picture, keeps predicting
  targets through blackouts with a Kalman filter, and cross-checks GPS against optical flow.

Work in progress.
