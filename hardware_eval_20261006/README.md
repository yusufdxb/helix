# Hardware evaluation 2026-10-06: loaded standing capture

One rosbag2 (sqlite3) capture of a running HELIX Tier-1 stack on a GO2 payload
computer (Jetson Orin NX), plus side evidence. Scored in
[`analysis/README.md`](../analysis/README.md) (section 7) by
[`analysis/score_capture.py`](../analysis/score_capture.py).

## Conditions

- Window: 2026-10-06 00:19:52Z to 00:38:16Z (bag duration 1103.2 s, 18.4 min).
  The planned length was 30 min; the operator ended it early.
- Robot standing still. No motion commands were sent. No faults were injected.
- HELIX Tier-1 only: heartbeat_monitor, anomaly_detector, log_parser,
  topic_rate_monitor, json_state_parser, pose_drift_monitor. No diagnosis,
  recovery or arbiter node was running.
- Build: commit b31a9ce (the arbiter branch). That the Tier-1 nodes behave
  identically to current main is an ASSUMPTION, not checked.
- Live detector parameters: the shipped `src/helix_bringup/config/helix_params.yaml`.
- topic_rate_monitor topics: `/utlidar/robot_odom`, `/utlidar/robot_pose`,
  `/utlidar/cloud`, `/gnss`, `/multiplestate`. `/utlidar/imu` was deliberately
  excluded to cut CPU.
- Not an idle machine: a separate voice-interaction stack (camera, speech and
  VLM client nodes) was running on the same computer, and a low-priority C++
  compile ran on it from about 00:34Z. The compile window is not separated in
  the analysis.
- Per-node CPU 1.4 to 3.6 %, RSS 39 to 56 MB.

## Layout

- `bags/loaded_standing_observe/`: `bag_0.db3`, `metadata.yaml`.
  Topics: `/helix/metrics` 28691, `/helix/faults` 1, `/helix/heartbeat` 55181,
  `/helix/node_health` 1103 messages.
- `notes/`: `start_utc.txt`, `end_utc.txt`, `lifecycle_state.txt`,
  `lifecycle.log` (all six nodes reached `active`), `cpu.txt` (periodic load and
  per-process CPU/RSS samples), `env.txt` (ROS domain and RMW only; the DDS
  config path was removed).
