# Hardware evaluation 2026-10-17: Python cost baseline

The full Python closed-loop stack ran on the GO2 payload computer (Jetson
Orin NX) for 30 minutes. This is the reference the native C++ profile is
measured against ([`docs/NATIVE_RUNTIME.md`](../docs/NATIVE_RUNTIME.md)),
and the fifth DEV capture of detector round 2
([`docs/DETECTOR_EVAL.md`](../docs/DETECTOR_EVAL.md)).

## Conditions

- Window: 2026-10-17 19:26:58Z to 19:57:50Z. The bag covers 1800.8 s, and
  the process samples cover 1796 s after a 30 s warmup.
- Build: commit 3f648a8, clean tree.
- The robot was lying down. No motion commands were sent and no faults were
  injected.
- Ten lifecycle nodes, all `active` (`pybaseline/lifecycle_state.txt`):
  heartbeat_monitor, anomaly_detector, log_parser, topic_rate_monitor,
  json_state_parser, pose_drift_monitor, context_buffer, diagnosis_node,
  recovery_node and arbiter. The sport sink and llm_explainer also ran, as
  plain nodes.
- The sink was in `dry_run`: no `/api/sport/request` publisher existed.
  Every StopMove in `sink.log` reads `sent_to_robot=False`.
- topic_rate_monitor included `/utlidar/imu` (250 Hz), the same topic set as
  the native profile.
- RMW: CycloneDDS. The DDS config path was removed from `env.txt`.

## Results

Per-process mean CPU (100 % is one core) and RSS, from `procs.jsonl`, with
a 5 s period:

| Process | CPU | RSS |
|---|---|---|
| topic_rate_monitor | 36.55 % | 62.4 MB |
| arbiter | 7.61 % | 55.5 MB |
| heartbeat_monitor | 7.04 % | 38.2 MB |
| go2_sport_sink | 5.85 % | 55.3 MB |
| context_buffer | 3.79 % | 38.4 MB |
| pose_drift_monitor | 3.66 % | 55.4 MB |
| recovery_node | 3.64 % | 54.2 MB |
| anomaly_detector | 3.43 % | 38.3 MB |
| json_state_parser | 2.36 % | 39.6 MB |
| diagnosis_node | 2.30 % | 40.0 MB |
| log_parser | 1.98 % | 38.3 MB |
| llm_explainer | 0.88 % | 47.5 MB |
| `ros2 run` wrapper | 0.00 % | 24.2 MB |
| **Total** | **79.08 %** | **587.4 MB** |

Over the first 600 s the total CPU averages 79.1 %.

`/helix/faults` received 1 message during the run: a `pose/` ANOMALY, which
is not stop-worthy. `/helix/recovery_hints` and `/helix/recovery_actions`
stayed empty.

## Layout

- `pybaseline/procs.jsonl`: one sample per process per 5 s (pid, cpu_pct,
  rss_kb, threads, context switches), followed by a summary line.
  `cpu_pct` is null on the first sample of each process.
- `pybaseline/bag/`: rosbag2 (sqlite3). `/helix/metrics` 50415,
  `/helix/hold` 36018, `/helix/node_health` 1801 and `/helix/faults` 1
  messages, plus the other HELIX topics.
- `pybaseline/load.txt`: load average and thermal zone 0 every 30 s.
- `pybaseline/stack.log`, `sink.log`, `run.log`, `bag.log`, `lifecycle_state.txt`, `env.txt`.

Recorded with `scripts/hw/py_baseline.sh <build dir> 1800`, which uses
`scripts/hw/proc_sampler.py`. The DDS config path in the committed script is
the GO2 default (`~/unitree_ros2`); `env.txt` here omits it.

## Native run (same day)

The first 10-minute native run (`scripts/hw/native_run.sh`, build 6ace4d7,
about 20:31Z) was lost when the Orin dropped off the network.

`native_partial/` is the second native run:
- Build 1254bbe, 22:25:48Z, robot standing.
- Stopped after 175 s of samples to free the robot for the motion trials.
- 6 processes, 7.06 % of one core, 83.5 MB (per-PID means, same sampler).
- `bag/` holds the HELIX topics. It is the held-out capture for detector
  round 2 (`docs/DETECTOR_EVAL.md`).

## Armed stop test

`native_stop/` holds every armed run of `scripts/hw/native_stop_hw.sh`
(ack, then motion), passes and failures alike. Each run has:
- `evidence/evidence.json`: every check and the full event log.
- `bag/`: sport request/response, SportModeState, LiDAR odometry and the
  HELIX topics.
- the stack, run and bag logs.

Run 221913 failed before the stack came up, so its bag was dropped.
`scripts/hw/stop_bag_check.py <run>/bag` re-checks a bag without the
runner's recorder. Results: `docs/NATIVE_RUNTIME.md`.
