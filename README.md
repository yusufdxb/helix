# HELIX

> Self-healing middleware for ROS 2 robots: detect faults, diagnose root cause,
> recover safely, explain what happened. Validated on a Unitree GO2 and Jetson
> Orin NX across eight hardware lab sessions.

[![CI](https://github.com/yusufdxb/helix/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/yusufdxb/helix/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.cppreference.com/w/cpp/17)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-blue)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## Demo

<p align="center">
  <a href="https://youtu.be/PbKXB91-NSY">
    <img src="https://img.youtube.com/vi/PbKXB91-NSY/maxresdefault.jpg" width="760" alt="HELIX demo: self-healing closed loop on a live GO2">
  </a>
</p>

A 90-second walkthrough of the four-tier closed loop (Sense, Diagnose, Recover,
Explain), replayed from the Session 8 hardware bag: 30 anomalies &rarr;
14 recovery hints &rarr; 14 recovery actions &rarr; 3,064 zero-twist
`/helix/cmd_vel` messages on a live Unitree GO2 and Jetson Orin NX.

---

## What HELIX Does

A robot fault rarely announces itself. A sensor's rate drifts, a topic goes
quiet, and the stack keeps driving on stale data until something downstream
fails. HELIX is a four-tier layer that watches the ROS 2 graph and closes the
loop on that failure mode.

1. **Sense.** Lifecycle nodes monitor the ROS 2 graph and emit structured
   `FaultEvent` messages: a rolling Z-score anomaly detector (Python reference
   plus a C++ port), a heartbeat monitor for topic-rate liveness, a regex log
   parser, and adapter nodes that bridge robot-specific telemetry to
   `/helix/metrics`.
2. **Diagnose.** A lifecycle node runs deterministic rules (R1 to R4) against
   the fault stream and context snapshots, publishing `RecoveryHint`
   suggestions. The pure-function rules are unit-testable without ROS 2.
3. **Recover.** A lifecycle node consumes hints, enforces a strict safety
   envelope (cooldown, an allowlist of `{STOP_AND_HOLD, RESUME, LOG_ONLY}`, and
   an enable flag), and is the only publisher of `cmd_vel`.
4. **Explain.** An advisory local LLM (Qwen2.5-1.5B-Instruct Q4_K_M via a
   llama-server sidecar) annotates events for operators with schema-constrained
   JSON output. It is never on the safety-critical path; the Recover allowlist
   is the hard gate.

Hot-path sensing nodes are being ported from Python to C++ so HELIX can coexist
on the Jetson with the robot's Nav2 and perception stack without RAM or CPU
pressure. Offline benchmarks (pure-Python ports of the detection logic) let you
evaluate algorithmic performance without a ROS 2 runtime.

## Architecture

```mermaid
graph LR
    GRAPH(["monitored ROS 2 graph"])
    SENSE["<b>SENSE</b><br/>helix_core<br/>helix_sensing_cpp (port)<br/>helix_adapter"]
    DIAGNOSE["<b>DIAGNOSE</b><br/>helix_diagnosis<br/>(rules R1-R4)"]
    RECOVER["<b>RECOVER</b><br/>helix_recovery<br/>(allowlist +<br/>safety env.)"]
    EXPLAIN["<b>EXPLAIN</b><br/>helix_explanation<br/>(local LLM,<br/>advisory)"]
    CMDVEL(["/cmd_vel + lifecycle"])

    GRAPH --> SENSE
    SENSE -- "/helix/faults" --> DIAGNOSE
    DIAGNOSE -- "/helix/recovery_hints" --> RECOVER
    RECOVER --> CMDVEL
    SENSE -- "/helix/faults" --> EXPLAIN

    classDef tier fill:#1f2937,stroke:#60a5fa,stroke-width:2px,color:#f9fafb;
    classDef io fill:#374151,stroke:#9ca3af,color:#f9fafb;
    class SENSE,DIAGNOSE,RECOVER,EXPLAIN tier;
    class GRAPH,CMDVEL io;
```

Full detail: [ARCHITECTURE.md](ARCHITECTURE.md). Design notes for the C++ port:
[`docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md`](docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md).

## Status

| Tier | State | Validation |
|---|---|---|
| **Sense** (`helix_core`, `helix_adapter`) | stable | Hardware-validated across 8 GO2 and Jetson lab sessions (2026-04-03 to 2026-04-23). |
| **Sense, C++ port** (`helix_sensing_cpp`) | work in progress | 30-min hardware parity run (2026-04-23): -56% RSS, -60% CPU vs Python, though 44% RSS missed the 30% design-doc target. Launch-gated (`use_cpp_anomaly=false`). |
| **Diagnose** (`helix_diagnosis`) | work in progress | 16/16 unit tests green on Jetson (live-schema fixtures). Closed-loop validated on a live GO2 in Session 8 after the R1 schema fix (branch `fix/r1-anomaly-schema-mismatch`, not yet on `main`). |
| **Recover** (`helix_recovery`) | work in progress | Validated end-to-end in Session 8: 14 hints consumed, allowlist and cooldown audited. Caveat: `/helix/cmd_vel` has 0 downstream subscribers, so STOP_AND_HOLD is currently a void publish (not yet wired to a twist_mux fallback). |
| **Explain** (`helix_explanation`) | work in progress | 26 unit tests green. Ships `llm_enabled=false`; Jetson llama-server deployment pending. |

Last stable release without the closed-loop stack:
[`v0.2.1`](https://github.com/yusufdxb/helix/releases/tag/v0.2.1). Current
self-healing work is tagged
[`v0.3.0-wip-self-healing`](https://github.com/yusufdxb/helix/releases/tag/v0.3.0-wip-self-healing).

## Packages

| Package | Language | Tier | Contents |
|---|---|---|---|
| `helix_msgs` | msg | shared | `FaultEvent`, `RecoveryHint`, `RecoveryAction`, `GetContext` srv |
| `helix_core` | Python | Sense | `anomaly_detector`, `heartbeat_monitor`, `log_parser` (reference implementation) |
| `helix_sensing_cpp` | C++ | Sense | C++ port of `anomaly_detector` (RollingStats kernel + LifecycleNode component). Launch-gated; Python stays default until hardware parity is re-confirmed. |
| `helix_adapter` | Python | Sense | Lifecycle nodes bridging robot-specific telemetry (topic-rate monitor, JSON state parser, pose drift) to `/helix/metrics` |
| `helix_diagnosis` | Python | Diagnose | `context_buffer`, `diagnosis_node` (IDLE / STOP_AND_HOLD state machine), pure-function `rules` |
| `helix_recovery` | Python | Recover | `recovery_node` with `SafetyEnvelope` (enable, cooldown, allowlist). Only publisher of `cmd_vel`. |
| `helix_explanation` | Python | Explain | `llm_explainer` and `llm_client`: llama-server sidecar client, `response_format: json_schema`, ThreadPoolExecutor, deterministic fallback. Advisory only. |
| `helix_bringup` | Python | Ops | Launch files, YAML config, `fault_injector` |

## Quick Start

### Build

```bash
mkdir -p ~/helix_ws/src
cd ~/helix_ws/src
git clone https://github.com/yusufdxb/helix.git
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# launch the sensing stack
ros2 launch helix_bringup helix_sensing.launch.py

# inject faults (separate terminal)
ros2 run helix_bringup fault_injector

# standalone benchmarks (no ROS 2 required)
python3 benchmark_helix.py
```

## Evaluation

Five benchmark suites evaluate the sensing components.

| Benchmark | Key Result | ROS 2? |
|-----------|-----------|--------|
| Algorithmic throughput | ~81K samples/sec (PC i7-7700), ~64K (Jetson Orin NX) | No |
| End-to-end ROS 2 latency | 1.16 ms mean (p95: 1.24 ms) | Yes |
| Realistic anomaly detection | 96.5% TPR at Z=3.0 with marginal anomalies; 0% TPR for 3-sigma in Laplace noise | No |
| Log parser accuracy | 22/22 correct, ~248K msg/sec throughput | No |
| GO2 attachability | 2/4 HELIX inputs natively available; 54 topics adaptable | No |
| Adapter-based detection | 4 real FaultEvents from a live GO2 LiDAR rate anomaly | Yes |

Full results, methodology, and caveats: [RESULTS.md](RESULTS.md). Known
limitations: [`docs/LIMITATIONS.md`](docs/LIMITATIONS.md).

## Testing

Unit tests exercise the `helix_core` nodes via `rclpy` in isolation. ROS 2
Humble is required.

```bash
cd ~/helix_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select helix_core
colcon test-result --verbose
```

Full details: [TESTING.md](TESTING.md).

## Reproducing the Closed-Loop Demo

The R1 schema fix the Session 8 closed loop depends on landed on `main` in
commit `82f7a15` (merged from `fix/r1-anomaly-schema-mismatch`), so the demo
reproduces from `main` via `ros2 launch helix_bringup helix_closedloop.launch.py`.
See
[`hardware_eval_20260423/results/closed_loop_demo.md`](hardware_eval_20260423/results/closed_loop_demo.md)
for the bag-level evidence (30 faults &rarr; 14 hints &rarr; 14 actions &rarr;
3,064 zero-twist commands in a 7m19s run).

An evaluator can otherwise reproduce, on a ROS 2 Humble environment
(the `ros:humble-ros-base` Docker image is a known-good setup):

1. **Build:** `colcon build`
2. **Unit tests:** `colcon test --packages-select helix_core`
3. **Standalone benchmarks:** `python3 benchmark_helix.py`,
   `python3 scripts/bench_realistic_anomalies.py`,
   `python3 scripts/bench_log_parser.py` (no ROS 2 required)
4. **End-to-end latency:** `python3 scripts/bench_e2e_latency.py`
5. **GO2 gap analysis:** `python3 scripts/go2_topic_gap_analysis.py` (no ROS 2 required)
6. **Live demo:** launch the sensing stack and inject faults in simulation

Result artifacts are stored in `results/`.

## Hardware Validation

Target platform: Unitree GO2 quadruped with an NVIDIA Jetson Orin NX 16 GB. The
sensing and recovery logic is platform-independent; adapter nodes isolate the
robot-specific telemetry.

Eight lab sessions (2026-04-03 to 2026-04-23) demonstrated:

- HELIX lifecycle nodes running persistently on the Jetson alongside the live GO2 stack
- A 30-minute persistent deployment with all success criteria green (Session 5)
- A 1-hour stability run with an RSS plateau, refuting earlier leak concerns (Session 7)
- IMU-excluded overhead: 6-node sum from 47.6% to 5.86% core CPU (-88%)
- Real `FaultEvent` detection from LiDAR rate anomalies on the GO2 via the adapter
- Ground-truth fault injection with ~1.8 s end-to-end detection latency
- Algorithmic benchmarks on the Jetson Orin NX (62-64K samples/sec)
- Session 8: end-to-end closed loop on a live GO2, 30 faults &rarr; 14 recovery
  hints (R1 STOP_AND_HOLD and R2 RESUME) &rarr; 14 audited actions (allowlist
  and cooldown); a 30-min C++ anomaly-detector parity run at -56% RSS and
  -60% CPU vs Python (RSS missed the 30% design target at 44%)

Full evidence, scope, and limitations: [`docs/GO2_HARDWARE_EVIDENCE.md`](docs/GO2_HARDWARE_EVIDENCE.md).

## Roadmap

HELIX ships as a public repo and demo video: a working self-healing system that
other roboticists can install and adapt. Three forward pillars:

1. **Close the recovery loop physically.** Wire `/helix/cmd_vel` through a
   `twist_mux` fallback so STOP_AND_HOLD reaches the robot, not a void publish.
2. **C/C++ on the hot paths.** Python is a RAM and latency liability on the
   Jetson at steady state. The anomaly detector port has landed; the heartbeat
   monitor, log parser, and adapter rate-monitor follow. Target: under 30% of
   the Python RSS baseline.
3. **Local LLM for heal, flag, and predict.** Qwen2.5-1.5B-Instruct Q4_K_M via
   a llama-server sidecar on the Jetson (~1.5 to 1.7 GB RSS), schema-constrained
   JSON output, advisory only. See
   [`docs/LLAMA_SERVER_JETSON_SETUP.md`](docs/LLAMA_SERVER_JETSON_SETUP.md) for
   the Jetson deployment runbook.

## Author

**Yusuf Guenena**
M.S. Robotics Engineering, Wayne State University
[linkedin.com/in/yusuf-guenena](https://linkedin.com/in/yusuf-guenena) &middot; [github.com/yusufdxb](https://github.com/yusufdxb)

## License

MIT. See [LICENSE](LICENSE).
