# C++ Port Plan: HELIX Hot-Path Sensing Nodes

**Status:** Design only. No implementation in this document.
**Scope:** `AnomalyDetector`, `HeartbeatMonitor`, `LogParser`, `topic_rate_monitor`.
**Target package:** `src/helix_sensing_cpp/` (already exists, currently holds the AnomalyDetector port).
**Supersedes / extends:** `docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md` (single-node design, 2026-04-18). That
doc stays as the per-node reference for the detector. This one covers the remaining three nodes, the
ordering, the shared parity method, and the re-benchmark protocol.
**Date:** 2026-08-24

---

## 0. Baseline provenance (read this before quoting any number)

The stated project targets are "under 30% of the Session-7 RSS baseline" and "under 10% of one core".
Both are ratios against a Session-7 (2026-04-17) measurement. Here is exactly where that measurement
can and cannot be found.

### 0.1 What is citable in-tree

| Number | Value | Citation |
|---|---|---|
| Session-7 total RSS across 6 lifecycle nodes | 257 to 282 MB | `docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md:16` |
| Session-7 adapter CPU with `/utlidar/imu` in the topic list | ~48% of one core | `docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md:17` |
| Session-7 per-node RSS attributed to `helix_anomaly_detector` | ~45 MB | `docs/cpp_parity_summary.md` "Python baseline (S7)" row |
| Session-7 per-node CPU attributed to `helix_anomaly_detector` | ~2.0% | `docs/cpp_parity_summary.md` "CPU mean" row |
| Session-7 existence and role | 1-hour Jetson stability run, RSS plateau, "baseline used for the Session 8 C++ comparison" | `docs/LIMITATIONS.md:16` |

### 0.2 What is citable only in the author's private engineering notes

| Number | Value | Citation |
|---|---|---|
| Session-7 headline | "257-282 MB RSS, up to 48% core CPU with IMU topic" | private engineering notes, HELIX status (Session 7 entry) |
| Phase-1 60 s measurement on the canonical adapter path | "49% of one core, 257 MB RSS total" | private engineering notes, Phase 2 summary 2026-04-16 |
| Phase-2 1200 s measurement | "50% mean / 57% peak, 250 to 263 MB (+13 MB)" | same note, same block |
| Total RSS vs projection | "~257 MB vs <50 MB projected" | private engineering notes, session summary 2026-04-16 |

### 0.3 The gap, stated plainly

**There is no raw Session-7 result artifact in this repository.** Two things follow.

1. `docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md:8` and `docs/cpp_parity_summary.md` both cite
   `docs/GO2_HARDWARE_EVIDENCE.md` sections 15 to 17 as the Session-7 baseline. **That citation is
   dangling.** The in-tree `docs/GO2_HARDWARE_EVIDENCE.md` is 240 lines and its last numbered section
   is 11 (`### 11. Operator-safety incident`, line 226). Section 3 still carries the pre-Session-7
   projected estimate and the sentence "HELIX ROS 2 nodes have not been measured running on the
   Jetson" (`docs/GO2_HARDWARE_EVIDENCE.md:64`). The private daily engineering log for 2026-04-17 records that a
   Session-6/7 rewrite of this doc was written and pushed on branch `hardware-eval/2026-04-16`
   (private daily engineering log, 2026-04-17), so the content exists somewhere, but it is
   not on the current `main` working tree.
2. The nearest in-tree overhead JSONs are Session-1 vintage and measure a **different, smaller node
   set** (3 nodes, PC not Jetson): `results/helix_overhead_with_adapter.json` reports
   `rss_mb.mean = 41.7`, `cpu_percent.mean = 22.3` over 60 s with
   `nodes_running = [log_parser, anomaly_detector, heartbeat_monitor]`. Do **not** substitute this for
   the Session-7 baseline. It is a PC measurement of half the node set.

**Action item P0-A (blocking for any pass/fail claim):** recover the Session-7 artifact before the
first re-benchmark. Two candidate sources, in order:

- `git show hardware-eval/2026-04-16:docs/GO2_HARDWARE_EVIDENCE.md` (branch may only exist on origin;
  `git fetch origin hardware-eval/2026-04-16` first).
- The offline lab-archive drive, under
  `LABWORK/HELIX/hardware_eval_20260416_adapter_live/jetson_live_copy_phase3/results/`. That drive was
  **not mounted** when this plan was written, so nothing under it was read.

Until P0-A closes, every ratio in this document is computed against the narrative figures in 0.1 and
0.2 and must be labelled as such. ASSUMPTION: the ~45 MB / ~2.0% per-node figures in
`docs/cpp_parity_summary.md` are a per-process slice of the same Session-7 run that produced the
257 to 282 MB total. That is consistent (6 nodes x ~45 MB is ~270 MB) but it is an inference from two
narrative documents, not a figure read off a result file.

### 0.4 Derived per-node targets

Using the 0.1 figures, and flagging them as narrative-sourced:

| Target | Derivation | Value |
|---|---|---|
| Per-node RSS budget | 30% of ~45 MB | **< 13.5 MB** |
| Whole-stack RSS budget | 30% of 257 MB (low end of the Session-7 band) | **< 77 MB** for the six-node stack |
| Per-node CPU budget | stated directly | **< 10% of one core** |
| Adapter CPU budget | 10% of one core, against the ~48% Session-7 figure | **< 10%**, a 4.8x reduction |

---

## 1. What already exists (do not re-port it)

| Item | State | Evidence |
|---|---|---|
| `helix_sensing_cpp` package | Exists, builds, ships the AnomalyDetector port | `src/helix_sensing_cpp/CMakeLists.txt` |
| `AnomalyDetectorNode` (C++) | Implemented as an `rclcpp_lifecycle` component + standalone executable | `src/helix_sensing_cpp/src/anomaly_detector_node.cpp` |
| `RollingStats` math kernel | Implemented, header-only, gtested | `src/helix_sensing_cpp/include/helix_sensing_cpp/rolling_stats.hpp`, `test/test_rolling_stats.cpp` |
| Hardware parity measurement | 30 min on Jetson, Session 8 | `docs/cpp_parity_summary.md`, `results/cpp_parity.json` |
| Launch gate `use_cpp_anomaly` | Wired, defaults to `false` | `src/helix_sensing_cpp/launch/anomaly_detector.launch.py:28` |

Measured result for the one node already ported, quoted from `docs/cpp_parity_summary.md`:

| Metric | C++ measured | Baseline quoted in that doc | Verdict against target |
|---|---:|---:|---|
| RSS mean | 19.81 MB | 45.0 MB (S7) | **MISS.** 44% of baseline, target was < 30% (< 13.5 MB) |
| CPU mean | 0.80% | ~2.0% (S7) | **PASS** by 12x against the < 10% target |

`helix_adapter_cpp`, referenced as a "pattern reference, already uses rclcpp_lifecycle" in
`docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md:8`, **does not exist**. `src/` contains
`helix_adapter` (Python) and `helix_sensing_cpp` only. Any plan step that assumes an existing C++
adapter package is wrong.

---

## 2. Blocking defects found while scoping this plan

These are correctness issues in the existing C++ port. They gate promotion of `use_cpp_anomaly` and
they set constraints on the three remaining ports.

### 2.1 The C++ AnomalyDetector does not emit a heartbeat

`grep -c heartbeat src/helix_sensing_cpp/src/anomaly_detector_node.cpp` returns **0**. The Python node
constructs `Heartbeat(self)` (`src/helix_core/helix_core/anomaly_detector.py:37`), which publishes the
node's own name on `/helix/heartbeat` at 10 Hz while active
(`src/helix_core/helix_core/heartbeat.py`).

`HeartbeatMonitor` builds its registry only from nodes it has seen publish, and only reports CRASH for
a node already in the registry (documented in the module docstring of `heartbeat.py`). So flipping
`use_cpp_anomaly:=true` today silently removes the detector from crash detection. That regression is
exactly the class of defect commit `a350401` ("emit heartbeats so node death is actually detectable")
was landed to fix.

**Requirement for every C++ port:** a `helix_sensing_cpp::Heartbeat` helper mirroring
`helix_core.heartbeat.Heartbeat`, started in `on_activate` and stopped in `on_deactivate`. This is
port item zero, before HeartbeatMonitor itself.

### 2.2 Parameter divergence between the two AnomalyDetector implementations

| Parameter | Python | C++ |
|---|---|---|
| `zscore_threshold` | declared, default 3.0 | declared, default 3.0 |
| `consecutive_trigger` | declared, default 3 | declared, default 3 |
| `window_size` | declared, default 60 | declared, default 60 |
| `min_anomaly_duration_s` | declared, default 2.0 (`anomaly_detector.py:42`) | declared, default 2.0 (`anomaly_detector_node.cpp:70`) |
| `emit_cooldown_s` | **not declared** | declared, default **1.0** (`anomaly_detector_node.cpp:69`) |

`emit_cooldown_s: 1.0` is a live behavior difference, not a tuning knob: the C++ path suppresses
repeat emissions per metric for 1 s and the Python path does not. Any bag-replay parity run must set
`emit_cooldown_s: 0.0` to compare like with like, and any promotion decision must state whether the
cooldown is wanted in production (it interacts with the R2 RESUME clear window, see
`docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md` section 9 Q1).

### 2.3 Two `helix_params.yaml` files have drifted

`src/helix_sensing_cpp/config/helix_params.yaml` and `src/helix_bringup/config/helix_params.yaml`
differ: the bringup copy carries `helix_heartbeat_monitor` and `helix_log_parser` blocks and no
`emit_cooldown_s`; the sensing_cpp copy carries `emit_cooldown_s: 1.0` and neither of the other two
node blocks. Two files, one node name, different values for the same node depending on which launch
file wins.

**Henki convention, enforced from here on:** exactly one parameter file per deployment, owned by
`helix_bringup/config/`. Package-local config directories may ship an *example* file only, and it must
be named `*.example.yaml` so it can never be loaded by accident. Fold `emit_cooldown_s` into the
bringup file (declared on both implementations) and delete the duplicate as part of port item zero.

---

## 3. Per-node analysis

### 3.1 `AnomalyDetector` (ported, needs remediation)

**What it does.** Consumes `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) and `/helix/metrics`
(`std_msgs/Float64MultiArray`), maintains a per-metric rolling window, computes a population Z-score
per sample with evaluate-before-append, and publishes `helix_msgs/FaultEvent` on `/helix/faults` once
a violation streak clears `consecutive_trigger` and `min_anomaly_duration_s`.

**Why it is hot.** Every sample from every adapter metric passes through a Python callback. At the
Session-8 adapter cadence (`publish_period_sec: 0.5` over 6 topics,
`src/helix_bringup/config/helix_adapter_params.yaml`) that is 12 messages/s at steady state, which is
modest, but the node also carries a full CPython interpreter plus rclpy bindings as fixed RSS. The
cost is the floor, not the loop: the Z-score math itself was already 0.049 ms per sample in Python
(`docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md` section 7).

**C++ equivalent.** Done. `AnomalyDetectorNode : rclcpp_lifecycle::LifecycleNode`, with `RollingStats`
as a ROS-free kernel.

**rclcpp APIs in use.** `rclcpp_lifecycle::LifecycleNode`, `create_publisher` returning a
`LifecyclePublisher`, `create_subscription`, `declare_parameter<T>` / `get_parameter().as_double()`,
`rclcpp_components_register_node`.

**QoS.** Preserved exactly: `/helix/faults` `rclcpp::QoS(10).reliable()`, `/diagnostics`
`rclcpp::QoS(10).reliable()`, `/helix/metrics` `rclcpp::QoS(100).reliable()`
(`anomaly_detector_node.cpp:90-98`), matching the Python depths of 10, 10, 100 with default reliable
QoS.

**Lifecycle:** yes, already a lifecycle node on both sides.

**Remaining work:** heartbeat emitter (2.1), parameter reconciliation (2.2), single params file (2.3),
and the RSS gap (44% actual vs 30% target).

**RSS gap options**, in the order they should be tried:

1. Measure where the 19.8 MB goes before optimising. `pmap -x <pid>` and a `heaptrack` or
   `valgrind --tool=massif` run on the workstation build will separate rclcpp/rmw fixed cost from node
   allocations. ASSUMPTION: most of the 19.8 MB is the Cyclone DDS participant plus rclcpp runtime,
   which no amount of node-level work will shrink. That assumption is testable in one afternoon and
   must be tested before item 2 or 3 is attempted.
2. If the fixed cost dominates: host the four ported nodes in one `component_container` so they share a
   single DDS participant and one rclcpp runtime. Four nodes at ~19.8 MB standalone should collapse
   well below 4 x 19.8 MB. This is the highest-leverage move available and it is the reason all four
   nodes are registered as components.
3. Only if 1 and 2 leave it short: replace `std::deque<double>` per metric with a fixed-capacity ring
   buffer (`std::array` or a flat `std::vector` with head/tail), and reserve the metric map. Six
   metrics x 60 doubles is 2.8 KB, so this will not move the needle; it is listed to be explicitly
   deprioritised.
4. If 1 shows the target is unreachable standalone: **retune the target and say so**, rather than
   quietly missing it. A per-node target expressed against a containerised deployment is the honest
   version of the 30% goal.

### 3.2 `HeartbeatMonitor`

**Source:** `src/helix_core/helix_core/heartbeat_monitor.py`, 227 lines.

**What it does.** Subscribes `/helix/heartbeat` (`std_msgs/String`, depth 100), keeps a
`{node_name: (last_seen, miss_count)}` registry behind a `threading.Lock`, runs a check timer at
`check_interval_sec` (0.5 s) that flags a node crashed after `miss_threshold` (3) consecutive misses
past `heartbeat_timeout_sec` (0.3 s), publishes `FaultEvent` CRASH on `/helix/faults` (depth 10) with
one-shot dedup via a `_crashed_nodes` set, and publishes `/helix/node_health`
(`diagnostic_msgs/DiagnosticArray`, depth 10) every 1.0 s.

**Why it is hot.** It is the highest-message-rate node in `helix_core`. Every active HELIX lifecycle
node beats at 10 Hz (`heartbeat.py`, `DEFAULT_PERIOD_SEC = 0.1`). With six active nodes that is 60
String callbacks/s into a Python callback that takes a lock and touches a dict, plus 2 timer
callbacks/s. Message rate scales linearly with the number of monitored nodes, so it gets worse exactly
as the stack grows.

**C++ equivalent.**

```
helix_sensing_cpp::HeartbeatRegistry     // ROS-free kernel
  void record(const std::string& name, double now);
  struct Verdict { std::vector<std::string> newly_crashed;
                   std::vector<std::string> recovered;
                   std::vector<Entry> snapshot; };
  Verdict check(double now, double timeout_s, int miss_threshold);

helix_sensing_cpp::HeartbeatMonitorNode : rclcpp_lifecycle::LifecycleNode
```

Registry as `std::unordered_map<std::string, Entry>` with `Entry { double last_seen; int misses; bool
crash_reported; }`. `std::string_view` on the callback path to avoid a copy until the map lookup
misses.

**rclcpp APIs.** `create_subscription<std_msgs::msg::String>`, two `create_wall_timer` calls created in
`on_activate` and `.reset()` in `on_deactivate` (the rclcpp analogue of rclpy `timer.cancel()`),
`LifecyclePublisher` for both `/helix/faults` and `/helix/node_health`.

**QoS preserved.** `/helix/heartbeat` `rclcpp::QoS(100).reliable()`, `/helix/faults`
`rclcpp::QoS(10).reliable()`, `/helix/node_health` `rclcpp::QoS(10).reliable()`.

Open question worth deciding at port time, not silently: a 10 Hz liveness stream is the textbook case
for `BEST_EFFORT`, and the repo already has a DDS-liveliness path
(`src/helix_core/helix_core/liveliness_monitor.py`, landed in `458e295`, default off per
`docs/POSITIONING.md`). Changing reliability changes detection semantics under load, so **the port
preserves RELIABLE** and any change is a separate, separately-measured commit.

**Lifecycle:** yes. It already is one, and it owns timers, which is the canonical reason to be one.

**Parameters (YAML, unchanged names/types):** `heartbeat_timeout_sec` (double 0.3), `miss_threshold`
(int 3), `check_interval_sec` (double 0.5). `HEALTH_PUBLISH_INTERVAL_SEC = 1.0` is currently a Python
module constant, not a parameter (`heartbeat_monitor.py:27`). Promote it to
`health_publish_interval_sec` in YAML during the port and add the same declaration to the Python node
so the two stay comparable.

**Port risk.** The `threading.Lock` is real: `_on_heartbeat` and `_check_heartbeats` genuinely race
under a multi-threaded executor. In C++ under the default `SingleThreadedExecutor` the lock is
unnecessary, but if the node ever lands in a `MultiThreadedExecutor` component container (see 3.1
option 2) it becomes necessary again. Ship a `std::mutex` in the registry from day one; an uncontended
mutex is a few nanoseconds and it removes a whole class of future bug.

### 3.3 `LogParser`

**Source:** `src/helix_core/helix_core/log_parser.py`, 191 lines.

**What it does.** Subscribes `/rosout` (`rcl_interfaces/msg/Log`, depth 100), drops anything below
level 40 (ERROR), runs each surviving message against a list of case-insensitive compiled regexes
loaded from a YAML rules file at configure time, deduplicates on `(rule_id, node_name)` within
`dedup_window_sec` (5.0 s), and emits `FaultEvent` on `/helix/faults`.

**Why it is hot.** `/rosout` is the single highest-volume topic in any ROS 2 system, and it is
adversarial: volume spikes exactly when something is going wrong, which is when HELIX must not fall
over. Every message crosses the rclpy C-to-Python boundary and allocates a Python `Log` object before
the severity filter can discard it. The benchmark numbers in `README.md:30` (777K msg/s PC, 156K msg/s
Jetson) measure the *pure-Python parsing kernel*, not the ROS callback path, so they overstate what
the node can absorb.

**C++ equivalent.**

```
helix_sensing_cpp::LogRuleSet            // ROS-free: rules + regex + dedup cache
  static LogRuleSet from_yaml(const std::string& path);   // throws on malformed
  std::optional<Match> match(int level, std::string_view msg,
                             std::string_view node_name, double now);
helix_sensing_cpp::LogParserNode : rclcpp_lifecycle::LifecycleNode
```

Rules as `std::vector<Rule>` with `Rule { std::string id, fault_type, detail_template; int severity;
std::regex compiled; }`. Dedup as `std::unordered_map<std::pair-key, double>`; key the map on a
concatenated `id + '\0' + node_name` string to avoid writing a pair hasher.

`std::regex` is notoriously slow. The rule set is small and the severity filter runs first, so it is
almost certainly fine, but this is the one node where the C++ version could plausibly be *slower* per
message than Python's `re` module (which is a C implementation). **Measure before assuming a win**:
benchmark `std::regex` against `re` on the real `log_rules.yaml`
(`src/helix_bringup/config/log_rules.yaml`). If `std::regex` loses, swap to RE2 (`libre2-dev`,
`rosdep` key `re2`) which is both faster and immune to catastrophic backtracking. Note this as a
decision point, not a foregone conclusion.

**rclcpp APIs.** `create_subscription<rcl_interfaces::msg::Log>`, `LifecyclePublisher<FaultEvent>`,
`rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds()` to match Python's `time.time()` in `msg.timestamp`.

**QoS preserved.** `/rosout` `rclcpp::QoS(100).reliable()` to match the Python depth-100 default.
Note for the implementer: `/rosout` is published by `rcl_logging` with its own QoS
(`TRANSIENT_LOCAL`, depth 1000 on most distros). A `VOLATILE` subscriber is compatible with a
`TRANSIENT_LOCAL` publisher, so the current Python subscription works; the C++ port must not
"helpfully" request `TRANSIENT_LOCAL` on the subscriber, or it will replay the entire durability
backlog into the dedup cache on every activate.

**Logging.** `_on_log` currently calls `get_logger().debug(...)` on every suppressed duplicate
(`log_parser.py:139`), and `get_logger().info(...)` on every emit. Logging from a `/rosout` handler is
a feedback loop: an INFO line published to `/rosout` comes straight back into the subscription. It is
harmless today only because the severity filter drops it at level 20. The C++ port must use
`RCLCPP_DEBUG_THROTTLE` / `RCLCPP_INFO_THROTTLE` with a 1000 ms period, and must never log at
`>= ERROR` from inside `_on_log`.

**Lifecycle:** yes. It loads a file in `on_configure` and can legitimately fail that transition
(`log_parser.py:69` returns `FAILURE`), which is the strongest possible argument for a lifecycle node.

**Parameters (YAML):** `dedup_window_sec` (double 5.0), `rules_file_path` (string, currently injected
programmatically by the launch file, per the comment in `helix_params.yaml`). Keep both, keep the
launch-time injection of the path so the rules file can be found via `get_package_share_directory`.

**Dependency added:** `yaml-cpp` (`<depend>yaml-cpp</depend>`, `find_package(yaml-cpp REQUIRED)`).
This is the only port that adds a third-party dependency.

### 3.4 `topic_rate_monitor`

**Source:** `src/helix_adapter/helix_adapter/topic_rate_monitor.py`, 164 lines, plus the ROS-free
`rate_window.py`, 60 lines.

**What it does.** Subscribes to each configured GO2 topic with a type looked up from `_TYPE_MAP`,
records a callback-arrival timestamp per message into a `RateWindow`, and every
`publish_period_sec` (0.5 s) publishes `rate_hz/<sanitised_topic>` as a labelled
`Float64MultiArray` on `/helix/metrics`. Stale topics emit NaN.

**Why it is hot: this is the real prize.** It is the node the Session-7 note blames for the ~48% core
CPU figure. Look at what its subscriptions carry: `/utlidar/imu` and `/utlidar/robot_odom`
(`helix_adapter_params.yaml`). The private notes record the aggregate as ~440 Hz, dominated by IMU at
250 Hz and odom at 148 Hz (session summary 2026-04-16). Every one of
those messages is fully deserialised into a Python `sensor_msgs/Imu` or `nav_msgs/Odometry` object,
dispatched through a lambda, and then **thrown away**: the callback body is
`self._windows[t].record()`. The node pays full deserialisation cost for data it never reads.

Two independent wins are available here, and they should not be conflated:

1. **The port.** C++ deserialisation of an `Imu` at 250 Hz is close to free compared to rclpy.
2. **Not deserialising at all.** `rclcpp::GenericSubscription` (`create_generic_subscription`)
   delivers a `rclcpp::SerializedMessage` without ever calling the typesupport deserialiser. Since the
   node only needs *arrival times*, this is the correct shape: one generic subscription per topic,
   type name taken from the YAML, `_TYPE_MAP` deleted entirely. That also removes the
   `Unknown topic type for ...; skipping` failure mode at `topic_rate_monitor.py:88`.

Do the port with typed subscriptions first (straight parity), then measure, then switch to
`GenericSubscription` as a second, separately-measured commit. Two changes at once means neither
number means anything.

**C++ equivalent.**

```
helix_adapter_cpp::RateWindow            // ROS-free, direct port of rate_window.py
  void record(double now_monotonic);
  double rate_or_nan(double now_monotonic) const;   // (n-1)/span, NaN if empty, 0.0 if n==1
helix_adapter_cpp::TopicRateMonitorNode : rclcpp_lifecycle::LifecycleNode
```

**Package placement.** New package `src/helix_adapter_cpp/`, not `helix_sensing_cpp`. The adapter tier
is a distinct architectural layer (it is the thing that knows GO2 topic names), and
`docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md` section 3.1 already made this call. Note that this package
is referenced as existing in that document but does not exist yet; creating it closes that
inconsistency.

**rclcpp APIs.** `create_generic_subscription` (phase 2) or `create_subscription<T>` (phase 1),
`create_wall_timer`, `LifecyclePublisher<std_msgs::msg::Float64MultiArray>`, `std::chrono::steady_clock`
for the window (matching Python's `time.monotonic`, **not** `now()`, which is ROS time and would break
under `use_sim_time`).

**QoS preserved, and this one matters.** The Python node applies best-effort to sensor types only:

```
_BEST_EFFORT_TYPES = {Imu, PointCloud2}   # topic_rate_monitor.py:50
```

producing `QoSProfile(depth=10, reliability=BEST_EFFORT, durability=VOLATILE)` for `/utlidar/imu` and
`/utlidar/cloud`, and plain depth-10 reliable for `/utlidar/robot_odom`, `/utlidar/robot_pose`,
`/gnss`, `/multiplestate`. A reliable subscriber against a best-effort publisher gets **no data at
all** in DDS, which for this node means a permanent NaN and a false stale-topic ANOMALY. Getting this
wrong is a silent, plausible-looking failure. The C++ port must carry the same per-topic mapping, and
the parity suite must assert it (test P4 below). With `GenericSubscription` the mapping moves into the
YAML as an explicit per-topic `reliability` field, which is the better long-term shape.

`/helix/metrics` publisher stays `rclcpp::QoS(10).reliable()`.

**Lifecycle:** yes, and it needs `on_shutdown` too (the Python node defines one at
`topic_rate_monitor.py:131`, delegating to `on_cleanup`).

**Parameters (YAML, unchanged):** `window_sec` (double 5.0), `publish_period_sec` (double 0.5),
`topics` (string array), `sim_mode` (bool false). `sim_mode` rewrites `/utlidar/cloud` to
`/utlidar/cloud_throttled`; preserve that rewrite exactly, it is what the Isaac Sim fault-injection
harness depends on.

---

## 4. Port order

| # | Item | Why here |
|---|---|---|
| 0 | **Remediation of the existing port**: heartbeat emitter, `emit_cooldown_s` reconciliation, single params file, RSS profiling (3.1 option 1) | Ports 1 to 3 copy whatever pattern the detector establishes. Copying a heartbeat-less node three more times turns one regression into four. The profiling result also decides whether the 30% target is reachable at all, which changes the acceptance criteria for everything after. Cheapest item, gates the rest. |
| 1 | **`topic_rate_monitor`** into a new `helix_adapter_cpp` | Largest measurable win by a wide margin: it is the ~48% CPU node, and it is the only one whose load (440 Hz) is set by the robot rather than by HELIX. Also the only one where a second, larger win (`GenericSubscription`) exists. Doing it first means the re-benchmark after port 1 produces a headline number instead of noise. |
| 2 | **`HeartbeatMonitor`** | Second-highest message rate (10 Hz x N nodes), and it is the node the whole crash-detection path depends on, so it benefits most from the tighter timer scheduling of a C++ executor. It is also structurally the closest sibling of the already-ported detector (registry + timer + FaultEvent), so it is the cheapest of the remaining three. |
| 3 | **`LogParser`** | Last, deliberately. It is the only port with an unresolved technical question (`std::regex` vs RE2), the only one adding a third-party dependency (`yaml-cpp`), and the only one whose steady-state load is near zero: `/rosout` at idle carries almost nothing, and the severity-40 filter discards most of what does arrive. Its value is *tail* robustness under a log storm, which is real but not a steady-state RSS or CPU win. Doing it last means its dependency churn cannot block the two ports that carry the actual measurements. |
| 4 | **Component-container consolidation** | Only meaningful once there are four C++ nodes to co-host. Expected to be where the per-node RSS target is actually met (3.1 option 2). |

Deliberately **not** in scope: `json_state_parser`, `pose_drift_monitor`, `liveliness_monitor`,
`liveliness_beacon`, and the whole diagnosis / recovery / explanation tier. They are low-rate or
off-hot-path, and porting them buys ops complexity rather than headroom.

---

## 5. Behavioral parity test strategy

Every port passes the same four tiers before its `use_cpp_*` flag can be flipped. Tiers 1 to 3 run in
CI on the workstation; tier 4 needs the robot.

### Tier 1: golden-vector unit tests on the ROS-free kernel

For each kernel (`RollingStats`, `HeartbeatRegistry`, `LogRuleSet`, `RateWindow`) a small Python
script drives the **existing Python implementation** over a fixed, seeded input stream and dumps a CSV
of `(input, output)` tuples into `src/<pkg>/test/fixtures/<kernel>_golden.csv`. A gtest replays the
same stream through the C++ kernel and asserts equality: exact for integers, booleans and strings;
`1e-12` absolute / `1e-9` relative for doubles.

This is the tier that catches real divergence, because it is the only one where the Python
implementation is the oracle rather than a second opinion. Generation scripts live in
`scripts/gen_golden/` and are re-runnable, so a deliberate Python behavior change regenerates the
fixture in the same commit that changes it.

Per-kernel streams:

| Kernel | Stream must include |
|---|---|
| `RollingStats` | noisy baseline, sustained spike, flat region (std < 1e-6), window-fill boundary, first-sample-no-decision, counter reset, counter persistence after emit |
| `HeartbeatRegistry` | steady beats, one node going silent past the threshold, that node recovering, a never-before-seen node appearing, exactly-at-threshold timing |
| `LogRuleSet` | level < 40 dropped, single match, multiple rules matching one message, dedup inside the window, dedup expiring, case-insensitivity, non-matching message |
| `RateWindow` | zero samples (NaN), one sample (0.0), steady rate, eviction at the window edge, span < 1e-6 (0.0), rate change across the window |

### Tier 2: node-level gtests

Port each existing pytest one-for-one, then add the cases the Python suite never had. The Python side
currently has `src/helix_core/test/test_anomaly_detector.py`, `test_heartbeat_monitor.py`,
`test_log_parser.py` and `src/helix_adapter/test/test_topic_rate_monitor.py`; treat each as the
minimum, not the target.

Mandatory additions that no current test covers:

- **P1 Heartbeat emission.** Assert the C++ node publishes its own name on `/helix/heartbeat` while
  active and stops when deactivated. Add the identical test on the Python side. This is defect 2.1
  turned into a regression test.
- **P2 Parameter binding.** Load `helix_bringup/config/helix_params.yaml` into the node under test and
  assert every parameter reads the YAML value, not the code default. Catches type mismatches such as
  `window_size: 60.0` failing a `declare_parameter<int>`.
- **P3 Lifecycle round-trip.** configure, activate, deactivate, cleanup, configure, activate. Assert
  no leaked timers or subscriptions and that internal state is empty after cleanup.
- **P4 QoS assertion.** For `topic_rate_monitor`, publish on `/utlidar/imu` from a `BEST_EFFORT`
  publisher and assert the rate metric goes non-NaN. Repeat with a `RELIABLE` publisher on
  `/utlidar/robot_odom`. This is the test that would catch the silent-NaN failure in 3.4.
- **P5 Throttled logging.** Assert no unthrottled log call exists on any subscription callback path.
  Enforced as a `grep`-based lint in `tests/test_docs_consistency.py` style rather than a runtime test.

### Tier 3: side-by-side bag replay (the actual parity gate)

For each ported node, run the Python and C++ implementations **simultaneously** in separate namespaces
(`/py/...` and `/cpp/...`) against one `ros2 bag play` of a real Jetson capture, and diff the output
streams.

- Input bags: the evidence bags under `hardware_eval_*/`. Prefer a bag containing real faults
  (Session 8 `post_fix_demo`) over an idle one, and run an idle bag as well so the false-positive rate
  is compared too.
- Compare on a canonical projection, not on bytes: for `FaultEvent`, the tuple
  `(node_name, fault_type, severity, sorted(zip(context_keys, context_values)))` and `timestamp`
  within 100 ms. For `/helix/metrics`, `(label, value)` with value within `1e-9`.
- Config for the run: `emit_cooldown_s: 0.0` so the C++ cooldown does not manufacture a diff (2.2).
- Acceptance: identical event count, identical ordering by metric, all values inside tolerance.
- Output: a committed report under `results/parity_<node>_<date>.json` plus a one-page summary in
  `docs/`, following the shape of the existing `docs/cpp_parity_summary.md`.
- A new script `scripts/compare_parity_streams.py` does the diff. It must be deterministic and it must
  fail loudly on an empty stream, so that "both produced nothing" cannot be reported as parity.

### Tier 4: hardware A/B

One CaresLab session per port, or one session covering several ports. Two separate cold boots of the
Jetson, since the node names collide and cannot run concurrently. Same GO2 state, same duration
(30 min minimum, matching the Session-8 C++ parity run), Python first then C++.

---

## 6. Build-system changes

### 6.1 New package `src/helix_adapter_cpp/`

`package.xml`:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>rclcpp_components</depend>
<depend>lifecycle_msgs</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>helix_msgs</depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
<test_depend>ament_cmake_gtest</test_depend>
```

`sensor_msgs` / `nav_msgs` / `geometry_msgs` are needed for phase-1 typed subscriptions and can be
dropped once phase 2 moves to `GenericSubscription`.

`CMakeLists.txt` follows the existing `helix_sensing_cpp` file exactly: a `SHARED` component library
per node, `rclcpp_components_register_node` to get executable and component from one declaration,
`install(DIRECTORY launch config ... OPTIONAL)`, and gtests under `if(BUILD_TESTING)`.

### 6.2 Changes to `src/helix_sensing_cpp/`

`package.xml` gains, for the two new nodes:

```xml
<depend>rcl_interfaces</depend>   <!-- LogParser: rcl_interfaces/msg/Log -->
<depend>yaml-cpp</depend>         <!-- LogParser: rules file -->
```

`CMakeLists.txt` gains, mirroring the existing detector block:

```cmake
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(heartbeat_monitor_component SHARED src/heartbeat_monitor_node.cpp)
# ... target_include_directories / ament_target_dependencies as for the detector
rclcpp_components_register_node(heartbeat_monitor_component
  PLUGIN "helix_sensing_cpp::HeartbeatMonitorNode"
  EXECUTABLE helix_heartbeat_monitor)

add_library(log_parser_component SHARED src/log_parser_node.cpp)
target_link_libraries(log_parser_component yaml-cpp)
rclcpp_components_register_node(log_parser_component
  PLUGIN "helix_sensing_cpp::LogParserNode"
  EXECUTABLE helix_log_parser)
```

plus one `ament_add_gtest` per kernel and per node, and

```cmake
install(DIRECTORY test/fixtures DESTINATION share/${PROJECT_NAME}/test OPTIONAL)
```

so golden CSVs are reachable from an installed test.

**Executable names stay identical to the Python console scripts** (`helix_heartbeat_monitor`,
`helix_log_parser`, `helix_topic_rate_monitor`). Node names stay identical too, so
`helix_params.yaml` keys keep binding and the swap is a `package=` change in the launch file and
nothing else.

### 6.3 Launch changes in `helix_bringup`

Add one boolean argument per node, all defaulting to `false`, in the same shape as the existing
`use_cpp_anomaly` (`src/helix_sensing_cpp/launch/anomaly_detector.launch.py:28`):
`use_cpp_heartbeat`, `use_cpp_log_parser`, `use_cpp_rate_monitor`. Add a convenience
`use_cpp_all:=false` that ORs into each. Each argument selects the `package` of a `LifecycleNode`
action; nothing else in the launch file changes.

### 6.4 CI

`.github/workflows` gains the two packages to the `colcon build --packages-select` list and their
gtests to `colcon test`. The tier-3 parity script is Python and stays under `tests/`; it must follow
the repo's lazy-import convention so importing it in a CI collection pass does not require a ROS
install.

---

## 7. Re-benchmark protocol

The point of this section is that each port produces **one number attributable to that port**, not a
cumulative blur.

### 7.1 Fix the baseline first

Close action item P0-A (section 0.3). Then, on the same Jetson, in the same power mode, with the GO2
in the same state, run a **fresh all-Python 30-minute capture** of the full six-node stack. That
becomes the reference for every subsequent delta, and it removes the caveat
`docs/cpp_parity_summary.md` already raises against itself ("The 45 MB Python baseline comes from
Session 7 notes, run under different conditions"). Commit it as
`results/baseline_python_<date>.json`.

### 7.2 Per-run procedure

Fixed for every run so runs are comparable:

1. Cold boot the Jetson. Record `nvpmodel -q`, `uname -a`, `free -m`, and the HELIX git SHA into an
   `env/` sidecar. Power mode differences alone can move CPU numbers by 2x.
2. GO2 in a fixed, documented state (standing idle, `motion_switcher` mode recorded, never
   `normal`).
3. Launch the stack, transition all nodes to `active`, wait 120 s for RSS to settle before the
   sampling window opens. Session 8's `+1.28 MB drift` over 30 min shows the settle is real.
4. Sample every 2 s for 30 minutes: per-PID `rss_mb`, `cpu_percent`, `num_threads`, via
   `psutil.Process(pid)`. Per-PID, not aggregate, so a single port's effect is visible.
5. Record the fault stream over the same window: counts per topic on `/helix/faults`,
   `/helix/recovery_hints`, `/helix/cmd_vel`.
6. Emit one JSON per run in the schema `results/cpp_parity.json` already uses, and one markdown
   summary.

### 7.3 Matrix

Each row is one 30-minute run. `A` is all-Python, `B0` is the current state.

| Run | Config | Answers |
|---|---|---|
| A | all Python | the reference (7.1) |
| B0 | `use_cpp_anomaly:=true` | re-measure the detector after remediation, on the fresh baseline |
| B1 | B0 + `use_cpp_rate_monitor:=true` (typed subs) | the headline number: what does killing 440 Hz of rclpy deserialisation buy |
| B1g | B1 with `GenericSubscription` | what does *not deserialising at all* buy on top |
| B2 | B1g + `use_cpp_heartbeat:=true` | heartbeat port delta |
| B3 | B2 + `use_cpp_log_parser:=true` | log parser delta at idle |
| B3s | B3 under a synthetic `/rosout` storm (1000 msg/s, `scripts/`-driven) | the tail-robustness claim the log parser port is actually for. Run the same storm against A for comparison. |
| C | B3 with all four nodes in one `component_container` | the consolidation win (3.1 option 2) |

Only `B1g`, `B2` and `B3` are single-variable deltas against their predecessor. `B1` changes both
implementation and package, and `C` changes the process topology, so both need their own control.

### 7.4 Reporting rules

- Every reported ratio names its denominator run and that run's file path. No ratio against a
  narrative number once 7.1 has produced a real baseline.
- RSS under `component_container` (run C) is **not** comparable per-node to standalone runs, because
  the DDS participant and rclcpp runtime are shared. Report run C as a stack total against run A's
  stack total, and say so in the same sentence.
- A missed target is reported as a missed target with the actual figure, as
  `docs/cpp_parity_summary.md` already does for the 44% RSS result. A retuned target is a separate,
  argued decision, recorded in `docs/decisions` or the project notes, never a silent edit of the number.
- Every run's JSON is committed. A summary with no committed JSON behind it does not count.

---

## 8. Definition of done, per port

- [ ] ROS-free kernel implemented, golden CSV generated from the Python oracle and committed
- [ ] Tier-1 gtests green
- [ ] Tier-2 node gtests green, including P1 heartbeat, P2 parameter binding, P3 lifecycle round-trip,
      and P4 QoS where applicable
- [ ] Tier-3 bag-replay parity report committed under `results/`
- [ ] Node emits a heartbeat while active
- [ ] Parameters read from `helix_bringup/config/helix_params.yaml` with unchanged names and types
- [ ] QoS profiles explicit in code and asserted in a test
- [ ] All subscription-callback logging throttled
- [ ] Launch flag wired, defaulting to the Python path
- [ ] `colcon build` and `colcon test` green in CI
- [ ] Tier-4 hardware A/B run, JSON committed, verdict recorded against a real baseline
- [ ] `ARCHITECTURE.md` updated
- [ ] Project tracking notes updated

---

## 9. Open questions requiring a decision before implementation

1. Is the 30% per-node RSS target achievable standalone, or only under a component container? Answer
   comes from the profiling in 3.1 option 1, and it changes the acceptance criteria for all four
   ports. **Blocking.**
2. `std::regex` or RE2 for `LogParser`? Answer comes from a microbenchmark against the real
   `log_rules.yaml`. Affects `package.xml` and the rosdep story.
3. Does `emit_cooldown_s` stay a C++-only behavior, get ported back to Python, or get removed? A
   behavior difference between two implementations of one node name is not a stable end state.
4. Should `HeartbeatMonitor` move to `BEST_EFFORT` on `/helix/heartbeat`, and does the existing
   DDS-liveliness path (`liveliness_monitor.py`, default off) make the whole heartbeat topic
   redundant? This is a design question that the port should not answer by accident.
5. Does `topic_rate_monitor` keep typed subscriptions or move to `GenericSubscription`? Decided by the
   `B1` vs `B1g` measurement, not in advance.

---

*Design only. No code in this change.*
