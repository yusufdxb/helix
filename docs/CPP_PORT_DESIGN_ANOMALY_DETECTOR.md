# C++ Port Design — `AnomalyDetector`

**Status:** Proposed
**Scope:** `helix_core.anomaly_detector` (Python, rclpy) → `helix_sensing_cpp::anomaly_detector` (C++, rclcpp_lifecycle)
**Target branch:** `feat/self-healing-closed-loop` (branch off for `feat/cpp-port-anomaly`)
**Author:** Yusuf Guenena (HELIX)
**Date:** 2026-04-18
**Related:** `docs/GO2_HARDWARE_EVIDENCE.md` §15–17 (Session 7 baseline), `src/helix_adapter_cpp/` (pattern reference — already uses `rclcpp_lifecycle` + `ament_cmake_gtest`)

---

## 1. Motivation

Python (`rclpy`) is the current RAM and CPU liability on the onboard Jetson Orin NX 16 GB:

- Session 7 1-hour plateau: **257 → 282 MB RSS** across 6 lifecycle nodes (~47 MB per process).
- Adapter CPU: **~48 % of one core** with `/utlidar/imu` in the topic list (Session 7 §17).
- `helix_anomaly_detector` specifically burns 1.80 %–2.21 % CPU and contributes ~45 MB RSS (rclpy runtime, interpreter, bound msg types).

The Python Z-score maths is already fast (0.049 ms mean per sample, Session 7 §16). The cost is **executor dispatch, Python msg deserialization, and the interpreter's baseline RSS** — none of which the algorithm owns. A 1:1 C++ rewrite eliminates that overhead without touching behavior.

**Project targets (from HELIX goals):**

- RSS: < 30 % of Session 7 baseline. Per-node: **< ~14 MB RSS** (vs ~47 MB Python).
- CPU: **< 10 %** of one Jetson core per node under realistic load.
- Latency: single-sample Z-score path **≤ 0.02 ms mean** (vs Python 0.0488 ms).

`AnomalyDetector` is ported first because it is the simplest hot-path node: two subscriptions, a pure-math inner loop, one publisher, no timers, no threads beyond the rclcpp executor. It becomes the template for `HeartbeatMonitor`, `LogParser`, and the rest of the rclpy stack.

---

## 2. Current Python behavior (parity source of truth)

File: `src/helix_core/helix_core/anomaly_detector.py` (225 LOC, reviewed 2026-04-18).

### 2.1 Node identity

- Node name: `helix_anomaly_detector`
- Base class: `rclpy.lifecycle.LifecycleNode`
- Entry point: `helix_anomaly_detector = helix_core.anomaly_detector:main` (in `setup.py`)

### 2.2 Subscriptions

| Topic | Type | QoS | Callback |
|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | depth 10, reliable (default) | `_on_diagnostics` |
| `/helix/metrics` | `std_msgs/Float64MultiArray` | depth 100, reliable (default) | `_on_metric` |

### 2.3 Publisher

| Topic | Type | QoS |
|---|---|---|
| `/helix/faults` | `helix_msgs/FaultEvent` | depth 10, reliable (default) |

### 2.4 Parameters

Declared in `on_configure`, defaults from `helix_params.yaml`:

| Param | Default | Type |
|---|---|---|
| `zscore_threshold` | `3.0` | double |
| `consecutive_trigger` | `3` | int |
| `window_size` | `60` | int |

Internal constant: `FLAT_SIGNAL_EPSILON = 1e-6` — skip Z-score if window std is below this.

### 2.5 Lifecycle transitions

- `on_configure`: read params, create publisher + two subscribers, log the three parameter values.
- `on_activate`: log only. (Python `rclpy.lifecycle` does not gate publishing on activation the way rclcpp does — see §7.)
- `on_deactivate`: log only.
- `on_cleanup`: destroy pub/subs, clear `_windows` and `_consecutive` under the lock.

### 2.6 Algorithm — Z-score with "evaluate-before-append"

Per incoming sample `(metric_name, value)`:

1. Lock `_data_lock`.
2. Lazy-init the rolling `deque(maxlen=window_size)` and consecutive counter for unseen metrics.
3. If window already has ≥ 2 samples:
   - Compute `mean = sum(samples)/N`, `variance = sum((s - mean)**2)/N` (**population variance**, not sample), `std = sqrt(variance)`.
   - If `std < 1e-6` → flat signal, log debug, **do not** touch the consecutive counter.
   - Else `zscore = abs((value - mean)/std)`.
     - If `zscore > threshold`: increment consecutive; if consecutive ≥ `consecutive_trigger`, emit a FaultEvent (and keep the counter — it is not reset on emit).
     - Else: reset consecutive to 0.
4. Append `value` to the deque **after** the check. This is the key correctness property: anomalous samples never poison the baseline.

### 2.7 `FaultEvent` payload

```
node_name        = metric_name        # spec: metric name doubles as identifier for ANOMALY
fault_type       = "ANOMALY"
severity         = 2                  # ERROR
detail           = "Metric '<name>' Z-score <zscore:.2f> exceeded threshold on <trigger> consecutive samples"
timestamp        = time.time()        # wall-clock seconds
context_keys     = ["metric_name","current_value","window_mean","window_std","zscore","consecutive_count"]
context_values   = [metric_name,
                    str(round(value, 4)),
                    str(round(mean, 4)),
                    str(round(std,   6)),
                    str(round(zscore,2)),
                    str(consecutive)]
```

The **formatting precision** (4, 4, 6, 2) and the `timestamp = time.time()` wall-clock source are load-bearing for byte-level parity.

### 2.8 `_on_diagnostics` quirks

- For every `status` in the array, for every KV, attempt `float(kv.value)`. Non-numeric KVs are silently skipped (bare `except ValueError`).
- Metric name is composed as `"{status.name}/{kv.key}"`.

### 2.9 `_on_metric` quirks

- Uses `msg.layout.dim[0].label` as the metric name. Empty label or missing dim → skip with a warn.
- Iterates `msg.data` and processes each value as an independent sample under the same label.

### 2.10 Current tests

`src/helix_core/test/test_anomaly_detector.py`:

- Only **one** functional test: `test_anomaly_detection` — publishes 25 noisy samples then 3 spikes on `/helix/metrics`, asserts at least one ANOMALY FaultEvent with `severity == 2`.
- No unit tests of the rolling-stats math itself (it's inlined in `_process_sample`).
- No test for `/diagnostics` path.
- No test for flat-signal epsilon skip.
- No test for lifecycle `on_cleanup` clearing state.

**Flagged gap:** the current Python test surface is thin. The C++ port is a good forcing function to back-fill with unit tests on the extracted algorithm core.

---

## 3. C++ port structure

### 3.1 Package choice: `helix_sensing_cpp/`

New ament_cmake package at `src/helix_sensing_cpp/`, mirroring the pattern already used by `helix_adapter_cpp/`. The three other Python nodes (`heartbeat_monitor`, `log_parser`, `topic_rate_monitor` lives in `helix_adapter_cpp`) will land here too, so the package is named for the **sensing tier**, not this one node.

Alternative considered: add alongside inside `helix_adapter_cpp/`. Rejected — `helix_adapter_cpp` is the adapter-layer package (rate monitoring lives there conceptually); sensing logic belongs in its own package.

### 3.2 File layout

```
src/helix_sensing_cpp/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── helix_sensing_cpp/
│       └── rolling_stats.hpp          # pure, header-only math (easy to gtest)
├── src/
│   ├── rolling_stats.cpp              # .cpp only if we keep the class out-of-line
│   └── anomaly_detector_node.cpp      # LifecycleNode class + component registration + main
└── test/
    ├── test_rolling_stats.cpp         # gtest of the math kernel
    └── test_anomaly_detector.cpp      # gtest that spins the node in-process
```

**Separation of concerns:** the rolling stats / decision logic is isolated from ROS plumbing so it can be tested at the raw-API level with no executor, no DDS, no discovery. This mirrors how `helix_adapter_cpp` separated `rate_window` from the node.

### 3.3 Class split

**`helix_sensing_cpp::RollingStats`** — pure C++, no ROS deps.

```cpp
class RollingStats {
 public:
  explicit RollingStats(std::size_t window_size);
  // Returns the decision for this sample, then appends.
  struct Decision {
    bool   skipped_flat;   // std < kFlatSignalEpsilon, counter untouched
    bool   have_enough;    // window had >= 2 samples
    double mean;
    double stdev;          // population std (divide by N, not N-1)
    double zscore;         // |value - mean| / stdev, 0 if skipped_flat
    int    consecutive;    // value of counter AFTER this sample (post-update)
    bool   triggered;      // consecutive >= trigger AND zscore > threshold
  };
  Decision evaluate(double value,
                    double zscore_threshold,
                    int consecutive_trigger);
  static constexpr double kFlatSignalEpsilon = 1e-6;
 private:
  std::size_t window_size_;
  std::deque<double> window_;
  int consecutive_{0};
};
```

Note: `RollingStats` is **per-metric**. The node owns an `std::unordered_map<std::string, RollingStats>`. No internal mutex in `RollingStats` — the node serializes access via the executor (single-threaded executor by default) or an outer mutex if we ever move to multi-threaded.

**`AnomalyDetectorNode : public rclcpp_lifecycle::LifecycleNode`** — in `anomaly_detector_node.cpp`.

- Owns the map of `RollingStats`.
- Holds `LifecyclePublisher<helix_msgs::msg::FaultEvent>` and two subscriptions.
- Three lifecycle callbacks mirror §2.5.
- Registered as an `rclcpp_components` component so `component_container` can host it (enables zero-copy intra-process with co-hosted nodes, e.g. the future `helix_sensing_cpp` topic rate monitor).

### 3.4 Lifecycle callback mapping

| Python | C++ |
|---|---|
| `__init__` → `declare_parameter` | ctor → `declare_parameter<T>(name, default)` |
| `on_configure` | `on_configure(const State&)` → get params, create pub/subs, keep them inactive |
| `on_activate` | `on_activate(...)` → `LifecycleNode::on_activate(state);` (activates publisher) |
| `on_deactivate` | `on_deactivate(...)` → `LifecycleNode::on_deactivate(state);` |
| `on_cleanup` | `on_cleanup(...)` → reset pub, subs, clear map |
| `main` | `main` → `rclcpp::init`, construct, `rclcpp::spin(node->get_node_base_interface())`, `shutdown` |

### 3.5 Executable name

- Executable / lifecycle node name: **`helix_anomaly_detector`** — identical to Python. This lets us swap the launch file via package name only.
- Installed to `lib/helix_sensing_cpp/helix_anomaly_detector`.

---

## 4. Dependencies

`package.xml`:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>rclcpp_components</depend>
<depend>lifecycle_msgs</depend>
<depend>std_msgs</depend>
<depend>diagnostic_msgs</depend>
<depend>helix_msgs</depend>

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
<test_depend>ament_cmake_gtest</test_depend>
```

**Not needed:**

- No `Eigen`. The window is N ≤ 60 doubles; a two-pass population mean/variance in plain C++ is faster than Eigen setup cost.
- No `<threads>`/std::mutex beyond what the executor provides. The default single-threaded executor already serializes callbacks.

---

## 5. CMakeLists (sketch)

```cmake
cmake_minimum_required(VERSION 3.8)
project(helix_sensing_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(helix_msgs REQUIRED)

add_library(rolling_stats src/rolling_stats.cpp)
target_include_directories(rolling_stats PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(anomaly_detector_component SHARED src/anomaly_detector_node.cpp)
target_include_directories(anomaly_detector_component PRIVATE include)
target_link_libraries(anomaly_detector_component rolling_stats)
ament_target_dependencies(anomaly_detector_component
  rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs
  std_msgs diagnostic_msgs helix_msgs)
rclcpp_components_register_node(anomaly_detector_component
  PLUGIN "helix_sensing_cpp::AnomalyDetectorNode"
  EXECUTABLE helix_anomaly_detector)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rolling_stats anomaly_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rolling_stats test/test_rolling_stats.cpp)
  target_include_directories(test_rolling_stats PRIVATE include)
  target_link_libraries(test_rolling_stats rolling_stats)

  ament_add_gtest(test_anomaly_detector test/test_anomaly_detector.cpp)
  target_include_directories(test_anomaly_detector PRIVATE include)
  target_link_libraries(test_anomaly_detector
    rolling_stats anomaly_detector_component)
  ament_target_dependencies(test_anomaly_detector
    rclcpp rclcpp_lifecycle std_msgs diagnostic_msgs helix_msgs)
endif()

ament_package()
```

`rclcpp_components_register_node` gives us **both** a standalone executable (`helix_anomaly_detector`) and a loadable component in one declaration — so the same artifact works standalone or inside a `component_container` for intra-process zero-copy.

---

## 6. Behavioral parity

### 6.1 Strategy

Three tiers, cheap → expensive:

1. **Unit — `RollingStats`.** A Python script dumps reference `(value, mean, std, zscore, consecutive, triggered)` tuples from the current Python `_process_sample` loop on a fixed seed + fixed sample stream (noisy baseline + spikes + flat regions). Check in as `test/fixtures/rolling_stats_golden.csv`. The gtest reads it, re-runs through `RollingStats::evaluate`, and asserts field equality within `1e-12` absolute / `1e-9` relative. Catches numerical drift vs the Python implementation directly.
2. **Node-level gtest.** Same shape as `test_anomaly_detector.py`: in-process publisher of `Float64MultiArray`, subscriber of `FaultEvent`, spin the node, assert at least one ANOMALY event. Ports the existing pytest 1:1.
3. **Bag-replay side-by-side.** Capture a 10-min rosbag from the Jetson (`/diagnostics`, `/helix/metrics`) during a known-good idle GO2 run (artifact already exists: `hardware_eval_20260416/jetson_live_copy_phase3/`). Replay once into the Python node (on a PC, namespace `/py/`), once into the C++ node (namespace `/cpp/`). Diff the resulting `FaultEvent` streams by `(metric_name, round(timestamp, 2), context_values)`. Accept if:
   - Same number of events.
   - Same metric names in the same order.
   - Matching `zscore` to 2 dp (Python rounds to 2 dp in `context_values`).
   - `timestamp` within 100 ms (wall-clock, different processes — can't be exact).

### 6.2 Test parity matrix

| # | Coverage | Python file/test | C++ file/test | New vs ported |
|---|---|---|---|---|
| 1 | Happy-path ANOMALY via `/helix/metrics` | `test_anomaly_detector.py::test_anomaly_detection` | `test_anomaly_detector.cpp::SpikeTriggersAnomaly` | Ported |
| 2 | Z-score math on golden stream (noisy baseline + spikes + flat) | (none) | `test_rolling_stats.cpp::GoldenStreamMatchesPython` | **New** |
| 3 | Flat signal (std < 1e-6) skips Z-score, counter untouched | (none) | `test_rolling_stats.cpp::FlatSignalSkipped` | **New** |
| 4 | `consecutive` counter resets on a non-violating sample | (none, relies on happy-path side effect) | `test_rolling_stats.cpp::CounterResetsBelowThreshold` | **New** |
| 5 | `consecutive` counter does NOT reset on emit (matches Python) | (none, implicit) | `test_rolling_stats.cpp::CounterPersistsAfterEmit` | **New** |
| 6 | Baseline not poisoned by anomalous samples (evaluate-before-append) | (none, implicit) | `test_rolling_stats.cpp::AnomaliesDoNotPoisonBaseline` | **New** |
| 7 | First sample never triggers (len < 2 guard) | (none) | `test_rolling_stats.cpp::TooFewSamplesNoDecision` | **New** |
| 8 | `/diagnostics` numeric KVs produce metric name `"<status.name>/<kv.key>"` | (none) | `test_anomaly_detector.cpp::DiagnosticsArrayParsed` | **New** |
| 9 | `/diagnostics` non-numeric KVs silently skipped | (none) | `test_anomaly_detector.cpp::NonNumericDiagnosticSkipped` | **New** |
| 10 | `/helix/metrics` with empty `layout.dim` emits no fault | (none) | `test_anomaly_detector.cpp::NoDimEmitsWarn` | **New** |
| 11 | `on_cleanup` clears rolling state (re-activation starts fresh) | (none) | `test_anomaly_detector.cpp::CleanupClearsState` | **New** |
| 12 | Parameter reads pick up launch YAML values, not defaults | (none) | `test_anomaly_detector.cpp::ParamsReadFromYaml` | **New** |
| 13 | `FaultEvent` context_values formatting: rounded to (4,4,6,2) dp | (none) | `test_anomaly_detector.cpp::ContextValuesFormatting` | **New** |

All ports use the same rclcpp in-process publisher/subscriber pattern that `helix_adapter_cpp/test/test_rate_window.cpp` already establishes (and extend it to spin a real `LifecycleNode`).

### 6.3 Numerical equivalence note

Python uses IEEE 754 double precision; C++ `double` is the same type on the Jetson (aarch64, GCC). Same summation order, same subtraction pattern → bitwise-equal mean and variance. The only float divergence risk is `std::round` vs Python `round()`: Python uses banker's rounding (round half to even), C++ `std::round` rounds half away from zero. **Mitigation:** match Python exactly by using `std::lrint` with `std::fesetround(FE_TONEAREST)`, or simpler — format through `std::ostringstream` with `std::fixed` + `std::setprecision`, which already rounds half-to-even on most libstdc++. Decide + unit-test in test #13.

---

## 7. Performance targets and expected results

| Metric | Python (Session 7) | Target | Expected C++ |
|---|---:|---:|---:|
| Per-node RSS | ~47 MB | < 14 MB | ~8–10 MB |
| CPU on one Jetson core | 1.80–2.21 % | < 10 % | < 0.3 % |
| Per-sample Z-score latency (mean) | 0.0488 ms | < 0.02 ms | < 0.005 ms |
| Per-sample Z-score latency (p95) | 0.0506 ms | < 0.02 ms | < 0.010 ms |

CPU was already below target in Python; the real win is **RSS** (≈ 5× reduction per node, dominated by removing the rclpy+interpreter baseline) and **tail latency headroom** (≈ 10× margin, useful when `/diagnostics` burst-fires 10+ KVs per message).

RSS estimation: `helix_topic_rate_monitor_cpp` (similar pattern, already built) measures well under 15 MB per process on the Jetson according to the adapter_cpp integration notes. Anomaly detector has a smaller per-metric footprint, so lower-bound 8 MB is realistic.

Benchmark: extend `benchmark_helix.py` with a `--cpp` flag that exercises the new node via rosbag replay + `/helix/faults` subscription; report RSS via `psutil.Process(pid).memory_info().rss` sampled at 10 Hz, and end-to-end latency via `time.monotonic()` deltas between input publish and FaultEvent arrival.

---

## 8. Migration plan

1. **Port + parity tests green (offline, PC).**
   - Branch: `feat/cpp-port-anomaly` off `feat/self-healing-closed-loop`.
   - Implement `RollingStats` + unit tests.
   - Generate golden CSV from Python.
   - Implement `AnomalyDetectorNode` + node-level gtest.
   - `colcon build` + `colcon test` clean on mewtwo.
2. **Side-by-side bag replay (PC).**
   - Replay `hardware_eval_20260416/jetson_live_copy_phase3/` bag.
   - Run Python node as `/py/helix_anomaly_detector`, C++ node as `/cpp/helix_anomaly_detector`.
   - Confirm parity diff (§6.1 tier 3).
   - Commit parity report under `hardware_eval_YYYYMMDD/cpp_port_parity/`.
3. **Launch-file flag.**
   - In `helix_bringup/launch/helix_sensing.launch.py`, add `use_cpp_anomaly:=false` (default false — Python remains the prod path until hardware-confirmed). When true, swap the `LifecycleNode(package='helix_core', executable='helix_anomaly_detector')` entry for `LifecycleNode(package='helix_sensing_cpp', executable='helix_anomaly_detector')`. Node **name** stays identical so `helix_params.yaml` keys still bind.
   - No change to `helix_params.yaml` — same three params, same names, same types.
4. **Hardware re-test (one CaresLab session, part of Phase 5).**
   - Cold-boot Jetson → launch with `use_cpp_anomaly:=true`.
   - 60-min run parallel to a concurrent 60-min run with `use_cpp_anomaly:=false` on separate boots (can't run both on same Jetson simultaneously because node names collide).
   - Compare RSS/CPU against Session 7 baseline; compare FaultEvent counts and labels on the same GO2 idle conditions.
5. **Promotion.** If RSS/CPU targets met and fault stream matches Python within tolerance, flip the launch default to `use_cpp_anomaly:=true`. Keep the flag for at least one more Phase for rollback.
6. **Retirement.** After a second Phase of clean hardware runs, delete `helix_core/anomaly_detector.py` and its console-script entry from `setup.py`. Remove `test_anomaly_detector.py` (already ported). Leave the launch flag wired for one release so downstream configs aren't broken; then drop it in the following release.

### 8.1 Rollback path

At every stage, rollback is a launch-file flag flip (`use_cpp_anomaly:=false`) or, post-retirement, a `git revert` of the deletion commit. The Python node stays in-tree and installable through step 5. If the flag is removed and a regression is found on hardware, `git revert` of the deletion commit restores the Python node within one build cycle (~2 min on the Jetson).

---

## 9. Risks and open questions

| # | Risk | Likelihood | Mitigation |
|---|---|---|---|
| R1 | Population-variance formula divergence (Python uses N, not N-1) — a future "clean up the maths" refactor could silently change this | Med | `RollingStats` doc-comments cite this explicitly; test #2 locks golden values |
| R2 | `std::round` half-away-from-zero vs Python banker's rounding diverging `context_values` strings | Med | Test #13; use stream formatter, verify on aarch64 libstdc++ |
| R3 | `time.time()` (Python wall-clock seconds) vs `now().seconds()` of a `rclcpp::Clock(RCL_SYSTEM_TIME)` — both are `CLOCK_REALTIME`-based on Linux so they match, but nonzero skew is theoretically possible | Low | Use `rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds()` explicitly, not `node->now()` (which is ROS time and can be sim-time) |
| R4 | Lifecycle timing difference: rclcpp `LifecyclePublisher` hard-gates publishing on active state; rclpy's does not. A call to `publish()` from an `on_configure`-time callback would silently drop in C++ but succeed in Python | Low | Publisher is only used from subscription callbacks, which fire post-activate; still, log a warn if `publish()` is called on an inactive publisher (via a helper) |
| R5 | Parameter typing: Python accepts any scalar; C++ `declare_parameter<int>("window_size", 60)` is strict, and YAML `window_size: 60` will bind — but `window_size: 60.0` in YAML would fail | Low | Keep `helix_params.yaml` integer-typed; `ParamsReadFromYaml` gtest confirms; also add a runtime check that `window_size >= 2` |
| R6 | Intra-process zero-copy changes the RSS accounting (shared allocations don't double-count) — comparing per-node RSS to the Python baseline is not apples-to-apples once we containerize | Med | For the first hardware test, run the C++ node standalone (not in a container) so RSS is directly comparable to Python. Container comparisons are a separate experiment |
| R7 | `helix_msgs` needs to be built in C++ mode | None | It already is — it's a `rosidl_generate_interfaces` package and `helix_adapter_cpp` consumes it via `find_package(helix_msgs REQUIRED)`. No change needed |
| R8 | Current Python test suite has only one functional test — there is no Python reference for most edge cases (flat signal, non-numeric diagnostics KV, on_cleanup behavior) | **Flagged** | The C++ port is the forcing function to write these. Tests 3–13 are new against the current-behavior spec captured in §2. If a Python behavior surprise turns up mid-port, file an issue and either amend §2 or fix Python first |

**Open questions for review:**

- ~~Q1~~ **RESOLVED 2026-04-18:** Level-triggered with cooldown. New param `emit_cooldown_s` (double, default `1.0`). On each emit, record `last_emit_time`; subsequent samples that meet the consecutive threshold are suppressed until `now - last_emit_time >= emit_cooldown_s`. `emit_cooldown_s = 0.0` reproduces the legacy Python flood behavior for back-compat. **Critical constraint:** cooldown MUST be strictly less than `ANOMALY_CLEAR_WINDOW_SECONDS` (3.0 s, defined in `helix_diagnosis/rules.py`) so the R2 RESUME rule does not fire spuriously during sustained anomalies. The DiagnosisNode state machine updates `_last_anomaly_time` on every incoming ANOMALY FaultEvent; R2 uses `now - _last_anomaly_time >= 3.0` as the clear test. A cooldown of 1.0 s keeps `_last_anomaly_time` refreshed at 1 Hz during sustained faults, which is well inside the 3.0 s window. **This is a behavior change from legacy Python** — enabled by default in the C++ port; Python can opt-in by adding the same cooldown logic (separate follow-up). Parity tests against legacy flood behavior use `emit_cooldown_s: 0.0`; new default-cooldown behavior gets its own dedicated tests.
- Q2: Should `/helix/metrics` subscription use `BestEffort` QoS to match the high-rate adapter? Current Python is reliable-depth-100; if the adapter switches to best-effort later, the detector must follow.

---

## 10. Out of scope for this doc

- Port of `HeartbeatMonitor` — own design doc, same pattern.
- Port of `LogParser` — own design doc; carries a YAML rules-file concern not present here.
- Port of `topic_rate_monitor` — already done in `helix_adapter_cpp`.
- Recovery layer (`helix_recovery`), diagnosis layer (`helix_diagnosis`), LLM/explanation tier (`helix_explanation`) — separate architecture work.
- Switching to multi-threaded executors or hardware-clock timestamps — orthogonal and not required to hit the performance targets.
- Packaging / DEB for Jetson deployment — current colcon-in-place flow is sufficient for Phase 5.

---

## 11. Checklist before merge

- [ ] `RollingStats` unit tests green on mewtwo
- [ ] Golden CSV regenerated and committed under `src/helix_sensing_cpp/test/fixtures/`
- [ ] Node-level gtest green on mewtwo
- [ ] Bag-replay parity report committed
- [ ] Launch file gated behind `use_cpp_anomaly` (default false)
- [ ] One CaresLab hardware run: RSS < 30 % of Session 7 per-node, CPU < 10 % on a Jetson core, FaultEvent stream matches within tolerance
- [ ] ARCHITECTURE.md updated with the new package
- [ ] Vault log in `Daily Claude Logs/YYYY-MM-DD.md` captures the port session

---

*End of design.*
