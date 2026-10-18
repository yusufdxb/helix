# Limitations

Known limitations of the HELIX prototype in its current state.

## Synthetic Benchmark Limits

The standalone benchmarks test pure-Python ports of the detection logic against synthetic Gaussian data with known injected anomalies. Real sensor data differs from this test regime in several ways: noise characteristics are non-Gaussian and sensor-specific; distributions may be multi-modal or non-stationary; failures can be correlated across multiple channels simultaneously; and transient environmental effects (vibration, thermal drift, communication dropouts) produce patterns not represented in the synthetic data. The reported detection latency (~0.065 ms), throughput (~46K samples/sec), and perfect TPR/FPR should be understood as measurements of algorithmic correctness on well-behaved inputs, not as predictions of operational performance.

## Bounded Hardware Validation

HELIX has been executed on real hardware in eight bounded sessions (2026-04-03 → 2026-04-23); it has **not** been deployed as a persistent monitoring service on any robot. Highlights:

- **Session 1 (2026-04-03)** ran the three `helix_core` lifecycle nodes on a PC observing the live Unitree GO2 graph for 60 s, with a passive adapter bridging GO2 topics into `/helix/metrics`. Four FaultEvents were emitted from a `/utlidar/cloud` rate fluctuation; the fluctuation was not ground-truth labeled.
- **Session 2 (2026-04-06)** reproduced Session 1 after a 3-day gap (CV < 0.001 across the gap) and emitted 2 new FaultEvents from a `/utlidar/robot_pose` rate anomaly.
- **Session 5 (2026-04-15)** ran the three `helix_core` lifecycle nodes natively on the Jetson Orin NX against the live GO2 stack for 1780 s with no out-of-memory, no thermal throttle, and no lifecycle deaths. A controlled `/rosout` log-pattern injection produced an end-to-end FaultEvent in ~1.8 s.
- **Session 7 (2026-04-17)** 1-hour Jetson stability run, RSS plateau confirmed; baseline used for the Session 8 C++ comparison.
- **Session 8 (2026-04-23)** ran the full SENSE → DIAGNOSE → RECOVER stack on Jetson against the live GO2: 30 ANOMALY faults, 14 recovery hints (9× R1 STOP_AND_HOLD on utlidar metrics, 5× R2 RESUME), 14 audited actions (7 ACCEPTED, 7 SUPPRESSED_COOLDOWN), 3064 zero-twist `/helix/cmd_vel` messages during STOP states. The R1 schema fix the demo depends on landed on `main` in `82f7a15`, merged from `fix/r1-anomaly-schema-mismatch`. Same session: 30-min C++ anomaly-detector parity vs the Python Session 7 plateau: RSS 42.51 MB to 19.81 MB (−53.4%), CPU 2.21% to 0.80% (−63.8%); RSS missed the 30% design-doc target at 46.6%, CPU passed 12× (see `docs/cpp_parity_summary.md`).

What this is **not**: continuous deployment, ground-truth-labeled fault detection, multi-day stability, or operation under environmental disturbance. Three controlled physical injections in Session 5 (LiDAR cover, USB mic disconnect, `/test/spam` flood) produced 0 FaultEvents, those signals are invisible without an adapter explicitly bridging them. A Session 8 UDP-block injection produced 0 ANOMALYs *during* the drop for a different mechanism (NaN handling, see *Detection Logic Limitations* below). See `docs/GO2_HARDWARE_EVIDENCE.md` and `hardware_eval_20260423/results/closed_loop_demo.md` for full evidence and bag inventory. Raw bags live under `hardware_eval_*/` on the T7 evidence drive; the main branch ships the algorithm, the tests, and the documented launch paths, not the bag artifacts.

## No Passive or Non-Intrusive Integration

The `helix_core` sensing nodes are not passive observers. The HeartbeatMonitor requires monitored nodes to explicitly publish heartbeat messages on a designated topic. HELIX's own lifecycle nodes now emit that heartbeat while active, so the stack can detect the death of one of its own tiers; until this was added no process published on `/helix/heartbeat` at all outside the demo fault injector, which left the monitor's registry permanently empty and made CRASH detection untestable against a real launch. GO2 nodes still do not emit heartbeats and remain outside its reach. The AnomalyDetector requires metric values to be forwarded onto `/helix/metrics`. The `helix_adapter` package narrows but does not eliminate this gap: it bridges a fixed, configured set of GO2 topics (rate, JSON state, pose drift) into `/helix/metrics` so the AnomalyDetector can run against a robot that does not natively publish HELIX-shaped inputs. There is still no automatic node discovery and no introspection of arbitrary topics, adapter coverage is exactly what the YAML configures and nothing more. Session 5's three physical injections (LiDAR cover, USB mic disconnect, unrelated topic flood) produced 0 FaultEvents for this reason.

## Physical Recovery Actuation: Command Path Proven, Walking Stop Unverified

As of Session 8 (2026-04-23), `helix_diagnosis` produces `RecoveryHint` messages and `helix_recovery` consumes them, applies the allowlist (`{STOP_AND_HOLD, RESUME, LOG_ONLY}`) and cooldown, and publishes audit events plus `/helix/cmd_vel`. The closed loop is observable end-to-end in the bag artifacts.

What is **not** validated: the Session 8 GO2 graph had 0 downstream subscribers on `/helix/cmd_vel`, so STOP_AND_HOLD published zero-twist into a void and did not physically command the robot. The `twist_mux` fallback was found unsafe (silent on idle and lock, forwards NaN). The current path is `helix_arbiter` (sole velocity output) to the GO2 sport sink (StopMove 1003 / Move 1008 only).

As of 2026-10-17 the native profile (`helix_native.launch.py`) tracks every stage of a stop: hold, arbiter ack, controller ack (`/api/sport/response` code 0), and the measured stop (`/lf/sportmodestate`). `scripts/hw/native_stop_test.py` checks all four. On 2026-10-17 the test passed on the real GO2 with the armed C++ sink: one ack stage and 2 of 4 motion attempts (`hardware_eval_20261017/native_stop/`, `docs/NATIVE_RUNTIME.md`, `docs/EVIDENCE_AUDIT.md`). The real controller answered 277 StopMove requests with code 0. In both passing motion trials the fault came about 0.35 s after the first Move at vx 0.25; the state estimate read 0.13 and 0.32 m/s at the fault and still 636 and 265 ms later. Limits:
- The robot never sustained a walk at 0.25 m/s or below, the sink's limit: the estimate shows a 6 to 8 cm body displacement that returned toward the start in two of three runs. Foot fields are zero in `mcf` and `/lowstate` was not recorded, so a body shift cannot be told from steps. A stop from steady walking is not shown.
- No counterfactual: how far the robot travels under the same Move stream without the hold is not measured.
- No independent motion sensor. `/utlidar/robot_odom` carries the same estimate as `/sportmodestate` (median velocity difference 0), so it cannot confirm it.
- The estimate spikes for one 3 ms sample at Move onset (up to 0.43 m/s). The first runner abort rule tripped on one (1 of 3 trials at vx 0.25). The runner now uses a persistence-plus-ceiling guard (`scripts/hw/stop_guard.py`); the HELIX stop path itself never depended on that rule.
- The fault was a synthetic `FaultEvent`, not a degraded sensor.
- Remote input was not recorded during those trials (it is now). Request ids show no other process sent sport requests.

HELIX is not a substitute for the remote or the e-stop at any stage.

## Native Runtime Limits (2026-10-17)

- Native CPU and RSS on the Orin: 7.35 % of one core and 91.5 MB over 10 min, with the robot lying. The Python baseline is 79.08 % and 587.4 MB over 30 min. The profiles do different work: the native profile has no LLM explainer, context buffer, pose-drift monitor or JSON parser. Ten minutes is not a soak.
- The native detector configuration was selected on five real captures (1.057 h). It passed a short held-out test on 0.216 h: 0 stop false alarms (95 % bound about 13.9 per hour), all synthetic onsets detected, 0 false clears. Fault onsets are synthetic, injected into real streams; no physical sensor fault has been scored. Round 1 of the same protocol failed held-out (2 stop false alarms from `rate_hz/utlidar_cloud` stalls); see `docs/DETECTOR_EVAL.md`.
- The chosen configuration still raises log-only faults on `pose/displacement_rate_m_s` (19 episodes in 30 min on a lying robot). They never hold, but they add audit noise.
- No live fault injection (topic loss, rate collapse, drift), SIGKILL crash test on the Orin, DDS-liveliness comparison, long soak or jitter measurement has been run on the native stack. Crash recovery is tested in simulation only (`scripts/sim_native_crash_recovery.py`).
- The native arbiter now owns a protective latch across supervisor restarts, epoch changes and heartbeat gaps. Startup and recovery require fresh supervisor health plus an explicit operator release bound to the current latch, epoch and supervisor revision. Regression evidence is from the fake GO2 only; physical restart behavior remains unverified. Legacy Python arbitration does not implement this native protocol.
- **Unresolved sink-crash failure:** if the sink process dies, no surviving process sends StopMove. The arbiter latch does not repair that missing output path. The physical controller's behavior when Move requests stop has not been measured; the fake GO2 timeout provides no hardware guarantee. This remains a separate deployment blocker.
- `helix_supervisor_cpp` has no Python counterpart to check parity against. Its behavior rests on its own 49 tests and the rehearsal.
- `json_state_parser` and `pose_drift_monitor` are not in the native profile.

## No End-to-End Field Deployment

The system has not been tested under real operational conditions. There is no evidence of sustained operation, no measurement of false alarm rates over extended periods, and no evaluation under the environmental disturbances (terrain variation, payload changes, weather, network instability) that characterize field robotics.

## No Multi-Robot or Distributed Testing

All testing assumes a single robot with co-located nodes. The architecture has not been evaluated in multi-robot scenarios, across network boundaries, or with the DDS discovery and QoS configurations required for distributed ROS 2 deployments.

## Detection Logic Limitations

- **AnomalyDetector**: The Z-score method assumes the baseline data is roughly Gaussian. For sensor channels with skewed, heavy-tailed, or multi-modal distributions, the Z-score threshold will produce systematically biased detection rates. No adaptive windowing, no distribution fitting, and no learning-based detection is implemented.
- **Stale-topic physical validation gap (AnomalyDetector to topic_rate_monitor)**: Session 8 confirmed that the then-current path rejected the NaN emitted for a silent topic, producing zero ANOMALY events during a 44 s UDP block. The current Python and C++ detectors include a dedicated stale-NaN path and unit tests that require a `violation_type=stale` fault. This repair has not yet been exercised against a physical GO2 sensor dropout, so its operational timing and false-alarm behavior remain unverified.
- **LogParser**: Detection depends entirely on pre-defined regex rules. Log messages that do not match any rule are silently ignored. There is no anomaly detection on log content, no frequency-based alerting, and no mechanism to learn new fault patterns from log data.
- **HeartbeatMonitor**: Uses a fixed timeout with a consecutive-miss threshold. There is no adaptive timeout based on observed communication latency, and no distinction between a node that has crashed and a node that is alive but slow.
- **Context tier had never actually run**: `context_buffer` raised `ValueError` inside its `/helix/node_health` callback on the first message after activation (rclpy delivers `DiagnosticStatus.level` as a one-byte `bytes`, and `int(b'\x00')` raises), so the node died roughly a second into every session, including Session 8. Nothing consumed its `GetContext` service, so the loss was silent and no result depending on the diagnosis tier's context snapshot should be assumed to have had one. Fixed with a regression test; the repaired node has run only in simulation so far.

## Threats to Validity

### Construct Validity

The benchmarks measure detection performance on synthetic Gaussian data with cleanly injected anomalies (values beyond a known threshold). This operationalization of "fault detection" does not capture the full complexity of real faults, which may manifest as subtle distributional shifts, correlated multi-channel deviations, or intermittent transient events. The construct measured (detection of point anomalies in Gaussian data) is a necessary but not sufficient proxy for the construct of interest (detection of real robot faults).

### Internal Validity

The standalone benchmarks isolate the detection logic from ROS 2 runtime effects (scheduling jitter, callback queuing, DDS serialization overhead, timer drift). Performance within a running ROS 2 graph may differ due to these factors. Additionally, the unit tests exercise individual nodes but do not test the full pipeline under sustained load or with realistic message rates.

### External Validity

The detection nodes have been tested against a narrow set of fault types: heartbeat timeout, Z-score threshold exceedance, and regex-matched log patterns. Generalization to fault types not represented in this set (hardware degradation, firmware errors, environmental interference, multi-node cascading failures) is not established. Similarly, the architecture has been developed and tested only within standard ROS 2 desktop environments; behavior on resource-constrained embedded platforms (Jetson Orin) and with non-standard DDS configurations is unknown.
