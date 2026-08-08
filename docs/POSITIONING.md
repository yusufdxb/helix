# Positioning: how HELIX is different from what ROS 2 already ships

This document answers one question honestly: given that `diagnostic_updater`,
`software_watchdogs`, `system_modes`, Nav2's recovery behaviors, and
`twist_mux` already exist in the ROS 2 ecosystem, what is HELIX actually
adding? Where the honest answer is "nothing, this duplicates an existing
package," it says so.

The claim under test: **ROS 2 has no central error/fault manager and no
centralized safety reaction mechanism.** Individual pieces exist (a
diagnostics aggregator, a liveliness watchdog, a mode-transition framework,
a per-behavior-tree recovery node, a command-arbitration mux). Nothing in
the ecosystem closes the loop from *detection* through a *policy decision*
to *gated actuation* as one system with one safety envelope. If that gap
does not hold up, HELIX has no reason to exist as anything other than a
config file wiring existing packages together, and this document should say
that too.

## The comparison table

| Package | What it does | What HELIX reuses / duplicates | What HELIX adds |
|---|---|---|---|
| `diagnostic_updater` / `diagnostic_aggregator` | Per-node status reporting (`DiagnosticStatus` OK/WARN/ERROR/STALE) aggregated into a tree (`DiagnosticArray` → `/diagnostics_agg`), typically viewed in `rqt_robot_monitor`. Aggregation is structural (group nodes by hardware/subsystem), not decision-making. | HELIX's `HeartbeatMonitor` publishes `/helix/node_health` as a flat `DiagnosticArray`, which is a smaller, unaggregated version of what `diagnostic_aggregator` already does well. This is duplication, not innovation, and should arguably be replaced by wiring HELIX's per-node status into a real `diagnostic_aggregator` group instead of hand-rolling a second diagnostics topic. | Nothing, on the diagnostics-reporting axis. What HELIX adds is downstream: `diagnostic_aggregator` has no concept of a `FaultEvent`, no severity-to-recovery-action mapping, and does not publish actuation commands. It ends at "this subsystem is RED"; it does not decide what the robot should do about it. |
| `ros-safety/software_watchdogs` | `SimpleWatchdog` (liveliness-only) and `WindowedWatchdog` (liveliness + deadline, tolerates N lease violations) transition a lifecycle node to `Inactive` on a DDS LIVELINESS/DEADLINE violation. Canonical, WG-blessed mechanism for detecting a dead or wedged publisher without extra topic traffic. | This is exactly the research finding that motivated the liveliness work in this change: `HeartbeatMonitor`'s original design (a bespoke `std_msgs/String` name published at 10 Hz, counted against a wall-clock timeout) is a strictly worse reimplementation of what `software_watchdogs` already does at the DDS layer. `helix_core/liveliness_beacon.py` and `helix_core/liveliness_monitor.py` (added alongside this document) reimplement the `SimpleWatchdog` half of that mechanism directly against rclpy QoS rather than depending on the external package, so it stays honest to say HELIX duplicates it rather than wraps it. | `software_watchdogs` transitions *the node itself* to Inactive on its own lease violation: it is a per-node self-defense primitive. It has no concept of a fault taxonomy (CRASH vs ANOMALY vs LOG_PATTERN), no rule engine mapping fault type to a recovery action, and no downstream actuation gate. A `WindowedWatchdog` firing tells you a node died; it does not tell a different, still-alive node ("recovery") what to do about it, and it does not stop `/cmd_vel`. |
| `system_modes` | Declarative operating-mode state machine (e.g. `NORMAL` / `DEGRADED` / `SAFE`) built on managed lifecycle nodes, with a mode manager that can drive a whole subsystem's mode transitions from one declarative model. | HELIX does not use `system_modes` and does not reimplement a general mode-transition framework. HELIX's four tiers (SENSE/CONTEXT/DIAGNOSE/RECOVER) are fixed lifecycle nodes wired by launch file, not a declarative mode graph. | `system_modes` answers "what mode is the robot in and how do modes transition." It does not answer "what fault occurred, why, and what specific bounded action should run right now." HELIX's rule engine (R1-R4) and `RecoveryHint` are a narrower, fault-triggered decision layer that `system_modes` does not provide; conversely, HELIX has no general-purpose mode declaration language and would need one to replace ad hoc per-node lifecycle wiring at larger scale. Fair critique: HELIX could plausibly be *rebuilt on top of* `system_modes` mode transitions as its actuation substrate instead of a bespoke `RecoveryNode`; that was not attempted here.|
| Nav2 recovery behaviors | Behavior-tree nodes (`Spin`, `BackUp`, `Wait`, `ClearCostmap`, etc.) invoked from within a Nav2 BT when navigation itself fails (planner/controller failure, costmap-detected collision risk). Scoped to navigation-stack failures the BT can observe. | None directly; HELIX does not depend on Nav2 and is not a BT plugin. | Nav2 recovery behaviors only fire from inside a Nav2 BT tick and only react to failures Nav2 itself detects (planning/controller failure). HELIX's fault sources are broader and Nav2-independent: process crash (liveliness/heartbeat), sensor-metric anomaly (Z-score), and log pattern match, none of which require Nav2 to be running or to have detected anything. HELIX cannot invoke Nav2's specific recovery primitives (it is not a BT node), so on a robot already running Nav2, the honest framing is that HELIX and Nav2 recovery cover disjoint failure classes and would need explicit integration (e.g. HELIX gating whether Nav2's BT is even allowed to run) rather than either replacing the other. |
| `twist_mux` | Priority-based arbitration across multiple `/cmd_vel` sources, with a lock topic to suppress lower-priority sources. Solves "who gets to drive" once multiple commanders exist. | HELIX's closed-loop launch path uses `twist_mux` as the actuation front end: `RecoveryNode` publishes `/helix/cmd_vel`, `twist_mux` arbitrates it against other commanders and forwards the muxed result to `/cmd_vel`. HELIX does not reimplement priority arbitration. | `twist_mux` has no concept of *why* a command should win, no fault taxonomy, no cooldown, and no allowlist: it only knows static priorities and an external lock signal. HELIX's `RecoveryNode` is what decides *whether* a `STOP_AND_HOLD` should be issued at all (enable flag, 5 s per-fault cooldown, action allowlist limited to `{STOP_AND_HOLD, RESUME, LOG_ONLY}`) before `twist_mux` ever sees a message. `twist_mux` is the actuation-arbitration layer HELIX sits upstream of, not a competitor. |

## Where HELIX genuinely duplicates existing work (no hedging)

- **`/helix/node_health` duplicates `diagnostic_aggregator`.** There is no
  technical reason HELIX's per-node status needs its own flat
  `DiagnosticArray` topic instead of publishing through the standard
  `diagnostic_updater` API and letting `diagnostic_aggregator` do the
  grouping. This was not fixed as part of this change (out of scope: it
  touches `heartbeat_monitor.py`, which this change does not modify), but it
  should be flagged as debt rather than defended as a feature.
- **The original heartbeat mechanism duplicated, worse, what
  `software_watchdogs` already does.** A hand-rolled `String` name on a
  timer, counted against wall-clock elapsed time with a miss threshold, is a
  slower, chattier, less standard version of a DDS LIVELINESS lease. This
  was the explicit motivation for the liveliness work in this change (see
  below); it is not being claimed as new detection *capability*, only as a
  better-aligned mechanism for a capability HELIX already had.
- **HELIX has no mode-declaration language.** Anything `system_modes`
  already does well at the "what state is this subsystem in" level, HELIX
  does with fixed lifecycle wiring and no general framework. If HELIX grows
  past four tiers, hand-wiring lifecycle transitions per node will not
  scale the way a declarative mode graph would.

## Where the gap claim holds up

None of the five packages above own the *closed loop*: a typed fault
(`FaultEvent`, one schema for CRASH/ANOMALY/LOG_PATTERN with severity and
context), a rule engine that maps fault conditions to a bounded recovery
decision (`RecoveryHint`, R1-R4), and a gated actuator that is the only
writer of `/helix/cmd_vel`, enforcing an enable flag, a 5 s per-fault
cooldown, and an action allowlist, before handing off to `twist_mux` for
arbitration against other commanders. `diagnostic_aggregator` stops at
status reporting. `software_watchdogs` stops at "this node is dead."
`system_modes` stops at mode transitions, not fault-specific actions.
Nav2 recovery stops at navigation-internal failures. `twist_mux` stops at
priority arbitration once a command already exists. HELIX's contribution is
the pipeline connecting a heterogeneous set of fault sources to one policy
layer to one gated actuator, not any individual stage in that pipeline:
several of which, per the section above, are themselves ROS 2-standard or
should be.

## The liveliness change in this document's context

Today's fix (`helix_core/heartbeat.py`) made `HeartbeatMonitor`'s existing
detection *reachable*: nothing published on `/helix/heartbeat` in
production before that, so CRASH detection was dead in every real
deployment despite passing tests against the demo fault injector. That fix
does not change *which mechanism* detects death: it is still a payload
message on a timer, counted against wall-clock elapsed time.

Alongside this document, `helix_core/liveliness_beacon.py` and
`helix_core/liveliness_monitor.py` add the DDS QoS liveliness path
(`LivelinessPolicy.MANUAL_BY_TOPIC`, a lease duration, and
`Publisher.assert_liveliness()` on the beacon side; a matching subscription
with `SubscriptionEventCallbacks(liveliness=...)` on the monitor side) as a
selectable, parameter-gated alternative, proven with its own unit tests and
one live rclpy/Cyclone DDS integration test
(`src/helix_core/test/test_liveliness_integration.py`) that stops a beacon
mid-run and confirms the monitor emits a `CRASH` `FaultEvent` from a real
DDS `LIVELINESS_CHANGED` event, not a mock. It defaults to disabled
(`enabled: false`) and is not wired into the nine production lifecycle
nodes or the bringup launch files by this change; that wiring is scoped out
because it touches files owned by concurrent work on this branch.

### Recommendation: do not switch the default yet

**Keep `HeartbeatMonitor` (the String-topic heartbeat) as the default.
Do not switch to DDS liveliness as the default detection path in this
change.** Tradeoffs, in order of weight:

1. **Per-node identity is worse under DDS liveliness as designed here.**
   The subscription-side `LIVELINESS_CHANGED` event reports only aggregate
   `alive_count` / `not_alive_count` across every writer matched on a
   subscription: it does not carry which writer changed state. This
   implementation works around that by giving every monitored node its own
   topic (`/helix/liveliness/<node_name>`), which restores identity but
   means N monitored nodes need N QoS-matched subscriptions instead of one
   shared topic. The heartbeat's String payload names the node directly on
   a single topic; that is architecturally simpler for nine nodes and would
   get more so at larger node counts under the per-topic liveliness design.
2. **Both mechanisms fail the same way under a wedged executor.** The
   stated advantage of DDS liveliness ("catches a publisher that is wedged
   rather than dead") only holds if the beacon's `assert_liveliness()` call
   runs on a *different* execution path than the wedge that killed the
   node's real work. In this implementation, `assert_liveliness()` is
   driven by the same rclpy timer/executor as the rest of the node's
   callbacks; a wedged single-threaded executor stops both the heartbeat
   timer and the liveliness-assert timer identically. The theoretical
   advantage is real (DDS AUTOMATIC liveliness, or a beacon run on a
   dedicated callback group/thread, would decouple assertion from the
   node's main work) but is not realized by the drop-in replacement built
   here, and claiming it would be would be describing the wrong node.
3. **DDS liveliness genuinely removes payload traffic** (no
   `std_msgs/String` message published, only lightweight RMW control
   messages) and is the WG-canonical mechanism, which matters for anyone
   reviewing HELIX against ROS 2 safety norms. That is a real, if modest,
   win, and the reason to keep this path available and tested rather than
   deleting it.
4. **Only `SimpleWatchdog`-equivalent behavior is implemented.** One lease
   violation is one immediate CRASH; there is no `WindowedWatchdog`-style
   tolerance for N missed asserts before declaring death (documented in
   `liveliness_monitor.py` as an explicit non-goal of this change, not an
   oversight). `HeartbeatMonitor`'s `miss_threshold` already gives that
   tolerance today. Switching defaults now would trade a tunable-tolerance
   mechanism for an intolerant one unless `WindowedWatchdog`-equivalent
   logic is built first.
5. **It is unwired and unproven at the fleet level.** The only evidence
   this document can point to is one emitter, one monitor, and one live
   integration test. `HeartbeatMonitor` has the sim-integration coverage
   (`tests/sim_integration/test_rule_r4_crash_logonly.py`) that a default
   detection path needs before it is trusted, and this change deliberately
   does not touch that test's dependency (a real process kill against
   `helix_context_buffer`) to avoid breaking work in flight elsewhere on
   this branch.

The honest path forward: keep both. Ship DDS liveliness as an
explicitly-opt-in mechanism for anyone who wants to align with the WG
convention now, use `HeartbeatMonitor` as the default until `WindowedWatchdog`
parity and per-node identity at scale are both worked out, and revisit the
default once the liveliness path has its own sim-integration evidence
equivalent to R4's process-kill test.
