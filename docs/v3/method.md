# HELIX v3: method

## What the healer does

`helix_healer` is one C++20 process that launches the nodes it manages (fork/exec,
`PR_SET_PDEATHSIG` so nothing outlives it) and supervises them from a plain-text
spec (`src/helix_healer/config/*.spec`). It ticks at 20 Hz:

1. **Observe** (no instrumentation inside the managed nodes): process alive and
   exit status (`waitpid`), CPU and RSS from `/proc`, every declared output's
   message age and rate (a generic subscription per topic), whether the node
   still advertises the topic, QoS-incompatible events on those subscriptions,
   and the upstream graph (who publishes each node's inputs, from the ROS graph
   plus the spec's `in` lines).
2. **Diagnose**, per node, in this order: crash (dead) or crash loop (`loop_deaths`
   quick deaths in a row); memory leak (RSS slope projects reaching `rss_max_mb`
   within `leak_horizon_s` and at least `leak_min_growth_mb` of growth); CPU
   runaway (above `cpu_max` for `cpu_sustain_s`); then output symptoms after the
   startup grace: QoS mismatch, *upstream* (an upstream node went bad first, or
   just recovered, or a resource runaway elsewhere is starving the machine),
   endpoint loss (stale and no longer advertised), hang (stale, still advertised),
   degraded (rate below `degraded_ratio` of expected). A non-exit symptom must
   persist `confirm_s` before anything is done.
3. **Repair**, from a closed set: restart (SIGTERM, SIGKILL after `term_grace_s`,
   backoff doubling per attempt), lifecycle reset (cleanup, configure, activate
   through the node's own lifecycle services; used for degraded and endpoint loss
   on lifecycle nodes), fallback (start the declared backup node), or report.
   Upstream-caused symptoms get no action: the root is repaired and its
   consumers get `victim_grace_s` to follow.
4. **Verify**: the node must be alive with healthy outputs within `verify_s`. A
   failed lifecycle reset escalates to a restart. A failed repair counts against
   a budget of `max_attempts` per `window_s`.
5. **Fall back**: when the budget is spent, or the fault is one a restart cannot
   fix (crash loop, QoS mismatch), the healer starts the backup if there is one;
   otherwise, for a critical node, it publishes `SAFE_STOP <node>` on
   `/helix/mode` (latched). Stopping the robot is left to whatever subscribes to
   that; the healer has no command publisher.

Every decision, verification and give-up is a line in `event_log` (JSONL) and on
`/helix/heal_events`; `/diagnostics` carries per-node state at 1 Hz.

## What no component may do

Nothing in the healer executes text it did not get from the spec. The action
set is the `Action` enum; the commands it can run are the spec's `exec` lines.
A learned diagnoser (tested in `eval/heal/learned.py`) could only replace step 2
and would output a (node, fault) pair from closed enums; steps 3 to 5 and the
budget stay deterministic.

## Baselines

Same binary, `policy:=` parameter:

- `none`: observe only.
- `respawn`: restart when the process exits (what `respawn=true` in a launch
  file gives), unbounded.
- `watchdog`: a strong conventional supervisor: restart on exit, on any stale
  output after the startup grace, or on RSS above `rss_max_mb`; the same budget,
  backoff, verification and fallback as HELIX. It differs from HELIX only in
  diagnosis (which node, which fault) and action choice.

## Evaluation

`eval/heal/run_episodes.py` runs one episode per (scenario, policy, rep): the
healer supervises a pipeline of `helix_demo/demo_node` stages in its own ROS
domain on its own 4-core slice (localhost-only Cyclone DDS). An independent
probe process subscribes to every output topic and, 7 s after start, tells one
node to fail (`helix_demo` fault injection: segv, exit, hang, endpoint loss,
CPU hog, leak, rate degradation, QoS mismatch, missing device). Faults that a
real system keeps across restarts (a missing device, a QoS misconfiguration)
are persisted in a per-episode fault file. A detector stage latches: once its
input has been gone for 1 s it stops publishing until restarted, even when the
input returns.

`eval/heal/score.py` scores from the probe, not from the healer:

- RESTORED: every output at >= 0.8 x nominal over the whole last 3 s
  (nominal = lowest declared rate among a topic's providers, so a working
  backup counts).
- SAFE: not restored, SAFE_STOP published. FAILED: neither.
- diagnosis correct, unnecessary decisions (on nodes other than the root, its
  backup, or a declared collateral node), futile repairs, recovery latency,
  outage time, resources still burning at the end, healer CPU and RSS.

DEV topology: `nav.spec` (`scenarios.py:NAV`), used for development. Held-out
topology: `manip.spec` (`scenarios.py:MANIP`), written before it ran, run once,
pre-registered in `docs/preregistration/2026-11-02-helix-v3-heldout-manip.md`.
