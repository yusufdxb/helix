# Observe mode (`act=0`, `observe_only`)

HELIX on the GO2 starts as a watcher, not a repairer. Observe mode lets the healer run its full
diagnosis on the real Orin stack while it is unable to change anything.

## Switches

- `act=0` on a `node` line: this node is observe-only. Every other node keeps its own setting.
- `observe_only:=true` (ROS parameter, default `false`): no node may be acted on, whatever its
  spec says. Use this for the first session on any new stack.

`may_act(node) = !observe_only && spec.act`.

## What an observe-only node gets

- The detector, the policy and the agent all run as usual: faults are diagnosed, candidates are
  ranked, and for `dtx` deploy verdicts are sampled.
- Where HELIX would act, it emits an `observe` event instead (`would <action> <detail>`). The
  same text for the same node is emitted at most once every 30 s. The policy's own decision
  record is logged as `would_act` or `would_given_up`, never `act` or `given_up`, so an event log
  from an observe session has no `act` line at all.
- The decision is answered with a no-op result (`Action::None`, success). The policy puts the node
  in `Verifying`, never `Retired`, so it is diagnosed again after `verify_s`. A node that stays dead
  after a no-op is not judged again until `verify_s` has passed, since nothing was done to it.
- An observe-only node that died is re-adopted on the next tick once its launcher starts it again
  (`attach ... (restarted elsewhere)`). If it then shows fresh outputs the log reads `observe`,
  `attach`, `verified`. That sequence measures recovery that happened outside HELIX.
- The policy's escalation still advances on each decision, so the `observe` trail shows the ladder
  HELIX would climb, up to the fallback or SAFE_STOP it would send. Those last steps are answered
  with `None` too, so the node is never retired and nothing is published. Agent memory (`tried`,
  `last_action`) is not updated for `None`, and no pending verdict is recorded, so no LKG or
  generation is committed from an observe decision.

## What is blocked

These are blocked for every node where `may_act` is false:

- `Restart`, `LifecycleReset`, `Fallback` (which would stop the node or publish SAFE_STOP),
  `Report`, `ParamRestore`, `restart_lkg`, and the plain `deploy_rollout` restart;
- the SIGKILL that the core-dump path sends to a node stuck dumping core.

These need no extra block:

- `release_orphans` and `terminate_all` stop only processes HELIX itself started, never adopted
  ones.
- A parameter rollback runs only after a `ParamRestore` was performed, and that is blocked.

## Fail-closed checks

- The healer refuses to start (exception at construction) if an observe-only node would be
  spawned by HELIX: `act=0` or `observe_only` with `autostart=1` and no `attach=`. An observe-only
  node must be started by someone else and adopted.
- An `attach=` node that is never adopted (no matching process) is skipped in the snapshot and is
  never judged, so Nav2 nodes that are off cause no events.

## Rules for the GO2

1. **Motion chain: `act=0`, permanently.** Anything that publishes or relays motion commands
   (`/cmd_vel`, `/api/sport/request`, `/mosaic/go2/cmd_velocity`), the bridges to the robot, the
   safety arbiter, the stop watchdog, the Nav2 controller and velocity smoother. HELIX never
   restarts these, even after observe mode is lifted elsewhere. A restart of a motion node can
   replay a command or drop a stop.
2. **First session: `observe_only:=true`.** The spec is not trusted until Stage 0 shows that each
   `attach=` pattern matches exactly one pid and the learned rates look sane.
3. **Acting on perception needs the owner's approval**, per node, after Stage 0. It is lifted node
   by node, by setting `act=1` on that node line, never by turning `observe_only` off for the
   whole stack.
4. Observe mode does not make the robot safe and does not prove anything about safety. It only
   guarantees that HELIX itself takes no action. Physical actuation stays disabled until it is
   authorized.

## Desk smoke (2026-11-03)

This was run on a desk machine with `eval/heal/humanoid/attach_test.py --observe`: the humanoid stack in
real ROS 2 processes, started by `ros2 launch`, with a real SIGSEGV to rsp at 8 s and to
controller_manager at 18 s. The source tree was dirty at the time and the change was not yet
committed.

| mode | `act` events | processes HELIX started | what the log shows |
|---|---|---|---|
| launch respawn | 0 | 0 | rsp: `would_act`, `observe`, re-adopted at 9.5 s, `verified` (/tf blackout 1.16 s, all from the launcher). controller_manager: re-adopted at 19.5 s, but its controllers were not respawned, so /joint_states stayed silent (blackout 12.97 s). HELIX judged `still_silent` and walked the ladder to `would fallback budget`. |
| no respawn | 0 | 0 | Both nodes stayed dead (0 copies at the end). camera_backup was never started. Decisions stayed bounded: rsp 3 `would_act` and 2 `would_given_up` in 24 s. |

The controller_manager case is a real gap that HELIX diagnosed correctly: a launcher that
respawns `ros2_control_node` does not re-run the one-shot spawner, so the node is alive and
silent. That is exactly the case where HELIX acting (the `then` helpers) would help, once acting
is authorized.

This smoke shows nothing about the Orin, the GO2 stack, or the `go2_orin.spec` patterns.

## Limits

- The interception is at HELIX's dispatch points. A bug in a new action path added later could
  bypass it. `test_policy` and `test_spec` cover the policy side and the parser, not every
  dispatch path in `healer_node.cpp`. The desk smoke (`eval/heal/humanoid/attach_test.py` style,
  with `observe_only:=true`) checks the integrated binary.
- Learned rates (an `out` line without hz) are learned over `learn_s` from whatever the stack
  does at startup. A node that is already degraded when HELIX starts teaches HELIX a bad rate.
