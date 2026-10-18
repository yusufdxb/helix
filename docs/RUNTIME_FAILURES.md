# Runtime failure follow-up

All new execution evidence here is from localhost simulation. No physical
motion or hardware access was used. The native protective-latch protocol and
the earlier evidence files are preserved. The PR remains draft.

Machine-readable results: [teardown checks](../results/rate_monitor_teardown.json),
[native latch campaign](../results/native_protective_latch_teardown_sim.json),
and [experimental Sport gate](../results/sport_guard_sim.json).

## Rate monitor teardown

The baseline reproduced SIGSEGV in two of four native campaign teardowns.
A debugger also caught three failures in twelve isolated starts with all six
default input streams under traffic. The failing thread was the DDS `gc`
worker at an unmapped program counter. At the same time, the main thread was
in `_dl_close_worker`, reached from `rcutils_unload_shared_library`,
`rcpputils::SharedLibrary::~SharedLibrary`, and `librmw_implementation` during
process finalization.

`Watched` owned a generic subscription whose callback strongly captured the
same `Watched`. That ownership cycle retained subscriptions, their node and
DDS resources after cleanup and destruction. LeakSanitizer confirmed the
retained `Watched`, `RateWindow`, and `GenericSubscription` allocations.
The leaked readers prevented normal DDS resource teardown before RMW unload.
Separate lifecycle defects left metrics and heartbeat timers scheduled when
shutdown was requested directly from ACTIVE.

The callback now captures a weak pointer. Cleanup, active shutdown and the
destructor cancel and release both timers before releasing subscriptions and
the metrics publisher. The destructor covers signal-driven destruction,
which does not invoke lifecycle shutdown. As with the generated component
main, the executor must stop using the node before it is destroyed. All
subscriptions, lifecycle callbacks and timers use the default mutually
exclusive callback group.

Validation:

- Two new regression cases fail on the baseline because cleanup and failed
  configuration leave input subscriptions registered.
- Fourteen process regressions pass after the fix: twelve configure/activate/
  deactivate/cleanup cycles, failed-configuration rollback, and twelve fresh
  starts with traffic, lifecycle shutdown and alternating SIGINT/SIGTERM.
- The same fourteen cases pass with AddressSanitizer and UndefinedBehaviorSanitizer,
  with leak detection disabled for that run. Twelve further debugger starts
  with all six default streams produce no SIGSEGV.
- Unsuppressed LeakSanitizer runs reduce the first six-topic report from
  21,575 bytes in 205 allocations to 1,095 bytes in 17 allocations. The remaining
  reports originate in ROS typesupport loading, class registration and the
  generated component main's retained class loader. No `Watched`, rate-window
  or generic-subscription allocation remains in those reports. This is not a
  claim of a completely leak-free ROS process.
- The complete native fake-GO2 crash campaign retains its protective-latch
  assertions and has no rate-monitor cleanup warnings after the fix. Sink
  death still reports `UNRESOLVED_NO_STOP_PATH` in the native topology.

Run the process regressions from a built, sourced workspace:

```bash
ROS_LOCALHOST_ONLY=1 colcon test --packages-select helix_sensing_cpp
colcon test-result --verbose
```

To reproduce the sanitizer build, use separate build/install directories and
`-fsanitize=address,undefined -fno-omit-frame-pointer` for C++ compilation,
plus `-fsanitize=address,undefined` for executable and shared-library linking.
Run `test_rate_monitor_teardown.py` with `RATE_MONITOR_EXECUTABLE` set to that
instrumented executable, `ROS_LOCALHOST_ONLY=1`, an isolated `ROS_DOMAIN_ID`,
and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Inspect an unsuppressed leak run
separately from the memory/undefined-behavior regression run.

## GO2 command silence is not a stop contract

The deployed firmware's behavior when Move requests cease is **unknown**.
There is no measured physical timeout in this evidence.

- The pinned official [GO2 client implementation](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/go2/sport/sport_client.py#L124)
  sends only `x`, `y`, and `z` for Move. It does not send a duration or expiry.
  This describes the public interface, not the closed controller's behavior.
- [RPC `SetTimeout`](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/rpc/client_base.py#L24)
  controls transport and response waiting. It is not a command lifetime.
- The official example's 0.5-second stop-on-silence comment is in the
  [B2W example](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/b2w/b2w_sport_client.cpp#L124).
  It is not evidence for GO2.
- Optional [lease support](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/go2/sport/sport_client.py#L28)
  defaults off. Its existence does not establish a GO2 velocity stop on lease
  loss. The current sink sends unleased requests.

The fake GO2's default one-second Move expiry is explicitly a simulation
assumption. `move_timeout_s:=0.0` now disables it. A regression with the real
C++ sink and this fake observes velocity above 0.15 m/s more than 1.1 seconds
after sink SIGKILL. The native arbiter latch cannot send a controller request
through a dead sink.

## Experimental independent Sport gate

The proposed mechanism is a separate C++ process downstream of the sink.
The sink publishes private intents. The guard is the sole publisher to the
private simulated controller request topic. This gives a surviving process
authority to block Move and repeatedly send StopMove after sink death.

The experiment is deliberately excluded from `helix_native.launch.py`.
`helix_sport_guard` defaults to `dry_run`; non-dry mode requires
`ROS_LOCALHOST_ONLY=1`. Resolved intent, request and response topics must stay
under `/helix/experimental/`. Fault output also defaults to the private
simulation namespace, including in dry_run. The controller egress topic in the tests is
`/helix/experimental/sport/request`.

The guard starts latched. It watches ordered sink request progress and
ordered arbiter status using steady-clock receipt times. Loss of progress
or status latches the gate and repeats StopMove every 50 ms, checked by a
20 ms timer. Moving-input timeout is 250 ms; idle-input timeout is 750 ms;
arbiter-status timeout is 300 ms. These are configured simulation values,
not real-time scheduling guarantees. Timeouts are checked before a later
message can replace the previous receipt time. Duplicate or regressing IDs
cannot extend a lease.

On protection events the guard publishes critical faults on its private
simulation fault topic. Rearming requires an observed held arbiter challenge, fresh
upstream progress after the trip, a later released status for the same
challenge, and explicit authorization bound to the guard's current random
instance/protection token. The token is advertised in guard status and
echoed on `/helix/experimental/sport/release`. Tokens rotate on startup,
protection events and entry into readiness. Expiring readiness invalidates its
token before a new message can refresh a receipt time. Authorization captured
before readiness remains invalid after recovery, including delayed delivery.
Premature authorization is rejected without banking it. This extra
operator step prevents a delayed old held/released status pair from reopening
a restarted guard. It neither releases the arbiter nor queues a Move for later.
Supervisor health and the native operator release remain the arbiter's
responsibility. A restarted sink cannot reopen the gate by itself.

Forwarded sink request IDs remain unchanged, so existing sink response
matching still works on the shared private controller response topic.
Guard-generated StopMove requests use negative IDs. Guard status separates
an acknowledged StopMove from measured stillness; it always reports
`measured_stopped=false` because this experiment does not certify motion
sensing or physical stopping.

Twenty-three vendor-independent C++ regressions and eleven ROS process tests cover
sink SIGKILL, repeated guard restarts, duplicate intents, stale release,
legitimate release, response identity, denied/dropped StopMove and arming
boundaries, including delayed pre-readiness and expired-ready authorization.
Two process cases run the existing fake GO2 with expiry disabled:
the unprotected case keeps moving, and the guarded case falls below
0.01 m/s while retaining the latch through sink restart. Other cases use a
controller stand-in that retains its target indefinitely and can deny or
drop StopMove. The arbiter status producer is a trusted protocol stand-in;
the complete native supervisor/arbiter/guard recovery wiring is not yet
integration-tested.

```bash
# Built workspace with the vendor message packages sourced; simulation only.
ROS_LOCALHOST_ONLY=1 python3 -m pytest tests/sim_integration/test_sport_guard.py -q
```

The experiment protects only against the upstream sink process failure under
its stated assumptions. Guard death, common host/power/DDS/network failure,
competing controller publishers, and rejected or lost StopMove still permit
motion. Tests explicitly retain those failures. Receipt ordering also cannot
prove the emission age of an unseen queued request; the protocol is not
authenticated. A lower request ID after restart fails closed, which can
require further operator recovery for availability.

The production sink-crash gap is still unresolved. Before enabling hardware
motion, this design needs integrated native fault/release validation and an
independent actuator/controller-side fail-safe for loss of the final guard
or communication path. A firmware-specific vendor stop contract or approved
physical measurement is still needed to establish GO2 command-silence behavior.
Physical motion remains disabled; no hardware motion test was performed.
