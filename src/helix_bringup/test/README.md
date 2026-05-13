# helix_bringup tests

## twist_mux priority arbitration: sim-only verification harness

Two layers of coverage for the priority-mux wiring landed in commit
`ce60db1`:

1. **`test_twist_mux_model.py`** (no ROS, no twist_mux binary): a
   pure-function model (`helix_bringup.twist_mux_model.PriorityMux`)
   that reimplements twist_mux's documented arbitration semantics
   (priority, timeout, dropout, idle->zero). Runs in the no-ROS CI
   job. Fast (<200 ms total). The YAML round-trip tests also assert
   that `config/twist_mux.yaml` matches the HELIX safety invariant
   (`teleop > helix_recovery > navigation`, all on a 0.5 s timeout).

2. **`test_twist_mux_integration.py`** (rclpy + real `twist_mux`
   binary, no hardware, no Isaac Sim, no GO2): starts the actual
   `twist_mux` node with the production YAML, fires synthetic
   publishers on the three input topics, asserts on `/cmd_vel`.
   Self-skips when rclpy or the `twist_mux` binary is unavailable, so
   it is safe to invoke from a no-ROS environment.

### Running

No-ROS, pure-function only (matches CI's `repo_consistency` job):

```bash
python3 -m pytest src/helix_bringup/test/test_twist_mux_model.py -v
```

Full suite, ROS sourced. Export `ROS_DOMAIN_ID` to isolate from any
other ROS nodes you have running:

```bash
. /opt/ros/humble/setup.bash
. install/setup.bash   # if you ran colcon build
export ROS_DOMAIN_ID=77
python3 -m pytest src/helix_bringup/test/ -v
```

The integration test's subprocess inherits `ROS_DOMAIN_ID` from the
parent shell. If you don't export one, you'll cross-talk with whatever
else is on the host's default domain (which is what bit us during
harness development: twist_mux on domain 0 vs. the test's rclpy.init
on domain 77 means zero messages cross over).

### Adding a new arbitration test case

The HELIX wiring is intentionally tiny (three inputs, one output, one
timeout per input), so most new behavior we want to verify expresses
naturally as a model test. The integration test is for binary-level
regressions (twist_mux version bumps, parameter-schema breakage, etc.).

To add a model test:

1. Open `test_twist_mux_model.py`. The helpers you'll need:
   * `FakeClock(t0=...)` for deterministic time control. `clock.advance(dt)`
     moves the clock forward; the mux re-evaluates staleness on every
     `arbitrate()` call.
   * `_make_helix_mux(clock)` constructs a `PriorityMux` with the
     canonical HELIX priorities (200/100/50, 0.5 s timeouts).
   * `mux.publish(name, Twist2D(linear_x, angular_z))` records a new
     message arrival at the current clock time.
   * `mux.arbitrate()` returns `ArbitrationResult(winner, twist, live_inputs)`.
     `mux.arbitrate_with_zero_on_idle()` returns `ZERO` instead of `None`
     when nothing is live (matches the real twist_mux idle behavior).

2. Write the scenario. Example skeleton:

   ```python
   def test_my_new_scenario():
       clock = FakeClock()
       mux = _make_helix_mux(clock)

       # set up: who publishes, when?
       mux.publish("helix_recovery", Twist2D(0.0, 0.0))
       clock.advance(0.1)
       mux.publish("teleop", Twist2D(0.3, 0.0))

       # assert: who wins?
       assert mux.arbitrate().winner == "teleop"
   ```

3. If your scenario depends on the *configured* priorities (rather
   than relative-priority semantics), reference the named constants
   (`HELIX_TELEOP_PRIO`, etc.) instead of hard-coding numbers, so that
   if the YAML changes the tests fail in one place.

4. If your scenario can only be verified against the real twist_mux
   binary (e.g. a QoS or DDS regression), add it to
   `test_twist_mux_integration.py` instead. Use the existing
   `twist_mux_proc` fixture; it spawns one node per test.

### Adding a new mux input

If a new `/some/cmd_vel` source is added to `config/twist_mux.yaml`,
update **both**:

* `test_twist_mux_model.py::test_bringup_yaml_loads_and_matches_helix_intent`
  (extend the expected input set and the priority/timeout assertions).
* The helper `_make_helix_mux(clock)` if you want canonical-wiring
  tests to use the new input.

### Misconfiguration sanity check

`test_misconfigured_priority_is_caught` deliberately constructs a
broken priority order (nav above helix) and verifies that the mux
actually picks nav under that broken config. This is the canary that
proves the assertions in the rest of the file are load-bearing rather
than green-by-default. Do not delete it.
