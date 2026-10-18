# GO2 controller inspection and native readiness

This report separates measured hardware behavior from vendor interface definitions
and simulation. No new software stop layer is proposed. No motion, controller-mode
change, service switch, lease acquisition or firmware update was performed in this
follow-up. The PR remains draft.

## Current evidence boundary

| Question | Verified evidence | Still unknown |
|---|---|---|
| Installed robot firmware and motion-controller build | No current hardware inspection completed | Firmware versions, build identifiers and applicable release notes |
| Current controller mode | Historical captures reported `mcf`; no current query completed | Current mode and configuration |
| StopMove / Move transport | Earlier real-controller bags contain matching code-0 replies for API 1003 and 1008; [audit](EVIDENCE_AUDIT.md) retains their limitations | Stop from sustained walking, independent stop distance and firmware-specific behavior on silence |
| Command timeout | No physical command-silence trial exists | Whether the installed GO2 autonomously brakes, after how long, under which mode and failure conditions |
| Separate operator stop | The official handheld manual documents a damping/emergency-stop command | Same-unit operation during payload/DDS failure, priority, response time and remaining common controller failures |
| Current native target readiness | Access attempts and discovery could not reach the payload | Current candidate ARM build, ABI consistency, live topic flow, CPU/RSS, thermal behavior and clean teardown |

The machine-readable [readiness record](../results/controller_readiness.json)
records the blocked hardware work. Prior hardware and simulation results remain
unchanged. The earlier six-process, ten-minute hardware resource result (7.35% of
one CPU core and 91.5 MB RSS) predates the current candidate. It is a historical
measurement, not validation of the current deployment.

## Firmware identity is not API compatibility

The robot controller and the payload computer are different systems. Payload
Ubuntu/kernel/JetPack versions and installed SDK commits do not identify robot
motion firmware. Similarly:

- [Internal API 1](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/internal/internal_api.hpp#L28)
  returns a service API version. `1.0.0.1` must not be recorded as a robot firmware
  build merely because it appears in a client header or server response.
- `LowState.version` contains two raw integers. Without a vendor definition,
  they remain uninterpreted version fields, not an identified locomotion build.
  Serial-number fields are deliberately excluded from the inspector.
- A read-only robot/app version report, with firmware component names and build
  identifiers, must be matched to the specific robot and motion-service release.
  Keep unique device identifiers in private evidence. No such current report
  was available here.

## Sport API and command silence

The pinned [GO2 Move implementation](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/go2/sport/sport_client.py#L124)
serializes velocity components only. Its Python call requests no reply; HELIX's
sink instead requests replies, which earlier hardware bags recorded successfully.
Neither form specifies motion expiry. An absent expiry parameter does not prove
that firmware lacks an internal timeout.

[Client timeout](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/rpc/client_base.py#L24)
limits transport/response waiting. The optional one-second ownership lease is a
different mechanism. The [reference lease server](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/rpc/lease_server.py#L51)
checks expiry during incoming operations and clears ownership bookkeeping; it
does not implement an autonomous actuator-stop callback. This does not reveal
the closed GO2 motion controller's behavior.

Two vendor examples describe silence behavior on other platforms:

- The [B2W example](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/b2w/b2w_sport_client.cpp#L124)
  mentions 0.5 seconds.
- The [GO2-W example](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/go2w/go2w_sport_client.cpp#L149)
  says a single Move lasts one second and continued calls sustain motion.

Neither statement establishes the timeout of the installed legged GO2 firmware.
FakeGo2's configurable expiry is also not controller evidence. A stationary
query cannot measure stopping after command loss while walking. Vendor confirmation
must specify the firmware/mode, timeout start event, queue handling, braking versus
coasting, latching/rearm behavior, and coverage of publisher death, payload loss and
communication loss. No inquiry was sent and no response is claimed.

## Supported stopping mechanisms and independence

| Mechanism | Vendor interface evidence | Failure boundary |
|---|---|---|
| Sport `StopMove` (1003) | Documented client API; code-0 transport replies measured earlier | Uses the same final Sport command path; acknowledgment does not establish physical braking |
| Sport `Damp` (1001) | Documented client API | Same Sport path; changes mode and may collapse the robot; not an independent fallback |
| Handheld damping/emergency stop | Official remote manual, page 9: hold L2 and click B | Separate operator handset/control path from the payload publisher; same-unit priority, latency and common onboard-controller dependence remain unverified |
| Automatic ownership lease expiry | SDK ownership mechanism exists | No established GO2 autonomous motion-stop contract |
| Hardwired safety-rated stop / drive inhibit | No supported interface established by this inspection | Cannot claim it exists, is fitted, or safely stops this robot |

The [official remote manual](https://marketing.unitree.com/article/en/Go2/Handheld_Remote_Control.html)
and its [page 9](https://doc-cdn.unitree.com/static/2025/12/22/dd0126329a594c34a36b936565dffe17_2481x3509.png)
support the handheld command mapping. They do not establish a certified stop
category, maximum stopping time, electrical independence, or immunity to a failed
onboard controller. The same manual cautions that obstacles weaken the radio path.
No remote button was pressed during this work. Mode-changing fallback validation
requires a separately authorized, physically supported test.

The walking experiment therefore requires both a validated separate operator stop
path for loss of the payload/Sport sender and independent physical containment for
failures that defeat that path. A second Sport sender, Ctrl-C, source silence or a
zero Twist cannot serve as the final-path contingency. Battery removal or an
unverified power switch must not be improvised as controlled braking.

## Reproducible non-actuating inspection

Use the project's access policy for the payload. Build the complete candidate
against its vendor-message underlay in an isolated workspace. Rebuild `helix_msgs`
and all native consumers together: the protective release protocol changed the
message/service ABI. Do not overlay new interfaces onto old running nodes. Record
the candidate commit, clean source state, compiler, ROS/RMW versions, vendor-message
commit and installed interface hashes alongside the output.

The native launch defaults are non-actuating: the sink is dry-run and the
supervisor cannot enable actuation. Keep those defaults during inspection:

```bash
source "${UNITREE_INSTALL}/setup.bash"
source "${HELIX_BUILD}/install/setup.bash"
ros2 launch helix_bringup helix_native.launch.py
```

In a second terminal with the same validated ROS environment:

```bash
python3 scripts/hw/controller_inspect.py --evidence-class HW-passive \
  --seconds 600 --warmup 30 --query-metadata --out inspection.json
```

The inspector defaults to subscriptions only. `--query-metadata` adds exactly
these five finite read-only requests, with payload `{}` and no retry:

| Purpose | Exact request topic | API |
|---|---|---:|
| Sport service API version | `/api/sport/request` | 1 |
| Robot-state service API version | `/api/robot_state/request` | 1 |
| Motion-switcher service API version | `/api/motion_switcher/request` | 1 |
| Service list | `/api/robot_state/request` | 1003 |
| Check current mode | `/api/motion_switcher/request` | 1001 |

The allowlist is service-scoped. Sport API 1001 is never sent. No command API,
arbitrary payload, service name or topic override is accepted. Responses must match
service, request ID and API before their documented fields are retained. Unknown
schemas, negative replies and missing responses remain explicit failures. Metadata
queries do not measure command timeout or test any stopping mechanism.

Do not run the vendor's [robot-state example](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/go2/go2_robot_state_client.cpp#L45)
as an inspector: it calls `SetReportFreq` and switches a service off and on. The
query-only tool does neither. The official [mode client](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/comm/motion_switcher/motion_switcher_client.py#L23)
also separates CheckMode from SelectMode and ReleaseMode.

For native readiness, require all six exact native executables, active lifecycle
nodes, fresh native status and sensor data, `dry_run`, actuation disabled, zero
arbiter output, and its protective latch retained throughout. The inspector samples
CPU using monotonic intervals and PID/start-time identity, filters the current ROS
domain, and reports mean/p95/max CPU, RSS and observed peak thermal-zone temperature.
CPU is percent of one core; RSS is KiB. The aggregate sums simultaneous samples.
It excludes the observer's own cost and records missing processes. Review observer
overhead and the representative payload load separately. A short capture does not
establish scheduling tails, thermal stability or a hardware stop guarantee.

The local [dry integration/resource record](../results/nonactuating_native_host_sim.json)
uses FakeGo2, synthetic LowState, localhost DDS and an x86 host. It validates the
inspection interface and non-actuating integration only. It is not ARM resource
validation. Run the process regressions with the vendor messages sourced:

```bash
python3 -m pytest tests/sim_integration/test_controller_inspection.py -v --timeout=60
```

The [supervised walking-stop protocol](WALKING_STOP_EXPERIMENT.md) specifies the
remaining physical evidence and the separate final-path contingency. Its preparation
does not authorize execution.
