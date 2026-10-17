# GO2 deployment preparation

Prepared on 2026-10-17 against HELIX revision `3f648a8`, including the
uncommitted preparation changes described here. This is an offline deployment
runbook. No Jetson connection, remote installation, real Sport API request,
robot motion, service change, model download, or training occurred during
preparation. Nothing was committed or pushed.

## Environment

The development host supports Humble natively. No container or host ROS
replacement was necessary. The target is Ubuntu 22.04, Python 3.10 and ROS 2
Humble on the Jetson Orin NX's aarch64 CPU. Build from source on the target;
the workstation's amd64 binaries must not be copied there. The current
Jetson OS and JetPack version cannot be confirmed while it is unavailable.

| Component | Validated workstation version |
|---|---|
| OS / architecture | Ubuntu 22.04.5 LTS / x86_64 |
| Python | 3.10.12 |
| ROS base | Humble, `0.10.0-1jammy.20260915.225919` |
| rclpy / rclcpp | 3.3.22 / 16.0.21 |
| Cyclone DDS RMW | 1.3.5 |
| twist_mux | 4.3.0 |
| CMake / g++ metapackage | 3.22.1 / 11.2.0 |
| colcon-core | 0.21.3 |
| NumPy / PyYAML / requests | 1.21.5 / 5.4.1 / 2.25.1, Ubuntu packages |
| pytest / pytest-timeout | 7.4.4 / 2.3.1, isolated virtual environment |
| Unitree API message sources | `unitree_ros2` revision `5204e6e098ee53f4bd929bd77eb1d387cd0fa842` |

`scripts/requirements-deployment.txt` pins the complete Python test-tool
dependency set. ROS bindings and runtime libraries come from Ubuntu/ROS apt
packages. `PYTHONNOUSERSITE=1` prevents user-installed pytest 9, NumPy 2, and
other packages from shadowing those libraries. Humble's legacy launch-testing
plugin requires pytest below version 8. Build artifacts, virtual environments,
portable wheels and detailed results live under ignored `.deployment/`.
Existing `build/`, `install/`, `log/`, configurations and hardware-result
directories are preserved.

Each build writes `.deployment/evidence/apt-versions.txt` and
`python-versions.txt`. These record installed package versions; apt installs
resolve against the configured Ubuntu/ROS repositories rather than an
immutable apt mirror. The Python wheelhouse and source bundle are frozen,
but the whole operating system is not a bitwise reproducible image.

## Architecture and command paths

| Location | Responsibility |
|---|---|
| GO2 controller | Existing sensors, telemetry, Sport API and stock services. Unmodified. |
| Jetson | HELIX messages, core sensing, adapters, context/diagnosis, recovery, arbiter and dry-run Sport sink. Keep the recovery chain on one companion computer. |
| External computer | Development, offline testing, source-bundle creation and optional recording/console observation. No external computer is required in HELIX's recovery loop. |

All nine HELIX packages build together: `helix_msgs`, `helix_core`,
`helix_adapter`, `helix_diagnosis`, `helix_recovery`, `helix_explanation`,
`helix_arbiter`, `helix_bringup`, and `helix_sensing_cpp`. The C++ anomaly
detector is built and tested, but the preparation launcher retains the
existing Python sensing path. This preparation does not switch runtime
backends or tune the detector. The LLM explainer stays in template mode;
no model server, CUDA stack, or inference environment is required.

The separate `helix_go2.launch.py` preparation profile fixes:

- `sim_mode=false`, using `helix_adapter_params.yaml` and the shipped
  `helix_params.yaml` (`zscore_threshold=4.0`, `window_size=60`).
- Sensing, adapter, context and diagnosis lifecycle nodes active.
- Recovery disabled and unconfigured; legacy velocity publication disabled.
- `enable_twist_mux=false`, with the existing `arbiter_dry.yaml` configuration.
- Arbiter output `/helix_dry/cmd_vel`, inputs `/helix_dry/nav/cmd_vel` and
  `/helix_dry/teleop/cmd_vel`.
- Sport sink mode fixed to `dry_run`, reading the isolated arbiter output.
  It creates no `/api/sport/request` publisher. There is no launch argument
  to arm this profile.

The preparation output stays zero while recovery is unconfigured. Missing
sensors can still produce diagnostic faults, which is expected offline and
does not establish a hardware fault. The profile must run alone when using
these node names. It is never started by either deployment script.

| Path | Actual transport and meaning |
|---|---|
| Simulation | Existing simulation launcher/bridge consumes `geometry_msgs/Twist` on `/cmd_vel`. The legacy `twist_mux` option remains available. |
| Preparation | Arbiter to `/helix_dry/cmd_vel` to dry-run sink. No robot command transport. |
| Real GO2, future operator-gated validation | Diagnosis to recovery to `/helix/hold` to arbiter to `/cmd_vel` to Sport sink to `unitree_api/Request` on `/api/sport/request`. Responses use `/api/sport/response`. |

A stock GO2 does not acquire a Twist consumer merely because `/cmd_vel`
exists. The sink is the required hardware boundary. Its allowlist is
StopMove (1003) and Move (1008); it rejects unrelated API IDs. `stop_only`
still sends real StopMove commands and is not a passive mode. Neither it
nor `armed` is selected during preparation. Unitree's
[ROS 2 integration](https://github.com/unitreerobotics/unitree_ros2) and
[Sport client definitions](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/include/common/ros2_sport_client.h)
document that request transport and API numbering.

The existing recovery enable flag, allowlist, cooldown, RESUME handling,
hold watchdogs and arbiter priorities are unchanged. The existing simulation
and `twist_mux` configuration are unchanged. `twist_mux` is not an equivalent
physical stopping path: its idle/lock silence and non-finite forwarding
limitations remain documented in [MOTION_ARBITRATION.md](MOTION_ARBITRATION.md).

## Telemetry configuration

The hardware adapter subscribes to `/utlidar/imu`, `/utlidar/robot_odom`,
`/utlidar/robot_pose`, `/utlidar/cloud`, `/gnss` and `/multiplestate`.
IMU and point-cloud rate subscriptions use best-effort, volatile QoS;
odometry, pose and JSON subscriptions use reliable, volatile QoS. Verify
these against the actual publishers when the robot returns. The simulator's
cloud remap and reduced JSON-source coverage are confined to its existing
simulation profile.

The adapter produces `/helix/metrics`; sensing produces `/helix/faults`;
diagnosis produces `/helix/recovery_hints`; enabled recovery owns
`/helix/hold` and `/helix/recovery_actions`. Stock telemetry is not treated
as native HELIX heartbeat data. Native `/diagnostics` availability depends
on the robot's active software mode. No new LowState adapter is introduced.

## Local preparation and validation

From the repository root:

```bash
./scripts/deploy_go2.sh --dry-run
./scripts/deploy_go2.sh --check
./scripts/deploy_go2.sh --build
./scripts/deploy_go2.sh --validate
```

Dry-run performs source/config checks and prints a plan. It needs neither
ROS nor remote access, does not create state, and reports YAML parsing as
unverified if PyYAML is absent. `--check` validates Ubuntu/Humble, rosdep,
Python ABI and imports. `--build` uses the existing `src/` colcon layout
with separate build/install/log paths, validates installed interfaces and
launch descriptions, and writes `.deployment/env.bash`. `--validate` runs
root tests, one Python invocation per package, and C++/ament checks. It
returns a failure if any required build or test fails.

If using an existing compatible Unitree installation, supply its install
prefix to build/check/validate:

```bash
./scripts/deploy_go2.sh --build --unitree-underlay "$UNITREE_INSTALL_PREFIX"
./scripts/deploy_go2.sh --validate --unitree-underlay "$UNITREE_INSTALL_PREFIX"
```

This is optional for preparation's dry-run sink. For a self-contained
hardware-ready source release, include Unitree API message sources in the
bundle below. That builds only `unitree_api`, not vendor motion examples,
drivers or a replacement Cyclone DDS. Interface/schema checks instantiate
messages without creating a publisher.

Offline tests select `config/cyclonedds_offline.xml`: loopback only,
multicast disabled, explicit localhost discovery and 64 available participant
indices. Do not combine explicit loopback selection with
`ROS_LOCALHOST_ONLY=1` on this Humble RMW; it selects the interface twice.
The validation script sets `ROS_LOCALHOST_ONLY=0` while retaining the explicit
loopback binding. Existing package-specific test domains remain in force.
Run validation suites serially because they use fixed DDS domains.

The offline XML is not the robot-network DDS configuration. Before live
monitoring, use a privately configured Cyclone DDS file for the actual robot
interface and the correct ROS domain. No interface, address, clock, firewall
or existing service configuration is changed by preparation tooling.

## Frozen source bundle

Stage Python wheels while internet is available. The pinned test dependencies
are pure-Python wheels usable on both workstation and target:

```bash
mkdir -p .deployment/wheels
PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages .deployment/download-env
PYTHONNOUSERSITE=1 .deployment/download-env/bin/python -m pip download \
  -r scripts/requirements-deployment.txt --only-binary=:all: \
  --dest .deployment/wheels
```

Use a local `unitree_ros2` checkout at the revision in the environment table,
then freeze the actual HELIX working tree:

```bash
export HELIX_UNITREE_SOURCE="$HOME/unitree_ros2"
python3 -s scripts/deploy_go2_remote.py --bundle .deployment/helix-go2.tar.gz \
  --unitree-source "$HELIX_UNITREE_SOURCE"
python3 -s scripts/deploy_go2_remote.py --dry-run .deployment/helix-go2.tar.gz
sha256sum .deployment/helix-go2.tar.gz
```

If needed, obtain the public interface sources before entering an offline lab:

```bash
git clone --no-checkout https://github.com/unitreerobotics/unitree_ros2.git .deployment/unitree_ros2
git -C .deployment/unitree_ros2 checkout --detach 5204e6e098ee53f4bd929bd77eb1d387cd0fa842
export HELIX_UNITREE_SOURCE="$PWD/.deployment/unitree_ros2"
```

Re-run the bundle command after selecting that source directory. The bundle
includes tracked working-tree files, the explicitly scoped preparation files,
Python wheels and optional Unitree API source/license. Its manifest records
source revisions and SHA-256 for every included file. It preserves uncommitted
tracked changes without creating commits. Agent instructions, untracked
experiments/results, raw `.db3`/`.mcap` recordings and local build outputs
are excluded. JSON replay fixtures and their provenance remain included.
Repeated bundling of unchanged inputs produces identical archive bytes.

The staged bundle was extracted and built locally with no external Unitree
underlay and with pip networking disabled. This validates the source/wheel
staging on amd64, not target-native arm64 compilation.

## Deployment when the Jetson is available

Remote installation is an explicit separate operation. It uses LabLink's
logical device name and identity checks, never remembered addresses or raw
SSH/scp. Select the Jetson's canonical logical name from local LabLink
configuration:

```bash
read -r -p 'Jetson LabLink logical name: ' HELIX_LAB_DEVICE
python3 -s scripts/deploy_go2_remote.py \
  --remote-install .deployment/helix-go2.tar.gz --device "$HELIX_LAB_DEVICE"
```

The script runs LabLink diagnostics and mandatory device status checks,
verifies the target is aarch64 Ubuntu 22.04 before writing, transfers and
checksums the archive, and builds under
`~/helix-deploy/releases/<bundle-sha256>/`. Existing workspaces and GO2
services are untouched. A matching successful-release marker makes a repeated
installation a no-op after transfer verification. Failed builds retain logs
for diagnosis and can be retried. No nodes, motion commands, systemd services
or boot configuration are started or enabled. Host-key and identity failures
stop the operation. An older LabLink without `doctor` may proceed only to
mandatory `status`, never around an identity failure.

If dependency checks fail, install missing packages only through a separate
explicit dependency operation. On an internet-enabled Ubuntu 22.04 target
with the official Humble apt repository already configured, use:

```bash
python3 -s scripts/deploy_go2_remote.py \
  --remote-install .deployment/helix-go2.tar.gz --device "$HELIX_LAB_DEVICE" \
  --install-deps
```

This flag permits the documented apt package install and rosdep cache setup.
It does not configure the ROS apt repository or upgrade JetPack/firmware.
The lab may have no internet: ensure Humble, the listed apt dependencies and
the rosdep cache are present before that visit, or bring a matching arm64
apt cache with its transitive dependencies. An amd64 package cache is not
usable on the Jetson. Wheels alone do not supply ROS or system libraries.

For manual installation from an already transferred and verified bundle,
extract it into a new directory on the Jetson, then run:

```bash
./scripts/deploy_go2.sh --check
./scripts/deploy_go2.sh --build
./scripts/deploy_go2.sh --validate
source .deployment/env.bash
```

The last command selects only the terminal's ROS/Python environment. After
checking the live NIC, ROS domain, DDS file, clock and exclusive node ownership,
monitoring is a separate manual step:

```bash
ros2 launch helix_bringup helix_go2.launch.py
```

This launch is still the non-actuating preparation profile. Real motion
validation uses the separate operator-gated
[HW_MOTION_TEST.md](HW_MOTION_TEST.md) procedure. Do not treat installation,
dry-run launch or a zero Twist trace as permission to select a live sink mode.

Rollback consists of leaving this environment/terminal and selecting the prior
workspace or release in a new shell. No active-release symlink or service is
changed. Once its processes are stopped and evidence is retained, a new
preparation release directory can be removed. Optional apt installs are shared
system dependencies; their removal is not automatic and requires a separate
dependency review.

## Results and remaining validation

The fresh native build completed all nine HELIX packages. A separately
extracted bundle completed ten packages including the staged `unitree_api`.
All five installed launch descriptions imported successfully. Rosdep reports
all system dependencies satisfied. The recorded GO2 `extended_5min` capture
was opened read-only: 82,367 IMU messages, 6,177 poses, 5,077 point clouds,
330 GNSS messages, 328 multistate messages and 46 rosout messages. Its measured
IMU rate was approximately 250 Hz. This is recorded-data inspection, not live
sensor validation. Existing replay tests also exercise real recorded telemetry
and the versioned Session 8 event fixture.

| Python suite | Passed | Failed / skipped |
|---|---:|---:|
| Root integrity, recorded-data replay and deployment tooling | 138 | 0 / 0 |
| helix_core | 33 | 0 / 0 |
| helix_adapter | 40 | 0 / 0 |
| helix_diagnosis | 24 | 0 / 0 |
| helix_recovery | 22 | 0 / 0 |
| helix_explanation | 26 | 0 / 0 |
| helix_arbiter | 156 | 0 / 0 |
| helix_bringup | 20 | 0 / 0 |
| Total | 459 | 0 / 0 |

The arbiter suite includes 19 process-level integration cases covering its
20-scenario matrix. Bringup includes real `twist_mux` tests, launch expansion
checks, and a complete installed preparation launch: monitoring and arbiter
active, recovery unconfigured, isolated zero output, and no publisher on
`/cmd_vel`, `/helix/cmd_vel`, `/helix/hold` or `/api/sport/request`.
Sixty-six root tests exercise preparation tooling, including mocked LabLink,
corrupt archives, target preflight and repeated-install behavior. These mocks
do not establish real network-transfer behavior.

C++ validation passed all **38 gtest cases** and all **10 CTest commands**.
`colcon test-result` reported **73 results, 0 errors, 0 failures, 7 skipped**;
that aggregate includes CTest wrappers and individual checks, so it is not
73 distinct C++ unit tests. The seven skips are ament's cppcheck checks,
explicitly disabled by the installed checker for cppcheck 2.7 performance
issues. They are not counted as passed. Ruff, shell syntax, manifest/XML/YAML
checks and `git diff --check` passed.

Machine-readable outputs remain in `.deployment/evidence/` and
`.deployment/build/helix_sensing_cpp/test_results/`. Isaac Sim scenarios,
real remote transfer/apt installation, arm64 execution and physical hardware
tests were not run. No skipped simulator or hardware test is classified as
passed.

Resolved preparation defects:

- Replaced unresolvable `ament_python` and `pytest` rosdep keys with
  `python3-setuptools` and `python3-pytest`; declared YAML and missing launch/test
  dependencies, including diagnosis/recovery dependencies for arbiter tests.
- Isolated incompatible user-site Python packages without changing the host's
  installed Python stack.
- Added loopback unicast DDS discovery and sufficient participant indices for
  the complete multi-process preparation graph. Original DDS files are preserved.
- Changed the Session 8 replay test to use the repository's byte-identical
  versioned event fixture instead of an external workstation-specific path.
- Fixed `twist_mux` integration-test process cleanup to terminate its child
  executable as well as the `ros2 run` wrapper, and gave the preparation-graph
  integration test its own DDS domain so repeated validation stays isolated.
- Corrected the lifecycle integration test's asynchronous boundary: ticks
  received while the deactivation request is still in flight are active-state
  output. The check now requires the shutdown zero and verifies that every
  subsequent output stays zero. Arbiter lifecycle behavior is unchanged.

Hardware-only checks remain:

1. Confirm Ubuntu/JetPack, Python ABI, free disk/RAM and native arm64 build on
   the returned Jetson. If it runs Ubuntu 20.04, use an Ubuntu 22.04 Humble
   container or a separately approved OS setup; do not force Jammy packages
   onto the older host.
2. Verify real NIC selection, ROS domain, DDS interoperability, current clock
   and robot timestamp skew. Topic discovery alone does not establish data flow.
3. Verify actual telemetry types, rates, QoS and freshness, including odometry
   and the mode-dependent JSON/diagnostic topics. The recorded capture lacks
   robot odometry and cannot prove that subscription.
4. Confirm Unitree interface compatibility, request leases, competing stock
   publishers and actual Sport API acknowledgements from this sink.
5. Conduct stages A through F under the existing hardware protocol. Previous
   remapped dry A/C evidence does not prove the real command path. Physical
   stopping time/distance, hold enforcement while upstream commands continue,
   and operator re-arm require real B/D/E/F evidence, especially stage E.
6. Measure end-to-end latency, CPU/RSS, drops, sustained stability, thermal
   behavior and detector false positives under representative Jetson load.
   Workstation tests and idle bag replay do not establish these measurements.

No hardware result or physical stopping claim is added by this preparation.

The hardware evidence runner additionally requires a clean Git checkout at
the exact candidate commit. The preparation archive is a source snapshot
with file hashes, without Git metadata, and includes uncommitted changes.
It is suitable for environment installation and monitoring, not for satisfying
that evidence gate. Before collecting new physical evidence, separately
authorize a candidate commit, use its clean checkout, and build the same
configuration there. The existing clean-tree requirement is preserved.
