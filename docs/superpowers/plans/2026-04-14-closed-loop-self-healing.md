# HELIX Closed-Loop Self-Healing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend HELIX from sensing-only to a closed-loop self-healing system that detects a LiDAR fault on the Unitree GO2, diagnoses it via deterministic rules, and commands a safe stop-and-resume via `twist_mux` priority arbitration — validated in Isaac Sim and on real hardware, filmed as an industry-facing demo.

**Architecture:** Four ROS 2 lifecycle-node tiers (Sensing → Context → Diagnosis → Recovery) plus a parallel `LLMExplainer` sidecar. Recovery publishes on a priority input to `twist_mux` for safe command arbitration. Safety envelope (enable flag, per-fault cooldown, action allowlist, `twist_mux` timeout) lives in the `RecoveryNode` alone.

**Tech Stack:** ROS 2 Humble, `rclpy`, `rosidl` (msgs), `ament_python` / `ament_cmake`, Isaac Sim 6.0 (via existing bridge), `pytest`, Graphviz, `ffmpeg`.

**Spec reference:** `docs/superpowers/specs/2026-04-14-closed-loop-self-healing-design.md`

**Branch:** `feat/self-healing-closed-loop` (already created).

---

## File structure

### New packages
- `src/helix_diagnosis/` — `ContextBuffer`, `DiagnosisNode`, `rules.py`
- `src/helix_recovery/` — `RecoveryNode` (safety envelope lives here)
- `src/helix_explanation/` — `LLMExplainer` (v1 stub, template-based)

### New messages / services (in existing `src/helix_msgs/`)
- Extend `msg/RecoveryHint.msg` — add `string rule_matched` field
- Create `msg/RecoveryAction.msg` — audit events
- Create `srv/GetContext.srv` — for `ContextBuffer` queries

### New configuration
- `config/twist_mux.yaml` — priority topology for HELIX's cmd_vel

### New tooling
- `scripts/sim_faults/inject_lidar_rate_drop.py` — relay throttle
- `scripts/sim_faults/inject_node_crash.py` — kill named node
- `scripts/sim_faults/run_closed_loop_scenario.py` — sim orchestrator

### New tests
- `src/helix_diagnosis/test/test_rules.py`
- `src/helix_diagnosis/test/test_context_buffer.py`
- `src/helix_diagnosis/test/test_diagnosis_node.py`
- `src/helix_recovery/test/test_recovery_node.py`
- `src/helix_explanation/test/test_llm_explainer.py`
- `tests/sim_integration/test_lidar_occlusion_recovery.py`

### Modified files
- `src/helix_msgs/CMakeLists.txt` — register new msg/srv
- `src/helix_bringup/helix_bringup/fault_injector.py` — add sim-targeted modes
- `launch/helix_closed_loop.launch.py` — new launch file for the full stack
- `Makefile` (create if missing) — `test-sim` target
- `README.md` — architecture section, demo GIF, quick-start
- `.github/workflows/ci.yml` — ensure new packages are built in CI

### Convention notes (from repo inspection)
- Lifecycle nodes inherit `rclpy.lifecycle.LifecycleNode`.
- Python packages use `ament_python` (setup.py + package.xml, `entry_points` for console scripts).
- Test directory has `conftest.py` with session-scoped `rclpy.init()` / `rclpy.shutdown()`.
- `helix_msgs` is `ament_cmake` (for `rosidl_generate_interfaces`).
- Commit messages: lower-case type prefix (`feat:`, `fix:`, `docs:`), no Claude co-author (per `~/.claude/CLAUDE.md`).

---

## Phase 1 — Scaffolding (msgs, srv, packages, build)

**Goal:** Every new interface resolves via `ros2 interface show`; every new package compiles. No business logic yet.

### Task 1.1 — Extend `RecoveryHint.msg`

**Files:**
- Modify: `src/helix_msgs/msg/RecoveryHint.msg`

- [ ] **Step 1: Edit the msg**

Replace contents with:

```
# RecoveryHint — emitted by DiagnosisNode, consumed by RecoveryNode.
#
# fault_id       correlates with the FaultEvent that triggered this hint
# suggested_action  STOP_AND_HOLD | RESUME | LOG_ONLY (enforced by RecoveryNode allowlist)
# confidence     0.0 - 1.0
# reasoning      short human-readable rationale
# rule_matched   rule identifier from rules.py (R1, R2, R3, R4, ...)

string fault_id
string suggested_action
float32 confidence
string reasoning
string rule_matched
```

- [ ] **Step 2: Build and verify**

Run: `cd ~/workspace/helix && colcon build --packages-select helix_msgs --cmake-clean-cache`
Expected: build succeeds.

Run: `source install/setup.bash && ros2 interface show helix_msgs/msg/RecoveryHint`
Expected: output contains the new `string rule_matched` line.

- [ ] **Step 3: Commit**

```bash
cd ~/workspace/helix
git add src/helix_msgs/msg/RecoveryHint.msg
git commit -m "msg: add rule_matched field to RecoveryHint"
```

### Task 1.2 — Create `RecoveryAction.msg`

**Files:**
- Create: `src/helix_msgs/msg/RecoveryAction.msg`
- Modify: `src/helix_msgs/CMakeLists.txt`

- [ ] **Step 1: Create the msg**

Contents of `src/helix_msgs/msg/RecoveryAction.msg`:

```
# RecoveryAction — audit log event from RecoveryNode.
#
# status values:
#   ACCEPTED              — hint passed all safety checks; action fired
#   SUPPRESSED_COOLDOWN   — same-type action fired within cooldown window
#   SUPPRESSED_ALLOWLIST  — hint requested action outside ALLOWED_ACTIONS
#   SUPPRESSED_DISABLED   — recovery.enabled param is false
#
# reason is always human-readable and never empty.

string fault_id
string action
string status
float64 timestamp
string reason
```

- [ ] **Step 2: Register in CMakeLists.txt**

Edit `src/helix_msgs/CMakeLists.txt`. Find the `rosidl_generate_interfaces` block and add `"msg/RecoveryAction.msg"`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaultEvent.msg"
  "msg/RecoveryHint.msg"
  "msg/RecoveryAction.msg"
  DEPENDENCIES std_msgs
)
```

- [ ] **Step 3: Build and verify**

Run: `cd ~/workspace/helix && colcon build --packages-select helix_msgs --cmake-clean-cache && source install/setup.bash && ros2 interface show helix_msgs/msg/RecoveryAction`
Expected: full message body prints, no errors.

- [ ] **Step 4: Commit**

```bash
git add src/helix_msgs/msg/RecoveryAction.msg src/helix_msgs/CMakeLists.txt
git commit -m "msg: add RecoveryAction audit message"
```

### Task 1.3 — Create `GetContext.srv`

**Files:**
- Create: `src/helix_msgs/srv/GetContext.srv`
- Modify: `src/helix_msgs/CMakeLists.txt`

- [ ] **Step 1: Create the srv**

Contents of `src/helix_msgs/srv/GetContext.srv`:

```
# GetContext — served by ContextBuffer, queried by DiagnosisNode.
# Request has no parameters; response carries the current context snapshot.
---
string[] rosout_ring
string metrics_json
string node_health_json
float64 snapshot_time
```

- [ ] **Step 2: Register in CMakeLists.txt**

Edit `src/helix_msgs/CMakeLists.txt` `rosidl_generate_interfaces` block to add the srv:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaultEvent.msg"
  "msg/RecoveryHint.msg"
  "msg/RecoveryAction.msg"
  "srv/GetContext.srv"
  DEPENDENCIES std_msgs
)
```

- [ ] **Step 3: Build and verify**

Run: `cd ~/workspace/helix && colcon build --packages-select helix_msgs --cmake-clean-cache && source install/setup.bash && ros2 interface show helix_msgs/srv/GetContext`
Expected: request + response sections show in output.

- [ ] **Step 4: Commit**

```bash
git add src/helix_msgs/srv/GetContext.srv src/helix_msgs/CMakeLists.txt
git commit -m "srv: add GetContext service for ContextBuffer queries"
```

### Task 1.4 — Scaffold `helix_diagnosis` package

**Files:**
- Create: `src/helix_diagnosis/package.xml`
- Create: `src/helix_diagnosis/setup.py`
- Create: `src/helix_diagnosis/setup.cfg`
- Create: `src/helix_diagnosis/resource/helix_diagnosis` (empty marker file)
- Create: `src/helix_diagnosis/helix_diagnosis/__init__.py` (empty)
- Create: `src/helix_diagnosis/test/conftest.py`

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>helix_diagnosis</name>
  <version>0.1.0</version>
  <description>HELIX diagnosis tier: context buffer + rule-based diagnosis node</description>
  <maintainer email="yusuf.a.guenena@gmail.com">yusufdxb</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>diagnostic_msgs</depend>
  <depend>rcl_interfaces</depend>
  <depend>helix_msgs</depend>
  <depend>lifecycle_msgs</depend>

  <test_depend>pytest</test_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>

  <buildtool_depend>ament_python</buildtool_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `setup.py`**

```python
from setuptools import setup

package_name = 'helix_diagnosis'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='HELIX diagnosis tier',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helix_context_buffer = helix_diagnosis.context_buffer:main',
            'helix_diagnosis_node = helix_diagnosis.diagnosis_node:main',
        ],
    },
)
```

- [ ] **Step 3: Create `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/helix_diagnosis
[install]
install_scripts=$base/lib/helix_diagnosis
```

- [ ] **Step 4: Create resource marker and package init**

```bash
mkdir -p ~/workspace/helix/src/helix_diagnosis/resource ~/workspace/helix/src/helix_diagnosis/helix_diagnosis ~/workspace/helix/src/helix_diagnosis/test
touch ~/workspace/helix/src/helix_diagnosis/resource/helix_diagnosis
touch ~/workspace/helix/src/helix_diagnosis/helix_diagnosis/__init__.py
```

- [ ] **Step 5: Create `test/conftest.py`** (mirror the existing helix_core convention)

```python
"""Session-scoped rclpy init/shutdown for all helix_diagnosis tests."""
import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()
```

- [ ] **Step 6: Build and verify**

Run: `cd ~/workspace/helix && colcon build --packages-select helix_diagnosis`
Expected: build succeeds (no python source yet; empty package is fine).

- [ ] **Step 7: Commit**

```bash
git add src/helix_diagnosis/
git commit -m "feat: scaffold helix_diagnosis package"
```

### Task 1.5 — Scaffold `helix_recovery` package

**Files:**
- Create: `src/helix_recovery/package.xml`
- Create: `src/helix_recovery/setup.py`
- Create: `src/helix_recovery/setup.cfg`
- Create: `src/helix_recovery/resource/helix_recovery`
- Create: `src/helix_recovery/helix_recovery/__init__.py`
- Create: `src/helix_recovery/test/conftest.py`

- [ ] **Step 1: Create all files** — same structure as Task 1.4, but package name `helix_recovery`, description "HELIX recovery tier: safety-envelope-enforcing actuation", entry points:

```python
entry_points={
    'console_scripts': [
        'helix_recovery_node = helix_recovery.recovery_node:main',
    ],
},
```

Additional `package.xml` depends (add `geometry_msgs`):

```xml
<depend>geometry_msgs</depend>
```

- [ ] **Step 2: conftest.py** — identical to Task 1.4 Step 5.

- [ ] **Step 3: Build**

Run: `cd ~/workspace/helix && colcon build --packages-select helix_recovery`
Expected: success.

- [ ] **Step 4: Commit**

```bash
git add src/helix_recovery/
git commit -m "feat: scaffold helix_recovery package"
```

### Task 1.6 — Scaffold `helix_explanation` package

**Files:** analogous to 1.4/1.5. Package name `helix_explanation`. Description "HELIX LLM explainer sidecar (v1 template stub)". Entry point `helix_llm_explainer = helix_explanation.llm_explainer:main`. No `geometry_msgs` dep; `std_msgs` is sufficient.

- [ ] **Step 1: Create all files** (same pattern)
- [ ] **Step 2: Build and verify** — `colcon build --packages-select helix_explanation`
- [ ] **Step 3: Commit**

```bash
git add src/helix_explanation/
git commit -m "feat: scaffold helix_explanation package"
```

### Task 1.7 — Verify full workspace build

- [ ] **Step 1: Clean-build everything**

Run: `cd ~/workspace/helix && colcon build --cmake-clean-cache`
Expected: all packages build. No errors. Warnings about empty python packages are OK.

- [ ] **Step 2: Source and list interfaces**

Run: `source install/setup.bash && ros2 interface list | grep helix_msgs`
Expected output (among others):
```
helix_msgs/msg/FaultEvent
helix_msgs/msg/RecoveryAction
helix_msgs/msg/RecoveryHint
helix_msgs/srv/GetContext
```

- [ ] **Step 3: No commit** — verification only.

---

## Phase 2 — Core implementation (TDD, unit tests first)

**Goal:** Every node has complete business logic + unit tests. Nodes launch cleanly in lifecycle `unconfigured → inactive → active`. No sim, no hardware yet.

### Task 2.1 — Implement `rules.py` (pure function, TDD)

**Files:**
- Create: `src/helix_diagnosis/test/test_rules.py`
- Create: `src/helix_diagnosis/helix_diagnosis/rules.py`

- [ ] **Step 1: Write the failing tests**

Contents of `test/test_rules.py`:

```python
"""Unit tests for helix_diagnosis.rules — pure function, no ROS 2 runtime."""
from types import SimpleNamespace

from helix_diagnosis.rules import evaluate, STATE_IDLE, STATE_STOP_AND_HOLD


def _fault(fault_type='ANOMALY', metric='utlidar_rate', severity=2, fault_id='f1'):
    """Build a minimal FaultEvent-shaped object for rule evaluation."""
    return SimpleNamespace(
        fault_id=fault_id,
        fault_type=fault_type,
        severity=severity,          # 1=WARN, 2=ERROR, 3=CRITICAL
        detail='',
        timestamp=0.0,
        context_keys=['metric'],
        context_values=[metric],
    )


def _ctx(anomaly_clear_seconds=0.0):
    return SimpleNamespace(
        rosout_ring=[],
        metrics_json='{}',
        node_health_json='{}',
        snapshot_time=0.0,
        anomaly_clear_seconds=anomaly_clear_seconds,
    )


def test_r1_fires_on_lidar_rate_anomaly_error():
    hint = evaluate(_fault(), _ctx(), current_state=STATE_IDLE)
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert hint.rule_matched == 'R1'
    assert hint.fault_id == 'f1'


def test_r1_does_not_fire_on_warn_severity():
    hint = evaluate(_fault(severity=1), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_r1_does_not_fire_on_unrelated_metric():
    hint = evaluate(_fault(metric='cpu_temp'), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_r2_fires_when_anomaly_has_cleared_and_state_is_stop():
    # No fault event, but we have state + clear window
    hint = evaluate(None, _ctx(anomaly_clear_seconds=3.5), current_state=STATE_STOP_AND_HOLD)
    assert hint is not None
    assert hint.suggested_action == 'RESUME'
    assert hint.rule_matched == 'R2'


def test_r2_does_not_fire_before_clear_window():
    hint = evaluate(None, _ctx(anomaly_clear_seconds=1.0), current_state=STATE_STOP_AND_HOLD)
    assert hint is None


def test_r2_does_not_fire_when_idle():
    hint = evaluate(None, _ctx(anomaly_clear_seconds=10.0), current_state=STATE_IDLE)
    assert hint is None


def test_r3_fires_on_critical_log_pattern():
    hint = evaluate(_fault(fault_type='LOG_PATTERN', severity=3), _ctx(), current_state=STATE_IDLE)
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert hint.rule_matched == 'R3'


def test_r3_does_not_fire_on_error_log_pattern():
    hint = evaluate(_fault(fault_type='LOG_PATTERN', severity=2), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_r4_log_only_on_crash():
    hint = evaluate(_fault(fault_type='CRASH', severity=3), _ctx(), current_state=STATE_IDLE)
    assert hint is not None
    assert hint.suggested_action == 'LOG_ONLY'
    assert hint.rule_matched == 'R4'


def test_no_match_returns_none():
    hint = evaluate(_fault(fault_type='NETWORK'), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_first_match_wins():
    # Craft a fault that matches both R3 and a hypothetical later rule — R3 must win.
    hint = evaluate(_fault(fault_type='LOG_PATTERN', severity=3), _ctx(), current_state=STATE_IDLE)
    assert hint.rule_matched == 'R1' or hint.rule_matched == 'R3'
    # Specifically R3 in current ordering:
    assert hint.rule_matched == 'R3'
```

- [ ] **Step 2: Run tests and confirm they fail**

Run: `cd ~/workspace/helix/src/helix_diagnosis && python -m pytest test/test_rules.py -v`
Expected: all tests FAIL with `ModuleNotFoundError: No module named 'helix_diagnosis.rules'` (or similar).

- [ ] **Step 3: Implement `rules.py`**

Contents of `helix_diagnosis/rules.py`:

```python
"""
Deterministic recovery rules for HELIX diagnosis tier.

Pure function `evaluate()` takes a FaultEvent-shaped object + a context snapshot
+ current recovery state, returns a RecoveryHint-shaped object or None.

No ROS 2 runtime dependency in this file — unit testable standalone.
First match wins. Rule order is the list order in `_RULES`.
"""
from dataclasses import dataclass
from typing import Any, List, Optional

STATE_IDLE = 'IDLE'
STATE_STOP_AND_HOLD = 'STOP_AND_HOLD'

SEVERITY_WARN = 1
SEVERITY_ERROR = 2
SEVERITY_CRITICAL = 3

ANOMALY_CLEAR_WINDOW_SECONDS = 3.0


@dataclass
class HintShape:
    """Shape-compatible with helix_msgs/RecoveryHint. Used so rules.py stays ROS-free."""
    fault_id: str
    suggested_action: str
    confidence: float
    reasoning: str
    rule_matched: str


def _metric_name(fault_event: Any) -> Optional[str]:
    """Pull metric_name from fault_event.context_keys/context_values."""
    try:
        idx = list(fault_event.context_keys).index('metric')
        return fault_event.context_values[idx]
    except (ValueError, AttributeError):
        return None


def _rule_r1(fault, ctx, state) -> Optional[HintShape]:
    if fault is None:
        return None
    if fault.fault_type != 'ANOMALY':
        return None
    if _metric_name(fault) != 'utlidar_rate':
        return None
    if fault.severity < SEVERITY_ERROR:
        return None
    return HintShape(
        fault_id=fault.fault_id,
        suggested_action='STOP_AND_HOLD',
        confidence=0.9,
        reasoning='LiDAR rate anomaly at ERROR severity — stop and hold',
        rule_matched='R1',
    )


def _rule_r2(fault, ctx, state) -> Optional[HintShape]:
    if state != STATE_STOP_AND_HOLD:
        return None
    clear_seconds = getattr(ctx, 'anomaly_clear_seconds', 0.0)
    if clear_seconds < ANOMALY_CLEAR_WINDOW_SECONDS:
        return None
    return HintShape(
        fault_id='',  # no fault correlation — RESUME is state-driven
        suggested_action='RESUME',
        confidence=0.9,
        reasoning=f'Anomaly cleared for {clear_seconds:.1f}s — resuming',
        rule_matched='R2',
    )


def _rule_r3(fault, ctx, state) -> Optional[HintShape]:
    if fault is None:
        return None
    if fault.fault_type != 'LOG_PATTERN':
        return None
    if fault.severity != SEVERITY_CRITICAL:
        return None
    return HintShape(
        fault_id=fault.fault_id,
        suggested_action='STOP_AND_HOLD',
        confidence=0.7,
        reasoning='Critical log pattern detected — defensive stop',
        rule_matched='R3',
    )


def _rule_r4(fault, ctx, state) -> Optional[HintShape]:
    if fault is None:
        return None
    if fault.fault_type != 'CRASH':
        return None
    return HintShape(
        fault_id=fault.fault_id,
        suggested_action='LOG_ONLY',
        confidence=0.5,
        reasoning='Node crash detected — logged, no actuation (scope limit)',
        rule_matched='R4',
    )


_RULES: List = [_rule_r1, _rule_r2, _rule_r3, _rule_r4]


def evaluate(fault_event: Any, context: Any, current_state: str) -> Optional[HintShape]:
    """Run rules in order; return first non-None hint, else None."""
    for rule in _RULES:
        hint = rule(fault_event, context, current_state)
        if hint is not None:
            return hint
    return None
```

- [ ] **Step 4: Run tests and confirm they pass**

Run: `cd ~/workspace/helix/src/helix_diagnosis && python -m pytest test/test_rules.py -v`
Expected: 11 passed.

- [ ] **Step 5: Commit**

```bash
cd ~/workspace/helix
git add src/helix_diagnosis/helix_diagnosis/rules.py src/helix_diagnosis/test/test_rules.py
git commit -m "feat(diagnosis): deterministic rule evaluator with unit tests"
```

### Task 2.2 — Implement `ContextBuffer` node

**Files:**
- Create: `src/helix_diagnosis/test/test_context_buffer.py`
- Create: `src/helix_diagnosis/helix_diagnosis/context_buffer.py`

- [ ] **Step 1: Write failing tests**

Contents of `test/test_context_buffer.py`:

```python
"""Tests for ContextBuffer — ring behavior and snapshot retrieval."""
from helix_diagnosis.context_buffer import RosoutRing


def test_ring_bounded_to_capacity():
    r = RosoutRing(capacity=3)
    for i in range(10):
        r.append(f'line_{i}')
    snap = r.snapshot()
    assert len(snap) == 3
    assert snap == ['line_7', 'line_8', 'line_9']


def test_ring_empty_by_default():
    r = RosoutRing(capacity=5)
    assert r.snapshot() == []


def test_ring_under_capacity():
    r = RosoutRing(capacity=5)
    r.append('a')
    r.append('b')
    assert r.snapshot() == ['a', 'b']
```

- [ ] **Step 2: Run test, confirm failure**

Run: `cd ~/workspace/helix/src/helix_diagnosis && python -m pytest test/test_context_buffer.py -v`
Expected: ImportError or similar.

- [ ] **Step 3: Implement `context_buffer.py`**

```python
"""
ContextBuffer — lifecycle node that maintains a bounded /rosout ring,
latest /helix/metrics snapshot, and latest /helix/node_health snapshot.
Serves GetContext srv.
"""
import json
import threading
from collections import deque
from typing import Deque, List

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray

from helix_msgs.srv import GetContext

ROSOUT_RING_CAPACITY: int = 200


class RosoutRing:
    """Pure bounded ring — unit-testable without ROS 2."""

    def __init__(self, capacity: int = ROSOUT_RING_CAPACITY):
        self._buf: Deque[str] = deque(maxlen=capacity)
        self._lock = threading.Lock()

    def append(self, line: str) -> None:
        with self._lock:
            self._buf.append(line)

    def snapshot(self) -> List[str]:
        with self._lock:
            return list(self._buf)


class ContextBuffer(LifecycleNode):
    """Serves current context snapshot via GetContext service."""

    def __init__(self):
        super().__init__('helix_context_buffer')
        self._ring = RosoutRing()
        self._latest_metrics: str = '{}'
        self._latest_health: str = '{}'
        self._sub_rosout = None
        self._sub_metrics = None
        self._sub_health = None
        self._srv = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._srv = self.create_service(GetContext, '/helix/get_context', self._handle_get_context)
        self.get_logger().info('ContextBuffer configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._sub_rosout = self.create_subscription(Log, '/rosout', self._on_rosout, 100)
        self._sub_metrics = self.create_subscription(
            Float64MultiArray, '/helix/metrics', self._on_metrics, 10)
        self._sub_health = self.create_subscription(
            DiagnosticArray, '/helix/node_health', self._on_health, 10)
        self.get_logger().info('ContextBuffer active')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        for sub in (self._sub_rosout, self._sub_metrics, self._sub_health):
            if sub is not None:
                self.destroy_subscription(sub)
        self._sub_rosout = self._sub_metrics = self._sub_health = None
        return TransitionCallbackReturn.SUCCESS

    def _on_rosout(self, msg: Log) -> None:
        self._ring.append(f'[{msg.level}] {msg.name}: {msg.msg}')

    def _on_metrics(self, msg: Float64MultiArray) -> None:
        self._latest_metrics = json.dumps({'data': list(msg.data)})

    def _on_health(self, msg: DiagnosticArray) -> None:
        statuses = [{'name': s.name, 'level': int(s.level), 'message': s.message}
                    for s in msg.status]
        self._latest_health = json.dumps({'status': statuses})

    def _handle_get_context(self, request, response):
        response.rosout_ring = self._ring.snapshot()
        response.metrics_json = self._latest_metrics
        response.node_health_json = self._latest_health
        response.snapshot_time = self.get_clock().now().nanoseconds / 1e9
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ContextBuffer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run unit tests, confirm pass**

Run: `cd ~/workspace/helix/src/helix_diagnosis && python -m pytest test/test_context_buffer.py -v`
Expected: 3 passed.

- [ ] **Step 5: Smoke test the node launches**

Run:
```bash
cd ~/workspace/helix && colcon build --packages-select helix_diagnosis && source install/setup.bash
timeout 3 ros2 run helix_diagnosis helix_context_buffer &
sleep 1
ros2 node list | grep helix_context_buffer
```
Expected: `helix_context_buffer` listed. Process ends cleanly after timeout.

- [ ] **Step 6: Commit**

```bash
git add src/helix_diagnosis/helix_diagnosis/context_buffer.py src/helix_diagnosis/test/test_context_buffer.py
git commit -m "feat(diagnosis): ContextBuffer lifecycle node with rosout ring"
```

### Task 2.3 — Implement `DiagnosisNode`

**Files:**
- Create: `src/helix_diagnosis/test/test_diagnosis_node.py`
- Create: `src/helix_diagnosis/helix_diagnosis/diagnosis_node.py`

- [ ] **Step 1: Write failing test for the state machine**

Contents of `test/test_diagnosis_node.py`:

```python
"""Tests for DiagnosisNode state machine (subscribed to /helix/faults)."""
from types import SimpleNamespace

from helix_diagnosis.diagnosis_node import DiagnosisStateMachine, STATE_IDLE, STATE_STOP_AND_HOLD


def _fault(ft='ANOMALY', sev=2, metric='utlidar_rate', fid='f1', ts=0.0):
    return SimpleNamespace(
        fault_id=fid,
        fault_type=ft,
        severity=sev,
        detail='',
        timestamp=ts,
        context_keys=['metric'],
        context_values=[metric],
    )


def test_starts_idle():
    sm = DiagnosisStateMachine()
    assert sm.current_state == STATE_IDLE


def test_anomaly_moves_to_stop():
    sm = DiagnosisStateMachine()
    hint = sm.process_fault(_fault(), now_seconds=1.0)
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert sm.current_state == STATE_STOP_AND_HOLD


def test_clear_window_resumes():
    sm = DiagnosisStateMachine()
    sm.process_fault(_fault(), now_seconds=1.0)
    # 4 seconds later with no faults, tick produces RESUME hint
    hint = sm.tick(now_seconds=5.0)
    assert hint is not None
    assert hint.suggested_action == 'RESUME'
    assert sm.current_state == STATE_IDLE


def test_clear_window_not_yet_elapsed():
    sm = DiagnosisStateMachine()
    sm.process_fault(_fault(), now_seconds=1.0)
    hint = sm.tick(now_seconds=2.0)   # only 1s
    assert hint is None
    assert sm.current_state == STATE_STOP_AND_HOLD


def test_new_anomaly_resets_clear_timer():
    sm = DiagnosisStateMachine()
    sm.process_fault(_fault(), now_seconds=1.0)
    sm.tick(now_seconds=2.5)                   # 1.5s clear
    sm.process_fault(_fault(fid='f2'), now_seconds=2.8)  # new fault resets
    hint = sm.tick(now_seconds=5.0)             # 2.2s since reset — should NOT resume
    assert hint is None
```

- [ ] **Step 2: Run tests, confirm failure**

Run: `cd ~/workspace/helix/src/helix_diagnosis && python -m pytest test/test_diagnosis_node.py -v`
Expected: ImportError.

- [ ] **Step 3: Implement `diagnosis_node.py`**

```python
"""
DiagnosisNode — lifecycle node.

Subscribes to /helix/faults (FaultEvent).
Queries /helix/get_context service on each fault.
Runs rules from helix_diagnosis.rules.evaluate.
Publishes /helix/recovery_hints (RecoveryHint).
Also ticks a timer to fire state-driven rules (R2 RESUME).
"""
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from helix_msgs.msg import FaultEvent, RecoveryHint
from helix_msgs.srv import GetContext

from helix_diagnosis.rules import evaluate, STATE_IDLE, STATE_STOP_AND_HOLD, HintShape

TICK_HZ: float = 10.0


class DiagnosisStateMachine:
    """Pure state machine — unit-testable without ROS 2."""

    def __init__(self):
        self.current_state = STATE_IDLE
        self._last_anomaly_time: Optional[float] = None

    def process_fault(self, fault_event, now_seconds: float) -> Optional[HintShape]:
        if fault_event.fault_type == 'ANOMALY':
            self._last_anomaly_time = now_seconds
        # Build a tiny context namespace for rules.evaluate()
        ctx = _Ctx(anomaly_clear_seconds=0.0)
        hint = evaluate(fault_event, ctx, self.current_state)
        if hint is not None and hint.suggested_action == 'STOP_AND_HOLD':
            self.current_state = STATE_STOP_AND_HOLD
        return hint

    def tick(self, now_seconds: float) -> Optional[HintShape]:
        if self.current_state != STATE_STOP_AND_HOLD:
            return None
        if self._last_anomaly_time is None:
            return None
        clear = now_seconds - self._last_anomaly_time
        ctx = _Ctx(anomaly_clear_seconds=clear)
        hint = evaluate(None, ctx, self.current_state)
        if hint is not None and hint.suggested_action == 'RESUME':
            self.current_state = STATE_IDLE
        return hint


class _Ctx:
    def __init__(self, anomaly_clear_seconds: float = 0.0):
        self.anomaly_clear_seconds = anomaly_clear_seconds
        self.rosout_ring = []
        self.metrics_json = '{}'
        self.node_health_json = '{}'
        self.snapshot_time = 0.0


class DiagnosisNode(LifecycleNode):

    def __init__(self):
        super().__init__('helix_diagnosis_node')
        self._sm = DiagnosisStateMachine()
        self._sub = None
        self._pub = None
        self._tick_timer = None
        self._ctx_client = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_lifecycle_publisher(RecoveryHint, '/helix/recovery_hints', 10)
        self._ctx_client = self.create_client(GetContext, '/helix/get_context')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(FaultEvent, '/helix/faults', self._on_fault, 10)
        self._tick_timer = self.create_timer(1.0 / TICK_HZ, self._on_tick)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._tick_timer is not None:
            self._tick_timer.cancel()
            self._tick_timer = None
        return super().on_deactivate(state)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_fault(self, msg: FaultEvent) -> None:
        hint_shape = self._sm.process_fault(msg, self._now())
        if hint_shape is not None:
            self._publish(hint_shape)

    def _on_tick(self) -> None:
        hint_shape = self._sm.tick(self._now())
        if hint_shape is not None:
            self._publish(hint_shape)

    def _publish(self, hint: HintShape) -> None:
        msg = RecoveryHint()
        msg.fault_id = hint.fault_id
        msg.suggested_action = hint.suggested_action
        msg.confidence = hint.confidence
        msg.reasoning = hint.reasoning
        msg.rule_matched = hint.rule_matched
        self._pub.publish(msg)
        self.get_logger().info(
            f'[{hint.rule_matched}] {hint.suggested_action}: {hint.reasoning}')


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosisNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run unit tests**

Run: `cd ~/workspace/helix/src/helix_diagnosis && python -m pytest test/test_diagnosis_node.py -v`
Expected: 5 passed.

- [ ] **Step 5: Build + smoke test**

```bash
cd ~/workspace/helix && colcon build --packages-select helix_diagnosis && source install/setup.bash
timeout 3 ros2 run helix_diagnosis helix_diagnosis_node &
sleep 1
ros2 node list | grep helix_diagnosis_node
```
Expected: node listed.

- [ ] **Step 6: Commit**

```bash
git add src/helix_diagnosis/helix_diagnosis/diagnosis_node.py src/helix_diagnosis/test/test_diagnosis_node.py
git commit -m "feat(diagnosis): DiagnosisNode with state machine and rule dispatch"
```

### Task 2.4 — Implement `RecoveryNode` (safety envelope)

**Files:**
- Create: `src/helix_recovery/test/test_recovery_node.py`
- Create: `src/helix_recovery/helix_recovery/recovery_node.py`

- [ ] **Step 1: Write failing tests for the safety envelope**

Contents of `test/test_recovery_node.py`:

```python
"""Tests for the safety envelope — pure, no ROS 2 spin."""
from helix_recovery.recovery_node import SafetyEnvelope, ACTION_STOP, ACTION_RESUME, ACTION_LOG_ONLY


def test_disabled_rejects_everything():
    env = SafetyEnvelope(enabled=False, cooldown_seconds=5.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    assert result.status == 'SUPPRESSED_DISABLED'
    assert result.publish is False


def test_enabled_accepts_allowlisted_action():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    assert result.status == 'ACCEPTED'
    assert result.publish is True


def test_cooldown_suppresses_second_action_same_type():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=2.0)
    assert result.status == 'SUPPRESSED_COOLDOWN'
    assert result.publish is False


def test_cooldown_allows_different_fault_type():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='LOG_PATTERN', now=2.0)
    assert result.status == 'ACCEPTED'


def test_cooldown_releases_after_window():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=7.0)
    assert result.status == 'ACCEPTED'


def test_allowlist_rejects_unknown_action():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    result = env.evaluate(action='SELF_DESTRUCT', fault_type='ANOMALY', now=1.0)
    assert result.status == 'SUPPRESSED_ALLOWLIST'
    assert result.publish is False


def test_log_only_is_never_published_but_is_accepted():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    result = env.evaluate(action=ACTION_LOG_ONLY, fault_type='CRASH', now=1.0)
    assert result.status == 'ACCEPTED'
    assert result.publish is False    # LOG_ONLY never actuates
```

- [ ] **Step 2: Run test, confirm failure**

Run: `cd ~/workspace/helix/src/helix_recovery && python -m pytest test/test_recovery_node.py -v`
Expected: ImportError.

- [ ] **Step 3: Implement `recovery_node.py`**

```python
"""
RecoveryNode — lifecycle node.

The only node in HELIX that publishes actuation commands.
All safety checks live here: enable flag, per-fault cooldown, action allowlist.
twist_mux fail-safe handles the case where this node crashes (input times out).
"""
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from helix_msgs.msg import RecoveryAction, RecoveryHint


ACTION_STOP = 'STOP_AND_HOLD'
ACTION_RESUME = 'RESUME'
ACTION_LOG_ONLY = 'LOG_ONLY'

ALLOWED_ACTIONS = {ACTION_STOP, ACTION_RESUME, ACTION_LOG_ONLY}

# LOG_ONLY is allowlisted but never actuates.
PUBLISHING_ACTIONS = {ACTION_STOP, ACTION_RESUME}

PUBLISH_HZ: float = 20.0


@dataclass
class EnvelopeResult:
    status: str          # 'ACCEPTED' | 'SUPPRESSED_*'
    publish: bool        # whether to actuate
    reason: str


class SafetyEnvelope:
    """Pure — unit-testable without ROS 2."""

    def __init__(self, enabled: bool, cooldown_seconds: float):
        self.enabled = enabled
        self.cooldown_seconds = cooldown_seconds
        self._last_action_time: Dict[str, float] = {}

    def evaluate(self, action: str, fault_type: str, now: float) -> EnvelopeResult:
        if not self.enabled:
            return EnvelopeResult('SUPPRESSED_DISABLED', False, 'recovery.enabled is false')
        if action not in ALLOWED_ACTIONS:
            return EnvelopeResult('SUPPRESSED_ALLOWLIST', False, f'{action} not in allowlist')
        last = self._last_action_time.get(fault_type)
        if last is not None and (now - last) < self.cooldown_seconds:
            return EnvelopeResult('SUPPRESSED_COOLDOWN', False,
                                  f'cooldown active for {fault_type} ({now - last:.2f}s)')
        self._last_action_time[fault_type] = now
        publish = action in PUBLISHING_ACTIONS
        return EnvelopeResult('ACCEPTED', publish, f'action {action} accepted')


class RecoveryNode(LifecycleNode):

    def __init__(self):
        super().__init__('helix_recovery_node')
        self.declare_parameter('enabled', False)
        self.declare_parameter('cooldown_seconds', 5.0)

        self._envelope: Optional[SafetyEnvelope] = None
        self._sub = None
        self._pub_cmd: Optional = None
        self._pub_audit: Optional = None
        self._publish_timer = None

        # Recovery state
        self._current_action: Optional[str] = None     # ACTION_STOP when holding; None when idle
        self._last_fault_type: Optional[str] = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        enabled = self.get_parameter('enabled').value
        cooldown = self.get_parameter('cooldown_seconds').value
        self._envelope = SafetyEnvelope(enabled=enabled, cooldown_seconds=cooldown)
        self._pub_cmd = self.create_lifecycle_publisher(Twist, '/helix/cmd_vel', 10)
        self._pub_audit = self.create_lifecycle_publisher(RecoveryAction, '/helix/recovery_actions', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(RecoveryHint, '/helix/recovery_hints', self._on_hint, 10)
        # Publish-timer is idle until _current_action set to STOP.
        self._publish_timer = self.create_timer(1.0 / PUBLISH_HZ, self._on_publish_tick)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None
        return super().on_deactivate(state)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_hint(self, msg: RecoveryHint) -> None:
        # Derive fault_type for cooldown keying. RecoveryHint doesn't carry it directly;
        # we use the rule_matched as a rough proxy since rule IDs map 1:1 to fault_type
        # in v1 (R1→ANOMALY, R2→no-fault, R3→LOG_PATTERN, R4→CRASH).
        fault_type = _rule_to_fault_type(msg.rule_matched)
        result = self._envelope.evaluate(msg.suggested_action, fault_type, self._now())
        self._audit(msg, result)
        if not result.publish:
            return
        if msg.suggested_action == ACTION_STOP:
            self._current_action = ACTION_STOP
        elif msg.suggested_action == ACTION_RESUME:
            self._current_action = None

    def _on_publish_tick(self) -> None:
        if self._current_action == ACTION_STOP:
            t = Twist()   # zero velocity
            self._pub_cmd.publish(t)

    def _audit(self, hint: RecoveryHint, result: EnvelopeResult) -> None:
        msg = RecoveryAction()
        msg.fault_id = hint.fault_id
        msg.action = hint.suggested_action
        msg.status = result.status
        msg.timestamp = self._now()
        msg.reason = result.reason
        self._pub_audit.publish(msg)
        self.get_logger().info(f'audit: {msg.action} {msg.status} {msg.reason}')


def _rule_to_fault_type(rule: str) -> str:
    return {
        'R1': 'ANOMALY',
        'R2': 'ANOMALY',     # RESUME also keyed under ANOMALY (same cooldown bucket)
        'R3': 'LOG_PATTERN',
        'R4': 'CRASH',
    }.get(rule, 'UNKNOWN')


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run unit tests**

Run: `cd ~/workspace/helix/src/helix_recovery && python -m pytest test/test_recovery_node.py -v`
Expected: 7 passed.

- [ ] **Step 5: Build + smoke test**

```bash
cd ~/workspace/helix && colcon build --packages-select helix_recovery && source install/setup.bash
timeout 3 ros2 run helix_recovery helix_recovery_node &
sleep 1
ros2 node list | grep helix_recovery_node
```
Expected: listed.

- [ ] **Step 6: Commit**

```bash
git add src/helix_recovery/
git commit -m "feat(recovery): RecoveryNode with safety envelope (enable/cooldown/allowlist)"
```

### Task 2.5 — Implement `LLMExplainer` (v1 stub)

**Files:**
- Create: `src/helix_explanation/test/test_llm_explainer.py`
- Create: `src/helix_explanation/helix_explanation/llm_explainer.py`

- [ ] **Step 1: Write failing test**

```python
"""Tests for the pure template-rendering function."""
from types import SimpleNamespace

from helix_explanation.llm_explainer import render_template


def _fault():
    return SimpleNamespace(
        fault_id='f1',
        fault_type='ANOMALY',
        severity=2,
        detail='',
        timestamp=1.234,
        context_keys=['metric'],
        context_values=['utlidar_rate'],
    )


def _hint():
    return SimpleNamespace(
        fault_id='f1',
        suggested_action='STOP_AND_HOLD',
        confidence=0.9,
        reasoning='LiDAR rate degraded',
        rule_matched='R1',
    )


def test_template_contains_key_fields():
    s = render_template(_fault(), _hint())
    assert 'ANOMALY' in s
    assert 'utlidar_rate' in s
    assert 'STOP_AND_HOLD' in s
    assert 'R1' in s


def test_template_handles_missing_fault():
    s = render_template(None, _hint())
    # Resume hints have no fault; render something still useful
    assert 'STOP_AND_HOLD' in s or 'RESUME' in s


def test_template_handles_empty_context():
    f = _fault()
    f.context_keys = []
    f.context_values = []
    s = render_template(f, _hint())
    assert 'ANOMALY' in s
```

- [ ] **Step 2: Confirm failure**

Run: `cd ~/workspace/helix/src/helix_explanation && python -m pytest test/test_llm_explainer.py -v`
Expected: ImportError.

- [ ] **Step 3: Implement `llm_explainer.py`**

```python
"""
LLMExplainer (v1 stub) — joins FaultEvent + RecoveryHint by fault_id,
emits a templated string on /helix/explanations.

NOT in the critical path. If this node crashes, the robot still recovers.

v2 upgrade: replace render_template() with an Ollama inference call.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from helix_msgs.msg import FaultEvent, RecoveryHint

SEVERITY_LABEL = {1: 'WARN', 2: 'ERROR', 3: 'CRITICAL'}


def render_template(fault_event, hint) -> str:
    """Pure formatter. Unit-testable without ROS 2."""
    if fault_event is None:
        return (f'Action: {hint.suggested_action} '
                f'(rule {hint.rule_matched}, confidence {hint.confidence:.2f}). '
                f'Reason: {hint.reasoning}.')
    sev = SEVERITY_LABEL.get(fault_event.severity, str(fault_event.severity))
    metric = _extract(fault_event.context_keys, fault_event.context_values, 'metric')
    return (
        f'Fault detected: {fault_event.fault_type}'
        + (f' on {metric}' if metric else '')
        + f' at t={fault_event.timestamp:.2f}. '
        f'Severity: {sev}. '
        f'Recovery action: {hint.suggested_action} '
        f'(rule {hint.rule_matched}, confidence {hint.confidence:.2f}). '
        f'Reason: {hint.reasoning}.'
    )


def _extract(keys, values, target) -> Optional[str]:
    try:
        idx = list(keys).index(target)
        return values[idx]
    except (ValueError, AttributeError, IndexError):
        return None


class LLMExplainer(Node):

    def __init__(self):
        super().__init__('helix_llm_explainer')
        self._pub = self.create_publisher(String, '/helix/explanations', 10)
        self._recent_fault = None         # last FaultEvent seen
        self.create_subscription(FaultEvent, '/helix/faults', self._on_fault, 20)
        self.create_subscription(RecoveryHint, '/helix/recovery_hints', self._on_hint, 10)

    def _on_fault(self, msg: FaultEvent) -> None:
        self._recent_fault = msg

    def _on_hint(self, msg: RecoveryHint) -> None:
        # Join on fault_id when possible; otherwise use most-recent.
        fault = self._recent_fault
        if msg.fault_id and fault is not None and fault.fault_id != msg.fault_id:
            fault = None
        out = String()
        out.data = render_template(fault, msg)
        self._pub.publish(out)
        self.get_logger().info(out.data)


def main(args=None):
    rclpy.init(args=args)
    node = LLMExplainer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Tests pass**

Run: `cd ~/workspace/helix/src/helix_explanation && python -m pytest test/test_llm_explainer.py -v`
Expected: 3 passed.

- [ ] **Step 5: Smoke test**

```bash
cd ~/workspace/helix && colcon build --packages-select helix_explanation && source install/setup.bash
timeout 3 ros2 run helix_explanation helix_llm_explainer &
sleep 1
ros2 node list | grep helix_llm_explainer
```

- [ ] **Step 6: Commit**

```bash
git add src/helix_explanation/
git commit -m "feat(explanation): LLMExplainer v1 stub (template-based)"
```

### Task 2.6 — Full-workspace build + lint

- [ ] **Step 1: Clean build**

Run: `cd ~/workspace/helix && colcon build --cmake-clean-cache`
Expected: all packages succeed.

- [ ] **Step 2: Run all pytest unit tests**

Run: `cd ~/workspace/helix && colcon test --pytest-args -v && colcon test-result --verbose`
Expected: all green.

- [ ] **Step 3: Ruff / flake8 pass**

Run: `cd ~/workspace/helix && ruff check src/helix_diagnosis src/helix_recovery src/helix_explanation`
Expected: clean (or auto-fix trivial issues).

- [ ] **Step 4: Commit any lint fixes**

```bash
git commit -am "style: satisfy ruff on new packages"  # only if edits were made
```

---

## Phase 3 — Sim integration

**Goal:** Full closed loop runs end-to-end in Isaac Sim with a scripted LiDAR-rate fault. No hardware yet.

### Task 3.1 — `twist_mux` config

**Files:**
- Create: `config/twist_mux.yaml`

- [ ] **Step 1: Create config**

```yaml
# twist_mux priority arbitration for HELIX closed loop.
# Higher priority wins when multiple topics publish.
topics:
  helix_recovery:
    topic:    /helix/cmd_vel
    timeout:  0.5
    priority: 100
  navigation:
    topic:    /nav/cmd_vel
    timeout:  0.5
    priority: 50
  teleop:
    topic:    /teleop/cmd_vel
    timeout:  0.5
    priority: 30

locks: {}
```

- [ ] **Step 2: Verify `twist_mux` is installed**

Run: `ros2 pkg prefix twist_mux 2>&1`
Expected: path prints. If not found: `sudo apt install ros-humble-twist-mux`.

- [ ] **Step 3: Dry-launch smoke test**

```bash
ros2 run twist_mux twist_mux --ros-args --params-file ~/workspace/helix/config/twist_mux.yaml &
sleep 1
ros2 node list | grep twist_mux
kill %1
```

- [ ] **Step 4: Commit**

```bash
cd ~/workspace/helix
git add config/twist_mux.yaml
git commit -m "config: twist_mux priority arbitration for helix/cmd_vel"
```

### Task 3.2 — Extend Isaac Sim bridge to publish synthetic `/utlidar/cloud`

**Files:**
- Modify: `~/IsaacLab/scripts/reinforcement_learning/rsl_rl/go2_ros2_bridge.py`

(Note: this file is outside the repo. Commit the diff as a patch inside the repo at `scripts/sim_patches/go2_ros2_bridge.utlidar.patch` so the change is tracked and reproducible.)

- [ ] **Step 1: Inspect existing bridge publishers** — find where `/go2/odom` etc. are declared.
- [ ] **Step 2: Add synthetic LiDAR publisher** — publish an empty `sensor_msgs/PointCloud2` on `/utlidar/cloud` at a configurable rate (default 10 Hz). Use a rospy timer.

Example insertion (adapt to existing code style):

```python
from sensor_msgs.msg import PointCloud2
# ... in bridge __init__:
self._utlidar_pub = self.create_publisher(PointCloud2, '/utlidar/cloud', 10)
self._utlidar_rate_hz = 10.0
self._utlidar_timer = self.create_timer(1.0 / self._utlidar_rate_hz, self._pub_utlidar)

def _pub_utlidar(self):
    msg = PointCloud2()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'utlidar'
    # width/height left zero — detector only cares about message rate.
    self._utlidar_pub.publish(msg)

def set_utlidar_rate_hz(self, hz: float) -> None:
    """External hook for injector to throttle rate."""
    self._utlidar_rate_hz = hz
    self._utlidar_timer.cancel()
    self._utlidar_timer = self.create_timer(1.0 / max(hz, 0.01), self._pub_utlidar)
```

- [ ] **Step 3: Run sim, verify topic is published**

Launch sim per the daily-log recipe (env -i clean launch). In a new shell:
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /utlidar/cloud
```
Expected: ~10 Hz.

- [ ] **Step 4: Save diff as a patch file in the repo**

```bash
cd ~/IsaacLab
git diff scripts/reinforcement_learning/rsl_rl/go2_ros2_bridge.py > ~/workspace/helix/scripts/sim_patches/go2_ros2_bridge.utlidar.patch
```

- [ ] **Step 5: Commit**

```bash
cd ~/workspace/helix
mkdir -p scripts/sim_patches
git add scripts/sim_patches/go2_ros2_bridge.utlidar.patch
git commit -m "sim: capture isaacsim bridge patch for synthetic /utlidar/cloud"
```

### Task 3.3 — `inject_lidar_rate_drop.py`

**Files:**
- Create: `scripts/sim_faults/inject_lidar_rate_drop.py`

- [ ] **Step 1: Implement** — subscribe to `/utlidar/cloud`, republish on a throttled topic `/utlidar/cloud_throttled` at a schedule-driven rate. (In sim, point `PassiveAdapter` at the throttled topic so we don't need to stop the source.)

```python
#!/usr/bin/env python3
"""Throttle-relay that simulates a LiDAR rate drop on /utlidar/cloud."""
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class ThrottleRelay(Node):
    """Relay /utlidar/cloud → /utlidar/cloud_throttled with scheduled Hz."""

    def __init__(self, schedule):
        super().__init__('lidar_throttle_relay')
        self._schedule = schedule       # list of (duration_s, target_hz)
        self._current_hz = schedule[0][1]
        self._start = time.time()
        self._last_publish = 0.0
        self._pub = self.create_publisher(PointCloud2, '/utlidar/cloud_throttled', 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self._on_msg, 10)
        self.get_logger().info(f'starting schedule: {schedule}')

    def _target_hz(self) -> float:
        elapsed = time.time() - self._start
        cumulative = 0.0
        for dur, hz in self._schedule:
            cumulative += dur
            if elapsed < cumulative:
                return hz
        return self._schedule[-1][1]

    def _on_msg(self, msg: PointCloud2) -> None:
        hz = self._target_hz()
        if hz <= 0:
            return   # drop entirely
        interval = 1.0 / hz
        now = time.time()
        if now - self._last_publish < interval:
            return
        self._last_publish = now
        self._pub.publish(msg)


def _parse_schedule(arg: str):
    # Format: "30:10,20:1,30:10" — duration_s:hz pairs.
    out = []
    for piece in arg.split(','):
        dur, hz = piece.split(':')
        out.append((float(dur), float(hz)))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--schedule', default='30:10,20:1,30:10')
    args = parser.parse_args()

    rclpy.init()
    node = ThrottleRelay(_parse_schedule(args.schedule))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Smoke test**

In one shell run the bridge-sim-provided `/utlidar/cloud`. In another:

```bash
python3 scripts/sim_faults/inject_lidar_rate_drop.py --schedule "10:10,10:1,10:10" &
sleep 1
ros2 topic hz /utlidar/cloud_throttled
```
Expected: ~10 Hz, then ~1 Hz after 10 s, then back to 10 Hz.

- [ ] **Step 3: Commit**

```bash
git add scripts/sim_faults/inject_lidar_rate_drop.py
git commit -m "sim: lidar-rate-drop injector (schedule-driven throttle relay)"
```

### Task 3.4 — `inject_node_crash.py`

**Files:**
- Create: `scripts/sim_faults/inject_node_crash.py`

- [ ] **Step 1: Implement**

```python
#!/usr/bin/env python3
"""Kill a named ROS 2 node after N seconds (for HeartbeatMonitor R4 tests)."""
import argparse
import subprocess
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--node', required=True, help='node name to kill (e.g., helix_context_buffer)')
    p.add_argument('--after', type=float, default=5.0)
    args = p.parse_args()

    print(f'[inject_node_crash] sleeping {args.after}s then killing {args.node}')
    time.sleep(args.after)
    result = subprocess.run(['pgrep', '-af', args.node], capture_output=True, text=True)
    lines = [ln for ln in result.stdout.splitlines() if args.node in ln]
    if not lines:
        print(f'[inject_node_crash] no process matches {args.node}')
        return 1
    pid = int(lines[0].split()[0])
    print(f'[inject_node_crash] SIGTERM to pid {pid}')
    subprocess.run(['kill', '-TERM', str(pid)])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
```

- [ ] **Step 2: Commit**

```bash
git add scripts/sim_faults/inject_node_crash.py
git commit -m "sim: node-crash injector for R4 testing"
```

### Task 3.5 — Closed-loop sim orchestrator

**Files:**
- Create: `scripts/sim_faults/run_closed_loop_scenario.py`
- Create: `launch/helix_sim_closed_loop.launch.py`

- [ ] **Step 1: Launch file** — launches all HELIX nodes + `twist_mux` for the sim scenario. Sets `recovery.enabled=true`.

```python
"""Launch file: full HELIX closed loop against sim (or real) GO2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    twist_mux_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'twist_mux.yaml')

    return LaunchDescription([
        # Sensing tier (existing)
        LifecycleNode(package='helix_core', executable='helix_anomaly_detector',
                      name='helix_anomaly_detector', namespace=''),
        LifecycleNode(package='helix_core', executable='helix_heartbeat_monitor',
                      name='helix_heartbeat_monitor', namespace=''),
        LifecycleNode(package='helix_core', executable='helix_log_parser',
                      name='helix_log_parser', namespace=''),
        # New tiers
        LifecycleNode(package='helix_diagnosis', executable='helix_context_buffer',
                      name='helix_context_buffer', namespace=''),
        LifecycleNode(package='helix_diagnosis', executable='helix_diagnosis_node',
                      name='helix_diagnosis_node', namespace=''),
        LifecycleNode(package='helix_recovery', executable='helix_recovery_node',
                      name='helix_recovery_node', namespace='',
                      parameters=[{'enabled': True, 'cooldown_seconds': 5.0}]),
        Node(package='helix_explanation', executable='helix_llm_explainer',
             name='helix_llm_explainer'),
        # Arbitration
        Node(package='twist_mux', executable='twist_mux',
             name='twist_mux', parameters=[twist_mux_config]),
    ])
```

Note: lifecycle nodes need explicit transitions via `ros2 lifecycle set` — handled by the orchestrator script below.

- [ ] **Step 2: Orchestrator script**

```python
#!/usr/bin/env python3
"""
Orchestrate a closed-loop sim scenario:
  1. launch file brings up HELIX nodes + twist_mux
  2. drive lifecycle transitions to ACTIVE
  3. start ros2 bag recording
  4. spawn inject_lidar_rate_drop.py
  5. wait for scenario duration
  6. terminate, archive artifacts
"""
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

HELIX_LIFECYCLE_NODES = [
    '/helix_anomaly_detector',
    '/helix_heartbeat_monitor',
    '/helix_log_parser',
    '/helix_context_buffer',
    '/helix_diagnosis_node',
    '/helix_recovery_node',
]


def _transition(node: str, t: str) -> None:
    subprocess.run(['ros2', 'lifecycle', 'set', node, t], check=False)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=90.0)
    p.add_argument('--schedule', default='30:10,20:1,30:10')
    p.add_argument('--artifact-dir', default=f'results/sim_run_{int(time.time())}')
    args = p.parse_args()

    artifact_dir = Path(args.artifact_dir)
    artifact_dir.mkdir(parents=True, exist_ok=True)

    print('[scenario] launching helix_sim_closed_loop')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'launch/helix_sim_closed_loop.launch.py'],
        preexec_fn=os.setsid)
    time.sleep(5.0)

    print('[scenario] configuring + activating lifecycle nodes')
    for n in HELIX_LIFECYCLE_NODES:
        _transition(n, 'configure')
    time.sleep(1.0)
    for n in HELIX_LIFECYCLE_NODES:
        _transition(n, 'activate')
    time.sleep(1.0)

    print(f'[scenario] recording bag to {artifact_dir}/bag')
    bag = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(artifact_dir / 'bag'),
         '/helix/faults', '/helix/recovery_hints', '/helix/recovery_actions',
         '/helix/explanations', '/helix/cmd_vel', '/cmd_vel', '/utlidar/cloud_throttled'],
        preexec_fn=os.setsid)

    print(f'[scenario] injecting lidar rate drop: {args.schedule}')
    injector = subprocess.Popen(
        ['python3', 'scripts/sim_faults/inject_lidar_rate_drop.py',
         '--schedule', args.schedule],
        preexec_fn=os.setsid)

    print(f'[scenario] running for {args.duration}s')
    time.sleep(args.duration)

    print('[scenario] terminating')
    for proc in (injector, bag, launch):
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    for proc in (injector, bag, launch):
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    print(f'[scenario] artifacts at {artifact_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
```

- [ ] **Step 3: Commit**

```bash
git add launch/helix_sim_closed_loop.launch.py scripts/sim_faults/run_closed_loop_scenario.py
git commit -m "sim: closed-loop orchestrator + launch file"
```

### Task 3.6 — PassiveAdapter wiring for sim topic

**Files:**
- Modify: (likely) `scripts/passive_adapter.py`

- [ ] **Step 1: Inspect current adapter input topic**

Run: `grep -n utlidar ~/workspace/helix/scripts/passive_adapter.py`
Find the topic the adapter currently subscribes to.

- [ ] **Step 2: Add sim mode** — argparse flag `--sim` switches the input to `/utlidar/cloud_throttled` instead of `/utlidar/cloud`. Everything else identical.

- [ ] **Step 3: Smoke test both modes**

- [ ] **Step 4: Commit**

```bash
git commit -am "adapter: --sim flag routes input to /utlidar/cloud_throttled"
```

---

## Phase 4 — Integration test (the load-bearing claim)

**Goal:** `tests/sim_integration/test_lidar_occlusion_recovery.py` passes reliably on mewtwo. `make test-sim` runs it.

### Task 4.1 — Integration test

**Files:**
- Create: `tests/sim_integration/__init__.py`
- Create: `tests/sim_integration/test_lidar_occlusion_recovery.py`

- [ ] **Step 1: Implement test** — runs `run_closed_loop_scenario.py`, parses the resulting bag, asserts the six claims from spec §8.

Key assertions (pseudo-code, fill concrete bag-reading code):

```python
"""Integration test: end-to-end closed loop with LiDAR rate drop."""
import json
import subprocess
from pathlib import Path

import pytest

from rosbags.highlevel import AnyReader


SCENARIO_CMD = [
    'python3', 'scripts/sim_faults/run_closed_loop_scenario.py',
    '--duration', '80',
    '--schedule', '20:10,20:1,30:10',
]


@pytest.fixture(scope='module')
def scenario_run(tmp_path_factory):
    out_dir = tmp_path_factory.mktemp('sim_run')
    cmd = SCENARIO_CMD + ['--artifact-dir', str(out_dir)]
    subprocess.run(cmd, check=True, timeout=180)
    return out_dir / 'bag'


def _read_topic(bag_dir: Path, topic: str):
    msgs = []
    with AnyReader([bag_dir]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            msgs.append((ts, msg))
    return msgs


def test_fault_event_fires_within_1s_of_rate_drop(scenario_run):
    # Rate drop starts at t=20s (per SCENARIO_CMD).
    faults = _read_topic(scenario_run, '/helix/faults')
    assert len(faults) > 0, 'no FaultEvent observed'
    first_ts_s = faults[0][0] / 1e9
    # Bag relative times start at 0, so fault should fire within ~21s of bag start.
    assert first_ts_s < 22.0


def test_stop_hint_follows_fault_within_100ms(scenario_run):
    faults = _read_topic(scenario_run, '/helix/faults')
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    stop_hints = [(ts, m) for ts, m in hints if m.suggested_action == 'STOP_AND_HOLD']
    assert stop_hints, 'no STOP_AND_HOLD hint'
    delta_ns = stop_hints[0][0] - faults[0][0]
    assert delta_ns < 100_000_000, f'stop hint {delta_ns / 1e6:.1f}ms after fault'


def test_cmd_vel_publishes_zero_within_100ms_of_hint(scenario_run):
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    stop_hints = [(ts, m) for ts, m in hints if m.suggested_action == 'STOP_AND_HOLD']
    cmds = _read_topic(scenario_run, '/helix/cmd_vel')
    assert cmds, 'no helix cmd_vel published'
    first_stop_ts = stop_hints[0][0]
    later = [ts for ts, _ in cmds if ts >= first_stop_ts]
    assert later, 'no cmd_vel after stop hint'
    delta_ns = later[0] - first_stop_ts
    assert delta_ns < 100_000_000


def test_resume_hint_after_rate_recovers(scenario_run):
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    resume = [ts for ts, m in hints if m.suggested_action == 'RESUME']
    assert resume, 'no RESUME hint observed'


def test_audit_log_has_accepted_events(scenario_run):
    actions = _read_topic(scenario_run, '/helix/recovery_actions')
    accepted = [m for _, m in actions if m.status == 'ACCEPTED']
    assert len(accepted) >= 2   # at least STOP + RESUME


def test_explanations_published(scenario_run):
    expl = _read_topic(scenario_run, '/helix/explanations')
    assert len(expl) >= 1
```

- [ ] **Step 2: Install `rosbags` library if missing**

Run: `pip install --user rosbags`

- [ ] **Step 3: Run the test (requires Isaac Sim running)**

Run: `python3 -m pytest tests/sim_integration/test_lidar_occlusion_recovery.py -v -s`
Expected: 6 passed. **If it fails**: do NOT mock failures away — read the bag manually with `ros2 bag info` and diagnose.

- [ ] **Step 4: Commit**

```bash
git add tests/sim_integration/
git commit -m "test(sim): closed-loop LiDAR-occlusion integration test"
```

### Task 4.2 — `make test-sim` target

**Files:**
- Create: `Makefile`

- [ ] **Step 1: Write Makefile**

```makefile
# HELIX development targets
.PHONY: test test-sim build clean

build:
	colcon build --cmake-clean-cache

test:
	colcon test --pytest-args -v
	colcon test-result --verbose

test-sim:
	@echo "REQUIREMENT: Isaac Sim must be running with the go2_ros2_bridge (see docs/)"
	python3 -m pytest tests/sim_integration/ -v -s

clean:
	rm -rf build install log
```

- [ ] **Step 2: Verify `make test` works**

Run: `cd ~/workspace/helix && make test`
Expected: unit tests green.

- [ ] **Step 3: Commit**

```bash
git add Makefile
git commit -m "chore: Makefile with build/test/test-sim targets"
```

### Task 4.3 — R3 + R4 regression coverage in sim

**Files:**
- Create: `tests/sim_integration/test_rule_r3_log_critical.py`
- Create: `tests/sim_integration/test_rule_r4_crash_logonly.py`

- [ ] **Step 1: R3 test** — injects a CRITICAL log line on `/rosout` during a scenario; asserts `LogParser` emits `LOG_PATTERN`, `DiagnosisNode` emits `STOP_AND_HOLD`, `RecoveryNode` fires it.

- [ ] **Step 2: R4 test** — kills `helix_context_buffer` during scenario via `inject_node_crash.py`. Asserts `HeartbeatMonitor` emits `CRASH`, `DiagnosisNode` emits `LOG_ONLY`, `RecoveryNode` audits ACCEPTED but `publish=false` (no cmd_vel fires).

Both tests follow the same structure as Task 4.1; only the injector and asserted predicates change.

- [ ] **Step 3: Run + pass**

Run: `make test-sim`

- [ ] **Step 4: Commit**

```bash
git add tests/sim_integration/test_rule_r3_log_critical.py tests/sim_integration/test_rule_r4_crash_logonly.py
git commit -m "test(sim): regression coverage for rules R3 and R4"
```

### Task 4.4 — Update CI (unit only — sim stays local)

**Files:**
- Modify: `.github/workflows/ci.yml`

- [ ] **Step 1: Ensure new packages are in the build + test matrix**

Edit the existing workflow; add `helix_diagnosis`, `helix_recovery`, `helix_explanation` to the build/test list. Do not add sim integration tests to CI — they require Isaac Sim.

- [ ] **Step 2: Push and verify CI green**

```bash
git push
```
Check GitHub Actions. Expected: green.

- [ ] **Step 3: Commit (if edits were made)**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: include new diagnosis/recovery/explanation packages"
```

---

## Phase 5 — Hardware validation (one lab session)

**Goal:** The same six asserts from Task 4.1 pass on the real GO2 + Jetson. Capture demo footage.

Hardware-only work. No TDD for this phase — tests exist; we're executing them on different hardware.

### Task 5.1 — Pre-flight checklist

**Files:**
- Create: `docs/hardware_preflight.md`

- [ ] **Step 1: Write checklist**

Checklist items (minimum): GO2 on same ROS_DOMAIN_ID as Jetson+PC; `ros2 topic list` shows `/utlidar/cloud`; `twist_mux` installed on the command path host; `helix_msgs` built on Jetson (per `hardware_run_prompt3` Task 1); feature branch checked out; `config/twist_mux.yaml` referenced correctly; `RecoveryNode` parameter `enabled=false` by default, flipped to `true` only for active demo recording.

- [ ] **Step 2: Commit**

```bash
git add docs/hardware_preflight.md
git commit -m "docs: hardware pre-flight checklist for closed-loop demo"
```

### Task 5.2 — Hardware demo launch file

**Files:**
- Create: `launch/helix_hardware_demo.launch.py`

- [ ] **Step 1: Identical to `helix_sim_closed_loop.launch.py` but:**
  - `PassiveAdapter` subscribes to real `/utlidar/cloud`, not the throttled topic.
  - `twist_mux` is configured to output on the GO2's actual `/cmd_vel` (or wherever the GO2's control subscribes).
  - Default `recovery.enabled=false` — demo script flips it on.

- [ ] **Step 2: Commit**

```bash
git add launch/helix_hardware_demo.launch.py
git commit -m "launch: hardware demo launch file (real /utlidar/cloud, twist_mux to GO2)"
```

### Task 5.3 — Execute hardware session

Manual, in-lab. No automation.

- [ ] **Step 1: Pre-flight** — run through `docs/hardware_preflight.md`.
- [ ] **Step 2: Baseline control** — confirm you can publish a teleop `/cmd_vel` and the GO2 responds (i.e., the command path works without HELIX). Record a short bag.
- [ ] **Step 3: HELIX-disabled run** — launch with `enabled=false`, occlude LiDAR. Confirm fault flows through but no action fires. Bag this.
- [ ] **Step 4: HELIX-enabled run** — flip `enabled=true` via `ros2 param set /helix_recovery_node enabled true`. Repeat occlusion. Robot must stop. Remove occlusion — robot must resume. Bag the full scenario.
- [ ] **Step 5: Multi-take for video** — record 3 takes for editing.
- [ ] **Step 6: Mirror bags + logs to T7** at `/media/yusuf/T7 Storage/LABWORK/HELIX/hardware_eval_<date>/` per Session 1+2 pattern.

### Task 5.4 — Run integration asserts against hardware bags

**Files:**
- Create: `tests/hardware_validation/test_hardware_closed_loop.py`

- [ ] **Step 1: Parameterize the Task 4.1 asserts over a hardware bag path** (instead of running the scenario live). Same six checks.

- [ ] **Step 2: Run against the Session 3 bag**

Run: `pytest tests/hardware_validation/test_hardware_closed_loop.py --bag hardware_eval_<date>/bags/closed_loop_demo`
Expected: all six pass. If any fail: that's the finding — document it honestly; do not hide.

- [ ] **Step 3: Commit test + result notes**

```bash
git add tests/hardware_validation/
git commit -m "test(hardware): closed-loop asserts over Session 3 bag"
```

---

## Phase 6 — Deliverables (video, blog, README, SVG, T7 mirror)

**Goal:** Every deliverable from spec §10 shipped. Merge to `main`.

### Task 6.1 — Regenerate architecture SVG

**Files:**
- Create: `docs/images/architecture_closed_loop.dot`
- Modify: existing architecture SVG reference in `README.md`

- [ ] **Step 1: DOT source for the 4-tier + sidecar architecture** (Graphviz). Mirror the ASCII diagram from spec §3.

- [ ] **Step 2: Render**

Run: `dot -Tsvg docs/images/architecture_closed_loop.dot > docs/images/architecture_closed_loop.svg`

- [ ] **Step 3: Commit**

```bash
git add docs/images/architecture_closed_loop.*
git commit -m "docs: architecture SVG for closed-loop design"
```

### Task 6.2 — Update `README.md`

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add a new top-level section "Closed-Loop Self-Healing"** immediately after the existing sensing sections. Include the new SVG, a 1-paragraph explanation, a quick-start (`make build && make test-sim`), and honest "what's hardware-validated vs sim-only" proof-boundary.

- [ ] **Step 2: Include demo video link** (placeholder until Task 6.3 completes).

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: README section for closed-loop self-healing"
```

### Task 6.3 — Demo video

Manual, non-TDD.

- [ ] **Step 1: Shotlist**
  1. Title card: "HELIX: Closed-Loop Self-Healing on a Unitree GO2."
  2. Baseline shot — GO2 walking, nothing monitored.
  3. HELIX-disabled fault shot — LiDAR occluded, robot keeps walking (dangerous).
  4. HELIX-enabled fault shot — LiDAR occluded, robot stops, overlay text from `/helix/explanations`, remove occlusion, robot resumes.
  5. Credits with GitHub link.

- [ ] **Step 2: Edit** — ffmpeg or a simple editor. Under 2:30.
- [ ] **Step 3: Upload** to YouTube unlisted / personal site.
- [ ] **Step 4: Update README with final link.**

```bash
git commit -am "docs: link final demo video"
```

### Task 6.4 — Blog post draft

**Files:**
- Create: `docs/blog/closed-loop-self-healing.md`

- [ ] **Step 1: Draft ~1500 words.** Sections:
  1. Problem — why runtime fault monitoring matters on real robots.
  2. Architecture — the 4-tier pipeline + `twist_mux` arbitration.
  3. Why deterministic rules in the critical path — with LLM explicitly out.
  4. LLM as explainer — the narrative, not the decision.
  5. Sim-first development — how the Isaac Sim bridge cut iteration time.
  6. Hardware validation — what reproduced, what didn't.
  7. Limitations and what's next.
  8. Code: link to repo + demo video.

- [ ] **Step 2: Commit draft**

```bash
git add docs/blog/closed-loop-self-healing.md
git commit -m "docs: blog post draft"
```

- [ ] **Step 3: Publish** to Yusuf's site / Medium (manual). Update README link.

### Task 6.5 — Session 3 T7 mirror

- [ ] **Step 1: rsync artifacts**

Run:
```bash
rsync -a hardware_eval_<date>/ "/media/yusuf/T7 Storage/LABWORK/HELIX/hardware_eval_<date>/"
```

- [ ] **Step 2: Commit the `docs/` + `results/` subsets** into the repo per Session 1+2 precedent (bags remain excluded via existing `.gitignore` rule).

```bash
git add hardware_eval_<date>/docs hardware_eval_<date>/results hardware_eval_<date>/notes
git commit -m "hw: Session 3 closed-loop demo artifacts"
```

### Task 6.6 — Update vault + merge

- [ ] **Step 1: Daily log entry** in `~/Documents/Obsidian Vault/Daily Claude Logs/<date>.md` summarizing the project completion.
- [ ] **Step 2: Update `~/Documents/Obsidian Vault/Projects/HELIX/status.md`** — flip closed-loop tasks to Done; add blog post + video links; note the thesis/workshop pivot.
- [ ] **Step 3: Merge branch to main**

```bash
git checkout main
git merge --no-ff feat/self-healing-closed-loop -m "feat: closed-loop self-healing (diagnosis + recovery)"
git push origin main
```

- [ ] **Step 4: Tag release**

```bash
git tag -a v0.2.0 -m "Closed-loop self-healing: diagnosis + recovery tiers"
git push origin v0.2.0
```

---

## Success criteria (copied from spec §14 for easy checking)

1. [ ] `tests/sim_integration/test_lidar_occlusion_recovery.py` passes ≥ 9/10 runs on mewtwo.
2. [ ] One hardware session produces demo footage with the same six asserts on real GO2.
3. [ ] Demo video under 2:30 with on-screen overlay from `/helix/explanations`.
4. [ ] Blog post (~1500 words) covers architecture + safety + LLM-as-explainer.
5. [ ] `README.md` updated; local `make test-sim` works on mewtwo.
6. [ ] Branch merged to `main` with clean history; tagged v0.2.0.
