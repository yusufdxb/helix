"""Off-robot integration harness: real HELIX processes, fake edges.

Starts the REAL arbiter, recovery and (optionally) diagnosis nodes and GO2
sport sink (dry_run) as separate OS processes, and provides one in-process
test node that plays every fake: upstream command sources, a fault source, a
direct hint/hold source, and the final robot consumer on the arbiter output.

Every message the test node sees is appended to ``events`` in the trace
format used by helix_arbiter.trace.analyze, stamped on this process's
monotonic clock.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from helix_msgs.msg import (
    ArbiterStatus,
    FaultEvent,
    HelixHold,
    RecoveryAction,
    RecoveryHint,
)

QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)


def share_config() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('helix_arbiter'),
                        'config', 'arbiter.yaml')


class Proc:
    def __init__(self, name: str, argv: List[str], log_dir: Path):
        self.name = name
        self.log_path = log_dir / f'{name}.log'
        self._fp = open(self.log_path, 'w')
        # New session so a signal reaches only this process tree.
        self.p = subprocess.Popen(argv, stdout=self._fp, stderr=subprocess.STDOUT,
                                  start_new_session=True)

    def pid_of_node(self) -> int:
        # `ros2 run` execs a python child; signal the real node, not the wrapper.
        out = subprocess.run(['pgrep', '-P', str(self.p.pid)], capture_output=True,
                             text=True).stdout.split()
        return int(out[0]) if out else self.p.pid

    def signal(self, sig) -> None:
        os.kill(self.pid_of_node(), sig)

    def alive(self) -> bool:
        return self.p.poll() is None

    def stop(self) -> None:
        if self.p.poll() is None:
            try:
                os.killpg(self.p.pid, signal.SIGINT)
                self.p.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(self.p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.p.wait(timeout=5)
        self._fp.close()

    def log(self) -> str:
        self._fp.flush()
        return self.log_path.read_text(errors='replace')


class Harness:

    def __init__(self, log_dir: Path, output_topic: str = '/cmd_vel'):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.procs: Dict[str, Proc] = {}
        self.node = rclpy.create_node('helix_harness')
        n = self.node
        self.pub_src = {
            'teleop': n.create_publisher(Twist, '/teleop/cmd_vel', 10),
            'nav': n.create_publisher(Twist, '/nav/cmd_vel', 10),
        }
        self.pub_fault = n.create_publisher(FaultEvent, '/helix/faults', 10)
        self.pub_hint = n.create_publisher(RecoveryHint, '/helix/recovery_hints', 10)
        self.pub_hold = n.create_publisher(HelixHold, '/helix/hold', 10)
        self.events: List[dict] = []
        self.output_topic = output_topic
        self._consumer = None
        self.attach_consumer()
        n.create_subscription(FaultEvent, '/helix/faults', lambda m: self._ev(
            'fault', node_name=m.node_name, fault_type=m.fault_type,
            src_stamp=m.timestamp), QOS)
        n.create_subscription(RecoveryHint, '/helix/recovery_hints', lambda m: self._ev(
            'hint', fault_id=m.fault_id, suggested_action=m.suggested_action,
            rule=m.rule_matched), QOS)
        n.create_subscription(RecoveryAction, '/helix/recovery_actions', lambda m: self._ev(
            'action', fault_id=m.fault_id, action=m.action, status=m.status,
            reason=m.reason, src_stamp=m.timestamp), QOS)
        n.create_subscription(HelixHold, '/helix/hold', lambda m: self._ev(
            'hold', hold=bool(m.hold), fault_id=m.fault_id, reason=m.reason,
            seq=int(m.seq), epoch=int(m.epoch), src_stamp=m.stamp), QOS)
        n.create_subscription(ArbiterStatus, '/helix/arbiter/status', lambda m: self._ev(
            'status', reason=m.reason, source=m.selected_source,
            hold_fault_id=m.hold_fault_id, vx=m.out_linear_x, wz=m.out_angular_z,
            rejected=int(m.rejected_total), sink_subscribers=int(m.sink_subscribers),
            src_stamp=m.stamp),
            QOS)
        n.create_subscription(String, '/helix/sink/trace', self._on_sink, QOS)

    # -- fake final consumer ----------------------------------------------------

    def attach_consumer(self) -> None:
        if self._consumer is None:
            self._consumer = self.node.create_subscription(
                Twist, self.output_topic, lambda m: self._ev(
                    'output', vx=m.linear.x, vy=m.linear.y, wz=m.angular.z,
                    zero=(m.linear.x == 0.0 and m.linear.y == 0.0
                          and m.angular.z == 0.0),
                    finite=all(math.isfinite(v) for v in (
                        m.linear.x, m.linear.y, m.linear.z,
                        m.angular.x, m.angular.y, m.angular.z))), QOS)

    def detach_consumer(self) -> None:
        if self._consumer is not None:
            self.node.destroy_subscription(self._consumer)
            self._consumer = None

    def _on_sink(self, m: String) -> None:
        d = json.loads(m.data)
        self._ev('sink', api_id=d['api_id'], reason=d['reason'],
                 sent_to_robot=d['sent_to_robot'], x=d['x'], src_stamp=d['t_wall'])

    def _ev(self, kind: str, **kw) -> None:
        kw.update(kind=kind, t=time.monotonic())
        self.events.append(kw)

    # -- processes --------------------------------------------------------------

    def start(self, name: str, argv: List[str]) -> Proc:
        p = Proc(name, argv, self.log_dir)
        self.procs[name] = p
        return p

    def start_arbiter(self, autostart: bool = True, extra: Optional[List[str]] = None,
                      config: Optional[str] = None) -> Proc:
        argv = ['ros2', 'run', 'helix_arbiter', 'helix_arbiter', '--ros-args',
                '--params-file', config or share_config(),
                '-p', f'autostart:={"true" if autostart else "false"}']
        return self.start('arbiter', argv + (extra or []))

    def start_recovery(self, cooldown: float = 5.0) -> Proc:
        p = self.start('recovery', [
            'ros2', 'run', 'helix_recovery', 'helix_recovery_node', '--ros-args',
            '-p', 'enabled:=true', '-p', f'cooldown_seconds:={cooldown}'])
        self.lifecycle('helix_recovery_node', Transition.TRANSITION_CONFIGURE)
        self.lifecycle('helix_recovery_node', Transition.TRANSITION_ACTIVATE)
        return p

    def start_diagnosis(self) -> Proc:
        p = self.start('diagnosis', ['ros2', 'run', 'helix_diagnosis',
                                     'helix_diagnosis_node'])
        self.lifecycle('helix_diagnosis_node', Transition.TRANSITION_CONFIGURE)
        self.lifecycle('helix_diagnosis_node', Transition.TRANSITION_ACTIVATE)
        return p

    def start_sink(self, mode: str = 'dry_run') -> Proc:
        return self.start('sink', ['ros2', 'run', 'helix_arbiter',
                                   'helix_go2_sport_sink', '--ros-args',
                                   '-p', f'mode:={mode}'])

    def lifecycle(self, node_name: str, transition_id: int, timeout: float = 15.0) -> bool:
        cli = self.node.create_client(ChangeState, f'/{node_name}/change_state')
        try:
            if not self._wait(lambda: cli.service_is_ready(), timeout):
                raise RuntimeError(f'{node_name} lifecycle service not available')
            req = ChangeState.Request()
            req.transition.id = transition_id
            fut = cli.call_async(req)
            self._wait(lambda: fut.done(), timeout)
            return bool(fut.done() and fut.result().success)
        finally:
            self.node.destroy_client(cli)

    def shutdown(self) -> None:
        for p in reversed(list(self.procs.values())):
            p.stop()
        self.node.destroy_node()

    # -- spinning / publishing --------------------------------------------------

    def _wait(self, pred: Callable[[], bool], timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if pred():
                return True
        return False

    def wait_for(self, pred: Callable[[], bool], timeout: float) -> bool:
        return self._wait(pred, timeout)

    def pump(self, duration: float, sources: Optional[Dict[str, tuple]] = None,
             hz: float = 20.0, each: Optional[Callable[[], None]] = None) -> None:
        """Spin for ``duration`` while streaming ``sources`` {name: (vx, wz)}."""
        end = time.monotonic() + duration
        nxt = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= nxt:
                for name, (vx, wz) in (sources or {}).items():
                    self.send(name, vx, wz)
                if each:
                    each()
                nxt = now + 1.0 / hz
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def send(self, source: str, vx: float, wz: float = 0.0) -> None:
        t = Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self.pub_src[source].publish(t)

    def inject_fault(self, metric: str = 'rate_hz/utlidar_cloud', severity: int = 2) -> None:
        """A benign synthetic ANOMALY that diagnosis rule R1 maps to STOP_AND_HOLD."""
        f = FaultEvent()
        f.node_name = metric
        f.fault_type = 'ANOMALY'
        f.severity = severity
        f.detail = f'harness-injected rate anomaly on {metric}'
        f.timestamp = time.time()
        f.context_keys = ['metric_name', 'zscore']
        f.context_values = [metric, '9.9']
        self.pub_fault.publish(f)

    def send_hint(self, action: str, fault_id: str = 'harness', rule: str = 'R1') -> None:
        h = RecoveryHint()
        h.fault_id, h.suggested_action, h.rule_matched = fault_id, action, rule
        h.confidence, h.reasoning = 0.9, 'harness'
        self.pub_hint.publish(h)

    def send_hold(self, hold: bool, epoch: int, seq: int, fault_id: str = 'harness') -> None:
        m = HelixHold()
        m.hold, m.fault_id, m.epoch, m.seq = hold, fault_id if hold else '', epoch, seq
        m.stamp = time.time()
        self.pub_hold.publish(m)

    # -- queries ----------------------------------------------------------------

    def outputs(self, since: float = 0.0, until: float = float('inf')) -> List[dict]:
        return [e for e in self.events if e['kind'] == 'output'
                and since <= e['t'] < until]

    def statuses(self, since: float = 0.0) -> List[dict]:
        return [e for e in self.events if e['kind'] == 'status' and e['t'] >= since]

    def last_output(self) -> Optional[dict]:
        o = self.outputs()
        return o[-1] if o else None

    def mark(self) -> float:
        return time.monotonic()
