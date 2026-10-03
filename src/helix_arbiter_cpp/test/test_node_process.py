"""
Process-level tests of both arbiter backends: Python reference and native.

Each test starts the real node executable as its own OS process on an
isolated, localhost-only DDS domain, with every topic under /helix_ptest/
(synthetic: nothing on a robot publishes or consumes them), drives it from an
in-process rclpy node and asserts on what is observable from outside: the
Twist stream, ArbiterStatus telemetry, lifecycle service results, signal
handling, parameters and graph QoS. Every test runs once per backend, so both
implementations are held to the same expectations. No hardware, no sink.
"""
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

# Forced, not defaulted: an exported ROS_DOMAIN_ID must not point these
# publishers at a live graph.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('HELIX_PTEST_DOMAIN', '91')
os.environ['ROS_LOCALHOST_ONLY'] = '1'

rclpy = pytest.importorskip('rclpy')
yaml = pytest.importorskip('yaml')

from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import State, Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from helix_msgs.msg import ArbiterStatus, HelixHold  # noqa: E402

PREFIX = '/helix_ptest'
OUT, STATUS, HOLD = PREFIX + '/cmd_vel', PREFIX + '/status', PREFIX + '/hold'
TOPICS = {'teleop': PREFIX + '/teleop', 'nav': PREFIX + '/nav'}
BURST = 7
NODE = '/helix_arbiter'
BACKENDS = ('python', 'cpp')


def _exe(backend: str) -> str:
    from ament_index_python.packages import get_package_prefix
    if backend == 'cpp':
        return os.environ.get('HELIX_ARBITER_CPP_EXE') or os.path.join(
            get_package_prefix('helix_arbiter_cpp'), 'lib', 'helix_arbiter_cpp', 'helix_arbiter')
    return os.path.join(get_package_prefix('helix_arbiter'), 'lib', 'helix_arbiter',
                        'helix_arbiter')


def _params(**over) -> dict:
    p = {'output_topic': OUT, 'status_topic': STATUS, 'hold_topic': HOLD, 'rate_hz': 50.0,
         'hold_timeout_sec': 0.5, 'max_abs_linear': 1.0, 'max_abs_angular': 1.5,
         'shutdown_zero_count': BURST,
         'sources': {'teleop': {'topic': TOPICS['teleop'], 'priority': 200, 'timeout': 0.5},
                     'nav': {'topic': TOPICS['nav'], 'priority': 50, 'timeout': 0.5}}}
    p.update(over)
    return p


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Rig:
    """One arbiter process plus the driver node that plays every other party."""

    def __init__(self, backend: str, tmp: Path, params: dict, autostart: bool = True):
        self.backend = backend
        self.node = rclpy.create_node('helix_ptest_driver')
        n = self.node
        self.pub = {k: n.create_publisher(Twist, t, 10) for k, t in TOPICS.items()}
        self.pub_hold = n.create_publisher(HelixHold, HOLD, 10)
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        self.events = []
        n.create_subscription(Twist, OUT, lambda m: self.events.append(
            ('out', time.monotonic(), m)), qos)
        n.create_subscription(ArbiterStatus, STATUS, lambda m: self.events.append(
            ('status', time.monotonic(), m)), qos)
        cfg = tmp / f'{backend}_arbiter.yaml'
        cfg.write_text(yaml.safe_dump({'helix_arbiter': {'ros__parameters': params}}))
        self.log_path = tmp / f'{backend}.log'
        self._log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(
            [_exe(backend), '--ros-args', '--params-file', str(cfg),
             '-p', f'autostart:={"true" if autostart else "false"}'],
            stdout=self._log, stderr=subprocess.STDOUT, start_new_session=True)
        self.epoch, self.seq = 1_700_000_000_000_000_000, 0

    # -- plumbing ---------------------------------------------------------------
    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)
        self._log.close()
        self.node.destroy_node()

    def log(self) -> str:
        self._log.flush()
        return self.log_path.read_text(errors='replace')

    def wait_for(self, pred, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if pred():
                return True
        return False

    def call(self, srv_type, name: str, req, timeout: float = 10.0):
        cli = self.node.create_client(srv_type, name)
        try:
            assert self.wait_for(cli.service_is_ready, timeout), f'{name} unavailable'
            fut = cli.call_async(req)
            assert self.wait_for(fut.done, timeout), f'{name} timed out'
            return fut.result()
        finally:
            self.node.destroy_client(cli)

    def transition(self, tid: int) -> bool:
        req = ChangeState.Request()
        req.transition.id = tid
        return bool(self.call(ChangeState, NODE + '/change_state', req).success)

    def state(self) -> int:
        return self.call(GetState, NODE + '/get_state', GetState.Request()).current_state.id

    def get_param(self, name: str):
        res = self.call(GetParameters, NODE + '/get_parameters',
                        GetParameters.Request(names=[name]))
        v = res.values[0]
        return {ParameterType.PARAMETER_STRING: v.string_value,
                ParameterType.PARAMETER_DOUBLE: v.double_value,
                ParameterType.PARAMETER_INTEGER: v.integer_value,
                ParameterType.PARAMETER_BOOL: v.bool_value}.get(v.type)

    def set_double(self, name: str, value: float) -> bool:
        p = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                      double_value=value))
        res = self.call(SetParameters, NODE + '/set_parameters', SetParameters.Request(
            parameters=[p]))
        return bool(res.results[0].successful)

    # -- stimulus -----------------------------------------------------------------
    def send(self, src: str, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0) -> None:
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(lx), float(ly), float(lz)
        t.angular.x, t.angular.y, t.angular.z = float(ax), float(ay), float(az)
        self.pub[src].publish(t)

    def send_hold(self, hold: bool, epoch=None, seq=None, fault: str = 'ptest') -> None:
        if seq is None:
            self.seq += 1
        m = HelixHold()
        m.hold = hold
        m.fault_id = fault if hold else ''
        m.epoch = self.epoch if epoch is None else epoch
        m.seq = self.seq if seq is None else seq
        m.stamp = time.time()
        self.pub_hold.publish(m)

    def pump(self, duration: float, sources=None, hold=None, fault='ptest', hz=20.0) -> None:
        """Spin for duration, streaming sources {name: kwargs} and the hold state."""
        end = time.monotonic() + duration
        nxt = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= nxt:
                for name, kw in (sources or {}).items():
                    self.send(name, **kw)
                if hold is not None:
                    self.send_hold(hold, fault=fault)
                nxt = now + 1.0 / hz
            rclpy.spin_once(self.node, timeout_sec=0.005)

    # -- observations ------------------------------------------------------------
    def mark(self) -> float:
        return time.monotonic()

    def outputs(self, since: float = 0.0):
        return [(t, m) for k, t, m in self.events if k == 'out' and t >= since]

    def statuses(self, since: float = 0.0):
        return [m for k, t, m in self.events if k == 'status' and t >= since]

    def last_status(self) -> ArbiterStatus:
        s = self.statuses()
        assert s, 'no status received:\n' + self.log()
        return s[-1]


@pytest.fixture(params=BACKENDS)
def backend(request):
    return request.param


@pytest.fixture
def rig_factory(backend, tmp_path):
    rigs = []

    def make(params=None, autostart=True) -> Rig:
        r = Rig(backend, tmp_path, params or _params(), autostart)
        rigs.append(r)
        return r

    yield make
    for r in rigs:
        r.close()


def _zero(m: Twist) -> bool:
    return m.linear.x == 0.0 and m.linear.y == 0.0 and m.angular.z == 0.0


def _started(r: Rig) -> None:
    assert r.wait_for(lambda: r.statuses(), 20), 'arbiter produced no output:\n' + r.log()


NAV_03 = {'nav': {'lx': 0.3, 'az': 0.1}}


def _burst_then_silence(r: Rig, t0: float) -> None:
    """Since t0: normal ticks, then exactly BURST zero SHUTDOWN ticks, then nothing."""
    reasons = [s.reason for s in r.statuses(t0)]
    assert 'SHUTDOWN' in reasons, reasons
    first = reasons.index('SHUTDOWN')
    assert reasons[first:] == ['SHUTDOWN'] * BURST, reasons
    outs = r.outputs(t0)
    assert len(outs) >= BURST and all(_zero(m) for _, m in outs[-BURST:])
    t_end = r.mark()
    r.pump(0.5, NAV_03, hold=False)
    assert not r.outputs(t_end) and not r.statuses(t_end), 'output after the zero burst'


def test_fails_closed_then_priority_then_hold(rig_factory):
    r = rig_factory()
    _started(r)
    t0 = r.mark()
    r.pump(0.6, NAV_03)                                  # no HELIX state yet
    assert all(_zero(m) for _, m in r.outputs(t0))
    s = r.last_status()
    assert (s.reason, s.hold_active) == ('HELIX_STATE_MISSING', True)

    r.pump(0.6, hold=False)
    assert (r.last_status().reason, r.last_status().hold_active) == ('NO_LIVE_INPUT', False)

    r.pump(0.6, NAV_03, hold=False)
    _, m = r.outputs()[-1]
    assert (m.linear.x, m.linear.y, m.angular.z) == (0.3, 0.0, 0.1)
    assert (r.last_status().reason, r.last_status().selected_source) == ('SOURCE', 'nav')

    # Fixed-rate output while nothing changes: 50 Hz, timer driven.
    t1 = r.mark()
    r.pump(1.0, NAV_03, hold=False)
    stamps = [t for t, _ in r.outputs(t1 + 0.1)]
    gaps = sorted(b - a for a, b in zip(stamps, stamps[1:]))
    assert 35 <= len(stamps) <= 60, len(stamps)
    assert 0.012 <= gaps[len(gaps) // 2] <= 0.028, gaps[len(gaps) // 2]

    both = {'nav': {'lx': 0.3}, 'teleop': {'lx': 0.5}}
    r.pump(0.6, both, hold=False)
    assert r.outputs()[-1][1].linear.x == 0.5 and r.last_status().selected_source == 'teleop'

    t2 = r.mark()
    r.pump(0.8, both, hold=True, fault='ptest-fault')     # teleop cannot override a hold
    assert all(_zero(m) for _, m in r.outputs(t2 + 0.1))
    s = r.last_status()
    assert (s.reason, s.hold_fault_id, s.hold_active) == ('HELIX_HOLD', 'ptest-fault', True)

    t3 = r.mark()
    r.pump(0.6, hold=False)                              # RESUME without new commands
    assert all(_zero(m) for _, m in r.outputs(t3))
    assert r.last_status().reason == 'NO_LIVE_INPUT'

    r.pump(0.9, NAV_03)                                  # recovery silent: stale
    assert r.last_status().reason == 'HELIX_STATE_STALE'
    assert _zero(r.outputs()[-1][1])


def test_hold_ordering_keeps_full_uint64_range_over_dds(rig_factory):
    r = rig_factory()
    _started(r)
    r.epoch, r.seq = 2 ** 64 - 1, 2 ** 64 - 60
    r.pump(0.6, NAV_03, hold=False)
    assert r.last_status().reason == 'SOURCE'
    t0 = r.mark()
    # A delayed STOP from earlier in the same epoch: dropped while fresh.
    r.send_hold(True, seq=2 ** 64 - 61, fault='stale-stop')
    r.pump(0.5, NAV_03, hold=False)
    assert not any(s.reason == 'HELIX_HOLD' for s in r.statuses(t0))
    # Highest possible (epoch, seq): accepted; anything after it is a duplicate.
    r.send_hold(True, seq=2 ** 64 - 1, fault='max')
    t1 = r.mark()
    r.pump(0.4, NAV_03, hold=False)                    # lower seqs: all dropped
    assert all(s.reason == 'HELIX_HOLD' and s.hold_fault_id == 'max'
               for s in r.statuses(t1 + 0.05))
    r.pump(0.8, NAV_03)                                # silence: stale
    assert r.last_status().reason == 'HELIX_STATE_STALE'
    r.epoch, r.seq = 0, 0                              # restart, clock stepped back
    r.pump(0.4, hold=False)
    assert r.last_status().reason == 'NO_LIVE_INPUT'


def test_malformed_input_rejected_counted_and_old_value_not_reused(rig_factory):
    r = rig_factory()
    _started(r)
    r.pump(0.6, {'teleop': {'lx': 0.4}}, hold=False)
    assert r.outputs()[-1][1].linear.x == 0.4
    before = r.last_status().rejected_total
    for bad in ({'ay': math.nan}, {'lz': math.inf}, {'lx': -math.inf}, {'lx': 1.2},
                {'az': -1.6}):
        t0 = r.mark()
        r.pump(0.4, {'teleop': bad}, hold=False)
        outs = r.outputs(t0 + 0.05)
        assert outs and all(_zero(m) for _, m in outs), bad
        assert all(math.isfinite(v) for _, m in outs for v in (
            m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z))
    assert r.last_status().rejected_total >= before + 5 * 6
    # A large but finite unactuated axis is accepted and dropped.
    r.pump(0.4, {'teleop': {'lx': 0.2, 'lz': 9.0, 'ax': 3.0}}, hold=False)
    _, m = r.outputs()[-1]
    assert (m.linear.x, m.linear.z, m.angular.x) == (0.2, 0.0, 0.0)


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM], ids=['sigint', 'sigterm'])
def test_signal_publishes_exact_zero_burst_then_exits_0(rig_factory, sig):
    r = rig_factory()
    _started(r)
    r.pump(0.6, NAV_03, hold=False)
    assert r.outputs()[-1][1].linear.x == 0.3
    t0 = r.mark()
    r.proc.send_signal(sig)
    r.pump(1.2, NAV_03, hold=False)
    assert r.proc.wait(timeout=10) == 0, r.log()
    st = r.statuses(t0)
    shut = [i for i, s in enumerate(st) if s.reason == 'SHUTDOWN']
    assert len(shut) == BURST, [s.reason for s in st]
    assert shut == list(range(shut[0], shut[0] + BURST)) and shut[-1] == len(st) - 1
    outs = r.outputs(t0)
    assert outs and all(_zero(m) for _, m in outs[-BURST:])


def test_lifecycle_deactivate_reactivate_reconfigure_shutdown(rig_factory):
    r = rig_factory(autostart=False)
    r.pump(1.0, NAV_03, hold=False)
    assert not r.outputs(), 'unconfigured arbiter published'
    assert r.state() == State.PRIMARY_STATE_UNCONFIGURED
    assert r.transition(Transition.TRANSITION_CONFIGURE)
    r.pump(0.5, NAV_03, hold=False)
    assert not r.outputs(), 'inactive arbiter published'
    assert r.transition(Transition.TRANSITION_ACTIVATE)
    r.pump(0.8, NAV_03, hold=False)
    assert r.outputs()[-1][1].linear.x == 0.3

    t0 = r.mark()
    assert r.transition(Transition.TRANSITION_DEACTIVATE)
    r.pump(0.3, NAV_03, hold=False)
    _burst_then_silence(r, t0)

    t1 = r.mark()
    assert r.transition(Transition.TRANSITION_ACTIVATE)
    r.pump(0.6, hold=False)                             # hold state but no fresh command
    assert all(_zero(m) for _, m in r.outputs(t1))
    assert r.last_status().reason == 'NO_LIVE_INPUT'

    # Reconfigure: parameters take effect on cleanup + configure.
    assert r.transition(Transition.TRANSITION_DEACTIVATE)
    assert r.transition(Transition.TRANSITION_CLEANUP)
    assert r.set_double('rate_hz', 20.0)
    assert r.transition(Transition.TRANSITION_CONFIGURE)
    assert r.transition(Transition.TRANSITION_ACTIVATE)
    r.pump(0.3, hold=False)
    t2 = r.mark()
    r.pump(1.0, hold=False)
    n = len(r.outputs(t2))
    assert 14 <= n <= 26, f'{n} outputs in 1 s at rate_hz 20'

    t3 = r.mark()
    assert r.transition(Transition.TRANSITION_ACTIVE_SHUTDOWN)
    r.pump(0.3, NAV_03, hold=False)
    _burst_then_silence(r, t3)
    assert r.state() == State.PRIMARY_STATE_FINALIZED


@pytest.mark.parametrize('case', ['relative_alias_of_output', 'float_priority',
                                  'unknown_source_field', 'shared_source_topic',
                                  'string_rate'])
def test_invalid_configuration_is_refused(rig_factory, case):
    sources = _params()['sources']
    over = {
        # Relative name: resolves to /helix_ptest/cmd_vel in the root namespace.
        'relative_alias_of_output': {'sources': {**sources, 'nav': {
            'topic': OUT.lstrip('/'), 'priority': 50, 'timeout': 0.5}}},
        'float_priority': {'sources': {**sources, 'nav': {
            'topic': TOPICS['nav'], 'priority': 50.5, 'timeout': 0.5}}},
        'unknown_source_field': {'sources': {**sources, 'nav': {
            'topic': TOPICS['nav'], 'priority': 50, 'timeout': 0.5, 'enabled': False}}},
        'shared_source_topic': {'sources': {**sources, 'nav': {
            'topic': TOPICS['teleop'], 'priority': 50, 'timeout': 0.5}}},
        'string_rate': {'rate_hz': 'fast'},
    }[case]
    r = rig_factory(_params(**over), autostart=False)
    assert not r.transition(Transition.TRANSITION_CONFIGURE)
    assert r.state() == State.PRIMARY_STATE_UNCONFIGURED
    r.pump(0.5, NAV_03, hold=False)
    assert not r.outputs()
    assert r.proc.poll() is None


def test_autostart_with_invalid_configuration_stays_silent(rig_factory):
    r = rig_factory(_params(max_abs_linear=-1.0), autostart=True)
    r.pump(2.0, NAV_03, hold=False)
    assert not r.outputs()
    assert r.state() == State.PRIMARY_STATE_UNCONFIGURED
    assert r.proc.poll() is None


def test_identity_qos_and_backend_label(rig_factory, backend):
    r = rig_factory(_params(arbiter_backend='bogus'))
    _started(r)
    r.pump(0.6, NAV_03, hold=False)
    pubs = r.node.get_publishers_info_by_topic(OUT)
    assert [(p.node_name, p.node_namespace) for p in pubs] == [('helix_arbiter', '/')]
    assert pubs[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert pubs[0].qos_profile.durability == DurabilityPolicy.VOLATILE
    st = r.node.get_publishers_info_by_topic(STATUS)
    assert st[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    for topic in (TOPICS['nav'], TOPICS['teleop'], HOLD):
        subs = [s for s in r.node.get_subscriptions_info_by_topic(topic)
                if s.node_name == 'helix_arbiter']
        assert len(subs) == 1, topic
        assert subs[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT, topic
    # Read-only and not relabelled by the parameter file.
    assert r.get_param('arbiter_backend') == backend
    assert r.last_status().sink_subscribers >= 1
    assert r.last_status().seq > 0


def test_changed_command_is_published_without_waiting_for_the_timer(rig_factory):
    """A source update that changes the decision is published from its callback.

    The timer runs at 2 Hz here, so a timer-only arbiter would take up to
    500 ms to show a change; both backends must show it well inside that,
    and a steady stream must not add publishes beyond the timer.
    """
    r = rig_factory(_params(rate_hz=2.0, hold_timeout_sec=5.0))
    _started(r)
    r.pump(1.2, NAV_03, hold=False)
    assert r.last_status().reason == 'SOURCE'

    latencies = []
    for i, lx in enumerate((0.11, 0.22, 0.33, 0.44, 0.55)):
        t = r.mark()
        r.send('nav', lx=lx, az=0.1)
        assert r.wait_for(lambda lx=lx: any(m.linear.x == lx for _, m in r.outputs(t)), 2.0), \
            'changed command never published:\n' + r.log()
        latencies.append(next(tt for tt, m in r.outputs(t) if m.linear.x == lx) - t)
        r.wait_for(lambda: False, 0.05 + 0.07 * i)   # vary the phase against the timer
    assert max(latencies) < 0.2, latencies

    t = r.mark()
    r.pump(1.0, {'nav': {'lx': 0.55, 'az': 0.1}}, hold=False)
    assert len(r.outputs(t)) <= 4, len(r.outputs(t))   # about 2 timer ticks, no echo


def test_redundant_lifecycle_request_is_refused_not_fatal(rig_factory):
    """A second activate (from a launch file or an operator) must not kill the node."""
    r = rig_factory()
    _started(r)
    assert r.state() == State.PRIMARY_STATE_ACTIVE
    assert r.transition(Transition.TRANSITION_ACTIVATE) is False
    assert r.transition(Transition.TRANSITION_CONFIGURE) is False
    t = r.mark()
    r.pump(0.6, NAV_03, hold=False)
    assert r.proc.poll() is None, 'arbiter exited:\n' + r.log()
    assert r.state() == State.PRIMARY_STATE_ACTIVE
    assert r.outputs(t) and r.last_status().reason == 'SOURCE'
