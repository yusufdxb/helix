"""Off-robot integration: real arbiter/recovery/diagnosis/sink processes over
real DDS, fake command sources and a fake final consumer (helix_arbiter.harness).

Numbered to match the required hardware-closure matrix. NO HARDWARE.
"""
import math
import signal

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('helix_msgs.msg')

from helix_arbiter.harness import Harness  # noqa: E402
from helix_arbiter.trace import analyze  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402

pytestmark = pytest.mark.timeout(120)
ARB_TICK = 1.0 / 50.0
HOLD_TIMEOUT = 0.5
SRC_TIMEOUT = 0.5


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def h(tmp_path):
    hx = Harness(tmp_path)
    yield hx
    hx.shutdown()


def _up(h, diagnosis=False, sink=False):
    h.start_arbiter()
    h.start_recovery()
    if diagnosis:
        h.start_diagnosis()
    if sink:
        h.start_sink('dry_run')
    ok = h.wait_for(lambda: any(s['reason'] == 'NO_LIVE_INPUT' for s in h.statuses()), 20)
    assert ok, 'arbiter never saw a live HELIX state:\n' + h.procs['arbiter'].log()


def _moving(h, vx=0.3, src='nav', t=0.6):
    h.pump(t, {src: (vx, 0.0)})
    last = h.last_output()
    assert last and abs(last['vx'] - vx) < 1e-9, f'not moving: {last}'


def _all_zero_after(h, t0, settle):
    outs = h.outputs(since=t0 + settle)
    assert outs, 'no output samples'
    bad = [o for o in outs if not o['zero']]
    assert not bad, f'{len(bad)} nonzero outputs, first {bad[0]}'
    return outs


def test_01_normal_command_passes_when_helix_healthy(h):
    _up(h)
    _moving(h, 0.3)
    s = h.statuses()[-1]
    assert s['reason'] == 'SOURCE' and s['source'] == 'nav'


def test_02_03_stop_forces_zero_and_persists(h):
    _up(h)
    _moving(h, 0.3)
    t0 = h.mark()
    h.send_hint('STOP_AND_HOLD', fault_id='f2')
    h.pump(2.5, {'nav': (0.3, 0.0), 'teleop': (0.5, 0.0)})   # both keep streaming
    outs = _all_zero_after(h, t0, 0.1)
    assert len(outs) > 100
    assert h.statuses()[-1]['reason'] == 'HELIX_HOLD'
    assert h.statuses()[-1]['hold_fault_id'] == 'f2'


def test_04_resume_releases_without_creating_motion(h):
    _up(h)
    _moving(h, 0.3)
    h.send_hint('STOP_AND_HOLD', fault_id='f4')
    h.pump(0.5, {'nav': (0.3, 0.0)})
    h.send_hint('RESUME', rule='R2')
    t0 = h.mark()
    h.pump(1.5)                                   # upstream silent after RESUME
    _all_zero_after(h, t0, 0.0)
    assert h.statuses()[-1]['reason'] == 'NO_LIVE_INPUT'
    _moving(h, 0.2)                               # a NEW command regains authority


def test_05_repeated_stop(h):
    _up(h)
    _moving(h, 0.3)
    t0 = h.mark()
    for _ in range(10):
        h.send_hint('STOP_AND_HOLD', fault_id='f5')
        h.pump(0.1, {'nav': (0.3, 0.0)})
    _all_zero_after(h, t0, 0.1)
    acts = [e for e in h.events if e['kind'] == 'action']
    assert acts[0]['status'] == 'ACCEPTED'
    assert all(a['status'] == 'SUPPRESSED_COOLDOWN' for a in acts[1:])


def test_06_repeated_resume(h):
    _up(h)
    h.send_hint('STOP_AND_HOLD', fault_id='f6')
    h.pump(0.3)
    t0 = h.mark()
    for _ in range(10):
        h.send_hint('RESUME', rule='R2')
        h.pump(0.1)
    _all_zero_after(h, t0, 0.0)
    _moving(h, 0.2)


def test_07_resume_without_prior_stop(h):
    _up(h)
    _moving(h, 0.25)
    h.send_hint('RESUME', rule='R2')
    h.pump(0.5, {'nav': (0.25, 0.0)})
    assert all(abs(o['vx'] - 0.25) < 1e-9 for o in h.outputs(since=h.mark() - 0.3))


def test_08_cooldown_never_suppresses_a_stop_after_resume(h):
    _up(h)
    h.send_hint('STOP_AND_HOLD', fault_id='f8')
    h.pump(0.3)
    h.send_hint('RESUME', rule='R2')
    h.pump(0.3)
    _moving(h, 0.3)
    t0 = h.mark()
    h.send_hint('STOP_AND_HOLD', fault_id='f8')      # 1 s after first STOP, cooldown 5 s
    h.pump(1.0, {'nav': (0.3, 0.0)})
    _all_zero_after(h, t0, 0.1)
    acts = [e for e in h.events if e['kind'] == 'action' and e['action'] == 'STOP_AND_HOLD']
    assert [a['status'] for a in acts] == ['ACCEPTED', 'ACCEPTED']


def test_09_helix_publisher_disappears(h):
    _up(h)
    _moving(h, 0.3)
    h.procs['recovery'].signal(signal.SIGKILL)       # no goodbye message possible
    t0 = h.mark()
    h.pump(1.5, {'nav': (0.3, 0.0)})
    zero_at = next(o['t'] for o in h.outputs(since=t0) if o['zero'])
    assert zero_at - t0 <= HOLD_TIMEOUT + 3 * ARB_TICK + 0.1
    _all_zero_after(h, zero_at, 0.0)
    assert h.statuses()[-1]['reason'] == 'HELIX_STATE_STALE'


def test_10_upstream_publisher_disappears(h):
    _up(h)
    _moving(h, 0.3)
    t0 = h.mark()
    h.pump(1.2)                                       # nav goes silent
    zero_at = next(o['t'] for o in h.outputs(since=t0) if o['zero'])
    assert zero_at - t0 <= SRC_TIMEOUT + 3 * ARB_TICK + 0.1
    _all_zero_after(h, zero_at, 0.0)


def test_11_final_consumer_disappears_and_returns(h):
    _up(h)
    _moving(h, 0.3)
    h.detach_consumer()
    h.pump(0.8, {'nav': (0.3, 0.0)})
    assert h.statuses()[-1]['sink_subscribers'] == 0   # visible, not silent
    h.send_hint('STOP_AND_HOLD', fault_id='f11')       # fault while consumer is gone
    h.pump(0.5, {'nav': (0.3, 0.0)})
    t0 = h.mark()
    h.attach_consumer()
    h.pump(1.0, {'nav': (0.3, 0.0)})
    outs = h.outputs(since=t0)
    assert outs and all(o['zero'] for o in outs)       # rejoining consumer sees the hold
    assert h.statuses()[-1]['sink_subscribers'] >= 1
    assert h.procs['arbiter'].alive()


def test_12_stale_hold_state_fails_safe(h):
    h.start_arbiter()
    h.pump(1.5, {'nav': (0.3, 0.0)})                   # no recovery ever started
    outs = h.outputs()
    assert outs and all(o['zero'] for o in outs)
    assert h.statuses()[-1]['reason'] == 'HELIX_STATE_MISSING'


def test_13_hold_reorder_and_delay(h):
    h.start_arbiter()
    epoch = 10_000
    seq = [0]

    def beat(hold):
        seq[0] += 1
        h.send_hold(hold, epoch, seq[0])

    h.pump(1.5, each=lambda: beat(False))
    h.pump(0.6, {'nav': (0.3, 0.0)}, each=lambda: beat(False))
    assert abs(h.last_output()['vx'] - 0.3) < 1e-9
    beat(True)
    stale_seq = seq[0] - 1
    h.pump(0.3, {'nav': (0.3, 0.0)}, each=lambda: beat(True))
    t0 = h.mark()
    h.send_hold(False, epoch, stale_seq)               # delayed, reordered RESUME
    h.pump(0.5, {'nav': (0.3, 0.0)}, each=lambda: beat(True))
    _all_zero_after(h, t0, 0.0)
    # DDS delay longer than hold_timeout: state goes stale -> zero
    beat(False)
    h.pump(0.6, {'nav': (0.3, 0.0)}, each=lambda: beat(False))
    assert abs(h.last_output()['vx'] - 0.3) < 1e-9
    t1 = h.mark()
    h.pump(1.0, {'nav': (0.3, 0.0)})                   # hold messages delayed
    zero_at = next(o['t'] for o in h.outputs(since=t1) if o['zero'])
    assert zero_at - t1 <= HOLD_TIMEOUT + 0.2


def test_14_nan_inf_never_reach_output(h):
    _up(h)
    _moving(h, 0.3, src='teleop')
    t0 = h.mark()
    for bad in (math.nan, math.inf, -math.inf):
        h.pump(0.3, {'teleop': (bad, 0.0)})
        h.pump(0.3, {'teleop': (0.1, bad)})
    outs = h.outputs(since=t0)
    assert outs and all(o['finite'] for o in outs)
    assert all(o['zero'] for o in h.outputs(since=t0 + 0.05))  # good 0.3 not reused
    assert h.statuses()[-1]['rejected'] >= 20


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM], ids=['15_sigint', '16_sigterm'])
def test_15_16_signal_leaves_output_zero(h, sig):
    _up(h)
    _moving(h, 0.3)
    arb = h.procs['arbiter']
    t0 = h.mark()
    arb.signal(sig)
    h.pump(1.5, {'nav': (0.3, 0.0)})
    assert arb.p.wait(timeout=5) == 0, arb.log()
    outs = h.outputs(since=t0)
    assert outs and outs[-1]['zero'], f'last output not zero: {outs[-3:]}'
    tail_zero = 0
    for o in reversed(outs):
        if not o['zero']:
            break
        tail_zero += 1
    assert tail_zero >= 5
    assert any(s['reason'] == 'SHUTDOWN' for s in h.statuses(since=t0))


def test_17_lifecycle_deactivate_reactivate(h):
    h.start_arbiter(autostart=False)
    h.start_recovery()
    assert h.lifecycle('helix_arbiter', Transition.TRANSITION_CONFIGURE)
    assert h.lifecycle('helix_arbiter', Transition.TRANSITION_ACTIVATE)
    h.pump(0.5)
    _moving(h, 0.3)
    t0 = h.mark()
    assert h.lifecycle('helix_arbiter', Transition.TRANSITION_DEACTIVATE)
    h.pump(1.0, {'nav': (0.3, 0.0)})
    outs = h.outputs(since=t0)
    assert outs and all(o['zero'] for o in outs), 'inactive arbiter emitted motion'
    t1 = h.mark()
    assert h.lifecycle('helix_arbiter', Transition.TRANSITION_ACTIVATE)
    h.pump(0.3)
    assert all(o['zero'] for o in h.outputs(since=t1))
    _moving(h, 0.2)
    # recovery deactivate asserts the hold immediately
    t2 = h.mark()
    assert h.lifecycle('helix_recovery_node', Transition.TRANSITION_DEACTIVATE)
    h.pump(1.0, {'nav': (0.2, 0.0)})
    _all_zero_after(h, t2, 0.1)


def test_18_fault_while_command_zero(h):
    _up(h, diagnosis=True)
    h.pump(0.5, {'nav': (0.0, 0.0)})
    h.inject_fault()
    h.pump(1.0, {'nav': (0.0, 0.0)})
    assert all(o['zero'] for o in h.outputs())
    assert any(s['reason'] == 'HELIX_HOLD' for s in h.statuses())


def test_19_fault_while_moving_full_chain_with_sink(h):
    _up(h, diagnosis=True, sink=True)
    _moving(h, 0.2)
    h.pump(0.5, {'nav': (0.2, 0.0)})
    t0 = h.mark()
    h.inject_fault('rate_hz/utlidar_cloud')
    h.pump(2.0, {'nav': (0.2, 0.0)})
    res = analyze(h.events)
    chains = [c for c in res['chains'] if c['fault_id'] == 'rate_hz/utlidar_cloud']
    assert chains and chains[0]['complete_to_output'], chains
    c = chains[0]
    assert c['times']['sink'] is not None              # StopMove decided by the sink
    assert h.events and [e for e in h.events if e['kind'] == 'sink'
                         and e['api_id'] == 1003 and e['t'] >= c['times']['hold']][0][
        'reason'] == 'ZERO'                            # caused by the zero, not periodic
    assert 0 < c['fault_to_output_src_ms'] < 200
    assert 0 < c['fault_to_sink_src_ms'] < 250
    assert res['nonzero_outputs_during_hold'] == 0
    _all_zero_after(h, t0, 0.2)
    # released by diagnosis R2 after the 3 s clear window, still no self-motion
    h.pump(4.0)
    assert any(e['kind'] == 'hint' and e['suggested_action'] == 'RESUME' for e in h.events)
    assert all(o['zero'] for o in h.outputs(since=t0 + 0.2))


def test_20_multiple_sources_priority_then_stop_overrides_all(h):
    _up(h)
    h.pump(0.6, {'nav': (0.2, 0.0), 'teleop': (0.4, 0.0)})
    assert abs(h.last_output()['vx'] - 0.4) < 1e-9
    h.pump(1.0, {'nav': (0.2, 0.0)})                   # teleop drops out
    assert abs(h.last_output()['vx'] - 0.2) < 1e-9
    t0 = h.mark()
    h.send_hint('STOP_AND_HOLD', fault_id='f20')
    h.pump(1.0, {'nav': (0.2, 0.0), 'teleop': (0.4, 0.0)})
    _all_zero_after(h, t0, 0.1)
