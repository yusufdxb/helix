"""Pure tests of every arbitration policy (no ROS). Numbers in test names
refer to the hardware-closure test matrix in docs/MOTION_ARBITRATION.md."""
import math

import pytest
from helix_arbiter.arbiter_core import (
    REASON_HOLD,
    REASON_MISSING,
    REASON_NO_INPUT,
    REASON_SOURCE,
    REASON_STALE,
    ZERO,
    Arbiter,
    Command,
    Limits,
    SourceSpec,
    validate_twist,
)

TELEOP = SourceSpec('teleop', '/teleop/cmd_vel', 200, 0.5)
NAV = SourceSpec('nav', '/nav/cmd_vel', 50, 0.5)
E = 1000  # hold epoch


class Rig:
    """Arbiter plus a HELIX hold publisher with its own seq counter."""

    def __init__(self, sources=(TELEOP, NAV)):
        self.arb = Arbiter(list(sources), hold_timeout_sec=0.5)
        self.seq = 0
        self.t = 0.0

    def hold(self, hold, fault='rate_hz/utlidar_cloud', epoch=E, at=None):
        self.seq += 1
        return self.arb.on_hold(hold, fault if hold else '', epoch, self.seq,
                                self.t if at is None else at)

    def src(self, name, vx, wz=0.0):
        return self.arb.on_source(name, (vx, 0.0, 0.0), (0.0, 0.0, wz), self.t)

    def step(self, dt):
        self.t += dt
        return self.arb.decide(self.t)


def healthy():
    r = Rig()
    r.hold(False)
    return r


def test_missing_helix_state_forces_zero():
    r = Rig()
    r.src('nav', 0.3)
    d = r.arb.decide(0.0)
    assert d.command == ZERO and d.reason == REASON_MISSING


def test_01_normal_command_passes_when_helix_healthy():
    r = healthy()
    r.src('nav', 0.3, 0.1)
    d = r.step(0.02)
    assert d.reason == REASON_SOURCE and d.source == 'nav'
    assert d.command == Command(0.3, 0.0, 0.1)


def test_02_stop_forces_zero_even_over_highest_priority():
    r = healthy()
    r.src('teleop', 0.5)
    assert r.step(0.02).command.vx == 0.5
    r.hold(True)
    r.src('teleop', 0.5)           # teleop keeps streaming
    d = r.step(0.02)
    assert d.command == ZERO and d.reason == REASON_HOLD
    assert d.hold_fault_id == 'rate_hz/utlidar_cloud'


def test_03_stop_persists_while_hold_is_refreshed():
    r = healthy()
    r.hold(True)
    for _ in range(500):           # 10 s at 50 Hz, sources streaming throughout
        r.src('teleop', 0.4)
        r.src('nav', 0.2)
        if round(r.t / 0.02) % 2 == 0:
            r.hold(True)           # recovery refreshes at 20-25 Hz
        assert r.step(0.02).command == ZERO


def test_04_resume_releases_but_does_not_create_motion():
    r = healthy()
    r.src('nav', 0.3)
    r.step(0.02)
    r.hold(True)
    r.step(0.02)
    r.hold(False)                  # RESUME
    d = r.step(0.02)
    assert d.command == ZERO and d.reason == REASON_NO_INPUT
    # The pre-hold 0.3 was never replayed; only a NEW command moves again.
    for _ in range(10):
        r.hold(False)
        assert r.step(0.02).command == ZERO
    r.src('nav', 0.3)
    assert r.step(0.02).command.vx == 0.3


def test_04b_command_received_during_hold_is_not_replayed_after_resume():
    r = healthy()
    r.hold(True)
    r.src('nav', 0.3)              # arrives during the hold
    r.step(0.01)
    r.hold(False)
    assert r.step(0.01).command == ZERO


def test_05_repeated_stop_is_idempotent():
    r = healthy()
    base = r.arb.counters.hold_transitions
    r.hold(True, fault='a')
    for _ in range(20):
        r.hold(True, fault='a')
        r.src('nav', 0.3)
        assert r.step(0.02).command == ZERO
    assert r.arb.counters.hold_transitions - base == 1


def test_06_repeated_resume_is_idempotent():
    r = healthy()
    r.hold(True)
    r.step(0.02)
    for _ in range(20):
        r.hold(False)
        assert r.step(0.02).command == ZERO
    r.src('nav', 0.2)
    assert r.step(0.02).command.vx == 0.2


def test_07_resume_without_prior_stop_is_a_noop():
    r = healthy()
    base = r.arb.counters.hold_transitions
    r.src('nav', 0.2)
    r.hold(False)                  # no transition
    d = r.step(0.02)
    assert d.command.vx == 0.2 and r.arb.counters.hold_transitions == base


def test_09_helix_publisher_disappears_fails_to_zero():
    r = healthy()
    for _ in range(10):
        r.src('nav', 0.3)
        r.hold(False)
        assert r.step(0.02).command.vx == 0.3
    # recovery dies: no more hold messages, nav keeps streaming
    outs = []
    for _ in range(40):
        r.src('nav', 0.3)
        outs.append(r.step(0.02))
    assert outs[-1].command == ZERO and outs[-1].reason == REASON_STALE
    # stale within hold_timeout (0.5 s) + one tick
    first_zero = next(i for i, d in enumerate(outs) if d.command == ZERO)
    assert (first_zero + 1) * 0.02 <= 0.5 + 0.02 + 1e-9


def test_09b_recovery_restart_with_new_epoch_rearms_without_replay():
    r = healthy()
    r.src('nav', 0.3)
    r.step(1.0)                    # recovery gone for 1 s -> stale
    assert r.arb.decide(r.t).reason == REASON_STALE
    r.seq = 0
    assert r.hold(False, epoch=E - 5)  # restarted, clock stepped BACK: still accepted
    assert r.step(0.02).command == ZERO   # stored nav command was discarded
    r.src('nav', 0.3)
    assert r.step(0.02).command.vx == 0.3


def test_10_upstream_disappears_falls_through_then_zero():
    r = healthy()
    r.src('teleop', 0.5)
    r.src('nav', 0.2)
    assert r.step(0.01).source == 'teleop'
    for _ in range(30):            # teleop silent, nav streaming
        r.hold(False)
        r.src('nav', 0.2)
        d = r.step(0.02)
    assert d.source == 'nav' and d.command.vx == 0.2
    for _ in range(30):            # nav silent too
        r.hold(False)
        d = r.step(0.02)
    assert d.command == ZERO and d.reason == REASON_NO_INPUT


def test_12_stale_source_exactly_at_timeout_boundary():
    r = healthy()
    r.src('nav', 0.2)
    r.hold(False, at=0.5)
    assert r.arb.decide(0.5).command.vx == 0.2       # age == timeout: live
    r.hold(False, at=0.5001)
    assert r.arb.decide(0.5001).command == ZERO      # age > timeout: stale


def test_13_reordered_or_duplicate_hold_is_dropped():
    r = healthy()
    r.arb.on_hold(True, 'f', E, 100, 0.0)
    assert not r.arb.on_hold(False, '', E, 99, 0.01)     # delayed older RESUME
    assert not r.arb.on_hold(False, '', E, 100, 0.01)    # duplicate seq
    assert r.arb.decide(0.02).reason == REASON_HOLD
    assert r.arb.counters.hold_reordered == 2
    assert not r.arb.on_hold(False, '', E - 1, 500, 0.03)  # older epoch
    assert r.arb.decide(0.03).reason == REASON_HOLD


def test_13b_delayed_stop_after_newer_resume_is_dropped():
    r = healthy()
    r.arb.on_hold(False, '', E, 10, 0.0)
    assert not r.arb.on_hold(True, 'f', E, 9, 0.01)
    r.arb.on_source('nav', (0.2, 0, 0), (0, 0, 0), 0.01)
    assert r.arb.decide(0.02).command.vx == 0.2


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', range(6))
def test_14_non_finite_never_reaches_output(bad, axis):
    r = healthy()
    r.src('teleop', 0.4)
    assert r.step(0.01).command.vx == 0.4
    vals = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    vals[axis] = bad
    assert not r.arb.on_source('teleop', vals[:3], vals[3:], r.t)
    d = r.step(0.01)
    # teleop's earlier good 0.4 must NOT be reused
    assert d.command == ZERO and d.reason == REASON_NO_INPUT
    assert all(math.isfinite(v) for v in (d.command.vx, d.command.vy, d.command.wz))


def test_14b_bad_high_priority_falls_to_valid_lower_priority():
    r = healthy()
    r.src('nav', 0.2)
    r.arb.on_source('teleop', (math.nan, 0, 0), (0, 0, 0), r.t)
    d = r.step(0.01)
    assert d.source == 'nav' and d.command.vx == 0.2


def test_14c_over_limit_rejected_not_clamped():
    cmd, why = validate_twist((1.5, 0, 0), (0, 0, 0), Limits(1.0, 1.5))
    assert cmd is None and 'limit' in why
    cmd, why = validate_twist((0, 0, 0), (0, 0, 2.0), Limits(1.0, 1.5))
    assert cmd is None


def test_14d_unactuated_axes_are_dropped():
    cmd, _ = validate_twist((0.1, 0.0, 9.0), (0.5, 0.5, 0.2), Limits())
    assert cmd == Command(0.1, 0.0, 0.2)


def test_18_fault_while_command_is_zero():
    r = healthy()
    r.src('nav', 0.0)
    assert r.step(0.02).command == ZERO
    r.hold(True)
    d = r.step(0.02)
    assert d.command == ZERO and d.reason == REASON_HOLD


def test_19_fault_while_moving_zero_on_next_decision():
    r = healthy()
    r.src('nav', 0.3, 0.2)
    assert not r.step(0.02).command.is_zero
    r.hold(True)
    d = r.arb.decide(r.t)          # same instant: no tick delay in the core
    assert d.command == ZERO


def test_20_priority_and_tiebreak():
    a = SourceSpec('a', '/a', 100, 0.5)
    b = SourceSpec('b', '/b', 100, 0.5)
    c = SourceSpec('c', '/c', 10, 0.5)
    r = Rig((a, b, c))
    r.hold(False)
    r.src('c', 0.1)
    assert r.step(0.01).source == 'c'
    r.src('a', 0.2)
    r.src('b', 0.3)
    assert r.step(0.01).source == 'b'     # tie: most recent wins
    r.src('a', 0.2)
    assert r.step(0.01).source == 'a'


def test_zero_timeout_source_refused():
    with pytest.raises(ValueError):
        Arbiter([SourceSpec('x', '/x', 1, 0.0)], 0.5)


def test_duplicate_source_refused():
    with pytest.raises(ValueError):
        Arbiter([TELEOP, TELEOP], 0.5)


def test_negative_zero_is_zero():
    cmd, _ = validate_twist((-0.0, -0.0, 0.0), (0.0, 0.0, -0.0), Limits())
    assert cmd.is_zero and math.copysign(1, cmd.vx) == 1
