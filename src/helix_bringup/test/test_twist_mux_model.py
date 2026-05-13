"""Sim-only verification harness for twist_mux priority arbitration.

These tests exercise ``PriorityMux`` (a pure-function model of twist_mux
semantics) plus ``load_input_specs`` (which parses the actual YAML the
real twist_mux node consumes). They do NOT spin a ROS node, do NOT
require twist_mux to be installed, and run in the no-ROS CI job.

Scope (from the lab-prep brief):
  * clean priority swaps (high beats low)
  * dropouts (high silence falls through to lower)
  * overlap (both producing simultaneously, highest priority wins)
  * stale data timing (winner switches exactly at the timeout boundary)
  * all-stale recovery (zero twist emitted when nothing is live)
  * tiebreak (equal-priority inputs: last-writer-wins, defensive)
  * yaml load + round-trip (config-file is the source of truth)
  * deliberately failing sanity check (proves assertions actually trip
    when the config is misordered)

To add a new arbitration test case: see test/README.md.
"""
from __future__ import annotations

from pathlib import Path

import pytest
from helix_bringup.twist_mux_model import (
    ZERO,
    InputSpec,
    PriorityMux,
    Twist2D,
    load_input_specs,
)

# ---------------------------------------------------------------------------
# Test fixtures / helpers
# ---------------------------------------------------------------------------

# Concrete HELIX wiring (mirrors src/helix_bringup/config/twist_mux.yaml,
# the file actually loaded by helix_closedloop.launch.py). Tests using
# these constants verify the *intended* priority order even if someone
# accidentally edits the YAML, and the YAML-loader tests verify the
# YAML matches.
HELIX_TELEOP_PRIO = 200
HELIX_RECOVERY_PRIO = 100
HELIX_NAV_PRIO = 50
HELIX_TIMEOUT_SEC = 0.5


class FakeClock:
    """Manually-advanced monotonic clock."""

    def __init__(self, t0: float = 1000.0) -> None:
        self._t = float(t0)

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += float(dt)


def _make_helix_mux(clock: FakeClock) -> PriorityMux:
    """Build the mux with HELIX's canonical priority wiring."""
    specs = [
        InputSpec("teleop", "/teleop/cmd_vel", HELIX_TIMEOUT_SEC, HELIX_TELEOP_PRIO),
        InputSpec("helix_recovery", "/helix/cmd_vel", HELIX_TIMEOUT_SEC, HELIX_RECOVERY_PRIO),
        InputSpec("navigation", "/nav/cmd_vel", HELIX_TIMEOUT_SEC, HELIX_NAV_PRIO),
    ]
    return PriorityMux(specs, clock=clock)


def _bringup_yaml_path() -> Path:
    """Path to the production twist_mux YAML the launch file actually loads."""
    return Path(__file__).resolve().parent.parent / "config" / "twist_mux.yaml"


# ---------------------------------------------------------------------------
# Sanity / wiring tests
# ---------------------------------------------------------------------------

def test_mux_rejects_empty_input_list():
    with pytest.raises(ValueError):
        PriorityMux([], clock=FakeClock())


def test_mux_rejects_duplicate_input_names():
    specs = [
        InputSpec("a", "/x", 0.5, 10),
        InputSpec("a", "/y", 0.5, 20),
    ]
    with pytest.raises(ValueError):
        PriorityMux(specs, clock=FakeClock())


def test_idle_mux_has_no_winner_and_zero_on_idle_emits_zero():
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    # No publisher has spoken yet.
    assert mux.arbitrate().winner is None
    assert mux.arbitrate().twist is None

    zero_result = mux.arbitrate_with_zero_on_idle()
    assert zero_result.winner is None
    assert zero_result.twist == ZERO


# ---------------------------------------------------------------------------
# Priority-order tests
# ---------------------------------------------------------------------------

def test_highest_priority_wins_when_all_live():
    """Overlap case: teleop + helix + nav all publishing -> teleop wins."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("navigation", Twist2D(0.3, 0.0))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))   # STOP_AND_HOLD twist
    mux.publish("teleop", Twist2D(0.7, 0.0))           # operator override

    result = mux.arbitrate()
    assert result.winner == "teleop"
    assert result.twist == Twist2D(0.7, 0.0)
    assert set(result.live_inputs) == {"teleop", "helix_recovery", "navigation"}


def test_helix_beats_navigation_when_teleop_silent():
    """STOP_AND_HOLD must preempt autonomy."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("navigation", Twist2D(0.4, 0.1))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))   # zero-twist stop

    result = mux.arbitrate()
    assert result.winner == "helix_recovery"
    assert result.twist == Twist2D(0.0, 0.0)


def test_navigation_wins_when_alone():
    """Steady-state autonomy: nav only -> nav wins."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("navigation", Twist2D(0.4, 0.0))

    result = mux.arbitrate()
    assert result.winner == "navigation"


# ---------------------------------------------------------------------------
# Dropout / stale-data tests
# ---------------------------------------------------------------------------

def test_high_priority_dropout_falls_through_to_next_tier():
    """Teleop publishes, then goes silent past its timeout -> helix wins."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("teleop", Twist2D(0.7, 0.0))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    mux.publish("navigation", Twist2D(0.3, 0.0))

    # Right after publish: teleop wins.
    assert mux.arbitrate().winner == "teleop"

    # Advance past teleop's timeout. Helix is still fresh.
    clock.advance(HELIX_TIMEOUT_SEC + 0.01)
    # Refresh helix and nav so they're still live; only teleop has gone stale.
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    mux.publish("navigation", Twist2D(0.3, 0.0))

    result = mux.arbitrate()
    assert result.winner == "helix_recovery"
    assert "teleop" not in result.live_inputs


def test_winner_switches_exactly_at_timeout_boundary():
    """Timing edge: just-before-timeout high-pri still wins, just-after loses."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("teleop", Twist2D(0.5, 0.0))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))

    # 1 us before timeout: teleop is still live.
    clock.advance(HELIX_TIMEOUT_SEC - 1e-6)
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))  # keep helix fresh
    assert mux.arbitrate().winner == "teleop"

    # Cross the boundary: teleop becomes stale, helix takes over.
    clock.advance(2e-6)
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    assert mux.arbitrate().winner == "helix_recovery"


def test_all_inputs_stale_emits_zero_twist():
    """Catastrophic dropout: every input went silent. Mux holds zero."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("teleop", Twist2D(0.5, 0.0))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    mux.publish("navigation", Twist2D(0.3, 0.0))

    clock.advance(HELIX_TIMEOUT_SEC + 1.0)

    plain = mux.arbitrate()
    assert plain.winner is None
    assert plain.twist is None

    with_zero = mux.arbitrate_with_zero_on_idle()
    assert with_zero.winner is None
    assert with_zero.twist == ZERO


def test_dropout_recovery_reclaims_priority():
    """After a teleop dropout, a new teleop message reclaims the mux."""
    clock = FakeClock()
    mux = _make_helix_mux(clock)

    mux.publish("teleop", Twist2D(0.7, 0.0))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))

    # Teleop goes silent.
    clock.advance(HELIX_TIMEOUT_SEC + 0.05)
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    assert mux.arbitrate().winner == "helix_recovery"

    # Operator grabs the stick again.
    mux.publish("teleop", Twist2D(-0.3, 0.0))
    assert mux.arbitrate().winner == "teleop"


# ---------------------------------------------------------------------------
# Tiebreak tests (defensive: HELIX config has distinct priorities, but the
# YAML schema permits equal priorities and the docs are silent on the
# resolution. We freeze last-writer-wins as the contract.)
# ---------------------------------------------------------------------------

def test_equal_priority_tiebreak_is_last_writer_wins():
    clock = FakeClock()
    specs = [
        InputSpec("a", "/a", 0.5, 100),
        InputSpec("b", "/b", 0.5, 100),
    ]
    mux = PriorityMux(specs, clock=clock)

    mux.publish("a", Twist2D(0.1, 0.0))
    mux.publish("b", Twist2D(0.2, 0.0))
    assert mux.arbitrate().winner == "b"

    mux.publish("a", Twist2D(0.3, 0.0))
    assert mux.arbitrate().winner == "a"


# ---------------------------------------------------------------------------
# YAML round-trip tests: the production config IS the source of truth.
# ---------------------------------------------------------------------------

def test_bringup_yaml_loads_and_matches_helix_intent():
    """The actual YAML the launch file consumes must match the HELIX
    safety invariant: teleop > helix > nav, all on a 0.5 s timeout."""
    specs = load_input_specs(_bringup_yaml_path())
    by_name = {s.name: s for s in specs}

    assert set(by_name) == {"teleop", "helix_recovery", "navigation"}, (
        f"unexpected input set in YAML: {set(by_name)}"
    )

    assert by_name["teleop"].priority == HELIX_TELEOP_PRIO
    assert by_name["helix_recovery"].priority == HELIX_RECOVERY_PRIO
    assert by_name["navigation"].priority == HELIX_NAV_PRIO

    # Order matters for human review: teleop > helix > nav.
    assert by_name["teleop"].priority > by_name["helix_recovery"].priority
    assert by_name["helix_recovery"].priority > by_name["navigation"].priority

    for s in specs:
        assert s.timeout_sec == HELIX_TIMEOUT_SEC, (
            f"input {s.name!r} timeout {s.timeout_sec} != "
            f"HELIX canonical {HELIX_TIMEOUT_SEC}"
        )

    # Topic names must match the documented wiring.
    assert by_name["teleop"].topic == "/teleop/cmd_vel"
    assert by_name["helix_recovery"].topic == "/helix/cmd_vel"
    assert by_name["navigation"].topic == "/nav/cmd_vel"


def test_yaml_loaded_specs_drive_the_model_end_to_end():
    """Sanity: feed the model from the production YAML, run a smoke arbitration."""
    specs = load_input_specs(_bringup_yaml_path())
    clock = FakeClock()
    mux = PriorityMux(specs, clock=clock)

    mux.publish("navigation", Twist2D(0.3, 0.0))
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    assert mux.arbitrate().winner == "helix_recovery"

    mux.publish("teleop", Twist2D(0.5, 0.0))
    assert mux.arbitrate().winner == "teleop"


# ---------------------------------------------------------------------------
# Misconfiguration sanity check: prove the assertions actually trip when the
# priority order is wrong. This is a meta-test: it builds a DELIBERATELY-broken
# mux config (nav above helix), confirms that the arbitration result is wrong
# under the broken config, and that the production YAML assertion above
# would reject it.
# ---------------------------------------------------------------------------

def test_misconfigured_priority_is_caught():
    """If someone inverts the priority order, the assertions in
    ``test_bringup_yaml_loads_and_matches_helix_intent`` MUST fail.
    Verify that here by constructing the broken state in-memory and
    checking that the safety invariant breaks observably."""
    clock = FakeClock()
    bad_specs = [
        # Wrong: nav above helix (autonomy would override a STOP).
        InputSpec("teleop", "/teleop/cmd_vel", HELIX_TIMEOUT_SEC, 200),
        InputSpec("helix_recovery", "/helix/cmd_vel", HELIX_TIMEOUT_SEC, 50),
        InputSpec("navigation", "/nav/cmd_vel", HELIX_TIMEOUT_SEC, 100),
    ]
    mux = PriorityMux(bad_specs, clock=clock)

    # Helix asks for STOP, nav asks to keep moving. Under the broken
    # priority, nav wins. This is the failure mode the test suite must
    # surface immediately.
    mux.publish("helix_recovery", Twist2D(0.0, 0.0))
    mux.publish("navigation", Twist2D(0.4, 0.0))

    bad_winner = mux.arbitrate().winner
    assert bad_winner == "navigation", (
        "Sanity check did not reproduce the misconfiguration: arbitration "
        f"chose {bad_winner!r} instead of 'navigation' under inverted priorities."
    )

    # And the human-readable invariant explicitly fails:
    by_name = {s.name: s for s in bad_specs}
    assert not (
        by_name["helix_recovery"].priority > by_name["navigation"].priority
    ), "broken config did not actually invert helix vs navigation"
