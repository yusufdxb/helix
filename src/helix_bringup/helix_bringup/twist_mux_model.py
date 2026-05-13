"""Pure-function model of the twist_mux priority arbitrator.

This module is a sim-only verification harness for the priority-arbitration
behavior wired up in commit ce60db1 ("twist_mux: wire priority arbitration
into closedloop pipeline"). It does NOT spin a real twist_mux node. It
implements the deterministic, side-effect-free function:

    f(active_inputs, now) -> arbitrated_twist or None

following the same semantics that the upstream twist_mux package documents:

  1. Each input has a ``timeout`` (seconds) and a ``priority`` (int, higher
     wins). Inputs are identified by their stable name as declared in the
     YAML file (e.g. ``teleop``, ``helix_recovery``, ``navigation``).
  2. An input is "live" at time ``now`` iff its most recent message arrived
     at ``t_last`` with ``now - t_last < timeout``. Stale inputs are removed
     from arbitration.
  3. Among live inputs, the one with the highest priority wins. Its twist
     is emitted unchanged.
  4. If two live inputs tie on priority (the YAML can technically allow
     this, even though the HELIX config does not), the tiebreaker is the
     most recently published message (last-writer-wins within a priority
     class). This mirrors upstream twist_mux behavior at the time of
     writing (see twist_mux::TwistMux::onTwistReceived).
  5. If no input is live, no twist is emitted. The real twist_mux node
     publishes zero in that case, which we model by returning the
     ``zero_twist`` sentinel from ``arbitrate_with_zero_on_idle`` if the
     caller asks for that explicit behavior.

Locks are intentionally NOT modeled here. The HELIX twist_mux config
declares a single dummy lock with priority 0 and timeout 0 to satisfy
twist_mux's parameter-declaration requirement; that lock can never preempt
any input. If we ever add real locks the model will need extending.

The model is driven by an injected monotonic clock so tests can advance
time deterministically without sleeping. Time units are seconds throughout.

Tests live in src/helix_bringup/test/test_twist_mux_model.py and
src/helix_bringup/test/test_twist_mux_integration.py.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional


@dataclass(frozen=True)
class Twist2D:
    """Minimal twist representation, enough for arbitration tests.

    The real geometry_msgs/Twist has six fields. We only need linear.x
    and angular.z to distinguish "which publisher won the mux", so we
    keep the model small.
    """
    linear_x: float = 0.0
    angular_z: float = 0.0


ZERO = Twist2D(0.0, 0.0)


@dataclass(frozen=True)
class InputSpec:
    """Static config for one mux input: matches the YAML schema."""
    name: str
    topic: str
    timeout_sec: float
    priority: int


@dataclass
class _InputState:
    """Mutable runtime state for one input: last message + arrival time."""
    spec: InputSpec
    last_twist: Optional[Twist2D] = None
    last_arrival: Optional[float] = None
    # Monotonic sequence counter, used for last-writer-wins on a priority tie.
    last_seq: int = -1


@dataclass
class ArbitrationResult:
    """What the mux decided at a given instant.

    ``winner`` is the input name, or None if every input is stale.
    ``twist`` is the emitted twist (None if no input is live and the
    caller used ``arbitrate``; ZERO if the caller used
    ``arbitrate_with_zero_on_idle``).
    ``live_inputs`` is the set of names that were live at decision time.
    """
    winner: Optional[str]
    twist: Optional[Twist2D]
    live_inputs: tuple = field(default_factory=tuple)


class PriorityMux:
    """Deterministic priority arbitrator with timeout-based dropout.

    Usage:
        mux = PriorityMux(specs, clock=fake_clock)
        mux.publish("teleop", Twist2D(0.5, 0.0))
        result = mux.arbitrate()
        # result.winner == "teleop"

    The mux is single-threaded. Tests advance ``clock`` to model dropouts
    and stale data. There is no spin loop and no ROS dependency.
    """

    def __init__(
        self,
        inputs: list[InputSpec],
        clock: Callable[[], float],
    ) -> None:
        if not inputs:
            raise ValueError("PriorityMux needs at least one input spec")
        names = [i.name for i in inputs]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate input names: {names}")
        self._states: dict[str, _InputState] = {
            i.name: _InputState(spec=i) for i in inputs
        }
        self._clock = clock
        self._seq = 0

    @property
    def input_names(self) -> tuple:
        return tuple(self._states.keys())

    def publish(self, name: str, twist: Twist2D) -> None:
        """Record a new message arrival for ``name`` at the current clock time."""
        if name not in self._states:
            raise KeyError(f"unknown mux input: {name!r}")
        state = self._states[name]
        state.last_twist = twist
        state.last_arrival = self._clock()
        self._seq += 1
        state.last_seq = self._seq

    def arbitrate(self) -> ArbitrationResult:
        """Return the current winner, or None if every input is stale.

        The "no live input" branch returns ``winner=None, twist=None``.
        If the caller wants the real twist_mux behavior of publishing zero
        on idle, see ``arbitrate_with_zero_on_idle``.
        """
        now = self._clock()
        live = self._live_inputs(now)
        if not live:
            return ArbitrationResult(winner=None, twist=None, live_inputs=())
        winner = self._pick_winner(live)
        return ArbitrationResult(
            winner=winner.spec.name,
            twist=winner.last_twist,
            live_inputs=tuple(s.spec.name for s in live),
        )

    def arbitrate_with_zero_on_idle(self) -> ArbitrationResult:
        """Same as ``arbitrate`` but emits ZERO twist when no input is live.

        This mirrors the upstream twist_mux behavior: when all topics go
        stale simultaneously, twist_mux publishes a zero twist rather
        than going silent. HELIX relies on this for the "robot holds when
        recovery and operator both fall silent" safety invariant.
        """
        result = self.arbitrate()
        if result.winner is None:
            return ArbitrationResult(winner=None, twist=ZERO, live_inputs=())
        return result

    def _live_inputs(self, now: float) -> list[_InputState]:
        out: list[_InputState] = []
        for state in self._states.values():
            if state.last_arrival is None or state.last_twist is None:
                continue
            age = now - state.last_arrival
            if age < state.spec.timeout_sec:
                out.append(state)
        return out

    @staticmethod
    def _pick_winner(live: list[_InputState]) -> _InputState:
        # Highest priority wins; ties broken by most recent publish (last_seq).
        return max(live, key=lambda s: (s.spec.priority, s.last_seq))


# ---------------------------------------------------------------------------
# YAML loader: keeps the model and the real twist_mux node in lockstep.
# ---------------------------------------------------------------------------

def load_input_specs(yaml_path: Path) -> list[InputSpec]:
    """Parse a twist_mux YAML config into a list of InputSpec.

    The YAML schema is the one twist_mux itself consumes:
        twist_mux:
          ros__parameters:
            topics:
              <name>:
                topic: <str>
                timeout: <float>
                priority: <int>
            locks: {...}   # ignored by this model

    Raises ValueError if the file is missing the expected structure.
    """
    import yaml  # local import; YAML is only needed when loading from disk.

    with Path(yaml_path).open("r", encoding="utf-8") as fp:
        doc = yaml.safe_load(fp)

    try:
        topics = doc["twist_mux"]["ros__parameters"]["topics"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"{yaml_path}: missing twist_mux.ros__parameters.topics"
        ) from exc

    if not isinstance(topics, dict) or not topics:
        raise ValueError(f"{yaml_path}: topics must be a non-empty mapping")

    specs: list[InputSpec] = []
    for name, cfg in topics.items():
        try:
            specs.append(
                InputSpec(
                    name=str(name),
                    topic=str(cfg["topic"]),
                    timeout_sec=float(cfg["timeout"]),
                    priority=int(cfg["priority"]),
                )
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"{yaml_path}: input {name!r} has malformed entry: {cfg!r}"
            ) from exc
    return specs
