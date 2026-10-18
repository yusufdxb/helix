"""Pure motion-arbitration core. No ROS imports; every safety policy is here.

The arbiter is the single authoritative velocity output of the HELIX stack.
Upstream command sources (teleop, navigation, ...) compete by priority. HELIX
does NOT compete: it contributes a hold STATE that, when asserted, forces the
output to zero regardless of any source's priority.

Policies (documented in docs/MOTION_ARBITRATION.md, tested in
test/test_arbiter_core.py):

P1  HELIX hold asserted                     -> output ZERO
P2  HELIX state never received              -> output ZERO  (HELIX_STATE_MISSING)
P3  HELIX state older than hold_timeout     -> output ZERO  (HELIX_STATE_STALE)
    A dead or partitioned recovery node therefore stops the robot. There is
    deliberately no "release on stale" option.
P4  Malformed input (NaN, Inf, over limit)  -> rejected, AND that source's
    previous command is discarded, so a stale good value is never reused.
P5  Source older than its timeout           -> dropped from arbitration.
P6  No valid fresh source                   -> output ZERO  (NO_LIVE_INPUT)
P7  Any hold transition (assert or release) discards every source's stored
    command. After a RESUME a source must publish a NEW command before it
    can move the robot, so RESUME never replays a pre-hold command.
P8  HELIX state ordered by (epoch, seq); an older or duplicate message is
    dropped while the current state is fresh. Once stale, any epoch is
    accepted so a restarted publisher with a stepped clock can recover.
P9  Only linear.x, linear.y, angular.z pass; the other three axes must be
    finite and are forced to zero in the output.
P10 Freshness uses the arbiter's receipt clock only. Publisher stamps are
    for tracing (robot and payload clocks are known to be skewed).

Priority ties are broken by most recent receipt, like twist_mux.

The C++ port (helix_arbiter_cpp) implements the same policies and the same
parameter contract (parse_config, check_topic_layout); its parity test runs
both implementations on identical inputs and compares every decision.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

REASON_SOURCE = 'SOURCE'
REASON_HOLD = 'HELIX_HOLD'
REASON_STALE = 'HELIX_STATE_STALE'
REASON_MISSING = 'HELIX_STATE_MISSING'
REASON_NO_INPUT = 'NO_LIVE_INPUT'
REASON_SHUTDOWN = 'SHUTDOWN'

HELIX_FORCED = (REASON_HOLD, REASON_STALE, REASON_MISSING)


@dataclass(frozen=True)
class Command:
    """The three GO2-actionable axes. Everything else is forced to zero."""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


ZERO = Command()


@dataclass(frozen=True)
class SourceSpec:
    name: str
    topic: str
    priority: int
    timeout_sec: float


@dataclass(frozen=True)
class Limits:
    max_abs_linear: float = 1.0
    max_abs_angular: float = 1.5


@dataclass(frozen=True)
class HoldState:
    hold: bool
    fault_id: str
    epoch: int
    seq: int
    received_at: float


@dataclass(frozen=True)
class Decision:
    command: Command
    reason: str
    source: str = ''
    hold_fault_id: str = ''

    @property
    def helix_forced(self) -> bool:
        return self.reason in HELIX_FORCED


@dataclass
class _Slot:
    spec: SourceSpec
    command: Optional[Command] = None
    received_at: Optional[float] = None
    order: int = -1


@dataclass
class Counters:
    rejected: int = 0
    hold_reordered: int = 0
    hold_transitions: int = 0
    by_source_rejected: Dict[str, int] = field(default_factory=dict)


def validate_twist(linear: Tuple[float, float, float],
                   angular: Tuple[float, float, float],
                   limits: Limits) -> Tuple[Optional[Command], str]:
    """Return (Command, '') if acceptable, else (None, why)."""
    for v in (*linear, *angular):
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            return None, 'non-finite component'
    lx, ly, _ = linear
    _, _, az = angular
    if abs(lx) > limits.max_abs_linear or abs(ly) > limits.max_abs_linear:
        return None, 'linear over limit'
    if abs(az) > limits.max_abs_angular:
        return None, 'angular over limit'
    # +0.0 normalises -0.0 so is_zero and equality are exact.
    return Command(float(lx) + 0.0, float(ly) + 0.0, float(az) + 0.0), ''


class Arbiter:
    """Deterministic arbiter; the caller supplies time on every call."""

    def __init__(self, sources: List[SourceSpec], hold_timeout_sec: float,
                 limits: Limits = Limits()) -> None:
        if not sources:
            raise ValueError('arbiter needs at least one source')
        names = [s.name for s in sources]
        if not all(names):
            raise ValueError('source names must be non-empty')
        if len(set(names)) != len(names):
            raise ValueError(f'duplicate source names: {names}')
        for s in sources:
            if not math.isfinite(s.timeout_sec) or s.timeout_sec <= 0.0:
                # twist_mux treats timeout 0 as "never expires"; that would let a
                # dead source keep authority forever, so it is refused here.
                # NaN and +inf are refused for the same reason.
                raise ValueError(f'source {s.name!r} needs a finite timeout > 0')
        if not math.isfinite(hold_timeout_sec) or hold_timeout_sec <= 0.0:
            raise ValueError('hold_timeout_sec must be finite and > 0')
        # A NaN limit would disable the bound (every comparison is False) and an
        # infinite one is no bound at all.
        for lim in (limits.max_abs_linear, limits.max_abs_angular):
            if not math.isfinite(lim) or lim < 0.0:
                raise ValueError('velocity limits must be finite and >= 0')
        self._slots: Dict[str, _Slot] = {s.name: _Slot(s) for s in sources}
        self.hold_timeout_sec = hold_timeout_sec
        self.limits = limits
        self._hold: Optional[HoldState] = None
        self._order = 0
        self.counters = Counters()

    @property
    def source_names(self) -> Tuple[str, ...]:
        return tuple(self._slots)

    def spec(self, name: str) -> SourceSpec:
        return self._slots[name].spec

    # -- inputs ---------------------------------------------------------------

    def on_source(self, name: str, linear, angular, now: float) -> bool:
        """Record a source message. Returns False if it was rejected."""
        slot = self._slots[name]
        cmd, _why = validate_twist(tuple(linear), tuple(angular), self.limits)
        if cmd is None:
            slot.command = None          # P4: never fall back to an older value
            slot.received_at = None
            self.counters.rejected += 1
            self.counters.by_source_rejected[name] = (
                self.counters.by_source_rejected.get(name, 0) + 1)
            return False
        self._order += 1
        slot.command, slot.received_at, slot.order = cmd, now, self._order
        return True

    def on_hold(self, hold: bool, fault_id: str, epoch: int, seq: int,
                now: float) -> bool:
        """Record a HELIX hold state. Returns False if dropped as out of order."""
        cur = self._hold
        if cur is not None and self._hold_fresh(now):
            if (epoch, seq) <= (cur.epoch, cur.seq):
                self.counters.hold_reordered += 1
                return False
        prev_effective = self._effective_hold(now)
        self._hold = HoldState(bool(hold), fault_id, int(epoch), int(seq), now)
        if self._effective_hold(now) != prev_effective:
            self._clear_sources()        # P7
            self.counters.hold_transitions += 1
        return True

    # -- decision -------------------------------------------------------------

    def decide(self, now: float) -> Decision:
        effective = self._effective_hold(now)
        if effective:
            # Staleness is also a hold transition: clear sources so a command
            # stored before the state went stale cannot resume motion later.
            self._clear_sources()
        if self._hold is None:
            return Decision(ZERO, REASON_MISSING)
        if not self._hold_fresh(now):
            return Decision(ZERO, REASON_STALE, hold_fault_id=self._hold.fault_id)
        if self._hold.hold:
            return Decision(ZERO, REASON_HOLD, hold_fault_id=self._hold.fault_id)
        live = [s for s in self._slots.values() if self._source_fresh(s, now)]
        if not live:
            return Decision(ZERO, REASON_NO_INPUT)
        win = max(live, key=lambda s: (s.spec.priority, s.order))
        return Decision(win.command, REASON_SOURCE, source=win.spec.name)

    def reset(self) -> None:
        """Forget all inputs and HELIX state (used on lifecycle deactivate)."""
        self._clear_sources()
        self._hold = None

    # -- internals ------------------------------------------------------------

    def _hold_fresh(self, now: float) -> bool:
        return (self._hold is not None
                and (now - self._hold.received_at) <= self.hold_timeout_sec)

    def _effective_hold(self, now: float) -> bool:
        """True whenever HELIX forces zero (asserted, stale or missing)."""
        return self._hold is None or not self._hold_fresh(now) or self._hold.hold

    @staticmethod
    def _source_fresh(slot: _Slot, now: float) -> bool:
        return (slot.command is not None and slot.received_at is not None
                and (now - slot.received_at) <= slot.spec.timeout_sec)

    def _clear_sources(self) -> None:
        for s in self._slots.values():
            s.command = None
            s.received_at = None


def load_sources(params: dict) -> List[SourceSpec]:
    """Build SourceSpecs from a flat {name: {topic, priority, timeout}} map."""
    out = []
    for name, cfg in params.items():
        out.append(SourceSpec(str(name), str(cfg['topic']), int(cfg['priority']),
                              float(cfg['timeout'])))
    return out


# -- node parameter contract (shared with helix_arbiter_cpp) -------------------
#
# Both nodes run with automatically_declare_parameters_from_overrides, so a YAML
# value keeps its YAML type. The node flattens its parameters into one
# {name: value} map ('sources.teleop.topic' -> '/teleop/cmd_vel') and hands it to
# parse_config. Typing is strict: a priority of 200.5, a timeout given as the
# string '0.5' or a boolean where a number is expected is refused, not coerced.
# Real-valued parameters accept YAML integers (rate_hz: 50).

MAX_RATE_HZ = 1000.0
MAX_SHUTDOWN_ZERO_COUNT = 1000
_SOURCES_PREFIX = 'sources.'
_SOURCE_FIELDS = ('topic', 'priority', 'timeout')
_KNOWN_TOP_LEVEL = frozenset((
    'output_topic', 'status_topic', 'hold_topic', 'rate_hz', 'hold_timeout_sec',
    'max_abs_linear', 'max_abs_angular', 'shutdown_zero_count', 'autostart',
    'use_sim_time', 'arbiter_backend'))

# Declared by the node when the parameter file does not set them.
DEFAULT_PARAMETERS: Tuple[Tuple[str, object], ...] = (
    ('output_topic', '/cmd_vel'),
    ('status_topic', '/helix/arbiter/status'),
    ('hold_topic', '/helix/hold'),
    ('rate_hz', 50.0),
    ('hold_timeout_sec', 0.5),
    ('max_abs_linear', 1.0),
    ('max_abs_angular', 1.5),
    ('shutdown_zero_count', 10),
    ('autostart', False),
)


class ConfigError(ValueError):
    """A parameter set the arbiter refuses to configure with."""


@dataclass(frozen=True)
class ArbiterConfig:
    output_topic: str
    status_topic: str
    hold_topic: str
    rate_hz: float
    hold_timeout_sec: float
    limits: Limits
    shutdown_zero_count: int
    autostart: bool
    sources: Tuple[SourceSpec, ...]


def _require(params: Mapping[str, object], name: str) -> object:
    if name not in params:
        raise ConfigError(f'missing parameter {name!r}')
    return params[name]


def _as_string(v: object, name: str) -> str:
    if not isinstance(v, str):
        raise ConfigError(f'{name!r} must be a string')
    if not v:
        raise ConfigError(f'{name!r} must be a non-empty string')
    return v


def _as_integer(v: object, name: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise ConfigError(f'{name!r} must be an integer')
    return v


def _as_number(v: object, name: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ConfigError(f'{name!r} must be a number')
    return float(v)


def _as_bool(v: object, name: str) -> bool:
    if not isinstance(v, bool):
        raise ConfigError(f'{name!r} must be a boolean')
    return v


def parse_sources(params: Mapping[str, object]) -> List[SourceSpec]:
    """SourceSpecs from the 'sources.<name>.<field>' entries, sorted by name.

    Every source needs exactly topic (non-empty string), priority (integer) and
    timeout (finite number > 0). Any other field is an error, so a misspelt or
    unsupported key (e.g. 'enabled: false') cannot be silently ignored.
    """
    raw: Dict[str, Dict[str, object]] = {}
    for key, value in params.items():
        if not key.startswith(_SOURCES_PREFIX):
            continue
        name, _, fld = key[len(_SOURCES_PREFIX):].partition('.')
        if not name:
            raise ConfigError(f'source parameter {key!r} has an empty source name')
        raw.setdefault(name, {})[fld] = value
    if not raw:
        raise ConfigError('no sources configured (expected sources.<name>.topic/priority/timeout)')
    specs = []
    for name in sorted(raw):
        fields = raw[name]
        extra = sorted(set(fields) - set(_SOURCE_FIELDS))
        if extra:
            raise ConfigError(f'source {name!r} has unsupported field {extra[0]!r} '
                              '(allowed: topic, priority, timeout)')
        for f in _SOURCE_FIELDS:
            if f not in fields:
                raise ConfigError(f'source {name!r} is missing {f!r}')
        base = f'{_SOURCES_PREFIX}{name}.'
        timeout = _as_number(fields['timeout'], base + 'timeout')
        topic = _as_string(fields['topic'], base + 'topic')
        priority = _as_integer(fields['priority'], base + 'priority')
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ConfigError(f'{base}timeout must be finite and > 0')
        specs.append(SourceSpec(name, topic, priority, timeout))
    return specs


def parse_config(params: Mapping[str, object]) -> ArbiterConfig:
    """Parse and validate the node's full parameter map. Raises ConfigError.

    Topic names are not resolved here; see check_topic_layout.
    """
    output = _as_string(_require(params, 'output_topic'), 'output_topic')
    status = _as_string(_require(params, 'status_topic'), 'status_topic')
    hold = _as_string(_require(params, 'hold_topic'), 'hold_topic')
    rate = _as_number(_require(params, 'rate_hz'), 'rate_hz')
    if not math.isfinite(rate) or rate <= 0.0 or rate > MAX_RATE_HZ:
        raise ConfigError("'rate_hz' must be finite, > 0 and <= 1000")
    hold_timeout = _as_number(_require(params, 'hold_timeout_sec'), 'hold_timeout_sec')
    if not math.isfinite(hold_timeout) or hold_timeout <= 0.0:
        raise ConfigError("'hold_timeout_sec' must be finite and > 0")
    lin = _as_number(_require(params, 'max_abs_linear'), 'max_abs_linear')
    ang = _as_number(_require(params, 'max_abs_angular'), 'max_abs_angular')
    for name, v in (('max_abs_linear', lin), ('max_abs_angular', ang)):
        if not math.isfinite(v) or v < 0.0:
            raise ConfigError(f'{name!r} must be finite and >= 0')
    count = _as_integer(_require(params, 'shutdown_zero_count'), 'shutdown_zero_count')
    if not 1 <= count <= MAX_SHUTDOWN_ZERO_COUNT:
        raise ConfigError("'shutdown_zero_count' must be between 1 and 1000")
    autostart = _as_bool(_require(params, 'autostart'), 'autostart')
    return ArbiterConfig(output, status, hold, rate, hold_timeout, Limits(lin, ang),
                         count, autostart, tuple(parse_sources(params)))


def unknown_parameters(params: Mapping[str, object]) -> List[str]:
    """Top-level parameter names the arbiter does not use (typo guard)."""
    return sorted(k for k in params
                  if k not in _KNOWN_TOP_LEVEL and not k.startswith(_SOURCES_PREFIX)
                  and not k.startswith('qos_overrides.'))


def check_topic_layout(output: str, status: str, hold: str,
                       source_topics: Sequence[Tuple[str, str]]) -> None:
    """Collision checks on RESOLVED topic names. Raises ConfigError.

    output, status and hold are pairwise distinct, and every source topic is
    distinct from those three and from every other source, so neither a
    relative alias ('cmd_vel' in the root namespace) nor a remapping can feed
    the arbiter its own output.
    """
    if len({output, status, hold}) != 3:
        raise ConfigError('output_topic, status_topic and hold_topic must be distinct '
                          f'(resolved: {output}, {status}, {hold})')
    seen: Dict[str, str] = {}
    for name, topic in source_topics:
        if topic == output:
            raise ConfigError(f'source {name} subscribes to the output topic {output}')
        if topic in (status, hold):
            raise ConfigError(f"source {name} uses the arbiter's own topic {topic}")
        if topic in seen:
            raise ConfigError(f'sources {seen[topic]} and {name} share the topic {topic}')
        seen[topic] = name
