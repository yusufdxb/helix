"""
Scenario generators for the arbiter replay protocol (see arbiter_replay.py).

Adversarial scenarios target the edges where a port is most likely to drift
from helix_arbiter.arbiter_core: inclusive freshness boundaries computed in
floating point, full-width uint64 hold ordering (values a double cannot hold
exactly), priority ties, per-axis NaN/Inf, exact limit boundaries, signed
zero, hold transitions, non-monotonic time, construction and parameter
validation, and topic aliasing. Seeded random scenarios mix all of these.

``bench_workload`` is a deterministic replay of a realistic session (50 Hz
decisions, 20 Hz hold state, nav and teleop sources, hold episodes, malformed
input) used by bench_arbiter_core.py for a matched Python/C++ comparison.
"""
from __future__ import annotations

import math
import random
from typing import Dict, Iterator, List, Sequence, Tuple

from arbiter_replay import enc, f64

U64_MAX = 2 ** 64 - 1
I64_MIN, I64_MAX = -2 ** 63, 2 ** 63 - 1
NAN, INF = math.nan, math.inf


class Script:
    """Builds protocol text; every method appends one command line."""

    def __init__(self) -> None:
        self.lines: List[str] = []

    def comment(self, text: str) -> 'Script':
        self.lines.append('# ' + text)
        return self

    def new(self, hold_timeout: float = 0.5, lin: float = 1.0, ang: float = 1.5) -> 'Script':
        self.lines.append(f'new {f64(hold_timeout)} {f64(lin)} {f64(ang)}')
        return self

    def spec(self, name: str, priority: int, timeout: float) -> 'Script':
        self.lines.append(f'spec {enc(name)} {priority} {f64(timeout)}')
        return self

    def build(self) -> 'Script':
        self.lines.append('build')
        return self

    def arbiter(self, sources: Sequence[Tuple[str, int, float]], hold_timeout: float = 0.5,
                lin: float = 1.0, ang: float = 1.5) -> 'Script':
        self.new(hold_timeout, lin, ang)
        for s in sources:
            self.spec(*s)
        return self.build()

    def src(self, name: str, now: float, lx: float = 0.0, ly: float = 0.0, lz: float = 0.0,
            ax: float = 0.0, ay: float = 0.0, az: float = 0.0) -> 'Script':
        vals = ' '.join(f64(v) for v in (lx, ly, lz, ax, ay, az))
        self.lines.append(f'src {enc(name)} {vals} {f64(now)}')
        return self

    def hold(self, hold: bool, epoch: int, seq: int, now: float, fault: str = '') -> 'Script':
        self.lines.append(f'hold {1 if hold else 0} {enc(fault)} {epoch} {seq} {f64(now)}')
        return self

    def decide(self, now: float) -> 'Script':
        self.lines.append(f'decide {f64(now)}')
        return self

    def reset(self) -> 'Script':
        self.lines.append('reset')
        return self

    def counters(self) -> 'Script':
        self.lines.append('counters')
        return self

    def param(self, name: str, value: object) -> 'Script':
        if isinstance(value, bool):
            kind, raw = 'b', '1' if value else '0'
        elif isinstance(value, int):
            kind, raw = 'i', str(value)
        elif isinstance(value, float):
            kind, raw = 'd', f64(value)
        elif isinstance(value, str):
            kind, raw = 's', enc(value)
        else:
            kind, raw = 'x', '-'
        self.lines.append(f'param {enc(name)} {kind} {raw}')
        return self

    def params(self, values: Dict[str, object]) -> 'Script':
        self.lines.append('clearparams')
        for k, v in values.items():
            self.param(k, v)
        self.lines.append('config')
        self.lines.append('unknown')
        return self

    def layout(self, out: str, status: str, hold: str,
               sources: Sequence[Tuple[str, str]]) -> 'Script':
        pairs = ''.join(f' {enc(n)} {enc(t)}' for n, t in sources)
        self.lines.append(f'layout {enc(out)} {enc(status)} {enc(hold)} {len(sources)}{pairs}')
        return self

    def text(self) -> str:
        return '\n'.join(self.lines) + '\n'


TELEOP = ('teleop', 200, 0.5)
NAV = ('nav', 50, 0.5)


# -- adversarial scenarios -------------------------------------------------------

def stale_boundaries() -> Script:
    """Inclusive (now - received) <= timeout at, just inside and just past the edge."""
    s = Script()
    for timeout in (0.5, 0.1, 0.3, 0.1 + 0.2, 1e-3, 0.02, 7.25):
        for t0 in (0.0, 0.1, 12345.678, 98765.4321, 2.0 ** 40 + 0.5):
            s.comment(f'source timeout {timeout!r} at t0 {t0!r}')
            s.arbiter([('nav', 50, timeout)], hold_timeout=timeout)
            s.hold(False, 1, 1, t0).src('nav', t0, lx=0.2)
            edge = t0 + timeout
            for now in (math.nextafter(edge, -INF), edge, math.nextafter(edge, INF),
                        t0 + timeout * (1 + 1e-12)):
                s.decide(now)
            # hold staleness edge with a fresh source
            s.hold(False, 1, 2, t0 + 10 * timeout)
            s.src('nav', t0 + 10 * timeout + timeout / 2, lx=0.3)
            hedge = t0 + 10 * timeout + timeout
            for now in (math.nextafter(hedge, -INF), hedge, math.nextafter(hedge, INF)):
                s.decide(now)
    return s


def hold_ordering_full_width() -> Script:
    """(epoch, seq) ordering on values a double cannot represent exactly."""
    s = Script().arbiter([NAV])
    big = [0, 1, 2 ** 53, 2 ** 53 + 1, 2 ** 63 - 1, 2 ** 63, 2 ** 64 - 2, U64_MAX]
    t = 0.0
    s.hold(False, 2 ** 53, 2 ** 53, t)
    for epoch in big:
        for seq in big:
            t += 0.001
            s.hold(seq % 2 == 0, epoch, seq, t, fault=f'f{epoch % 7}').decide(t)
    # adjacent integers above 2**53 collapse to one double: must still order
    s.reset()
    t += 1.0
    s.hold(False, 2 ** 53 + 1, 5, t).hold(True, 2 ** 53, 9, t, 'older-epoch').decide(t)
    s.hold(True, 2 ** 53 + 1, 5, t, 'duplicate').decide(t)
    s.hold(True, 2 ** 53 + 1, 6, t, 'newer').decide(t)
    s.hold(False, U64_MAX, U64_MAX, t).hold(False, U64_MAX, U64_MAX, t).decide(t)
    # once stale, any epoch is accepted (restart with a stepped-back clock)
    t += 0.6
    s.decide(t).hold(False, 0, 0, t).decide(t).src('nav', t, lx=0.1).decide(t)
    s.hold(False, 0, 0, t).hold(False, 0, 1, t).decide(t + 0.5).decide(t + 0.51)
    return s.counters()


def priority_ties() -> Script:
    s = Script().arbiter([('a', 100, 0.5), ('b', 100, 0.5), ('c', 10, 0.5),
                          ('lo', I64_MIN, 0.5), ('hi', I64_MAX, 0.25)])
    t = 0.0
    s.hold(False, 1, 1, t)
    for name in ('c', 'a', 'b', 'a', 'lo', 'b'):
        t += 0.01
        s.src(name, t, lx=0.1).decide(t)
    s.src('a', t, lx=0.2).src('b', t, lx=0.3).decide(t)       # same instant: later receipt wins
    s.src('hi', t, az=0.4).decide(t).decide(t + 0.25).decide(t + 0.2500001)
    for k in range(12):
        t += 0.1
        s.hold(False, 1, 2 + k, t)
        if k % 3 == 0:
            s.src('b', t, lx=0.05 * k)
        if k % 4 == 0:
            s.src('a', t, ly=-0.05 * k)
        s.decide(t).decide(t + 0.45).decide(t + 0.55)
    return s.counters()


def invalid_inputs() -> Script:
    """P4: rejected input discards the source's previous good command."""
    s = Script().arbiter([TELEOP, NAV], lin=1.0, ang=1.5)
    t = 0.0
    s.hold(False, 7, 1, t)
    axes = ('lx', 'ly', 'lz', 'ax', 'ay', 'az')
    for axis in axes:
        for bad in (NAN, INF, -INF, -NAN):
            t += 0.01
            s.src('teleop', t, lx=0.4).decide(t)
            s.src('teleop', t, **{axis: bad}).decide(t)
            s.src('nav', t, lx=0.2).src('teleop', t, **{'lx': 0.1, axis: bad}).decide(t)
            s.hold(False, 7, 2 + len(s.lines), t)
    lim_l, lim_a = 1.0, 1.5
    for kw in ({'lx': lim_l}, {'lx': -lim_l}, {'ly': lim_l}, {'az': lim_a}, {'az': -lim_a},
               {'lx': math.nextafter(lim_l, INF)}, {'ly': -math.nextafter(lim_l, INF)},
               {'az': math.nextafter(lim_a, INF)}, {'lz': 1e300}, {'ax': -1e300},
               {'ay': 5e-324}, {'lx': 5e-324}, {'lx': -0.0, 'ly': -0.0, 'az': -0.0},
               {'lz': 9.0, 'ax': 0.5, 'ay': 0.5, 'lx': 0.1, 'az': 0.2}, {}):
        t += 0.01
        s.hold(False, 7, 10_000 + len(s.lines), t)
        s.src('teleop', t, **kw).decide(t)
    return s.counters()


def zero_and_negative_limits() -> Script:
    s = Script().arbiter([NAV], lin=0.0, ang=0.0)
    s.hold(False, 1, 1, 0.0)
    s.src('nav', 0.0).decide(0.0)
    s.src('nav', 0.0, lx=-0.0).decide(0.0)
    s.src('nav', 0.0, lx=5e-324).decide(0.0)
    s.src('nav', 0.0, ax=3.0, lz=-2.0).decide(0.0)
    for lin, ang in ((-1.0, 1.5), (1.0, -0.1), (NAN, 1.5), (1.0, NAN), (INF, 1.5), (1.0, -INF),
                     (0.0, 0.0), (1e308, 1e308)):
        s.arbiter([NAV], lin=lin, ang=ang)
    return s


def hold_transitions() -> Script:
    """P7: assert, release and staleness all discard stored commands."""
    s = Script().arbiter([TELEOP, NAV])
    t, seq = 0.0, 0

    def beat(hold: bool, fault: str = 'rate_hz/utlidar_cloud') -> None:
        nonlocal seq
        seq += 1
        s.hold(hold, 1000, seq, t, fault if hold else '')

    beat(False)
    s.src('nav', t, lx=0.3).decide(t)
    t += 0.02
    beat(True)
    s.decide(t)
    s.src('teleop', t, lx=0.5).decide(t)             # teleop cannot override a hold
    t += 0.02
    s.src('nav', t, lx=0.3)                           # received during the hold
    beat(False)                                        # RESUME
    s.decide(t)                                        # zero: pre-hold command not replayed
    for _ in range(5):
        t += 0.02
        beat(False)
        s.decide(t)
    s.src('nav', t, lx=0.3).decide(t)
    beat(False)
    s.src('nav', t, lx=0.25).decide(t)
    t += 0.6                                           # recovery silent: stale
    s.decide(t)
    beat(False)                                        # comes back: stale -> released transition
    s.decide(t)
    s.src('nav', t, lx=0.2).decide(t)
    for fault in ('', ' spaced id ', 'p%rcent', 'unicodé', 'tab\tid', 'a' * 300):
        t += 0.02
        beat(True, fault)
        s.decide(t)
        t += 0.02
        beat(False)
        s.src('nav', t, lx=0.1).decide(t)
    return s.counters()


def fail_closed() -> Script:
    s = Script().arbiter([TELEOP, NAV])
    s.decide(0.0).src('teleop', 0.0, lx=0.5).decide(0.0)          # missing
    s.hold(False, 3, 1, 0.1).decide(0.1)                             # released, no input
    s.src('nav', 0.1, lx=0.2).decide(0.1)
    s.hold(True, 3, 2, 0.2, 'f').decide(0.2).decide(0.71)           # stale keeps fault id
    s.hold(False, 3, 3, 0.8).decide(0.8).decide(1.31)
    s.src('nav', 1.31, lx=0.2).reset().decide(1.31)                  # reset: missing again
    s.hold(False, 3, 4, 1.4).src('nav', 1.4, lx=0.2).decide(1.4)
    s.hold(False, 3, 4, 1.4).decide(1.4)                             # duplicate dropped
    return s.counters()


def nonmonotonic_time() -> Script:
    s = Script().arbiter([TELEOP, NAV])
    for now in (5.0, 4.0, 4.5, 3.0, 3.0, 10.0, 9.5, 9.49, 100.0, 0.0):
        s.hold(False, 9, int(now * 1000), now).src('nav', now - 0.1, lx=0.1)
        s.decide(now).decide(now - 1.0).decide(now + 0.4)
    return s.counters()


def build_validation() -> Script:
    s = Script()
    for sources, hold_timeout in (
            ([], 0.5), ([('a', 1, 0.5), ('a', 2, 0.5)], 0.5), ([('', 1, 0.5)], 0.5),
            ([('a', 1, 0.0)], 0.5), ([('a', 1, -0.1)], 0.5), ([('a', 1, NAN)], 0.5),
            ([('a', 1, INF)], 0.5), ([('a', 1, 0.5)], 0.0), ([('a', 1, 0.5)], NAN),
            ([('a', 1, 0.5)], INF), ([('a', 1, 0.5)], -1.0), ([('a', 1, 5e-324)], 5e-324),
            ([('a', 1, 1e300)], 1e300), ([('b', 1, 0.5), ('a', 1, 0.5)], 0.5)):
        s.arbiter(sources, hold_timeout=hold_timeout)
    return s


def config_cases() -> Script:
    """parse_config / unknown_parameters on valid, boundary and type-confused maps."""
    base: Dict[str, object] = {
        'output_topic': '/cmd_vel', 'status_topic': '/helix/arbiter/status',
        'hold_topic': '/helix/hold', 'rate_hz': 50.0, 'hold_timeout_sec': 0.5,
        'max_abs_linear': 1.0, 'max_abs_angular': 1.5, 'shutdown_zero_count': 10,
        'autostart': False, 'use_sim_time': False,
        'sources.teleop.topic': '/teleop/cmd_vel', 'sources.teleop.priority': 200,
        'sources.teleop.timeout': 0.5, 'sources.nav.topic': '/nav/cmd_vel',
        'sources.nav.priority': 50, 'sources.nav.timeout': 0.5,
    }
    s = Script().params(base)
    variants: List[Tuple[str, object]] = [
        ('rate_hz', 50), ('rate_hz', 1000.0), ('rate_hz', math.nextafter(1000.0, INF)),
        ('rate_hz', 0.0), ('rate_hz', -5.0), ('rate_hz', NAN), ('rate_hz', INF), ('rate_hz', '50'),
        ('rate_hz', True), ('rate_hz', []), ('hold_timeout_sec', 1), ('hold_timeout_sec', 0),
        ('hold_timeout_sec', NAN), ('hold_timeout_sec', 5e-324), ('max_abs_linear', 0),
        ('max_abs_linear', -0.0), ('max_abs_linear', -1e-9), ('max_abs_linear', INF),
        ('max_abs_angular', NAN), ('max_abs_angular', 2), ('shutdown_zero_count', 1),
        ('shutdown_zero_count', 1000), ('shutdown_zero_count', 0), ('shutdown_zero_count', 1001),
        ('shutdown_zero_count', 10.0), ('shutdown_zero_count', -3), ('autostart', True),
        ('autostart', 'true'), ('autostart', 1), ('output_topic', ''), ('output_topic', 5),
        ('status_topic', False), ('hold_topic', 'helix/hold'),
        ('sources.nav.priority', 50.0), ('sources.nav.priority', True),
        ('sources.nav.priority', '50'), ('sources.nav.priority', I64_MIN),
        ('sources.nav.priority', I64_MAX), ('sources.nav.timeout', 1),
        ('sources.nav.timeout', '0.5'), ('sources.nav.timeout', 0.0),
        ('sources.nav.timeout', NAN), ('sources.nav.timeout', INF),
        ('sources.nav.timeout', False), ('sources.nav.topic', ''),
        ('sources.nav.topic', 7), ('sources.nav.enabled', False), ('sources.nav', 5),
        ('sources.nav.topic.extra', 'x'), ('sources.zeta.topic', '/z'),
        ('hold_timout_sec', 2.0), ('max_abs_linaer', 0.3), ('qos_overrides./cmd_vel.depth', 5),
        ('arbiter_backend', 'cpp'), ('sources', []),
    ]
    for key, value in variants:
        p = dict(base)
        p[key] = value
        s.comment(f'{key} = {value!r}').params(p)
    for missing in ('output_topic', 'rate_hz', 'autostart', 'sources.nav.topic',
                    'sources.nav.priority', 'sources.nav.timeout', 'shutdown_zero_count'):
        p = {k: v for k, v in base.items() if k != missing}
        s.comment(f'missing {missing}').params(p)
    only = {k: v for k, v in base.items() if not k.startswith('sources.')}
    s.comment('no sources').params(only)
    return s


def layout_cases() -> Script:
    s = Script()
    out, st, hold = '/cmd_vel', '/helix/arbiter/status', '/helix/hold'
    for o, t, h, srcs in (
            (out, st, hold, [('teleop', '/teleop/cmd_vel'), ('nav', '/nav/cmd_vel')]),
            (out, st, hold, [('nav', out)]), (out, st, hold, [('nav', hold)]),
            (out, st, hold, [('nav', st)]),
            (out, st, hold, [('a', '/x'), ('b', '/x')]), (out, out, hold, [('a', '/x')]),
            (out, st, out, [('a', '/x')]), (out, hold, hold, [('a', '/x')]),
            ('/ns/cmd_vel', st, hold, [('a', '/cmd_vel')]), (out, st, hold, [])):
        s.layout(o, t, h, srcs)
    return s


ADVERSARIAL = {
    'stale_boundaries': stale_boundaries,
    'hold_ordering_full_width': hold_ordering_full_width,
    'priority_ties': priority_ties,
    'invalid_inputs': invalid_inputs,
    'zero_and_negative_limits': zero_and_negative_limits,
    'hold_transitions': hold_transitions,
    'fail_closed': fail_closed,
    'nonmonotonic_time': nonmonotonic_time,
    'build_validation': build_validation,
    'config_cases': config_cases,
    'layout_cases': layout_cases,
}


# -- seeded random scenarios -----------------------------------------------------

_VALUES = (0.0, -0.0, 0.1, -0.25, 0.3, 0.5, 1.0, -1.0, 1.5, -1.5, 0.999999, 5e-324, 1e-310,
           2.0, -7.5, 1e300, NAN, INF, -INF)
_TIMEOUTS = (0.5, 0.25, 0.1, 0.02, 1e-3, 0.1 + 0.2, 1.0)


def random_scenario(seed: int, events: int = 400) -> Script:
    rng = random.Random(seed)
    n = rng.randint(1, 5)
    prios = [rng.choice((0, 10, 50, 50, 200, -3, I64_MAX)) for _ in range(n)]
    sources = [(f's{i}', prios[i], rng.choice(_TIMEOUTS)) for i in range(n)]
    hold_timeout = rng.choice(_TIMEOUTS)
    lin, ang = rng.choice(((1.0, 1.5), (0.25, 0.5), (0.0, 0.0), (2.0, 3.0)))
    s = Script().comment(f'seed {seed}').arbiter(sources, hold_timeout, lin, ang)
    t = rng.choice((0.0, 1000.0, 86400.123, 2.0 ** 33))
    epoch, seq = rng.choice(((0, 0), (1_700_000_000_000_000_000, 0), (U64_MAX - 50, U64_MAX - 50)))
    for _ in range(events):
        r = rng.random()
        if r < 0.1:
            t = math.nextafter(t, INF)
        elif r < 0.6:
            t += rng.choice((0.0, 0.001, 0.02, 0.05, hold_timeout, sources[0][2], 0.6))
        elif r < 0.62:
            t -= rng.choice((0.001, 0.3))
        op = rng.random()
        if op < 0.35:
            name = rng.choice(sources)[0]
            vals = {ax: rng.choice(_VALUES) if rng.random() < 0.3 else rng.uniform(-1.6, 1.6)
                    for ax in ('lx', 'ly', 'az')}
            vals.update({ax: rng.choice(_VALUES) if rng.random() < 0.1 else 0.0
                         for ax in ('lz', 'ax', 'ay')})
            s.src(name, t, **vals)
        elif op < 0.6:
            k = rng.random()
            if k < 0.7:
                seq = min(seq + 1, U64_MAX)
            elif k < 0.8:
                pass                                   # duplicate
            elif k < 0.9:
                seq = max(seq - rng.randint(1, 3), 0)  # reordered
            elif k < 0.95:
                epoch, seq = rng.randrange(0, 2 ** 64), rng.randrange(0, 2 ** 64)
            else:
                epoch = max(epoch - 1, 0)              # restart with a stepped-back clock
            hold = rng.random() < 0.3
            s.hold(hold, epoch, seq, t, rng.choice(('', 'f1', 'rate_hz/x')) if hold else '')
        elif op < 0.97:
            s.decide(t)
        elif op < 0.985:
            s.reset()
        else:
            s.counters()
    return s.counters()


def random_config(seed: int) -> Script:
    rng = random.Random(seed)
    s = Script()
    pool: List[object] = [0, 1, -1, 50, 1000, 1001, 0.0, -0.0, 0.5, 50.0, 1e-300, NAN, INF, -INF,
                          True, False, '', '/x', 'rel/topic', '0.5', [], 200, 10, 1.5]
    keys = ['output_topic', 'status_topic', 'hold_topic', 'rate_hz', 'hold_timeout_sec',
            'max_abs_linear', 'max_abs_angular', 'shutdown_zero_count', 'autostart']
    good: Dict[str, object] = {'output_topic': '/o', 'status_topic': '/s', 'hold_topic': '/h',
                               'rate_hz': 50.0, 'hold_timeout_sec': 0.5,
                               'max_abs_linear': 1.0, 'max_abs_angular': 1.5,
                               'shutdown_zero_count': 10, 'autostart': False}
    for _ in range(60):
        p = dict(good)
        for k in rng.sample(keys, rng.randint(0, 3)):
            p[k] = rng.choice(pool)
        for i in range(rng.randint(0, 3)):
            for fld, v in (('topic', f'/t{i}'), ('priority', 10 * i), ('timeout', 0.5)):
                if rng.random() < 0.85:
                    p[f'sources.n{i}.{fld}'] = v if rng.random() < 0.8 else rng.choice(pool)
            if rng.random() < 0.05:
                p[f'sources.n{i}.extra'] = 1
        if rng.random() < 0.2:
            p[rng.choice(('typo_param', 'qos_overrides.x', 'use_sim_time'))] = 1
        s.params(p)
    return s


def all_random(count: int = 200, base_seed: int = 20260917) -> Iterator[Tuple[str, Script]]:
    for i in range(count):
        yield f'random_{i}', random_scenario(base_seed + i)
    for i in range(20):
        yield f'random_config_{i}', random_config(base_seed * 7 + i)


# -- benchmark workload ----------------------------------------------------------

def bench_workload(seconds: float = 600.0, seed: int = 7) -> Script:
    """
    Build a realistic session: 50 Hz decide, 20 Hz hold and nav, teleop bursts.

    Holds are asserted for ~3 s roughly every 30 s; about 0.5% of source messages
    are malformed. Times are seconds on a monotonic clock starting at 5000 s.
    """
    rng = random.Random(seed)
    s = Script().comment(f'bench workload {seconds}s seed {seed}').arbiter([TELEOP, NAV])
    t0 = 5000.0
    ticks = int(seconds * 100)             # 100 Hz base grid (10 ms)
    seq, holding_until = 0, -1.0
    for k in range(ticks):
        t = t0 + k * 0.01
        if k % 5 == 0:                     # 20 Hz hold state
            if holding_until < t and rng.random() < 1 / 600:
                holding_until = t + 3.0
            seq += 1
            hold = t < holding_until
            s.hold(hold, 1_700_000_000_000_000_000, seq, t,
                   'rate_hz/utlidar_cloud' if hold else '')
        if k % 5 == 2:                     # 20 Hz nav
            bad = rng.random() < 0.005
            s.src('nav', t, lx=NAN if bad else rng.uniform(-0.3, 0.3), az=rng.uniform(-0.4, 0.4))
        if (k // 300) % 4 == 1 and k % 5 == 3:   # teleop bursts
            s.src('teleop', t, lx=rng.uniform(-0.5, 0.5), ly=rng.uniform(-0.2, 0.2))
        if k % 2 == 0:                     # 50 Hz output
            s.decide(t)
    return s
