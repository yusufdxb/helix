#!/usr/bin/env python3
"""
Reference side of the arbiter replay protocol (helix_arbiter.arbiter_core).

The native tool ``helix_arbiter_replay`` and this module execute the same line
protocol, so identical input text must produce identical output text when the
C++ core and the Python reference agree, decision by decision.

Usage::

    python3 arbiter_replay.py < scenario.txt > decisions.txt
    python3 arbiter_replay.py --bench scenario.txt [--repeat N] [--warmup N]

Encoding: f64 is the 16-hex-digit IEEE-754 bit pattern (exact, keeps -0.0 and
NaN), str is ``s:`` plus percent-encoded UTF-8 bytes, integers are decimal and
booleans are 0/1. One output line per command:

==============================================  =====================================
command                                         output
==============================================  =====================================
new <hold_timeout> <max_lin> <max_ang>          new
spec <name> <priority> <timeout>                spec
build                                           build ok | build error
src <name> <lx> <ly> <lz> <ax> <ay> <az> <now>  src 1 | src 0 <why> | src unknown
hold <0|1> <fault> <epoch> <seq> <now>          hold 1 | hold 0
decide <now>                                    decide <REASON> <source> <fault>
                                                <vx> <vy> <wz> <helix_forced>
reset                                           reset
counters                                        counters <rejected> <reordered>
                                                <transitions> <name>=<rejected>...
param <name> <b|i|d|s|x> <value>                param
clearparams                                     clearparams
config                                          config ok <fields...> | config error
unknown                                         unknown <name>...
layout <out> <status> <hold> <n> [<name> <topic>]*  layout ok | layout error
==============================================  =====================================
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from typing import Dict, Iterable, List, Optional

from helix_arbiter.arbiter_core import (
    Arbiter,
    ConfigError,
    Limits,
    SourceSpec,
    check_topic_layout,
    parse_config,
    unknown_parameters,
    validate_twist,
)

_SAFE = frozenset(b'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_./~{}-')
_WHY = {'non-finite component': 'nonfinite', 'linear over limit': 'linear',
        'angular over limit': 'angular'}


def f64(x: float) -> str:
    return struct.pack('>d', x).hex()


def from_f64(tok: str) -> float:
    if len(tok) != 16:
        raise ValueError(f'bad f64 token {tok!r}')
    return struct.unpack('>d', bytes.fromhex(tok))[0]


def enc(s: str) -> str:
    return 's:' + ''.join(chr(c) if c in _SAFE else '%%%02X' % c for c in s.encode('utf-8'))


def dec(tok: str) -> str:
    if not tok.startswith('s:'):
        raise ValueError(f'bad str token {tok!r}')
    out = bytearray()
    i = 2
    while i < len(tok):
        if tok[i] == '%' and i + 2 < len(tok):
            out.append(int(tok[i + 1:i + 3], 16))
            i += 3
        else:
            out += tok[i].encode('utf-8')
            i += 1
    return out.decode('utf-8', errors='surrogateescape')


def u64(tok: str) -> int:
    v = int(tok)
    if not 0 <= v < 2 ** 64:
        raise ValueError(f'bad u64 token {tok!r}')
    return v


class Session:
    """Executes protocol commands against the Python reference core."""

    def __init__(self) -> None:
        self.hold_timeout = 0.5
        self.limits = Limits()
        self.specs: List[SourceSpec] = []
        self.arb: Optional[Arbiter] = None
        self.params: Dict[str, object] = {}

    def arbiter(self) -> Arbiter:
        if self.arb is None:
            raise RuntimeError('no arbiter built')
        return self.arb

    def run(self, t: List[str]) -> str:  # noqa: C901 (one branch per command)
        cmd = t[0]
        if cmd == 'new':
            self.hold_timeout = from_f64(t[1])
            self.limits = Limits(from_f64(t[2]), from_f64(t[3]))
            self.specs, self.arb = [], None
            return 'new'
        if cmd == 'spec':
            self.specs.append(SourceSpec(dec(t[1]), '/unused', int(t[2]), from_f64(t[3])))
            return 'spec'
        if cmd == 'build':
            try:
                self.arb = Arbiter(list(self.specs), self.hold_timeout, self.limits)
                return 'build ok'
            except ValueError:
                self.arb = None
                return 'build error'
        if cmd == 'src':
            name = dec(t[1])
            if name not in self.arbiter().source_names:
                return 'src unknown'
            vals = [from_f64(x) for x in t[2:8]]
            linear, angular = tuple(vals[:3]), tuple(vals[3:])
            if self.arbiter().on_source(name, linear, angular, from_f64(t[8])):
                return 'src 1'
            _, why = validate_twist(linear, angular, self.arbiter().limits)
            return f'src 0 {_WHY[why]}'
        if cmd == 'hold':
            ok = self.arbiter().on_hold(t[1] == '1', dec(t[2]), u64(t[3]), u64(t[4]),
                                        from_f64(t[5]))
            return 'hold 1' if ok else 'hold 0'
        if cmd == 'decide':
            d = self.arbiter().decide(from_f64(t[1]))
            c = d.command
            return (f'decide {d.reason} {enc(d.source)} {enc(d.hold_fault_id)} '
                    f'{f64(c.vx)} {f64(c.vy)} {f64(c.wz)} {1 if d.helix_forced else 0}')
        if cmd == 'reset':
            self.arbiter().reset()
            return 'reset'
        if cmd == 'counters':
            a = self.arbiter()
            c = a.counters
            per = ' '.join(f'{enc(n)}={c.by_source_rejected.get(n, 0)}' for n in a.source_names)
            return (f'counters {c.rejected} {c.hold_reordered} {c.hold_transitions}'
                    + (f' {per}' if per else ''))
        if cmd == 'param':
            kind, raw = t[2], t[3]
            value: object = {'b': lambda: raw == '1', 'i': lambda: int(raw),
                             'd': lambda: from_f64(raw), 's': lambda: dec(raw),
                             'x': lambda: []}[kind]()
            self.params[dec(t[1])] = value
            return 'param'
        if cmd == 'clearparams':
            self.params = {}
            return 'clearparams'
        if cmd == 'config':
            try:
                c = parse_config(self.params)
            except ConfigError:
                return 'config error'
            src = ''.join(f' {enc(s.name)} {enc(s.topic)} {s.priority} {f64(s.timeout_sec)}'
                          for s in c.sources)
            return (f'config ok {enc(c.output_topic)} {enc(c.status_topic)} '
                    f'{enc(c.hold_topic)} {f64(c.rate_hz)} {f64(c.hold_timeout_sec)} '
                    f'{f64(c.limits.max_abs_linear)} {f64(c.limits.max_abs_angular)} '
                    f'{c.shutdown_zero_count} {1 if c.autostart else 0} {len(c.sources)}{src}')
        if cmd == 'unknown':
            names = unknown_parameters(self.params)
            return 'unknown' + ''.join(f' {enc(n)}' for n in names)
        if cmd == 'layout':
            n = int(t[4])
            pairs = [(dec(t[5 + 2 * i]), dec(t[6 + 2 * i])) for i in range(n)]
            try:
                check_topic_layout(dec(t[1]), dec(t[2]), dec(t[3]), pairs)
                return 'layout ok'
            except ConfigError:
                return 'layout error'
        raise ValueError(f'unknown command {cmd!r}')


def replay(lines: Iterable[str]) -> List[str]:
    s = Session()
    out = []
    for line in lines:
        t = line.split()
        if not t or t[0].startswith('#'):
            continue
        out.append(s.run(t))
    return out


# -- benchmark ------------------------------------------------------------------

def _percentiles(ns: List[int]) -> dict:
    if not ns:
        return {'count': 0}
    v = sorted(ns)

    def pct(p: float) -> int:
        # Nearest-rank percentile, the same definition the C++ bench uses.
        k = max(1, min(len(v), math.ceil(p / 100.0 * len(v))))
        return v[k - 1]

    return {'count': len(v), 'p50_ns': pct(50), 'p95_ns': pct(95), 'p99_ns': pct(99),
            'max_ns': v[-1], 'mean_ns': sum(v) / len(v)}


def bench(path: str, repeat: int, warmup: int) -> dict:
    """Time every core call of the scenario's runtime events, as the C++ tool does."""
    setup = Session()
    events = []
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t or t[0].startswith('#'):
                continue
            if t[0] in ('new', 'spec', 'build'):
                setup.run(t)
            elif t[0] == 'src':
                v = [from_f64(x) for x in t[2:8]]
                events.append(('src', dec(t[1]), tuple(v[:3]), tuple(v[3:]), from_f64(t[8])))
            elif t[0] == 'hold':
                events.append(('hold', t[1] == '1', dec(t[2]), u64(t[3]), u64(t[4]),
                               from_f64(t[5])))
            elif t[0] == 'decide':
                events.append(('decide', from_f64(t[1])))
            elif t[0] == 'reset':
                events.append(('reset',))
    specs, hold_timeout, limits = list(setup.specs), setup.hold_timeout, setup.limits
    clock = time.perf_counter_ns
    sink = 0

    def run_once(per_op: Optional[Dict[str, List[int]]]) -> None:
        nonlocal sink
        arb = Arbiter(list(specs), hold_timeout, limits)
        for e in events:
            op = e[0]
            t0 = clock() if per_op is not None else 0
            if op == 'src':
                sink += arb.on_source(e[1], e[2], e[3], e[4])
            elif op == 'hold':
                sink += arb.on_hold(e[1], e[2], e[3], e[4], e[5])
            elif op == 'decide':
                d = arb.decide(e[1])
                sink += len(d.reason) + len(d.source)
            else:
                arb.reset()
            if per_op is not None:
                per_op[op].append(clock() - t0)

    for _ in range(warmup):
        run_once(None)
    per_op: Dict[str, List[int]] = {'src': [], 'hold': [], 'decide': [], 'reset': []}
    for _ in range(repeat):
        run_once(per_op)
    t0 = time.perf_counter()
    for _ in range(repeat):
        run_once(None)
    elapsed = time.perf_counter() - t0
    overhead = []
    for _ in range(100000):
        a = clock()
        overhead.append(clock() - a)
    return {'implementation': 'python', 'python': sys.version.split()[0],
            'events_per_replay': len(events), 'repeat': repeat, 'warmup': warmup,
            'throughput_events_per_s': len(events) * repeat / elapsed,
            'timer_overhead': _percentiles(overhead),
            'ops': {k: _percentiles(v) for k, v in per_op.items()}, 'checksum': sink}


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--bench', metavar='FILE')
    ap.add_argument('--repeat', type=int, default=20)
    ap.add_argument('--warmup', type=int, default=3)
    a = ap.parse_args(argv)
    if a.bench:
        print(json.dumps(bench(a.bench, max(1, a.repeat), max(0, a.warmup))))
        return 0
    for line in replay(sys.stdin):
        print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
