#!/usr/bin/env python3
"""
Reference side of the anomaly replay protocol (helix_core.anomaly_detector).

Executes the Python node's own methods (_process_sample, _cooldown_expired,
_emit_anomaly_fault, _emit_stale_fault) on a stand-in object, with the
module's ``time`` replaced by a scripted clock and the fault publisher
replaced by a recorder. No ROS graph is created and no detection logic is
re-implemented here. The native tool helix_anomaly_replay executes the same
protocol against AnomalyCore and pyfmt, so identical input must give
identical output text.

One known divergence: when a squared deviation overflows a double
(|s - mean| > ~1.34e154) the reference raises OverflowError, which in the real
node escapes the subscription callback and stops the process; the native core
continues with an infinite window std (z = 0) until the value leaves the
window. Generated scenarios stay below that range; test_python_parity.py pins
the divergence separately.

Usage::

    python3 anomaly_replay.py < scenario.txt > results.txt
    python3 anomaly_replay.py --bench scenario.txt [--repeat N] [--warmup N]

Commands (f64 = 16 hex digits of the IEEE-754 bits, str = ``s:`` + percent-
encoded UTF-8)::

    params <zscore:f64> <trigger:int> <window:int> <cooldown:f64> <min_duration:f64>
        -> params ok | params error           (window <= 0 is refused, as on_configure does)
    sample <metric:str> <value:f64> <mono:f64> <wall:f64>
        -> ok <streak> | fault <node> <type> <severity> <detail> <stamp:f64> <n>
           [<key> <value>]* <streak> | error OverflowError (Python only, see below)
    square <x:f64>          -> square <f64>                   x ** 2 (libm pow, not x * x)
    parse <text:str>        -> parse <f64> | parse none       float(text)
    repr <x:f64>            -> repr <str>                     repr(x)
    round <x:f64> <n>       -> round <str>                    str(round(x, n))
    fixed <x:f64> <n>       -> fixed <str>                    f'{x:.{n}f}'
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import threading
import time
from typing import Iterable, List, Optional

from helix_core import anomaly_detector as reference

_SAFE = frozenset(b'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_./~{}-')


def f64(x: float) -> str:
    return struct.pack('>d', x).hex()


def from_f64(tok: str) -> float:
    if len(tok) != 16:
        raise ValueError(f'bad f64 token {tok!r}')
    return struct.unpack('>d', bytes.fromhex(tok))[0]


def enc(s: str) -> str:
    raw = s.encode('utf-8', errors='surrogateescape')
    return 's:' + ''.join(chr(c) if c in _SAFE else '%%%02X' % c for c in raw)


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


class ScriptedClock:
    """Stands in for the ``time`` module inside helix_core.anomaly_detector."""

    def __init__(self) -> None:
        self.mono = 0.0
        self.wall = 0.0

    def monotonic(self) -> float:
        return self.mono

    def time(self) -> float:
        return self.wall


class _SilentLogger:
    def debug(self, *a, **k) -> None:
        pass

    info = warn = warning = error = debug


class Reference:
    """The reference detector's state and methods, without a ROS node."""

    _process_sample = reference.AnomalyDetector._process_sample
    _cooldown_expired = reference.AnomalyDetector._cooldown_expired
    _emit_anomaly_fault = reference.AnomalyDetector._emit_anomaly_fault
    _emit_stale_fault = reference.AnomalyDetector._emit_stale_fault

    def __init__(self, zscore: float, trigger: int, window: int, cooldown: float,
                 min_duration: float) -> None:
        self._zscore_threshold = zscore
        self._consecutive_trigger = trigger
        self._window_size = window
        self._emit_cooldown_s = cooldown
        self._min_anomaly_duration_s = min_duration
        self._windows: dict = {}
        self._consecutive: dict = {}
        self._anomaly_start: dict = {}
        self._last_emit: dict = {}
        self._data_lock = threading.Lock()
        self._fault_pub = self
        self.published: list = []
        self._logger = _SilentLogger()

    def get_logger(self) -> _SilentLogger:
        return self._logger

    def publish(self, msg) -> None:   # the _fault_pub stand-in
        self.published.append(msg)


class Session:
    def __init__(self) -> None:
        self.clock = ScriptedClock()
        self.ref: Optional[Reference] = None

    def run(self, t: List[str]) -> str:
        cmd = t[0]
        if cmd == 'params':
            window = int(t[3])
            if window <= 0:      # AnomalyDetector.on_configure refuses this
                self.ref = None
                return 'params error'
            self.ref = Reference(from_f64(t[1]), int(t[2]), window, from_f64(t[4]),
                                 from_f64(t[5]))
            return 'params ok'
        if cmd == 'sample':
            if self.ref is None:
                raise RuntimeError('no reference configured')
            metric = dec(t[1])
            self.clock.mono, self.clock.wall = from_f64(t[3]), from_f64(t[4])
            before = len(self.ref.published)
            saved = reference.time
            reference.time = self.clock
            try:
                self.ref._process_sample(metric, from_f64(t[2]))
            except OverflowError:
                # (s - mean) ** 2 overflowed: the reference raises here and,
                # in the real node, the exception ends rclpy.spin. Reported so
                # the documented divergence can be asserted, never hidden.
                return 'error OverflowError'
            finally:
                reference.time = saved
            streak = self.ref._consecutive.get(metric, 0)
            new = self.ref.published[before:]
            if not new:
                return f'ok {streak}'
            assert len(new) == 1
            m = new[0]
            pairs = ''.join(f' {enc(k)} {enc(v)}'
                            for k, v in zip(m.context_keys, m.context_values))
            return (f'fault {enc(m.node_name)} {enc(m.fault_type)} {m.severity} '
                    f'{enc(m.detail)} {f64(m.timestamp)} {len(m.context_keys)}{pairs} {streak}')
        if cmd == 'square':
            # The operator the reference applies to every deviation: (s - mean) ** 2.
            x = from_f64(t[1])
            try:
                return f'square {f64(x ** 2)}'
            except OverflowError:
                return 'error OverflowError'
        if cmd == 'parse':
            try:
                return f'parse {f64(float(dec(t[1])))}'
            except ValueError:
                return 'parse none'
        if cmd == 'repr':
            return f'repr {enc(repr(from_f64(t[1])))}'
        if cmd == 'round':
            return f'round {enc(str(round(from_f64(t[1]), int(t[2]))))}'
        if cmd == 'fixed':
            return f'fixed {enc(format(from_f64(t[1]), "." + t[2] + "f"))}'
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


def _percentiles(ns: List[int]) -> dict:
    if not ns:
        return {'count': 0}
    v = sorted(ns)

    def pct(p: float) -> int:
        k = max(1, min(len(v), math.ceil(p / 100.0 * len(v))))
        return v[k - 1]

    return {'count': len(v), 'p50_ns': pct(50), 'p95_ns': pct(95), 'p99_ns': pct(99),
            'max_ns': v[-1], 'mean_ns': sum(v) / len(v)}


def bench(path: str, repeat: int, warmup: int) -> dict:
    """Time _process_sample per sample line; logging is a no-op stand-in."""
    params, samples = None, []
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t or t[0].startswith('#'):
                continue
            if t[0] == 'params':
                params = (from_f64(t[1]), int(t[2]), int(t[3]), from_f64(t[4]), from_f64(t[5]))
            elif t[0] == 'sample':
                samples.append((dec(t[1]), from_f64(t[2]), from_f64(t[3]), from_f64(t[4])))
    if params is None:
        raise ValueError('no params line')
    clock = ScriptedClock()
    saved = reference.time
    reference.time = clock
    perf = time.perf_counter_ns
    try:
        def run_once(ns: Optional[List[int]]) -> None:
            ref = Reference(*params)
            for metric, value, mono, wall in samples:
                clock.mono, clock.wall = mono, wall
                t0 = perf() if ns is not None else 0
                ref._process_sample(metric, value)
                if ns is not None:
                    ns.append(perf() - t0)

        for _ in range(warmup):
            run_once(None)
        ns: List[int] = []
        for _ in range(repeat):
            run_once(ns)
        t0 = time.perf_counter()
        for _ in range(repeat):
            run_once(None)
        elapsed = time.perf_counter() - t0
    finally:
        reference.time = saved
    overhead = []
    for _ in range(100000):
        a = perf()
        overhead.append(perf() - a)
    return {'implementation': 'python', 'python': sys.version.split()[0],
            'samples_per_replay': len(samples), 'repeat': repeat, 'warmup': warmup,
            'throughput_samples_per_s': len(samples) * repeat / elapsed,
            'timer_overhead': _percentiles(overhead), 'process': _percentiles(ns)}


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
