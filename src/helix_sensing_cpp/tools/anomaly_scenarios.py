"""
Scenario generators for the anomaly replay protocol (see anomaly_replay.py).

Adversarial scenarios target where a port drifts from the reference: streak
and reset rules, NaN (stale) handling, flat and insufficient windows, exact
duration and cooldown boundaries, wall-clock steps, window rollover, values
that overflow the window sum, formatting of rounded context values, and the
float() parser. Seeded random scenarios mix all of these.

``bench_workload`` is a deterministic stream shaped like the GO2 adapter
output (several rate and state metrics at 10 Hz each, occasional spikes and
stale periods), used by bench_anomaly_core.py for a matched comparison.
"""
from __future__ import annotations

import math
import random
import struct
from typing import Iterator, List, Tuple

from anomaly_replay import enc, f64

NAN, INF = math.nan, math.inf


class Script:
    def __init__(self) -> None:
        self.lines: List[str] = []
        self.mono = 1000.0
        self.wall = 1.79e9

    def comment(self, text: str) -> 'Script':
        self.lines.append('# ' + text)
        return self

    def params(self, zscore=3.0, trigger=3, window=60, cooldown=1.0,
               min_duration=2.0) -> 'Script':
        self.lines.append(f'params {f64(zscore)} {trigger} {window} {f64(cooldown)} '
                          f'{f64(min_duration)}')
        return self

    def sample(self, metric: str, value: float, dt: float = 0.1, dwall=None) -> 'Script':
        """Advance both clocks (wall by ``dwall``, default ``dt``), then sample."""
        self.mono += dt
        self.wall += dt if dwall is None else dwall
        self.lines.append(f'sample {enc(metric)} {f64(value)} {f64(self.mono)} {f64(self.wall)}')
        return self

    def series(self, metric: str, values, dt: float = 0.1) -> 'Script':
        for v in values:
            self.sample(metric, v, dt)
        return self

    def text(self) -> str:
        return '\n'.join(self.lines) + '\n'


def _noise(n: int, base: float = 10.0, step: float = 0.1) -> List[float]:
    return [base + (i % 3) * step for i in range(n)]


def streaks_and_resets() -> Script:
    s = Script()
    for trigger in (1, 3, 0, -1):
        s.params(trigger=trigger, cooldown=0.0, min_duration=0.0)
        s.series('m', _noise(25)).series('m', [100.0, 100.0, 10.1, 100.0, 100.0, 100.0, 100.0])
    s.params(zscore=0.0, cooldown=0.0, min_duration=0.0).series('z0', _noise(10) + [10.1, 10.0])
    s.params(zscore=-1.0, cooldown=0.0, min_duration=0.0).series('zneg', _noise(6))
    s.params(zscore=NAN, cooldown=0.0, min_duration=0.0).series('znan', _noise(6) + [1e6] * 4)
    return s


def stale_nan() -> Script:
    s = Script().params(cooldown=0.0, min_duration=0.0)
    s.series('rate_hz/x', [NAN] * 5).series('rate_hz/x', _noise(10)).series('rate_hz/x', [NAN] * 3)
    s.series('rate_hz/x', [100.0] * 3 + [NAN, 100.0, -NAN])
    s.params(cooldown=0.0, min_duration=0.5)
    s.series('rate_hz/y', [NAN] * 3, dt=0.2).series('rate_hz/y', [NAN] * 3, dt=0.05)
    return s


def flat_and_insufficient() -> Script:
    s = Script()
    for window in (1, 2, 3, 5):
        s.params(window=window, cooldown=0.0, min_duration=0.0)
        s.series(f'w{window}', [7.0] * 6 + [1000.0] * 4 + [7.0, 7.0000001, 7.0000002])
    s.params(cooldown=0.0, min_duration=0.0).series('nearflat', [5.0, 5.0 + 1e-7, 5.0, 5.0 + 2e-6,
                                                                 5.0, 9.0, 9.0, 9.0])
    for bad in (0, -3):
        s.params(window=bad)
    return s


def duration_and_cooldown_boundaries() -> Script:
    s = Script().params(cooldown=1.0, min_duration=0.5)
    s.series('b', _noise(25))
    # Streak starts; mono advances in exact binary steps so elapsed == 0.5.
    s.sample('b', 100.0, dt=0.25).sample('b', 100.0, dt=0.125).sample('b', 100.0, dt=0.125)
    s.sample('b', 100.0, dt=0.0)                       # elapsed == min_duration: emits
    s.sample('b', 100.0, dt=0.5, dwall=1.0)            # wall delta == cooldown: emits again
    s.sample('b', 100.0, dt=0.1, dwall=0.999)            # inside the cooldown
    s.sample('b', 100.0, dt=0.1, dwall=-5.0)           # wall clock stepped back: suppressed
    s.sample('b', 100.0, dt=0.1, dwall=10.0)
    s.params(cooldown=-1.0, min_duration=-1.0).series('neg', _noise(25) + [100.0] * 5)
    s.params(cooldown=30.0, min_duration=0.0).series('long', _noise(500) + [100.0] * 10)
    return s


def extreme_values() -> Script:
    s = Script().params(cooldown=0.0, min_duration=0.0)
    s.series('inf', _noise(10) + [INF, INF, INF, 10.0, -INF, -INF, -INF])
    # Huge but with squared deviations that still fit a double; beyond about
    # 1.34e154 of deviation the reference raises OverflowError (see
    # anomaly_replay.py and test_documented_overflow_divergence).
    s.series('huge', [1e150, -1e150, 1.3e150, 10.0, 10.0, 10.0, -1.3e150, 10.0])
    s.series('tiny', [5e-324, 1e-320, 0.0, -0.0, 5e-324, 1e-300, 1.0, 1.0, 1.0])
    s.series('negzero', [-0.0, 0.0, -0.0, 0.0, -0.0, -1e-10, 1e-10] + [-5.0] * 3)
    s.series('big', [1e16, 1e16 + 2, 1e16 - 2, 1e16 + 4, 3e16, 3e16, 3e16])
    return s


def rounding_and_repr() -> Script:
    """Context values that exercise repr: exponents, ties, trailing zeros."""
    s = Script().params(zscore=1.0, trigger=1, cooldown=0.0, min_duration=0.0)
    for metric, base, spike in (('int', 100.0, 200.0), ('tie', 2.675, 2.68),
                                ('small', 1e-5, 3e-5), ('tiny', 1e-7, 5e-7),
                                ('neg', -123456.78905, -123000.0), ('e16', 1e16, 5e16),
                                ('half', 0.00005, 0.00015), ('mixed', 0.1, 0.7)):
        s.series(metric, [base, base * 1.0001 if base else 1e-9, base] + [spike])
    return s


def multi_metric_names() -> Script:
    s = Script().params(cooldown=0.5, min_duration=0.2)
    names = ['rate_hz/utlidar_cloud', 'nodeA/cpu_pct', 'spaced name', 'unicodé/μ', '', 'a' * 200]
    rng = random.Random(3)
    for k in range(400):
        name = names[k % len(names)]
        v = 10.0 + rng.gauss(0, 0.2) if rng.random() > 0.08 else rng.choice((100.0, NAN, -50.0))
        s.sample(name, v, dt=0.02)
    return s


PARSE_CASES = [
    '1', '-1', '+1', ' 1.5 ', '\t2\n', '\x0b3\x0c', '\x1c4\x1f', ' 5 ', '　6　',
    '1_000', '1__0', '_1', '1_', '1_0.5_5e1_0', '1._5', '1_.5', '1e_5', '0_0', '00', '-0', '+0.0',
    '1e5', '1E-5', '1e+5', '1e', 'e5', '.5', '5.', '.', '-.5e-3', '1.5.2', '1,5', '1 2', '++1',
    '+', '-', '', ' ', 'inf', '-Infinity', 'INF', 'iNfInItY', 'nan', '-nan', 'NaN', '+nan',
    'nan(1)', 'infinit', 'infinityx', '0x10', '0X1p3', '1e999', '-1e999', '1e-400', '4.9e-324',
    '2.4703282292062328e-324', '1.7976931348623157e308', '1.7976931348623159e308',
    '0.1', '123456789012345678901234567890', 'True', 'OK', '1\x00', '١', '1d', '1f', '١٢',
]
REPR_VALUES = [0.0, -0.0, 1.0, -1.0, 0.1, 100.0, 1e15, 1e16, 1.5e16, 9999999999999998.0,
               1e-4, 1e-5, 1.5e-5, 0.00012345, 123.456, 2.675, 1e300, 5e-324,
               2.2250738585072014e-308,
               INF, -INF, NAN, 1 / 3, 2 / 3, 12345678.9, 0.30000000000000004]


def formatting() -> Script:
    s = Script()
    rng = random.Random(11)
    values = list(REPR_VALUES)
    for _ in range(300):
        values.append(struct.unpack('>d', rng.getrandbits(64).to_bytes(8, 'big'))[0])
        values.append(round(rng.uniform(-1e4, 1e4), rng.randint(0, 8)))
        values.append(rng.choice((1.0, -1.0)) * 10 ** rng.uniform(-12, 20))
    for x in values:
        s.lines.append(f'repr {f64(x)}')
        for n in (0, 2, 4, 6):
            s.lines.append(f'round {f64(x)} {n}')
        s.lines.append(f'fixed {f64(x)} 2')
    return s


def parsing() -> Script:
    s = Script()
    rng = random.Random(5)
    cases = [c for c in PARSE_CASES if not any(ch in c for ch in '١١٢')]
    for _ in range(200):
        cases.append(repr(rng.uniform(-1e6, 1e6)))
        cases.append(f'{rng.uniform(-10, 10):.{rng.randint(0, 20)}e}')
    for c in cases:
        s.lines.append(f'parse {enc(c)}')
    return s


def squares() -> Script:
    """
    Square deviations exactly as the reference does.

    (s - mean) ** 2 calls libm pow() in CPython, which differs from x * x in
    the last bit for about 0.07% of inputs. The kernel must match pow().
    """
    s = Script()
    rng = random.Random(17)
    for _ in range(20000):
        x = rng.choice((rng.uniform(-1, 1), rng.gauss(0, 1e-3), rng.uniform(-1e6, 1e6),
                        rng.uniform(-1e150, 1e150),
                        rng.gauss(0, 1) * 10 ** rng.uniform(-150, 150)))
        s.lines.append(f'square {f64(x)}')
    for x in (0.0, -0.0, 1.0, 6.365728736090825e-05, 0.46618712722307665, 0.6875287757043953,
              1.6787426909940675e-09, INF, -INF, NAN, 5e-324):
        s.lines.append(f'square {f64(x)}')
    return s


def large_magnitude_std() -> Script:
    """
    Emit faults whose window_std is about 1e10.

    At that magnitude round(std, 6) keeps every bit, so a last-bit difference
    in the variance shows in the published context string.
    """
    s = Script().params(zscore=0.5, trigger=1, window=60, cooldown=0.0, min_duration=0.0)
    rng = random.Random(23)
    for _ in range(3000):
        s.sample('wide', rng.gauss(0.0, 1e10) * rng.choice((1.0, 3.0)), dt=0.01)
    return s


ADVERSARIAL = {
    'streaks_and_resets': streaks_and_resets,
    'stale_nan': stale_nan,
    'flat_and_insufficient': flat_and_insufficient,
    'duration_and_cooldown_boundaries': duration_and_cooldown_boundaries,
    'extreme_values': extreme_values,
    'rounding_and_repr': rounding_and_repr,
    'multi_metric_names': multi_metric_names,
    'formatting': formatting,
    'parsing': parsing,
    'squares': squares,
    'large_magnitude_std': large_magnitude_std,
}


def random_scenario(seed: int, samples: int = 600) -> Script:
    rng = random.Random(seed)
    s = Script().comment(f'seed {seed}')
    s.params(zscore=rng.choice((2.0, 3.0, 4.0, 1.5)), trigger=rng.choice((1, 2, 3, 5)),
             window=rng.choice((2, 3, 10, 60)), cooldown=rng.choice((0.0, 0.25, 1.0)),
             min_duration=rng.choice((0.0, 0.1, 0.3, 2.0)))
    metrics = [f'm{i}' for i in range(rng.randint(1, 4))]
    level = {m: rng.uniform(-50, 50) for m in metrics}
    for _ in range(samples):
        m = rng.choice(metrics)
        r = rng.random()
        if r < 0.05:
            v = NAN
        elif r < 0.12:
            v = level[m] + rng.choice((-1, 1)) * rng.uniform(5, 500)
        elif r < 0.13:
            v = rng.choice((INF, -INF, 1e150, 5e-324, -0.0))
        else:
            level[m] += rng.gauss(0, 0.05)
            v = level[m] + rng.gauss(0, 1.0)
        dt = rng.choice((0.0, 0.01, 0.05, 0.1, 0.25))
        dwall = dt if rng.random() < 0.97 else rng.choice((-2.0, 3.0))
        s.sample(m, v, dt=dt, dwall=dwall)
    return s


def all_random(count: int = 120, base_seed: int = 20261002) -> Iterator[Tuple[str, Script]]:
    for i in range(count):
        yield f'random_{i}', random_scenario(base_seed + i)


def bench_workload(seconds: float = 300.0, seed: int = 9) -> Script:
    """
    Build eight adapter-style metrics at 10 Hz each, deployed parameters.

    zscore 4.0, trigger 3, window 60, cooldown 1.0 s, min duration 2.0 s
    (config/helix_params.yaml). About 1% spikes, occasional stale periods.
    """
    rng = random.Random(seed)
    s = Script().comment(f'bench workload {seconds}s seed {seed}')
    s.params(zscore=4.0, trigger=3, window=60, cooldown=1.0, min_duration=2.0)
    metrics = ['rate_hz/utlidar_cloud', 'rate_hz/utlidar_imu', 'rate_hz/robot_odom',
               'rate_hz/lowstate', 'rate_hz/sportmodestate', 'state/body_height',
               'state/foot_force_sum', 'pose/drift_m']
    base = {m: rng.uniform(5, 50) for m in metrics}
    stale_until = {m: -1.0 for m in metrics}
    steps = int(seconds * 10)
    for k in range(steps):
        for m in metrics:
            t = k * 0.1
            if stale_until[m] < t and rng.random() < 0.0005:
                stale_until[m] = t + 3.0
            if t < stale_until[m]:
                v = NAN
            elif rng.random() < 0.01:
                v = base[m] * rng.uniform(1.5, 3.0)
            else:
                v = base[m] + rng.gauss(0, 0.05 * base[m])
            s.sample(m, v, dt=0.1 / len(metrics))
    return s
