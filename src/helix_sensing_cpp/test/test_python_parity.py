"""
Differential parity: native AnomalyCore and pyfmt against helix_core.

Every scenario (tools/anomaly_scenarios.py) runs through the C++ tool
helix_anomaly_replay and through the Python reference node's own methods
(tools/anomaly_replay.py); the transcripts must match line for line. A
"fault" line carries every FaultEvent field the reference sets, including
the rounded context strings and the detail text, so this checks the
published contract byte for byte, not only the decision.

The replay binary comes from $HELIX_ANOMALY_REPLAY (set by colcon test) or
the installed helix_sensing_cpp package.
"""
import os
import subprocess
import sys
from collections import Counter

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir, 'tools'))

import anomaly_replay  # noqa: E402
import anomaly_scenarios  # noqa: E402


def _replay_binary() -> str:
    path = os.environ.get('HELIX_ANOMALY_REPLAY')
    if path:
        return path
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('helix_sensing_cpp'), 'lib', 'helix_sensing_cpp',
                        'helix_anomaly_replay')


def _native(text: str):
    proc = subprocess.run([_replay_binary()], input=text.encode('utf-8'), capture_output=True,
                          timeout=120)
    assert proc.returncode == 0, proc.stderr.decode(errors='replace')
    return proc.stdout.decode('utf-8').splitlines()


SCENARIOS = dict(anomaly_scenarios.ADVERSARIAL)
SCENARIOS.update({name: (lambda s=s: s) for name, s in anomaly_scenarios.all_random()})


@pytest.mark.parametrize('name', sorted(SCENARIOS))
def test_native_matches_python_reference(name):
    text = SCENARIOS[name]().text()
    lines = [ln for ln in text.splitlines() if ln.split() and not ln.startswith('#')]
    native = _native(text)
    reference = anomaly_replay.replay(lines)
    assert len(native) == len(reference) == len(lines)
    for i, (cmd, n, r) in enumerate(zip(lines, native, reference)):
        assert n == r, (f'{name}: first divergence at command {i}\n  input:  {cmd}\n'
                        f'  native: {n}\n  python: {r}')


def test_fixtures_reach_every_outcome():
    """Guard against a vacuous pass."""
    seen = Counter()
    for build in anomaly_scenarios.ADVERSARIAL.values():
        for out in anomaly_replay.replay(build().text().splitlines()):
            t = out.split()
            if t[0] == 'fault':
                seen['fault stale' if 's:stale' in t else 'fault zscore'] += 1
            elif t[0] in ('params', 'parse'):
                seen[t[0] + (' none' if t[1] == 'none' else ' ok' if t[1] != 'error'
                             else ' error')] += 1
            else:
                seen[t[0]] += 1
    for key in ('fault stale', 'fault zscore', 'ok', 'params ok', 'params error', 'parse ok',
                'parse none', 'repr', 'round', 'fixed', 'square'):
        assert seen[key] > 0, f'no fixture reaches {key!r}: {sorted(seen)}'


def test_documented_unicode_digit_limit():
    """
    float() accepts non-ASCII decimal digits; the native parser does not.

    Documented in pyfmt.hpp: such a /diagnostics value is skipped instead of
    processed. This pins the difference so it cannot widen silently.
    """
    s = anomaly_scenarios.Script()
    for text in ('١٢', '٣.5'):
        s.lines.append(f'parse {anomaly_replay.enc(text)}')
    native = _native(s.text())
    reference = anomaly_replay.replay(s.lines)
    assert native == ['parse none', 'parse none']
    assert all(r != 'parse none' for r in reference)


def test_documented_overflow_divergence():
    """
    The reference raises OverflowError on an overflowing squared deviation.

    In the Python node that exception escapes the /helix/metrics callback and
    ends rclpy.spin, so the detector process exits. The native core keeps
    running: the huge sample itself is a violation against the normal window,
    and while it stays in the window the std is infinite, so z = 0 and the
    streak resets. Pinned here so the behaviour cannot change unnoticed.
    """
    s = anomaly_scenarios.Script().params(cooldown=0.0, min_duration=0.0)
    s.series('m', [10.0, 10.1, 10.2, 10.0, 1e200, 10.0, 10.1])
    lines = [ln for ln in s.text().splitlines() if ln.split() and not ln.startswith('#')]
    native = _native(s.text())
    reference = anomaly_replay.replay(lines)
    assert native[:6] == reference[:6]          # params + 4 normal + the 1e200 sample
    assert native[5] == 'ok 1'                  # 1e200 is a violation (streak 1)
    assert reference[6] == 'error OverflowError'
    assert native[6:] == ['ok 0', 'ok 0']       # native: infinite std, z = 0, reset
