"""
Differential parity: the native arbiter core against helix_arbiter.arbiter_core.

Every scenario (tools/arbiter_scenarios.py) is executed twice, by the C++ tool
helix_arbiter_replay and by the Python reference through the same protocol
(tools/arbiter_replay.py), and the two transcripts must be identical line for
line. Floats travel as IEEE-754 bit patterns, so "identical" is bit-exact,
including the sign of zero.

The replay binary comes from $HELIX_ARBITER_REPLAY (set by colcon test) or the
installed helix_arbiter_cpp package.
"""
import os
import subprocess
import sys
from collections import Counter

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir, 'tools'))

import arbiter_replay  # noqa: E402
import arbiter_scenarios  # noqa: E402


def _replay_binary() -> str:
    path = os.environ.get('HELIX_ARBITER_REPLAY')
    if path:
        return path
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix('helix_arbiter_cpp'), 'lib', 'helix_arbiter_cpp',
                        'helix_arbiter_replay')


SCENARIOS = dict(arbiter_scenarios.ADVERSARIAL)
SCENARIOS.update({name: (lambda s=s: s) for name, s in arbiter_scenarios.all_random()})


def _both(script_text: str):
    lines = [ln for ln in script_text.splitlines()
             if ln.split() and not ln.startswith('#')]
    proc = subprocess.run([_replay_binary()], input=script_text, capture_output=True,
                          text=True, timeout=60)
    assert proc.returncode == 0, proc.stderr
    native = proc.stdout.splitlines()
    reference = arbiter_replay.replay(lines)
    return lines, native, reference


@pytest.mark.parametrize('name', sorted(SCENARIOS))
def test_native_core_matches_python_reference(name):
    lines, native, reference = _both(SCENARIOS[name]().text())
    assert len(native) == len(reference) == len(lines)
    for i, (cmd, n, r) in enumerate(zip(lines, native, reference)):
        assert n == r, (f'{name}: first divergence at command {i}\n  input:  {cmd}\n'
                        f'  native: {n}\n  python: {r}')


def test_adversarial_scenarios_exercise_every_outcome():
    """Guard against a vacuous pass: the fixtures must reach every branch."""
    seen = Counter()
    for name, build in arbiter_scenarios.ADVERSARIAL.items():
        for out in arbiter_replay.replay(build().text().splitlines()):
            t = out.split()
            seen[' '.join(t[:2]) if t[0] in ('decide', 'build', 'config', 'layout', 'hold')
                 else ' '.join(t[:3])] += 1
    for key in ('decide SOURCE', 'decide HELIX_HOLD', 'decide HELIX_STATE_STALE',
                'decide HELIX_STATE_MISSING', 'decide NO_LIVE_INPUT', 'hold 1', 'hold 0',
                'src 1', 'src 0 nonfinite', 'src 0 linear', 'src 0 angular', 'build ok',
                'build error', 'config ok', 'config error', 'layout ok', 'layout error'):
        assert seen[key] > 0, f'no fixture reaches {key!r}: {sorted(seen)}'


def test_full_width_hold_ordering_is_not_rounded():
    """2**53 + 1 vs 2**53 must order as integers on both sides."""
    s = arbiter_scenarios.Script().arbiter([arbiter_scenarios.NAV])
    e = 2 ** 53
    s.hold(False, e, e, 0.0).hold(False, e, e + 1, 0.0).hold(False, e, e + 1, 0.0)
    s.hold(False, 2 ** 64 - 1, 2 ** 64 - 1, 0.0).hold(False, 2 ** 64 - 1, 2 ** 64 - 2, 0.0)
    _, native, reference = _both(s.text())
    assert native == reference
    assert native[-5:] == ['hold 1', 'hold 1', 'hold 0', 'hold 1', 'hold 0']
