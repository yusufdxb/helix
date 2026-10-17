"""
Native LogMatcher vs helix_core.log_parser.LogParser, record by record.

The Python reference methods (_load_rules, _on_log, _emit_log_fault) run
unbound on a small shim with a fake clock, so no rclpy node is needed. Both
sides read the shipped log_rules.yaml and see the same /rosout stream.
"""
import os
import random
import subprocess
import threading
import types

import pytest

REPLAY = os.environ.get('HELIX_LOG_REPLAY', '')
RULES = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'helix_bringup', 'config', 'log_rules.yaml'))

MSGS = [
    'SLAM diverged: covariance exceeded maximum threshold',
    'slam has DIVERGED again',
    'Costmap layer failed to initialize',
    'costmap: plugin FAILED TO INITIALIZE',
    'participant lost: 0x01',
    'DDS discovery timeout after 5 s',
    'TF extrapolation into the future',
    'Lookup would require extrapolation; transform timeout',
    'emergency stop pressed',
    'E-STOP engaged',
    'robot estop latched',
    'all good here',
    'covariance within limits',
    'Ünïcode ' * 30 + 'SLAM diverged',
    'x' * 200 + ' estop',
]
NAMES = ['slam', 'nav2_costmap', 'go2_bridge', 'helix_x']


class Shim:
    """Just enough of LogParser for its rule and callback methods."""

    def __init__(self, dedup):
        from helix_core.log_parser import LogParser
        self.LogParser = LogParser
        self._dedup_window = dedup
        self._dedup_cache = {}
        self._dedup_lock = threading.Lock()
        self.out = []
        self._fault_pub = types.SimpleNamespace(publish=self.out.append)

    def get_logger(self):
        return types.SimpleNamespace(warn=lambda *a: None, debug=lambda *a: None,
                                     info=lambda *a: None)


def python_faults(records, dedup, monkeypatch):
    from helix_core import log_parser as lp
    from rcl_interfaces.msg import Log
    shim = Shim(dedup)
    shim._rules = lp.LogParser._load_rules(shim, RULES)
    clock = {'t': 0.0}
    monkeypatch.setattr(lp.time, 'time', lambda: clock['t'])
    shim._emit_log_fault = types.MethodType(lp.LogParser._emit_log_fault, shim)
    lines = []
    for t, level, name, file, msg in records:
        clock['t'] = t
        log = Log(level=level, name=name, file=file, msg=msg)
        before = len(shim.out)
        lp.LogParser._on_log(shim, log)
        for f in shim.out[before:]:
            lines.append('\t'.join([repr(f.timestamp), f.node_name, f.fault_type,
                                    str(f.severity), f.detail, *f.context_values]))
    return lines


def stream(seed, n=400):
    rng = random.Random(seed)
    t, out = 1_700_000_000.0, []
    for _ in range(n):
        t += rng.choice([0.01, 0.5, 1.0, 2.0, 4.9, 5.1])
        out.append((round(t, 3), rng.choice([10, 20, 30, 40, 40, 50]), rng.choice(NAMES),
                    rng.choice(['a.cpp', 'b.py']), rng.choice(MSGS)))
    return out


@pytest.mark.skipif(not REPLAY, reason='HELIX_LOG_REPLAY not set')
@pytest.mark.parametrize('seed', range(8))
@pytest.mark.parametrize('dedup', [0.0, 5.0])
def test_native_matches_reference(seed, dedup, monkeypatch):
    records = stream(seed)
    text = ''.join(f'{t!r}\t{lv}\t{nm}\t{fl}\t{msg}\n' for t, lv, nm, fl, msg in records)
    cpp = subprocess.run([REPLAY, RULES, str(dedup)], input=text, capture_output=True,
                         text=True, check=True).stdout.splitlines()
    py = python_faults(records, dedup, monkeypatch)
    assert py, 'stream produced no faults'
    # timestamps are compared numerically, the text spelling differs
    norm = [line.split('\t', 1)[1] for line in cpp]
    assert norm == [line.split('\t', 1)[1] for line in py]
    assert [float(line.split('\t', 1)[0]) for line in cpp] == pytest.approx(
        [float(line.split('\t', 1)[0]) for line in py], abs=1e-3)
