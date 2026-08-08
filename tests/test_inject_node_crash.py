"""Regression tests for crash-injector process selection.

A naive `pgrep -af <node>` matches the injector's own command line and the
pytest process that spawned it. Killing one of those leaves the target node
running, and every downstream assertion then blames the detector for a
missing CRASH fault. These tests pin the selection rules instead.
"""
import os
import subprocess
from types import SimpleNamespace

import pytest

from scripts.sim_faults.inject_node_crash import find_node_pids

NODE = 'helix_context_buffer'
REAL = f'/usr/bin/python3 /home/x/ws/install/helix_diagnosis/lib/helix_diagnosis/{NODE} --ros-args'


def fake_ps(monkeypatch, lines):
    def run(cmd, capture_output=False, text=False):
        return SimpleNamespace(stdout='\n'.join(lines) + '\n', returncode=0)
    monkeypatch.setattr(subprocess, 'run', run)


def test_finds_the_installed_node_executable(monkeypatch):
    fake_ps(monkeypatch, [f'  4242 {REAL}'])
    assert find_node_pids(NODE) == [4242]


def test_ignores_the_injector_itself(monkeypatch):
    fake_ps(monkeypatch, [
        f'  1111 python3 scripts/sim_faults/inject_node_crash.py --node {NODE} --after 15',
        f'  4242 {REAL}',
    ])
    assert find_node_pids(NODE) == [4242]


def test_ignores_a_shell_that_merely_mentions_the_node(monkeypatch):
    fake_ps(monkeypatch, [
        f'  2222 /bin/bash -c ros2 lifecycle set /{NODE} activate',
        f'  4242 {REAL}',
    ])
    assert find_node_pids(NODE) == [4242]


def test_ignores_own_and_parent_process(monkeypatch):
    fake_ps(monkeypatch, [
        f'  {os.getpid()} {REAL}',
        f'  {os.getppid()} {REAL}',
    ])
    assert find_node_pids(NODE) == []


def test_does_not_match_a_different_node_with_a_shared_prefix(monkeypatch):
    other = '/usr/bin/python3 /home/x/ws/install/helix_diagnosis/lib/helix_diagnosis/helix_context_buffer_v2'
    fake_ps(monkeypatch, [f'  4243 {other} --ros-args'])
    assert find_node_pids(NODE) == []


def test_returns_every_match_so_the_caller_can_refuse_to_guess(monkeypatch):
    fake_ps(monkeypatch, [f'  4242 {REAL}', f'  4244 {REAL}'])
    assert find_node_pids(NODE) == [4242, 4244]


def test_no_match_is_empty_not_an_error(monkeypatch):
    fake_ps(monkeypatch, ['  9999 /usr/bin/python3 -m unrelated.thing'])
    assert find_node_pids(NODE) == []


@pytest.mark.parametrize('args', [
    f'/home/x/ws/install/helix_diagnosis/lib/helix_diagnosis/{NODE}',
    f'python3 /opt/ros/install/pkg/lib/pkg/{NODE} --ros-args -r __ns:=/',
])
def test_matches_both_direct_exec_and_interpreter_forms(monkeypatch, args):
    fake_ps(monkeypatch, [f'  4242 {args}'])
    assert find_node_pids(NODE) == [4242]


def test_ps_is_invoked_without_width_truncation():
    """`ps -eo args=` truncates to 80 columns off a tty.

    An installed node path is longer than that, so the executable name is cut
    off and the node looks absent while it is running. -ww disables it.
    """
    from scripts.sim_faults.inject_node_crash import PS_COMMAND
    assert '-eww' in PS_COMMAND or '-ww' in PS_COMMAND


def test_finds_a_node_behind_a_long_install_path(monkeypatch):
    long_path = (
        '/usr/bin/python3 /home/someone/very/deeply/nested/workspace/'
        f'install/helix_diagnosis/lib/helix_diagnosis/{NODE} --ros-args -r __ns:=/'
    )
    assert len(long_path) > 80
    fake_ps(monkeypatch, [f'  4242 {long_path}'])
    assert find_node_pids(NODE) == [4242]
