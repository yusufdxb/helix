"""Backend selection for the nodes that have a C++ port.

Three layers:
  * the pure resolvers in helix_bringup.backends (no ROS needed);
  * the launch files' OpaqueFunctions, evaluated in a LaunchContext, so the
    package each choice launches is checked without starting processes;
  * real ``ros2 launch`` runs on an isolated domain that wait for the
    anomaly detector to reach ``active`` and read which binary is running.

NO HARDWARE. The anomaly detector only subscribes and publishes fault events;
no motion node is started by these tests.
"""
from __future__ import annotations

import importlib.util
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest
from helix_bringup.backends import (
    executable,
    parse_bool,
    resolve_anomaly_backend,
    resolve_arbiter_backend,
    resolve_backend,
)

LAUNCH_DIR = Path(__file__).resolve().parent.parent / 'launch'


# --- pure resolvers ---------------------------------------------------------

@pytest.mark.parametrize('backend, legacy, expected', [
    ('', 'false', 'python'),        # all defaults
    ('', 'true', 'cpp'),            # legacy flag alone
    ('cpp', 'false', 'cpp'),
    ('python', 'false', 'python'),
    ('cpp', 'true', 'cpp'),         # both agree
    (' CPP ', 'False', 'cpp'),      # case and whitespace tolerated
])
def test_anomaly_backend_resolution(backend, legacy, expected):
    assert resolve_anomaly_backend(backend, legacy) == expected


def test_anomaly_backend_conflict_is_refused():
    with pytest.raises(ValueError, match='conflicts'):
        resolve_anomaly_backend('python', 'true')


@pytest.mark.parametrize('backend, legacy', [('rust', 'false'), ('', 'maybe')])
def test_anomaly_backend_invalid_values_are_refused(backend, legacy):
    with pytest.raises(ValueError):
        resolve_anomaly_backend(backend, legacy)


@pytest.mark.parametrize('backend, twist_mux, expected', [
    ('python', 'false', 'python'),
    ('cpp', 'false', 'cpp'),
    ('python', 'true', None),       # twist_mux replaces the arbiter
])
def test_arbiter_backend_resolution(backend, twist_mux, expected):
    assert resolve_arbiter_backend(backend, twist_mux) == expected


def test_cpp_arbiter_with_twist_mux_is_refused():
    with pytest.raises(ValueError, match='twist_mux'):
        resolve_arbiter_backend('cpp', 'true')


def test_backend_names_and_bools_are_validated():
    with pytest.raises(ValueError):
        resolve_backend('arbiter_backend', '')
    with pytest.raises(ValueError):
        parse_bool('enable_twist_mux', '2')


def test_both_backends_keep_the_node_executable_name():
    for component in ('anomaly_detector', 'arbiter'):
        (_, py_exe), (_, cpp_exe) = (executable(component, 'python'),
                                     executable(component, 'cpp'))
        assert py_exe == cpp_exe


# --- launch-file OpaqueFunctions --------------------------------------------

launch = pytest.importorskip('launch', reason='launch not available (ROS not sourced)')
pytest.importorskip('launch_ros', reason='launch_ros not available')
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

from launch import LaunchContext  # noqa: E402


def _load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(context, value):
    # launch_ros keeps a literal package/executable as a plain string and a
    # substitution as a list of Substitutions.
    return value if isinstance(value, str) else perform_substitutions(context, value)


def _nodes(actions, context):
    found = []
    for action in actions:
        if isinstance(action, Node):
            found.append((_text(context, action.node_package),
                          _text(context, action.node_executable)))
    return found


def _context(**configs):
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


@pytest.mark.parametrize('configs, package', [
    ({'anomaly_backend': '', 'use_cpp_anomaly': 'false'}, 'helix_core'),
    ({'anomaly_backend': 'cpp', 'use_cpp_anomaly': 'false'}, 'helix_sensing_cpp'),
    ({'anomaly_backend': '', 'use_cpp_anomaly': 'true'}, 'helix_sensing_cpp'),
])
def test_sensing_launch_picks_one_anomaly_detector(configs, package):
    sensing = _load('helix_sensing.launch.py')
    context = _context(auto_activate='true', **configs)
    nodes = _nodes(sensing.anomaly_detector_actions(context), context)
    assert nodes == [(package, 'helix_anomaly_detector')]


@pytest.mark.parametrize('configs, expected', [
    ({'arbiter_backend': 'python', 'enable_twist_mux': 'false'},
     [('helix_arbiter', 'helix_arbiter')]),
    ({'arbiter_backend': 'cpp', 'enable_twist_mux': 'false'},
     [('helix_arbiter_cpp', 'helix_arbiter')]),
    ({'arbiter_backend': 'python', 'enable_twist_mux': 'true'}, []),
])
def test_closedloop_launch_picks_at_most_one_arbiter(configs, expected):
    closedloop = _load('helix_closedloop.launch.py')
    context = _context(arbiter_config='/dev/null', cmd_vel_out='/cmd_vel', **configs)
    assert _nodes(closedloop.arbiter_actions(context), context) == expected


def test_auto_activate_registers_the_handler_before_configuring():
    # The activate handler must exist before configure is emitted, or a fast
    # configure transition is missed and the node stays inactive.
    from launch.actions import EmitEvent, RegisterEventHandler
    from launch_ros.actions import LifecycleNode
    for name in ('helix_sensing.launch.py', 'helix_closedloop.launch.py'):
        module = _load(name)
        node = LifecycleNode(package='helix_core', executable='helix_heartbeat_monitor',
                             name='probe', namespace='')
        actions = module._auto_activate(node, None)
        kinds = [type(a) for a in actions]
        assert kinds == [RegisterEventHandler, EmitEvent], name


# --- real launches ----------------------------------------------------------

def _ros2_available():
    return shutil.which('ros2') is not None


def _package_installed(package):
    try:
        out = subprocess.run(['ros2', 'pkg', 'prefix', package], capture_output=True,
                             text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return out.returncode == 0


def _descendants(pid):
    kids, frontier = [], [pid]
    while frontier:
        parent = frontier.pop()
        path = Path(f'/proc/{parent}/task/{parent}/children')
        try:
            children = [int(c) for c in path.read_text().split()]
        except OSError:
            continue
        kids.extend(children)
        frontier.extend(children)
    return kids


def _cmdline(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
    except OSError:
        return ''


def _lifecycle_state(node, env):
    out = subprocess.run(['ros2', 'lifecycle', 'get', node], capture_output=True,
                         text=True, timeout=20, env=env)
    return out.stdout.strip()


@pytest.fixture
def ros_env():
    if not _ros2_available():
        pytest.skip('ros2 CLI not on PATH')
    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'
    return env


def _launch_sensing(env, *args):
    return subprocess.Popen(
        ['ros2', 'launch', 'helix_bringup', 'helix_sensing.launch.py', *args],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
        start_new_session=True)


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def _stop(proc):
    """Stop the launch and every node it started (they share its process group)."""
    pgid = proc.pid
    if proc.poll() is None:
        os.killpg(pgid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=5)
    # ros2 launch can exit before a child that ignored SIGINT; reap stragglers.
    deadline = time.monotonic() + 5.0
    while _group_alive(pgid) and time.monotonic() < deadline:
        os.killpg(pgid, signal.SIGTERM)
        time.sleep(0.2)
    if _group_alive(pgid):
        os.killpg(pgid, signal.SIGKILL)


@pytest.mark.parametrize('args, package', [
    ((), 'helix_core'),
    (('anomaly_backend:=cpp',), 'helix_sensing_cpp'),
])
def test_launched_anomaly_detector_reaches_active(ros_env, args, package):
    if not _package_installed(package):
        pytest.skip(f'{package} not installed')
    proc = _launch_sensing(ros_env, *args)
    try:
        deadline, state = time.monotonic() + 45.0, ''
        while time.monotonic() < deadline and proc.poll() is None:
            state = _lifecycle_state('/helix_anomaly_detector', ros_env)
            if state.startswith('active'):
                break
            time.sleep(1.0)
        assert state.startswith('active'), f'state={state!r}'
        detectors = [_cmdline(p) for p in _descendants(proc.pid)
                     if 'helix_anomaly_detector' in _cmdline(p)]
        assert len(detectors) == 1, detectors
        assert f'/{package}/' in detectors[0], detectors[0]
    finally:
        _stop(proc)


def test_conflicting_anomaly_arguments_stop_the_launch(ros_env):
    proc = _launch_sensing(ros_env, 'anomaly_backend:=python', 'use_cpp_anomaly:=true')
    try:
        out, _ = proc.communicate(timeout=30)
    finally:
        _stop(proc)
    assert 'conflicts' in out
    assert 'helix_anomaly_detector' not in ''.join(
        line for line in out.splitlines() if 'process started' in line)
