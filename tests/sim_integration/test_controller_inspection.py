"""Finite query-only inspector and dry native validation on localhost.

These tests use synthetic metadata, LowState and FakeGo2. No firmware or physical
stop behavior is tested. Vendor messages and built native executables required.
"""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('unitree_api')
pytest.importorskip('unitree_go')

from scripts.hw.controller_inspect import NATIVE, QUERIES  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def rig(tmp_path, monkeypatch):
    import rclpy
    from rclpy.node import Node
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '207')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init(args=[])
    node = Node('inspection_test_fixture')
    children = []

    def start(argv, label):
        with (tmp_path / (label+'.log')).open('w') as log:
            child = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        children.append(child)
        return child

    def wait(child, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end and child.poll() is None:
            rclpy.spin_once(node, timeout_sec=.01)
        assert child.poll() is not None, 'inspector did not terminate'
        assert child.returncode == 0, (tmp_path / 'inspect.log').read_text()

    yield node, start, wait
    for child in reversed(children):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
    for child in children:
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    node.destroy_node()
    rclpy.shutdown()


def test_only_allowlisted_queries_are_sent_and_no_firmware_is_invented(rig, tmp_path):
    from unitree_api.msg import Request, Response
    node, start, wait = rig
    observed = []
    replies = {service: node.create_publisher(Response, f'/api/{service}/response', 10)
               for _, service, _ in QUERIES}

    def request(service, m):
        observed.append((service, m.header.identity.api_id, m.parameter))
        r = Response()
        r.header.identity = m.header.identity
        r.header.status.code = 0
        r.data = ('1.0.0.1' if m.header.identity.api_id == 1 else
                  '[{"name":"sport_mode","status":1,"protect":1}]' if service == 'robot_state'
                  else '{"name":"synthetic_mode","form":"0"}')
        replies[service].publish(r)
    for service in replies:
        node.create_subscription(Request, f'/api/{service}/request',
                                 lambda m, service=service: request(service, m), 100)
    output = tmp_path / 'query.json'
    child = start([sys.executable, str(REPO/'scripts/hw/controller_inspect.py'), '--out',
                   str(output), '--seconds', '10', '--warmup', '0', '--query-metadata',
                   '--evidence-class', 'Sim'], 'inspect')
    wait(child, 20)
    assert sorted(observed) == sorted((service, api, '{}') for _, service, api in QUERIES)
    result = json.loads(output.read_text())
    assert len(result['requests_sent']) == 5
    assert all('metadata' in q for q in result['queries'].values())
    assert result['firmware_build'] is None and result['controller_motion_timeout_s'] is None
    assert not result['motion_timeout_measured'] and not result['native_validation_complete']


def test_default_native_profile_is_nonactuating_and_resources_are_measured(rig, tmp_path):
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import qos_profile_sensor_data
    from unitree_api.msg import Request
    from unitree_go.msg import LowState
    node, start, wait = rig
    try:
        fake = Path(get_package_prefix('helix_arbiter'))/'lib/helix_arbiter/helix_fake_go2'
        sink = Path(get_package_prefix('helix_go2_sink_cpp'))/'lib/helix_go2_sink_cpp/helix_go2_sport_sink'
    except LookupError:
        pytest.skip('requires built native packages')
    if not sink.exists():
        pytest.skip('requires native vendor-message build')
    requests = []
    node.create_subscription(Request, '/api/sport/request', lambda m: requests.append(m), 100)
    low = node.create_publisher(LowState, '/lowstate', qos_profile_sensor_data)
    node.create_timer(.002, lambda: low.publish(LowState()))
    start([str(fake), '--ros-args', '-p', 'move_timeout_s:=0.0'], 'fake')
    start(['ros2', 'launch', 'helix_bringup', 'helix_native.launch.py'], 'native')
    output = tmp_path/'native.json'
    child = start([sys.executable, str(REPO/'scripts/hw/controller_inspect.py'), '--out',
                   str(output), '--seconds', '20', '--warmup', '5', '--evidence-class', 'Sim'], 'inspect')
    wait(child, 35)
    result = json.loads(output.read_text())
    assert not requests and not result['requests_sent']
    assert not result['nonactuating_profile_violations']
    assert result['native_validation_complete'], result
    assert set(result['resources']) == set(NATIVE)
    assert all(r['samples'] > 5 and r['rss_max_kb'] > 0 for r in result['resources'].values())
