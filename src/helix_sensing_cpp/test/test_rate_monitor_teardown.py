"""Exercise the installed component executable under traffic and lifecycle teardown."""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from std_msgs.msg import Float64MultiArray, String


@pytest.fixture
def driver():
    rclpy.init()
    node = rclpy.create_node('rate_teardown_driver')
    pub = node.create_publisher(String, '/rate_teardown/input', 10)
    node.create_timer(0.002, lambda: pub.publish(String(data='traffic')))
    yield node
    node.destroy_node()
    rclpy.shutdown()


def wait(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        if predicate():
            return True
    return False


class Monitor:
    def __init__(self, driver, tmp_path, iteration, types=None):
        self.driver = driver
        self.path = f'/rate_teardown/run_{iteration}/helix_topic_rate_monitor'
        self.log_path = tmp_path / f'node_{iteration}.log'
        self.log = self.log_path.open('w')
        types = types or ['std_msgs/msg/String']
        topics = ['/rate_teardown/input'] * len(types)
        self.proc = subprocess.Popen([
            os.environ['RATE_MONITOR_EXECUTABLE'], '--ros-args',
            '-r', f'__ns:=/rate_teardown/run_{iteration}',
            '-p', f'topics:={topics}', '-p', f'topic_types:={types}',
            '-p', 'publish_period_sec:=0.005',
        ], stdout=self.log, stderr=subprocess.STDOUT)
        self.change = driver.create_client(ChangeState, self.path + '/change_state')
        self.state = driver.create_client(GetState, self.path + '/get_state')
        assert wait(driver, self.state.service_is_ready), self.output()
        future = self.state.call_async(GetState.Request())
        assert wait(driver, future.done), self.output()

    def output(self):
        return self.log_path.read_text()

    def transition(self, transition_id, success=True):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change.call_async(req)
        assert wait(self.driver, future.done), self.output()
        assert future.result().success is success, self.output()

    def stop(self, sig=signal.SIGINT):
        self.proc.send_signal(sig)
        assert self.proc.wait(timeout=10) == 0, self.output()

    def close(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(timeout=5)
        self.driver.destroy_client(self.change)
        self.driver.destroy_client(self.state)
        self.log.close()


def test_repeated_cleanup_releases_subscriptions(driver, tmp_path):
    monitor = Monitor(driver, tmp_path, 'cleanup')
    try:
        for _ in range(12):
            monitor.transition(1)  # configure
            assert wait(driver, lambda: driver.count_subscribers('/rate_teardown/input') == 1)
            monitor.transition(3)  # activate
            monitor.transition(4)  # deactivate
            monitor.transition(2)  # cleanup
            assert wait(driver, lambda: driver.count_subscribers('/rate_teardown/input') == 0)
        monitor.stop()
    finally:
        monitor.close()


def test_failed_configuration_rolls_back_subscriptions(driver, tmp_path):
    monitor = Monitor(driver, tmp_path, 'failure', [
        'std_msgs/msg/String', 'missing_package/msg/Absent'])
    try:
        for _ in range(3):
            monitor.transition(1, success=False)
            assert wait(driver, lambda: driver.count_subscribers('/rate_teardown/input') == 0)
        monitor.stop()
    finally:
        monitor.close()


@pytest.mark.parametrize('iteration', range(12))
def test_repeated_start_shutdown_under_traffic(driver, tmp_path, iteration):
    monitor = Monitor(driver, tmp_path, iteration)
    metrics, beats = [], []
    metric_sub = driver.create_subscription(
        Float64MultiArray, '/helix/metrics', metrics.append, 10)
    beat_sub = driver.create_subscription(String, '/helix/heartbeat', beats.append, 10)
    try:
        monitor.transition(1)
        monitor.transition(3)
        assert wait(driver, lambda: len(metrics) >= 10 and len(beats) >= 1)
        if iteration % 3 == 0:
            # Shutdown directly from ACTIVE must stop both timers and release inputs.
            monitor.transition(7)
            assert wait(driver, lambda: driver.count_subscribers('/rate_teardown/input') == 0)
            wait(driver, lambda: False, 0.15)  # drain already transmitted data
            before = len(metrics), len(beats)
            wait(driver, lambda: False, 0.25)
            assert (len(metrics), len(beats)) == before
        elif iteration % 3 == 1:
            monitor.transition(4)
            monitor.transition(2)
            monitor.transition(5)  # shutdown from UNCONFIGURED
        monitor.stop(signal.SIGINT if iteration % 2 == 0 else signal.SIGTERM)
    finally:
        monitor.close()
        driver.destroy_subscription(metric_sub)
        driver.destroy_subscription(beat_sub)
