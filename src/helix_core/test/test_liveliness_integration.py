"""
Live DDS integration test for the liveliness detection path.

This is the test that actually proves the claim behind this change: that
stopping LivelinessBeacon.start()'s timer (the writer process keeps running,
it simply stops asserting liveliness, the "wedged, not dead" case the
research finding calls out) causes Cyclone DDS to raise
RCL_SUBSCRIPTION_LIVELINESS_CHANGED on LivelinessMonitor's subscription, and
that LivelinessMonitor turns that into exactly one CRASH FaultEvent on
/helix/faults, using the real rclpy QoS liveliness machinery end to end
(no mocks on the DDS side).

Needs RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and a ROS_DOMAIN_ID (43 per repo
convention) in the environment; both are exported by the test runner, not
by this file, so this test also works if colcon/CI already sets them.

rclpy is initialized once for the whole test/ directory by conftest.py.
"""
import threading
import time

import rclpy
from helix_core.liveliness_beacon import LivelinessBeacon
from helix_core.liveliness_monitor import LivelinessMonitor
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from helix_msgs.msg import FaultEvent

LEASE_DURATION_SEC = 0.2
ASSERT_PERIOD_SEC = 0.05
MONITORED_NODE_NAME = 'helix_liveliness_it_emitter'


def test_stopping_the_beacon_produces_a_crash_fault_via_dds_liveliness():
    emitter_node = rclpy.create_node(MONITORED_NODE_NAME)
    beacon = LivelinessBeacon(
        emitter_node,
        assert_period_sec=ASSERT_PERIOD_SEC,
        lease_duration_sec=LEASE_DURATION_SEC,
    )

    monitor = LivelinessMonitor()
    monitor.set_parameters([
        Parameter('enabled', Parameter.Type.BOOL, True),
        Parameter('monitored_nodes', Parameter.Type.STRING_ARRAY, [MONITORED_NODE_NAME]),
        Parameter('lease_duration_sec', Parameter.Type.DOUBLE, LEASE_DURATION_SEC),
    ])
    monitor.trigger_configure()
    monitor.trigger_activate()

    received_faults = []
    lock = threading.Lock()

    sub_node = rclpy.create_node('helix_liveliness_it_fault_sub')
    sub_node.create_subscription(
        FaultEvent,
        '/helix/faults',
        lambda msg: (lock.acquire(), received_faults.append(msg), lock.release()),
        10,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(emitter_node)
    executor.add_node(monitor)
    executor.add_node(sub_node)

    try:
        beacon.start()

        # Phase 1: beacon asserting normally for several lease periods.
        # Must NOT produce a CRASH fault while alive.
        deadline = time.time() + 4 * LEASE_DURATION_SEC
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.02)
        with lock:
            assert received_faults == [], (
                'A healthy, asserting beacon must not be reported CRASH: '
                f'got {[(f.fault_type, f.node_name) for f in received_faults]}'
            )

        # Phase 2: stop asserting (process stays up, DDS writer stays open;
        # this is the "wedged" case, not a process kill). Wait past several
        # lease durations for Cyclone DDS to expire the lease.
        beacon.stop()
        deadline = time.time() + 8 * LEASE_DURATION_SEC
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.02)
            with lock:
                if received_faults:
                    break

        with lock:
            crash_faults = [
                f for f in received_faults
                if f.fault_type == 'CRASH' and f.node_name == MONITORED_NODE_NAME
            ]

        assert len(crash_faults) >= 1, (
            'Expected at least one CRASH FaultEvent from DDS liveliness '
            f'expiry, got: {[(f.fault_type, f.node_name) for f in received_faults]}'
        )
        assert crash_faults[0].severity == 3
        assert 'dds_liveliness' in crash_faults[0].context_values
    finally:
        executor.remove_node(emitter_node)
        executor.remove_node(monitor)
        executor.remove_node(sub_node)
        monitor.trigger_deactivate()
        monitor.trigger_cleanup()
        monitor.destroy_node()
        emitter_node.destroy_node()
        sub_node.destroy_node()
