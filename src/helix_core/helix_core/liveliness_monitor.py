"""
LivelinessMonitor: DDS QoS liveliness-based node-death detection.

Alternative detection path to helix_core.heartbeat_monitor.HeartbeatMonitor.
Where HeartbeatMonitor infers node death by counting elapsed wall-clock time
between /helix/heartbeat String messages, this node asks Cyclone DDS itself:
each monitored node runs a LivelinessBeacon (helix_core.liveliness_beacon)
that calls Publisher.assert_liveliness() on a per-node topic
(/helix/liveliness/<node_name>) with QoS liveliness=MANUAL_BY_TOPIC and a
lease duration. This node subscribes to each of those topics with a matching
QoS profile and a SubscriptionEventCallbacks(liveliness=...) handler, so the
RMW raises RCL_SUBSCRIPTION_LIVELINESS_CHANGED the instant a lease expires,
no periodic polling of message content required.

This mirrors what ros-safety/software_watchdogs' SimpleWatchdog does
(liveliness-only, no tolerance for missed asserts): one lease violation is
one dead node. It intentionally does NOT implement the WindowedWatchdog
variant (liveliness + deadline, tolerating N violations): that needs a
second QoS policy (deadline) and a violation counter per node, which is a
real extension but out of scope for proving the mechanism here. See
docs/comparison.md for the tradeoff writeup.

SCOPE NOTE: this node is net-new and is not wired into helix_bringup's
launch files or into the nine existing lifecycle nodes by this change: that
wiring touches files owned by other in-flight work on this branch. This node
proves the DDS-liveliness mechanism works (own tests, including a live
rclpy/Cyclone DDS integration test) and is ready to be adopted; it does not
change any behavior of HeartbeatMonitor or the current CRASH-detection path,
and the `enabled` parameter defaults to False so an accidental launch is
inert by default.

Emits the same FaultEvent wire contract as HeartbeatMonitor
(fault_type="CRASH", severity=3) on /helix/faults, so nothing downstream
(DIAGNOSE, RECOVER) needs to know which detector produced it.
"""
import time
from typing import Dict, List

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos_event import QoSLivelinessChangedInfo, SubscriptionEventCallbacks
from std_msgs.msg import Empty

from helix_core.liveliness_beacon import build_liveliness_qos, liveliness_topic_for
from helix_msgs.msg import FaultEvent

DEFAULT_LEASE_DURATION_SEC: float = 0.3
DEFAULT_ENABLED: bool = False


class LivelinessMonitor(LifecycleNode):
    """Lifecycle node that detects node death via DDS LIVELINESS QoS."""

    def __init__(self) -> None:
        super().__init__('helix_liveliness_monitor')

        self.declare_parameter('enabled', DEFAULT_ENABLED)
        self.declare_parameter('monitored_nodes', [''])
        self.declare_parameter('lease_duration_sec', DEFAULT_LEASE_DURATION_SEC)

        self._fault_pub = None
        self._subs: List = []
        # node_name -> True if we have already emitted CRASH for the current
        # not-alive episode; cleared when a fresh writer is seen alive again.
        self._crashed: Dict[str, bool] = {}
        self._enabled = DEFAULT_ENABLED
        self._lease_duration_sec = DEFAULT_LEASE_DURATION_SEC

    # -- Lifecycle callbacks --------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._enabled = self.get_parameter('enabled').value
        self._lease_duration_sec = self.get_parameter('lease_duration_sec').value
        raw_nodes = self.get_parameter('monitored_nodes').value
        monitored_nodes = [n for n in raw_nodes if n]

        self._fault_pub = self.create_publisher(FaultEvent, '/helix/faults', 10)

        if not self._enabled:
            self.get_logger().info(
                'LivelinessMonitor configured but disabled (enabled=False); '
                'no liveliness subscriptions created.'
            )
            return TransitionCallbackReturn.SUCCESS

        qos = build_liveliness_qos(self._lease_duration_sec)
        for node_name in monitored_nodes:
            self._crashed[node_name] = False
            topic = liveliness_topic_for(node_name)
            sub = self.create_subscription(
                Empty,
                topic,
                lambda _msg: None,  # no payload is ever published on this topic
                qos,
                event_callbacks=SubscriptionEventCallbacks(
                    liveliness=self._make_liveliness_callback(node_name)
                ),
            )
            self._subs.append(sub)

        self.get_logger().info(
            f'LivelinessMonitor configured, enabled=True '
            f'lease_duration={self._lease_duration_sec}s '
            f'monitoring={monitored_nodes}'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('LivelinessMonitor activated.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('LivelinessMonitor deactivated.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        if self._fault_pub:
            self.destroy_publisher(self._fault_pub)
            self._fault_pub = None
        self._crashed = {}
        return TransitionCallbackReturn.SUCCESS

    # -- Liveliness handling ----------------------------------------------------

    def _make_liveliness_callback(self, node_name: str):
        def _callback(event: QoSLivelinessChangedInfo) -> None:
            self._on_liveliness_changed(node_name, event)
        return _callback

    def _on_liveliness_changed(
        self, node_name: str, event: QoSLivelinessChangedInfo
    ) -> None:
        """
        SimpleWatchdog-equivalent policy: any not_alive_count_change > 0 is
        an immediate CRASH (no tolerance for repeated violations). Any
        alive_count_change > 0 clears the crashed flag so a restarted node's
        next lease violation can raise a fresh fault.
        """
        if event.not_alive_count_change > 0:
            if not self._crashed.get(node_name, False):
                self._crashed[node_name] = True
                self._emit_crash_fault(node_name, event)
        if event.alive_count_change > 0:
            self._crashed[node_name] = False

    def _emit_crash_fault(
        self, node_name: str, event: QoSLivelinessChangedInfo
    ) -> None:
        msg = FaultEvent()
        msg.node_name = node_name
        msg.fault_type = 'CRASH'
        msg.severity = 3
        msg.detail = (
            f'DDS liveliness lease expired for {node_name} '
            f'(lease={self._lease_duration_sec}s, MANUAL_BY_TOPIC).'
        )
        msg.timestamp = time.time()
        msg.context_keys = ['detection_mechanism', 'lease_duration_sec', 'alive_count']
        msg.context_values = [
            'dds_liveliness',
            str(self._lease_duration_sec),
            str(event.alive_count),
        ]
        self._fault_pub.publish(msg)
        self.get_logger().info(
            f'FaultEvent emitted: CRASH for {node_name} via DDS liveliness '
            f'(alive_count={event.alive_count}, not_alive_count={event.not_alive_count})'
        )


def main(args=None) -> None:
    """Entry point (not yet wired to a console_script; run as a module)."""
    rclpy.init(args=args)
    node = LivelinessMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
