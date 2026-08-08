"""
LivelinessBeacon: DDS QoS liveliness emitter, the publish side of
LivelinessMonitor.

This is the alternative to helix_core.heartbeat.Heartbeat. Instead of
publishing the node's name as payload on a shared /helix/heartbeat topic and
having the monitor count elapsed wall-clock time between messages, this
emitter relies on the DDS LIVELINESS QoS policy: it opens a publisher with
liveliness=MANUAL_BY_TOPIC and a lease duration, then calls
Publisher.assert_liveliness() on a timer. The RMW (Cyclone DDS under HELIX)
tracks the writer's liveliness itself and raises a LIVELINESS_CHANGED event
on any matched subscription the instant the lease expires, no user-data
message required.

Read against the installed rclpy source before writing this (Humble,
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/):
  - qos.py: LivelinessPolicy enum (SYSTEM_DEFAULT=0, AUTOMATIC=1,
    MANUAL_BY_TOPIC=3) and QoSProfile.liveliness / liveliness_lease_duration
    fields.
  - publisher.py: Publisher.assert_liveliness() calls
    _rclpy.rclpy_assert_liveliness(self.handle); docstring says the
    application must call this at least as often as
    QoSProfile.liveliness_lease_duration.
  - duration.py: Duration(seconds=...) builds the C-level rcl_duration_t
    used for both `deadline` and `liveliness_lease_duration`.

Each monitored node gets its own topic (/helix/liveliness/<node_name>)
rather than sharing one topic the way /helix/heartbeat does. This is
deliberate: the subscription-side LIVELINESS_CHANGED event only reports
aggregate alive_count / not_alive_count across every writer matched on that
subscription, it does not carry the identity of which writer changed state.
Sharing one topic across nine nodes would make a single node's death
indistinguishable from eight other nodes staying alive. One topic per node
keeps the DDS-level detection as identifiable as the current heartbeat.
"""
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Empty

# Cyclone DDS gotcha (ros-safety/software_watchdogs README): the lease
# duration is compared against wall-clock gaps between assert_liveliness()
# calls, so it must be comfortably larger than the assert period or normal
# scheduling jitter reads as a false death. 3x mirrors the WindowedWatchdog
# guidance of sizing the lease for the number of tolerated missed asserts.
DEFAULT_ASSERT_PERIOD_SEC: float = 0.1
DEFAULT_LEASE_DURATION_SEC: float = 3 * DEFAULT_ASSERT_PERIOD_SEC

LIVELINESS_TOPIC_PREFIX: str = '/helix/liveliness/'


def liveliness_topic_for(node_name: str) -> str:
    """Return the per-node liveliness topic name for a monitored node."""
    return f'{LIVELINESS_TOPIC_PREFIX}{node_name}'


def build_liveliness_qos(lease_duration_sec: float) -> QoSProfile:
    """
    Build the QoSProfile both the beacon publisher and the watchdog
    subscription must use. LIVELINESS QoS compatibility requires the
    subscription's requested lease to be >= the publisher's offered lease,
    so both sides of HELIX's own mechanism use this one builder to avoid an
    accidental mismatch producing a silently-incompatible pair.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=lease_duration_sec),
    )


class LivelinessBeacon:
    """
    Asserts DDS liveliness for the owning node while active.

    Usage in a lifecycle node, alongside (not instead of) Heartbeat:

        self._liveliness = LivelinessBeacon(self)
        ...
        def on_activate(self, state):
            self._liveliness.start()
            ...
        def on_deactivate(self, state):
            self._liveliness.stop()

    No payload is ever published: assert_liveliness() is a lightweight RMW
    control call, not a user-data sample, so this adds no traffic on
    /helix/liveliness/<node_name> beyond the DDS liveliness protocol itself.
    """

    def __init__(
        self,
        node,
        assert_period_sec: float = DEFAULT_ASSERT_PERIOD_SEC,
        lease_duration_sec: float = DEFAULT_LEASE_DURATION_SEC,
    ) -> None:
        self._node = node
        self._assert_period_sec = assert_period_sec
        self._lease_duration_sec = lease_duration_sec
        self._topic = liveliness_topic_for(node.get_name())
        self._qos = build_liveliness_qos(lease_duration_sec)
        self._pub = node.create_publisher(Empty, self._topic, self._qos)
        self._timer = None

    def start(self) -> None:
        if self._timer is None:
            self._timer = self._node.create_timer(
                self._assert_period_sec, self._assert
            )

    def stop(self) -> None:
        if self._timer is not None:
            self._node.destroy_timer(self._timer)
            self._timer = None

    def _assert(self) -> None:
        self._pub.assert_liveliness()
