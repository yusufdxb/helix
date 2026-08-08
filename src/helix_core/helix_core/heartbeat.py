"""
Heartbeat emitter: the publish side of HeartbeatMonitor.

HeartbeatMonitor builds its registry from whoever publishes on
/helix/heartbeat, and only reports a CRASH for a node it has already seen
alive. Without an emitter on the monitored nodes the registry stays empty and
no node death is ever detectable, which is what left R4 (CRASH -> LOG_ONLY)
exercised by the fault injector alone.

Usage in a lifecycle node:

    self._heartbeat = Heartbeat(self)
    ...
    def on_activate(self, state):
        self._heartbeat.start()
        ...
    def on_deactivate(self, state):
        self._heartbeat.stop()

The emitter is deliberately dumb: a name on a topic at a fixed rate. It says
"this process is still scheduling timers", not "this node is healthy".
"""
from std_msgs.msg import String

# 10 Hz against the monitor's 0.3 s timeout and 3-miss threshold leaves room
# for roughly two dropped beats before a node is called crashed.
DEFAULT_PERIOD_SEC: float = 0.1

HEARTBEAT_TOPIC: str = '/helix/heartbeat'


class Heartbeat:
    """Publishes the owning node's name on /helix/heartbeat while active."""

    def __init__(self, node, period_sec: float = DEFAULT_PERIOD_SEC) -> None:
        self._node = node
        self._period_sec = period_sec
        self._msg = String(data=node.get_name())
        self._pub = node.create_publisher(String, HEARTBEAT_TOPIC, 10)
        self._timer = None

    def start(self) -> None:
        if self._timer is None:
            self._timer = self._node.create_timer(self._period_sec, self._beat)

    def stop(self) -> None:
        if self._timer is not None:
            self._node.destroy_timer(self._timer)
            self._timer = None

    def _beat(self) -> None:
        self._pub.publish(self._msg)
