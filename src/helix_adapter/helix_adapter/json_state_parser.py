"""JsonStateParser — HELIX adapter lifecycle node.

Subscribes to configured ``std_msgs/String`` topics (GNSS / multiplestate /
servicestate on the GO2), parses JSON payloads, and republishes numeric and
boolean fields as labeled metrics on ``/helix/metrics``.

Each source is configured by prefix + numeric-key list + bool-key list.
Unknown keys are ignored; unparseable or non-object JSON is silently dropped
(the anomaly detector already handles "no data for this metric this tick").

Parameters:
    publish_period_sec (float, default 0.5): metric publish cadence.
    sources (list[str]): ``"<topic>|<prefix>|k1,k2|b1,b2"`` tuples.
        Default configures ``/gnss`` and ``/multiplestate`` per the GO2
        hardware-eval schema.
"""
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray, String

from helix_adapter._metrics import make_metric
from helix_adapter.json_parse import extract_numeric_fields, try_load_json

# Wire-format: "topic|prefix|num_k1,num_k2|bool_k1,bool_k2"
# Defaults match GO2 hardware_eval_20260403 / 20260406 sample payloads.
_DEFAULT_SOURCES = [
    "/gnss|gnss|satellite_total,satellite_inuse,hdop|",
    "/multiplestate|go2_state|volume,brightness|obstaclesAvoidSwitch,uwbSwitch",
]


def _parse_source_spec(spec: str):
    """Parse ``topic|prefix|numeric_keys|bool_keys`` into a dict."""
    parts = spec.split("|")
    if len(parts) != 4:
        raise ValueError(
            f"invalid source spec {spec!r}; expected 4 pipe-separated fields"
        )
    topic, prefix, nums, bools = parts
    if not topic or not prefix:
        raise ValueError(f"source spec {spec!r} missing topic or prefix")
    return {
        "topic": topic,
        "prefix": prefix,
        "numeric_keys": [k for k in nums.split(",") if k],
        "bool_keys": [k for k in bools.split(",") if k],
    }


class JsonStateParser(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("helix_json_state_parser")
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("sources", _DEFAULT_SOURCES)

        self._sources: list = []
        self._latest: dict = {}  # topic -> parsed dict
        self._subs: list = []
        self._metrics_pub = None
        self._timer = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            specs = [_parse_source_spec(s)
                     for s in self.get_parameter("sources").value]
        except ValueError as exc:
            self.get_logger().error(f"Invalid source spec: {exc}")
            return TransitionCallbackReturn.FAILURE
        for spec in specs:
            topic = spec["topic"]
            self._sources.append(spec)
            self._latest[topic] = {}
            sub = self.create_subscription(
                String, topic,
                lambda msg, t=topic: self._on_string(t, msg),
                10,
            )
            self._subs.append(sub)

        self._metrics_pub = self.create_publisher(
            Float64MultiArray, "/helix/metrics", 10)
        self.get_logger().info(
            f"JsonStateParser configured — {len(self._sources)} sources"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        period = float(self.get_parameter("publish_period_sec").value)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info("JsonStateParser activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info("JsonStateParser deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs.clear()
        self._sources.clear()
        self._latest.clear()
        if self._metrics_pub is not None:
            self.destroy_publisher(self._metrics_pub)
            self._metrics_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return self.on_cleanup(state)

    def _on_string(self, topic: str, msg: String) -> None:
        data = try_load_json(msg.data)
        if data is not None:
            self._latest[topic] = data

    def _publish(self) -> None:
        if self._metrics_pub is None:
            return
        for spec in self._sources:
            data = self._latest.get(spec["topic"], {})
            if not data:
                continue
            for name, value in extract_numeric_fields(
                data,
                numeric_keys=spec["numeric_keys"],
                bool_keys=spec["bool_keys"],
                prefix=spec["prefix"],
            ):
                self._metrics_pub.publish(make_metric(name, value))


def main(args=None):
    rclpy.init(args=args)
    node = JsonStateParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
