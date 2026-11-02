"""Independent observer and fault injector for one episode.

Runs in its own process and domain participant, so outcome measurement never
depends on the healer under test. Records the arrival time of every message on
the given topics and every /helix/mode message, injects one fault at a fixed
wall time, and writes a JSON record at the end.
"""

import argparse
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topics", required=True, help="comma separated TOPIC or TOPIC=pkg/msg/Type")
    ap.add_argument("--inject", default="", help="node:fault[:arg], empty = nominal")
    ap.add_argument("--t-inject", type=float, required=True, help="wall time")
    ap.add_argument("--t-end", type=float, required=True, help="wall time")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("helix_eval_probe")
    types = dict((t.split("=", 1) + ["std_msgs/msg/String"])[:2] for t in a.topics.split(",") if t)
    arrivals: dict[str, list[float]] = {t: [] for t in types}
    modes: list[tuple[float, str]] = []
    data_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    for topic in arrivals:
        # raw=True: only the arrival time matters, so nothing is deserialized.
        node.create_subscription(get_message(types[topic]), topic, lambda _m, t=topic: arrivals[t].append(time.time()),
                                 data_qos, raw=True)
    mode_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/helix/mode", lambda m: modes.append((time.time(), m.data)), mode_qos)
    inj = node.create_publisher(String, "/helix_demo/inject", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    injected_at = None
    inject_subs = 0
    while time.time() < a.t_end:
        rclpy.spin_once(node, timeout_sec=0.005)
        if a.inject and injected_at is None and time.time() >= a.t_inject:
            inject_subs = inj.get_subscription_count()
            inj.publish(String(data=a.inject))
            injected_at = time.time()
    # Let the reliable inject message drain before the participant goes away.
    node.destroy_node()
    rclpy.shutdown()
    with open(a.out, "w") as f:
        json.dump({"inject": a.inject, "t_inject": injected_at, "inject_subs": inject_subs,
                   "t_end": a.t_end, "arrivals": arrivals, "modes": modes}, f)


if __name__ == "__main__":
    main()
