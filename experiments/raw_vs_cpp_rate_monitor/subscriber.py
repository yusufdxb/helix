#!/usr/bin/env python3
"""
Benchmark: measure self-CPU and throughput of a high-rate Imu subscription
with and without raw=True. Invoked as:
    subscriber.py --raw 0|1 --duration 60 --warmup 5
"""
import argparse
import os
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class Sub(Node):
    def __init__(self, raw: bool, topic: str):
        super().__init__("bench_sub")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.received = 0
        self.create_subscription(
            Imu, topic, self._cb, qos, raw=raw,
        )

    def _cb(self, _msg):
        self.received += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", type=int, required=True, choices=[0, 1])
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--topic", default="/bench/imu")
    args = ap.parse_args()

    rclpy.init()
    node = Sub(raw=bool(args.raw), topic=args.topic)

    proc = psutil.Process(os.getpid())
    proc.cpu_percent(interval=None)  # prime

    # Spin in a background loop while we sample CPU in main thread.
    # We use rclpy.spin_once in a tight loop with a small timeout so the
    # scheduler can still run our sampling loop.
    import threading

    stop = threading.Event()

    def spin_loop():
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)

    t = threading.Thread(target=spin_loop, daemon=True)
    t.start()

    # Warmup: let DDS discovery settle, do not count.
    time.sleep(args.warmup)
    proc.cpu_percent(interval=None)
    received_start = node.received
    wall_start = time.monotonic()
    cpu_samples = []

    end_at = wall_start + args.duration
    while time.monotonic() < end_at:
        cpu_samples.append(proc.cpu_percent(interval=1.0))

    wall = time.monotonic() - wall_start
    received = node.received - received_start

    stop.set()
    t.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()

    cpu_samples = [c for c in cpu_samples if c > 0.0]
    if not cpu_samples:
        cpu_samples = [0.0]
    mean_cpu = sum(cpu_samples) / len(cpu_samples)
    peak_cpu = max(cpu_samples)
    hz = received / wall if wall > 0 else 0.0
    print(
        f"raw={bool(args.raw)} received={received} wall_s={wall:.2f} "
        f"hz={hz:.2f} cpu_mean_pct={mean_cpu:.2f} cpu_peak_pct={peak_cpu:.2f} "
        f"samples={len(cpu_samples)}",
        flush=True,
    )


if __name__ == "__main__":
    main()
