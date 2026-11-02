#!/usr/bin/env python3
"""Plumbing-check node for the agent's parameter and config repairs. Not a stock package.

Publishes std_msgs/String on /dev_rate at the `rate` parameter (Hz), and re-arms its timer
when `rate` changes at runtime, so a parameter fault degrades its output and restoring the
parameter repairs it. A params file with a non-numeric `rate` makes it exit at start.
It exists only so ParamRestore, RestartLKG and rollback can be exercised before the freeze
without running the withheld families (rsp and controller_manager); its episodes are never
used as training or history data.
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String


class DevRate(Node):
    def __init__(self) -> None:
        super().__init__("dev_rate")
        rate = self.declare_parameter("rate", 50.0).value
        if not isinstance(rate, (int, float)) or rate <= 0:
            raise SystemExit(f"dev_rate: bad rate {rate!r}")
        self.pub = self.create_publisher(String, "/dev_rate", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.add_on_set_parameters_callback(self.on_set)

    def on_set(self, params):
        for p in params:
            if p.name == "rate":
                if not isinstance(p.value, (int, float)) or p.value <= 0:
                    return SetParametersResult(successful=False, reason="rate must be > 0")
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / p.value, self.tick)
        return SetParametersResult(successful=True)

    def tick(self) -> None:
        self.pub.publish(String(data="x"))


def main() -> None:
    rclpy.init()
    try:
        rclpy.spin(DevRate())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
