#!/usr/bin/env python3
"""Inject a single CRITICAL-severity log line matching a configured pattern.

Used by the Phase 4 R3 regression test to exercise LogParser →
DiagnosisNode → RecoveryNode on the critical-log rule.

The emitted text defaults to "Costmap failed to initialize", which matches
the `nav2_costmap_fail` rule (severity=3, fault_type=LOG_PATTERN) in
config/log_rules.yaml.
"""
import argparse
import time

import rclpy
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--after', type=float, default=10.0,
                        help='Seconds to wait after startup before emitting the log')
    parser.add_argument('--message', default='Costmap failed to initialize',
                        help='Log text (must match an entry in log_rules.yaml)')
    args = parser.parse_args()

    rclpy.init()
    node = Node('helix_critical_log_injector')
    time.sleep(args.after)
    # .fatal() emits severity=FATAL (50), well above LogParser's ERROR threshold.
    node.get_logger().fatal(args.message)
    # Give rosout a beat to flush before shutting the node down.
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
