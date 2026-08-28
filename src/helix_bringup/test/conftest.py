"""Test fixtures for helix_bringup.

The twist_mux model tests are pure-function and do NOT need rclpy.
The twist_mux integration tests handle rclpy lifecycle themselves
(per-test, not session-scoped, so they can be skipped cleanly when
twist_mux is not installed).
"""
import os

# Test isolation. colcon runs package test binaries CONCURRENTLY and every
# suite used to share DDS domain 0, so publishers from one package leaked
# into another package's assertions: helix_core's test_anomaly_detection
# failed on a 'nodeA/cpu_pct' FaultEvent that a sibling suite had published,
# and helix_adapter's rate-metric test saw foreign traffic the same way.
# Each package therefore gets its own domain. This is set at import time,
# before rclpy.init() reads it, and is forced rather than defaulted so a
# developer's exported ROS_DOMAIN_ID cannot silently reintroduce crosstalk
# (or point a test run at a live robot).
os.environ["ROS_DOMAIN_ID"] = "88"

