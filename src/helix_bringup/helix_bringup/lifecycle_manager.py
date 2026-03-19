"""
helix_lifecycle_manager — configure and activate all HELIX lifecycle nodes.

Runs as a plain rclpy node within the launch, uses service calls (not CLI)
to transition lifecycle nodes. Much more reliable than external ros2 commands
because it participates directly in the DDS graph.
"""
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition, State
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

MANAGED_NODES = [
    "helix_heartbeat_monitor",
    "helix_anomaly_detector",
    "helix_log_parser",
    "helix_recovery_planner",
    "helix_llm_advisor",
]

WAIT_TIMEOUT = 30.0   # seconds to wait for each service
CALL_TIMEOUT = 10.0   # seconds to wait for each service call


def _call(node: Node, client, request):
    """Spin until a service call completes; return the result or None."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)
    return future.result()


def manage_node(node: Node, name: str) -> bool:
    change_srv = f"/{name}/change_state"
    get_srv = f"/{name}/get_state"

    change_client = node.create_client(ChangeState, change_srv)
    get_client = node.create_client(GetState, get_srv)

    node.get_logger().info(f"Waiting for {name} services…")
    if not change_client.wait_for_service(timeout_sec=WAIT_TIMEOUT):
        node.get_logger().error(f"Timeout waiting for {change_srv}")
        return False

    # --- get current state ---
    result = _call(node, get_client, GetState.Request())
    if result is None:
        node.get_logger().error(f"Failed to get state of {name}")
        return False
    current = result.current_state.id
    node.get_logger().info(f"{name}: state={result.current_state.label}")

    # --- configure if unconfigured ---
    if current == State.PRIMARY_STATE_UNCONFIGURED:
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        result = _call(node, change_client, req)
        if result is None or not result.success:
            node.get_logger().error(f"configure failed for {name}")
            return False
        node.get_logger().info(f"{name}: configured ✓")
        current = State.PRIMARY_STATE_INACTIVE

    # --- activate if inactive ---
    if current == State.PRIMARY_STATE_INACTIVE:
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        result = _call(node, change_client, req)
        if result is None or not result.success:
            node.get_logger().error(f"activate failed for {name}")
            return False
        node.get_logger().info(f"{name}: activated ✓")
        return True

    if current == State.PRIMARY_STATE_ACTIVE:
        node.get_logger().info(f"{name}: already active ✓")
        return True

    node.get_logger().error(f"{name}: unexpected state {current}, skipping")
    return False


def main():
    rclpy.init()
    node = Node("helix_lifecycle_manager")
    node.get_logger().info("HELIX Lifecycle Manager starting…")

    # Give nodes a moment to finish DDS registration
    time.sleep(3.0)

    ok = 0
    for name in MANAGED_NODES:
        if manage_node(node, name):
            ok += 1

    node.get_logger().info(
        f"Lifecycle Manager done: {ok}/{len(MANAGED_NODES)} nodes active"
    )
    rclpy.shutdown()
    sys.exit(0 if ok == len(MANAGED_NODES) else 1)
