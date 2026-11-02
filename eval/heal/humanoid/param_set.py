#!/usr/bin/env python3
"""Set one parameter on a live node at a fixed wall time, for the param: injector.

`ros2 param set` takes about 6 s to start, so the moment the node applies the change is not
the moment the CLI was launched. This helper starts during warm-up, waits for the node's
set_parameters service, sleeps until T_INJECT, sends the request and prints one JSON line:
{"ok": bool, "t_sent": wall, "t_done": wall, "reason": str}. Values are parsed the way the
CLI parses them (ros2param.api.get_parameter_value).

Usage: param_set.py NODE NAME VALUE T_INJECT
"""
import json
import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from ros2param.api import get_parameter_value


def main() -> None:
    node_name, name, value, t_inject = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
    out = dict(ok=False, t_sent=None, t_done=None, reason="")
    rclpy.init()
    node = rclpy.create_node("helix_param_injector")
    try:
        cli = node.create_client(SetParameters, f"/{node_name.lstrip('/')}/set_parameters")
        if not cli.wait_for_service(timeout_sec=max(2.0, t_inject - time.time() + 5.0)):
            out["reason"] = "set_parameters service not found"
            return
        req = SetParameters.Request(parameters=[Parameter(name=name, value=get_parameter_value(string_value=value))])
        while (d := t_inject - time.time()) > 0:
            time.sleep(min(d, 0.01))
        out["t_sent"] = time.time()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        out["t_done"] = time.time()
        res = fut.result()
        if res is None:
            out["reason"] = "no response"
        else:
            r = res.results[0]
            out["ok"], out["reason"] = bool(r.successful), r.reason
    finally:
        print(json.dumps(out), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
