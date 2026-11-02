#!/usr/bin/env python3
"""Extract GO2 streams from a ROS 2 bag into a compact .npz for the HELIX v2 evaluation.

Every stream keeps the bag receive time (t, seconds) so the offline evaluation sees
messages in the order and at the times the robot computer saw them.

Usage: extract_bag.py <bag_dir> <out.npz> [--msgs <unitree msg root>]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

DEFAULT_MSGS = Path.home() / "Projects/unitree_ros2/cyclonedds_ws/src/unitree"


def make_typestore(msg_root: Path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for pkg in ("unitree_go", "unitree_api"):
        for f in sorted((msg_root / pkg / "msg").glob("*.msg")):
            text = f.read_text()
            if pkg == "unitree_go" and f.stem == "SportModeState":
                # The GO2 firmware in these recordings publishes the older layout with a
                # trailing PathPoint[10] (236 + 10 * 28 = 516 bytes on the wire).
                text = text.rstrip() + "\nPathPoint[10] path_point\n"
            types.update(get_types_from_msg(text, f"{pkg}/msg/{f.stem}"))
    ts.register(types)
    return ts


def imu_state(m):
    return list(m.gyroscope) + list(m.accelerometer)


EXTRACT = {
    # topic: (key, row function)
    "/utlidar/imu": ("lidar_imu", lambda m: [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                              m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                                              m.linear_acceleration.x, m.linear_acceleration.y,
                                              m.linear_acceleration.z]),
    "/sportmodestate": ("sport", lambda m: [m.stamp.sec + m.stamp.nanosec * 1e-9]
                        + imu_state(m.imu_state) + list(m.velocity) + [m.yaw_speed, m.mode, m.gait_type]),
    "/lowstate": ("low", lambda m: [float(m.tick)] + imu_state(m.imu_state)
                  + [m.motor_state[i].dq for i in range(12)] + [m.motor_state[i].q for i in range(12)]),
    "/utlidar/robot_odom": ("odom", lambda m: [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                               m.twist.twist.linear.x, m.twist.twist.linear.y,
                                               m.twist.twist.linear.z, m.twist.twist.angular.z]),
}
COLUMNS = {
    "lidar_imu": "stamp,gx,gy,gz,ax,ay,az",
    "sport": "stamp,gx,gy,gz,ax,ay,az,vx,vy,vz,yaw_speed,mode,gait_type",
    "low": "tick,gx,gy,gz,ax,ay,az," + ",".join(f"dq{i}" for i in range(12)) + "," + ",".join(f"q{i}" for i in range(12)),
    "odom": "stamp,vx,vy,vz,wz",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--msgs", default=str(DEFAULT_MSGS))
    a = ap.parse_args()
    ts = make_typestore(Path(a.msgs))
    rows = {k: [] for k, _ in EXTRACT.values()}
    times = {k: [] for k in rows}
    req_t, req_api, req_param = [], [], []
    with AnyReader([Path(a.bag)], default_typestore=ts) as r:
        wanted = [c for c in r.connections if c.topic in EXTRACT or c.topic == "/api/sport/request"]
        for c, t, raw in r.messages(connections=wanted):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == "/api/sport/request":
                req_t.append(t * 1e-9)
                req_api.append(int(m.header.identity.api_id))
                req_param.append(m.parameter)
                continue
            key, fn = EXTRACT[c.topic]
            times[key].append(t * 1e-9)
            rows[key].append(fn(m))
    out = {}
    for k in rows:
        if rows[k]:
            out[f"{k}_t"] = np.asarray(times[k])
            out[k] = np.asarray(rows[k], dtype=np.float64)
            out[f"{k}_cols"] = np.asarray(COLUMNS[k])
    if req_t:
        out["req_t"] = np.asarray(req_t)
        out["req_api"] = np.asarray(req_api)
        out["req_param"] = np.asarray(req_param)
    np.savez_compressed(a.out, **out)
    summary = {k: len(v) for k, v in rows.items()}
    summary["requests"] = len(req_t)
    print(Path(a.bag).name, summary, file=sys.stderr)


if __name__ == "__main__":
    main()
