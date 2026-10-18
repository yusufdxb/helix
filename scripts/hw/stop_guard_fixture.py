#!/usr/bin/env python3
"""Extract the speed and position samples OverspeedGuard sees from stop-test bags.

Writes {run: {"state": [[t_ms, v, x, y], ...], "odom": [[t_ms, v], ...]}} with t
relative to the first Move request, from 0.1 s before it to the end of the
streaming window. Used by tests/test_stop_guard.py.

Usage: stop_guard_fixture.py <out.json> <run dir>...
"""
import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MOVE = 1008


def extract(run: Path, after_s: float = 4.0) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    state, odom, first_move = [], [], None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/sportmodestate':
            m = deserialize_message(data, get_message(types[topic]))
            state.append((stamp, math.hypot(m.velocity[0], m.velocity[1]),
                          m.position[0], m.position[1]))
        elif topic == '/utlidar/robot_odom':
            m = deserialize_message(data, get_message(types[topic]))
            v = m.twist.twist.linear
            odom.append((stamp, math.hypot(v.x, v.y)))
        elif topic == '/api/sport/request' and first_move is None:
            m = deserialize_message(data, get_message(types[topic]))
            if m.header.identity.api_id == MOVE:
                first_move = stamp

    def rel(stamp):
        return round((stamp - first_move) / 1e6, 3)

    def keep(stamp):
        return -100.0 <= rel(stamp) <= after_s * 1000.0

    return {'state': [[rel(t), round(float(v), 4), round(float(x), 4), round(float(y), 4)]
                      for t, v, x, y in state if keep(t)],
            'odom': [[rel(t), round(float(v), 4)] for t, v in odom if keep(t)]}


def main() -> int:
    out = {Path(r).name: extract(Path(r)) for r in sys.argv[2:]}
    Path(sys.argv[1]).write_text(json.dumps(out, separators=(',', ':')) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
