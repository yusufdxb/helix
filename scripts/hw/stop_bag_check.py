#!/usr/bin/env python3
"""Check a native stop-test bag without the test runner's own recorder.

Reads /api/sport/request, /api/sport/response and /helix/faults and prints
the Move count before and after the first fault, the first request after the
fault with its response, and the (api_id, response code) histogram. It also
reports requests outside the sink's consecutive id sequence, responses to
ids nobody in the bag sent, and /wirelesscontroller (remote) messages when
the bag recorded that topic. Bags without a fault (an aborted stage) still
get the foreign-traffic report.

Usage: stop_bag_check.py <bag dir>
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MOVE = 1008


def main(uri: str) -> int:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    requests, responses, faults, remote = [], {}, [], 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/api/sport/request':
            m = deserialize_message(data, get_message(types[topic]))
            requests.append((stamp, m.header.identity.api_id, m.header.identity.id))
        elif topic == '/api/sport/response':
            m = deserialize_message(data, get_message(types[topic]))
            responses.setdefault(m.header.identity.id,
                                 (stamp, m.header.identity.api_id, m.header.status.code))
        elif topic == '/helix/faults':
            faults.append(stamp)
        elif topic == '/wirelesscontroller':
            remote += 1
    ids = [r[2] for r in requests]
    breaks = [(a, b) for a, b in zip(ids, ids[1:]) if b != a + 1]
    unknown = [i for i in responses if i not in set(ids)]
    print('foreign traffic: id sequence breaks', len(breaks), breaks[:3],
          '| responses to unknown ids', len(unknown),
          '| remote messages', remote if '/wirelesscontroller' in types else 'not recorded')
    if not faults:
        print('no /helix/faults message in the bag')
        return 1
    fault = faults[0]
    print('requests', len(requests), 'api ids', sorted({r[1] for r in requests}))
    print('Move before fault', sum(1 for r in requests if r[1] == MOVE and r[0] < fault),
          'Move after fault', sum(1 for r in requests if r[1] == MOVE and r[0] > fault))
    first = next(r for r in requests if r[0] > fault)
    reply = responses.get(first[2])
    print('first request after fault: api', first[1], 'dt_ms', round((first[0] - fault) / 1e6, 2),
          'response', reply and (reply[1], reply[2], round((reply[0] - fault) / 1e6, 2)))
    codes = {}
    for r in requests:
        reply = responses.get(r[2])
        key = (r[1], reply[2] if reply else None)
        codes[key] = codes.get(key, 0) + 1
    print('(api, response code): count', codes)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
