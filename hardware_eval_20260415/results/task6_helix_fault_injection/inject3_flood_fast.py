#!/usr/bin/env python3
"""Fast topic flood — 10000 String msgs to /test/spam via single rclpy publisher."""
import time, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

N = int(sys.argv[1]) if len(sys.argv) > 1 else 10000

rclpy.init()
node = Node("helix_t6_flood")
pub = node.create_publisher(String, "/test/spam", 10)
# Let discovery settle
time.sleep(1.0)

ts_start = time.time()
m = String()
for i in range(N):
    m.data = f"spam{i}"
    pub.publish(m)
ts_end = time.time()
node.destroy_node()
rclpy.shutdown()

print(f"FLOOD_PUBLISHED={N} duration_s={ts_end - ts_start:.2f}")
print(f"TS_FLOOD_START={int(ts_start)}")
print(f"TS_FLOOD_END={int(ts_end)}")
