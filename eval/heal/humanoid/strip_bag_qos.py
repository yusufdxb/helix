"""Copy a bag, keeping only the listed topics and dropping offered-QoS metadata Humble cannot parse.

Bags recorded on newer distros store QoS profiles that Humble's player rejects
("yaml-cpp: bad conversion"). Usage: strip_bag_qos.py SRC.mcap DST_DIR /topic_a,/topic_b
"""
import sys, rosbag2_py
src, dst, keep = sys.argv[1], sys.argv[2], set(sys.argv[3].split(','))
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
for t in r.get_all_topics_and_types():
    if t.name in keep:
        w.create_topic(rosbag2_py.TopicMetadata(name=t.name, type=t.type, serialization_format=t.serialization_format, offered_qos_profiles=''))
n = 0
while r.has_next():
    topic, data, ts = r.read_next()
    if topic in keep:
        w.write(topic, data, ts); n += 1
print('wrote', n)
