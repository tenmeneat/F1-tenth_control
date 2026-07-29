#!/usr/bin/env python3
"""bag 에서 map->odom TF 를 제거한 새 bag 을 만든다 (slam_toolbox 오프라인 재현용).

원본 bag 에는 그때 돌던 slam_toolbox 의 map->odom 이 들어있다. 그대로 재생하면
새로 띄운 slam_toolbox 가 발행하는 map->odom 과 충돌해 TF 트리가 깨진다.
odom->base_link (vesc_to_odom) 만 남기면 오프라인 재현이 성립한다.

사용: filter_bag.py <입력bag> <출력bag>
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

src, dst = sys.argv[1], sys.argv[2]

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
tt = reader.get_all_topics_and_types()

writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
for t in tt:
    writer.create_topic(t)

kept = dropped = 0
TF = get_message('tf2_msgs/msg/TFMessage')
while reader.has_next():
    topic, data, ts = reader.read_next()
    if topic == '/tf':
        m = deserialize_message(data, TF)
        keep = [tr for tr in m.transforms if tr.header.frame_id != 'map']
        dropped += len(m.transforms) - len(keep)
        if not keep:
            continue
        m.transforms = keep
        data = serialize_message(m)
        kept += len(keep)
    writer.write(topic, data, ts)

print(f'✅ {dst}')
print(f'   odom->base_link 등 유지 {kept}개, map->odom 제거 {dropped}개')
