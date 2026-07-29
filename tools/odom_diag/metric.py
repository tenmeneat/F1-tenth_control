#!/usr/bin/env python3
"""slam 재현 실행 하나를 숫자 하나로 요약 — 파라미터 A/B 비교용.

odom 이 랩당 0.21도 오차임을 별도로 검증했으므로(gyro_scale_loop.py, 닫힌 루프 기준),
map->odom 표류량은 곧 slam 쪽 오차다. 작을수록 좋다.

⚠️ map->odom = 0 이 곧 좋은 맵은 아니다 — use_scan_matching:=false 로 꺼도 0 이 나온다.
   반드시 map_quality.py 로 실제 맵도 같이 볼 것.

사용: metric.py /tmp/slamrun/<라벨> [...]
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def metric(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    rec = []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic != '/tf':
            continue
        m = deserialize_message(data, get_message(types[topic]))
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                rec.append((tr.transform.translation.x, tr.transform.translation.y,
                            yaw_of(tr.transform.rotation)))
    if not rec:
        return None
    a = np.array(rec)
    th = np.unwrap(a[:, 2])
    return dict(n=len(a), yaw_end=math.degrees(th[-1] - th[0]),
                yaw_span=math.degrees(th.max() - th.min()),
                xy_end=float(math.hypot(a[-1, 0] - a[0, 0], a[-1, 1] - a[0, 1])))


if __name__ == '__main__':
    print(f'{"실행":<22} {"n":>6} {"최종 yaw":>10} {"yaw 폭":>9} {"최종 xy":>9}')
    for p in sys.argv[1:]:
        m = metric(p.rstrip('/') + '/tfbag')
        label = p.rstrip('/').split('/')[-1]
        if m is None:
            print(f'{label:<22} map->odom 없음 (노드가 죽었을 수 있음 — slam.log 확인)')
        else:
            print(f'{label:<22} {m["n"]:6d} {m["yaw_end"]:+9.2f}° '
                  f'{m["yaw_span"]:8.2f}° {m["xy_end"]:8.3f} m')
