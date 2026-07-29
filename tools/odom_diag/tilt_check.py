#!/usr/bin/env python3
"""방위각별 최대 관측 거리로 하향 기울기를 판정한다.

바닥에 닿는 거리 r = h / tan(alpha*cos(theta)) 는 차 위치와 무관한 기하값이라,
기울어져 있으면 정면 관측이 그 값을 절대 못 넘는다.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

H = 0.11

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
A, ang, meta = [], None, None
while r.has_next():
    topic, data, ts = r.read_next()
    if topic != '/scan':
        continue
    m = deserialize_message(data, get_message(types[topic]))
    if ang is None:
        ang = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        meta = (m.range_min, m.range_max)
    A.append(np.array(m.ranges, dtype=float))

A = np.vstack(A)
A[~np.isfinite(A) | (A < meta[0]) | (A >= meta[1])] = np.nan
deg = np.degrees(ang)
mx = np.nanmax(A, axis=0)
p99 = np.nanpercentile(A, 99, axis=0)

print(f'스캔 {A.shape[0]}개, 빔 {len(ang)}개')
print(f'{"방위각":>8} {"최대":>9} {"p99":>9} {"1° 예측 바닥":>13}')
for t in (-120, -90, -60, -30, -15, 0, 15, 30, 60, 90, 120):
    i = int(np.argmin(np.abs(deg - t)))
    a = math.radians(1.0) * math.cos(math.radians(t))
    pred = H / math.tan(a) if a > 1e-9 else float('inf')
    ps = f'{pred:10.1f} m' if pred < 100 else '       n/a'
    print(f'{t:+7.0f}° {mx[i]:7.2f} m {p99[i]:7.2f} m {ps}')

f = np.abs(deg) < 20
fmax = float(np.nanmax(mx[f]))
print(f'\n정면(±20°) 최대 {fmax:.2f} m   전체 최대 {np.nanmax(mx):.2f} m')
print(f'1° 기울기면 정면은 6.3 m 를 못 넘는다.')
if fmax > 7.0:
    print(f'-> 정면으로 {fmax:.1f} m 관측. 바닥에 막히지 않는다.')
    print(f'   이 관측과 양립하는 최대 기울기 = {math.degrees(math.atan(H/fmax)):.2f}°')
else:
    print(f'-> 정면이 {fmax:.1f} m 에서 잘린다. 역산 기울기 '
          f'{math.degrees(math.atan(H/fmax)):.2f}°')
