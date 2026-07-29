#!/usr/bin/env python3
"""점프가 '고정 시간 지연'인지 '고정 거리 오차'인지 가른다.

지연 보상이 과하면 보정량 = 속도 x 시간오차 이므로 점프가 속도에 비례한다.
초과 이동량을 속도로 나눈 값(=등가 시간)이 일정하면 시간 문제,
초과 이동량 자체가 일정하면 거리/맵 문제다.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
P = []
while r.has_next():
    topic, data, ts = r.read_next()
    if topic == '/pf/pose/odom':
        m = deserialize_message(data, get_message(types[topic]))
        P.append((ts * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                  m.twist.twist.linear.x))

a = np.array(P)
t, x, y, v = a[:, 0] - a[0, 0], a[:, 1], a[:, 2], a[:, 3]
dt = np.diff(t)
step = np.hypot(np.diff(x), np.diff(y))
vv = np.abs(v[:-1])
lim = vv * dt * 3.0 + 0.05
J = np.nonzero(step > np.maximum(lim, 0.15))[0]

excess = step[J] - vv[J] * dt[J]              # 정상 이동분을 뺀 초과량 [m]
terr = excess / np.maximum(vv[J], 1e-6)       # 초과량을 시간으로 환산 [s]

print(f'점프 {len(J)}건')
print(f'\n[거리 해석] 초과 이동량')
print(f'  중앙 {np.median(excess):.3f} m   범위 {excess.min():.3f}~{excess.max():.3f} m'
      f'   변동계수 {np.std(excess)/np.mean(excess):.3f}')
print(f'\n[시간 해석] 초과량 / 속도 = 등가 시간 오차')
print(f'  중앙 {np.median(terr)*1000:.1f} ms   범위 {terr.min()*1000:.0f}~'
      f'{terr.max()*1000:.0f} ms   변동계수 {np.std(terr)/np.mean(terr):.3f}')
print(f'\n  그때 속도 중앙 {np.median(vv[J]):.2f} m/s '
      f'(범위 {vv[J].min():.2f}~{vv[J].max():.2f})')

cv_t = np.std(terr) / np.mean(terr)
cv_d = np.std(excess) / np.mean(excess)
print(f'\n=== 판정 ===')
if cv_t < cv_d * 0.8:
    print(f'  🔴 시간 해석의 변동계수가 더 작다 ({cv_t:.3f} < {cv_d:.3f})')
    print(f'     -> 고정 시간 지연이 속도에 곱해져 나오는 오차다.')
    print(f'     delay_compensation_factor=3.0 이면 '
          f'mcl_processing_time ≈ {np.median(terr)*1000/3:.0f} ms 에 해당')
elif cv_d < cv_t * 0.8:
    print(f'  거리 해석이 더 일정 ({cv_d:.3f} < {cv_t:.3f}) -> 맵/환경 쪽 고정 오차')
else:
    print(f'  둘이 비슷하다 ({cv_t:.3f} vs {cv_d:.3f}) — 속도 범위가 좁아 구분이 안 된다.')
    print(f'  속도 {vv[J].min():.1f}~{vv[J].max():.1f} m/s 로 배율이 '
          f'{vv[J].max()/max(vv[J].min(),1e-9):.1f}배뿐이라 판별력이 약하다.')
    print(f'  느린 랩과 빠른 랩을 같이 담은 주행이 필요하다.')
