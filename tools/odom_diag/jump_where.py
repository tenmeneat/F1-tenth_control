#!/usr/bin/env python3
"""포즈 점프가 '어디서' 나는지 — 트랙상 특정 지점인가, 그냥 고속이면 나는가.

  * 특정 지점에 몰림 -> 그 구간의 맵/환경 문제 (특징 부족, 대칭, 맵 오류)
  * 고루 분포        -> 속도 문제 (MCL 갱신율/모션모델이 못 따라감)
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


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
                  yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x))

a = np.array(P)
t, x, y, th, v = a[:, 0] - a[0, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4]
dt = np.diff(t)
step = np.hypot(np.diff(x), np.diff(y))
lim = np.abs(v[:-1]) * dt * 3.0 + 0.05
jump = np.nonzero(step > np.maximum(lim, 0.15))[0]

print(f'주행 {t[-1]:.1f}초, 궤적 x {x.min():.1f}~{x.max():.1f}  y {y.min():.1f}~{y.max():.1f}')
print(f'점프 {len(jump)}건\n')

# 점프 위치를 서로 묶어 본다 (1.5 m 이내면 같은 지점)
pts = np.stack([x[jump], y[jump]], 1)
used = np.zeros(len(pts), bool)
clusters = []
for i in range(len(pts)):
    if used[i]:
        continue
    d = np.hypot(pts[:, 0] - pts[i, 0], pts[:, 1] - pts[i, 1])
    mem = (d < 1.5) & ~used
    used |= mem
    clusters.append((mem.sum(), pts[mem].mean(0), t[jump][mem]))
clusters.sort(key=lambda c: -c[0])

print(f'{"군집":>4} {"건수":>5} {"위치(x, y)":>20}   점프 시각[s]')
for k, (n, c, ts_) in enumerate(clusters, 1):
    print(f'{k:>4} {n:>5}   ({c[0]:+7.2f}, {c[1]:+7.2f})   '
          f'{", ".join(f"{s:.1f}" for s in sorted(ts_)[:8])}')

print(f'\n=== 판정 ===')
big = [c for c in clusters if c[0] >= 3]
print(f'  3건 이상 반복된 지점 {len(big)}개 / 전체 군집 {len(clusters)}개')
cov = sum(c[0] for c in big)
print(f'  그 지점들이 전체 점프의 {100*cov/len(jump):.0f}% 를 설명')
if cov / len(jump) > 0.5:
    print('  🔴 특정 지점 반복 -> 그 구간의 맵/환경 문제. 속도 탓이 아니다.')
else:
    print('  -> 특정 지점에 안 몰린다. 속도/갱신율 쪽을 봐야 한다.')

# 속도 의존성
print(f'\n=== 속도 분포 ===')
allv, jv = np.abs(v), np.abs(v[jump])
for lo, hi in ((0, 1), (1, 2), (2, 3), (3, 4), (4, 6)):
    m = (allv >= lo) & (allv < hi)
    mj = (jv >= lo) & (jv < hi)
    if m.sum():
        print(f'  {lo}~{hi} m/s: 전체 {100*m.sum()/len(allv):5.1f}% 시간, '
              f'점프 {mj.sum():2d}건 ({100*mj.sum()/max(len(jv),1):5.1f}%)')
