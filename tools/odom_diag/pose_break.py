#!/usr/bin/env python3
"""MCL 포즈 붕괴 탐지 — /pf/pose/odom 이 물리적으로 불가능하게 튀는 순간을 찾고,
그 시점의 스캔 상태·속도를 같이 본다.

/pf/pose/odom 은 pose=MCL 추정, twist=VESC 패스스루라 출처가 다르다.
따라서 twist 로 예측한 이동량과 pose 실제 변화량을 비교하면 '점프'가 분리된다.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}

    P, S = [], []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == '/pf/pose/odom':
            m = deserialize_message(data, get_message(types[topic]))
            P.append((ts * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                      yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x))
        elif topic == '/scan':
            m = deserialize_message(data, get_message(types[topic]))
            rr = np.array(m.ranges, dtype=float)
            ok = np.isfinite(rr) & (rr > m.range_min) & (rr < m.range_max)
            S.append((ts * 1e-9, int(ok.sum()), len(rr),
                      float(np.nanmedian(rr[ok])) if ok.any() else float('nan')))
    if len(P) < 20:
        print('pf/pose/odom 표본 부족')
        return

    a = np.array(P)
    t, x, y, th, v = a[:, 0] - a[0, 0], a[:, 1], a[:, 2], np.unwrap(a[:, 3]), a[:, 4]
    dt = np.diff(t)
    step = np.hypot(np.diff(x), np.diff(y))
    dth = np.abs(np.diff(th))
    # twist 로 가능한 최대 이동량 (여유 3배)
    lim = np.abs(v[:-1]) * dt * 3.0 + 0.05

    jump = np.nonzero(step > np.maximum(lim, 0.15))[0]
    print(f'/pf/pose/odom {len(a)}개, {t[-1]:.1f}초')
    print(f'속도 범위 {v.min():+.2f}~{v.max():+.2f} m/s')
    print(f'\n=== 물리적으로 불가능한 위치 점프 {len(jump)}건 ===')
    print(f'{"t[s]":>7} {"점프[m]":>9} {"허용[m]":>9} {"dyaw":>8} {"v":>7} '
          f'{"유효빔%":>8} {"스캔중앙":>9}')
    for i in jump[:25]:
        ts_ = t[i]
        k = int(np.argmin(np.abs(np.array([s[0] for s in S]) - (a[i, 0]))))
        okpct = 100 * S[k][1] / S[k][2] if S else float('nan')
        print(f'{ts_:7.1f} {step[i]:9.3f} {lim[i]:9.3f} '
              f'{math.degrees(dth[i]):+7.1f}° {v[i]:+6.2f} {okpct:7.1f}% '
              f'{S[k][3]:8.2f} m')

    if len(S) > 10:
        s = np.array([[q[1] / q[2], q[3]] for q in S])
        print(f'\n=== 스캔 품질 전체 ===')
        print(f'  유효빔 비율  중앙 {100*np.median(s[:,0]):.1f}%  '
              f'최소 {100*s[:,0].min():.1f}%')
        print(f'  스캔 거리 중앙값의 중앙 {np.nanmedian(s[:,1]):.2f} m')


if __name__ == '__main__':
    main()
