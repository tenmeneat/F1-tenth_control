#!/usr/bin/env python3
"""닫힌 루프로 자이로 스케일을 절대 보정한다.

차가 트랙을 한 바퀴 돌아 같은 지점으로 돌아오면 참 헤딩 변화는 정확히 360도다.
이건 어떤 센서와도 무관한 기하학적 사실이라 자이로 스케일의 **절대 기준자**가 된다.
(줄자도, 다른 센서도 필요 없다.)

    imu_angular_scale_새 = imu_angular_scale_현재 * (360 / odom이_적분한_각도)

재방문 구간은 위치로 찾는다 — 시간이 10초 이상 떨어졌는데 가장 가까운 두 시점.

사용: gyro_scale_loop.py <bag경로>
전제: /odom 의 헤딩이 이미 자이로 기반일 것(vesc_to_odom use_imu_for_angular_velocity).

2026-07-28 실측: odom 360.21° / 참 360° -> 스케일 오차 +0.06%
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CUR_SCALE = 0.017453292519943295   # pi/180 (VESC 자이로는 deg/s 발행)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    T, X, Y, YAW = [], [], [], []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic != '/odom':
            continue
        m = deserialize_message(data, get_message(types[topic]))
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        X.append(m.pose.pose.position.x)
        Y.append(m.pose.pose.position.y)
        YAW.append(yaw_of(m.pose.pose.orientation))
    if len(T) < 100:
        print('/odom 표본 부족')
        return
    t = np.array(T) - T[0]
    x, y = np.array(X), np.array(Y)
    yaw = np.unwrap(np.array(YAW))

    # 재방문 쌍: 시간이 10초 이상 떨어졌는데 위치가 가장 가까운 두 시점
    best = (1e9, 0, 0)
    for i in range(len(t)):
        far = np.nonzero(t > t[i] + 10.0)[0]
        if len(far) == 0:
            break
        dd = np.hypot(x[far] - x[i], y[far] - y[i])
        j = int(np.argmin(dd))
        if dd[j] < best[0]:
            best = (float(dd[j]), i, int(far[j]))
    gap, i, j = best
    print(f'재방문 쌍: t={t[i]:.1f}s <-> t={t[j]:.1f}s,  위치차 {gap*100:.1f} cm')

    dyaw = math.degrees(yaw[j] - yaw[i])
    truth = round(dyaw / 360) * 360
    print(f'\n=== 그 구간의 헤딩 변화 ===')
    print(f'  odom(자이로 적분)  {dyaw:+.2f}°')
    print(f'  닫힌 루프의 참값    {truth:+.1f}°')
    if truth == 0:
        print('  ⚠️ 한 바퀴가 안 된다 — 루프 구간이 아님')
        return
    k = truth / dyaw
    print(f'  오차               {dyaw-truth:+.2f}°  ({100*(dyaw/truth-1):+.2f}%)')
    print(f'\n  자이로가 {100*(1-k):+.2f}% {"과소" if k > 1 else "과대"} 측정 중')
    print(f'  ✅ imu_angular_scale: {CUR_SCALE:.10f} -> {CUR_SCALE*k:.10f}')

    path = float(np.sum(np.hypot(np.diff(x[i:j+1]), np.diff(y[i:j+1]))))
    print(f'\n=== 참고 ===')
    print(f'  이 구간 경로길이 {path:.2f} m,  위치 폐합 오차 {gap*100:.1f} cm '
          f'({100*gap/max(path,1e-9):.2f}%)')
    print(f'\n  ⚠️ 한 바퀴만으론 거칠다(재방문 위치차 {gap*100:.0f} cm 가 각도로 섞임).')
    print(f'     여러 바퀴 연속 주행이면 나눗셈이라 정밀도가 바퀴 수만큼 좋아진다.')


if __name__ == '__main__':
    main()
