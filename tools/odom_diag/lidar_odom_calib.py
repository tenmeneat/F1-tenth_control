#!/usr/bin/env python3
"""라이다 기준 거리 스케일 실측 — 줄자 불필요, 출발/정지 아티팩트 무관.

원리
----
벽을 향해(또는 등지고) 직선 주행하면 정면 빔의 거리 변화량이 곧 이동거리다.
라이다는 절대 거리계(mm 정밀도)라 ERPM·게인·슬립과 완전히 독립이다.

    이동거리_참 = |전방거리(시작) - 전방거리(끝)|
    이동거리_휠 = integral |odom.twist.linear.x| dt
    speed_to_erpm_gain_새 = 4420 * (휠 / 참)
    (odom_speed = erpm/gain 이므로 odom 이 과소보고면 gain 이 너무 높은 것 = 낮춰야 함)

왜 줄자 방식을 버렸나 (2026-07-28)
----------------------------------
출발 임계 이전 구간 누락, 정지 판정 시점, 표시선 오차, 저속 ERPM 신뢰도가 전부 섞여
같은 차에서 두 번 재면 -5.6% 와 -19.0% 가 나왔다(k=0.61, c=-73cm 라는 물리적으로
불가능한 피팅). 이 방식은 두 센서가 매 순간 같은 것을 동시에 재므로 그런 게 없다.

⚠️ 끝점 두 개만 쓰면 안 된다 — lidar_fit.py 로 전 구간 회귀할 것.
   벽이 비스듬하거나 빔이 다른 면으로 옮겨가면 끝점 계산이 통째로 틀어진다
   (실측: 9구간 중 7구간이 그래서 +17%/-4% 같은 쓰레기값을 냈다).

사용: 벽을 정면에 두고 직선으로 3~8 m 접근/후퇴. 조향 중립.
"""
import csv
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# 현재 젯슨 vesc.yaml의 speed_to_erpm_gain. 새 게인을 이 값 기준으로 산출하므로
# 젯슨 값이 바뀌면 여기도 같이 바꿔야 한다.
# 2026-08-04 세미슬릭 타이어 교체 재보정으로 4232.0 → 4336.0 (08-05 확정).
# 2026-08-19 타이어 마모 진행으로 4336.0 → 4420.0 (+1.94%). 외경이 그만큼 줄었다는 뜻.
# 예전 bag을 다시 돌릴 땐 그 시점 게인으로 바꿀 것(08-04 이전 4232 / 08-04~08-18 4336).
GAIN = 4420.0
HALF_CONE = math.radians(3.0)   # 정면 ±3도 빔의 중앙값


class Calib(Node):
    def __init__(self, out):
        super().__init__('lidar_odom_calib')
        self.out = out
        self.front = float('nan')
        self.rows = []
        self.last_t = None
        self.reset()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.get_logger().info(
            '대기 중 — 벽을 정면에 두고 직선 주행. 출발하면 자동 측정, 2초 정지 시 결과.')

    def reset(self):
        self.moving = False
        self.dist = 0.0
        self.yaw = 0.0
        self.vmax = 0.0
        self.idle = 0.0
        self.front_start = float('nan')
        self.seg = []

    def scan_cb(self, m):
        n = len(m.ranges)
        ang = m.angle_min + np.arange(n) * m.angle_increment
        r = np.array(m.ranges, dtype=float)[np.abs(ang) <= HALF_CONE]
        r = r[np.isfinite(r) & (r > m.range_min) & (r < m.range_max)]
        self.front = float(np.median(r)) if len(r) >= 3 else float('nan')

    def odom_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.1)
        self.last_t = t
        v = m.twist.twist.linear.x

        # 출발 임계를 낮게(0.08) 잡는다 — 0.3 이면 초반 거리를 통째로 놓친다
        if abs(v) > 0.08 and not self.moving:
            if math.isnan(self.front):
                self.get_logger().warn('정면 빔 없음 — 벽을 정면에 두세요')
                return
            self.reset()
            self.moving = True
            self.front_start = self.front
            print(f'\n>>> 출발. 시작 전방거리 {self.front_start:.3f} m')
        if not self.moving:
            return

        self.dist += abs(v) * dt
        self.yaw += m.twist.twist.angular.z * dt
        self.vmax = max(self.vmax, abs(v))
        self.idle = self.idle + dt if abs(v) < 0.03 else 0.0
        self.seg.append((t, v, self.dist, self.front))
        print(f'  휠 {self.dist:6.3f} m | 전방 {self.front:6.3f} m | '
              f'라이다이동 {abs(self.front_start-self.front):6.3f} m', end='\r')

        if self.idle > 2.0 and self.dist > 0.3:
            self.finish()

    def finish(self):
        lid, wheel = abs(self.front_start - self.front), self.dist
        print(f'\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        print(f'  라이다 이동거리 (참)  {lid:7.3f} m   '
              f'[{self.front_start:.3f} -> {self.front:.3f}]')
        print(f'  휠 적분 거리 (ERPM)   {wheel:7.3f} m')
        print(f'  최고속도 {self.vmax:.2f} m/s   헤딩 변화 {math.degrees(self.yaw):+.1f}°')
        if abs(math.degrees(self.yaw)) > 5:
            print('  ⚠️ 헤딩이 5° 넘게 변했다 — 정면 빔이 딴 곳을 봤을 수 있음. 재측정')
        elif lid >= 0.5:
            k = wheel / lid
            print(f'\n  비율 휠/참 = {k:.4f}  ({100*(k-1):+.1f}%)')
            print(f'  ✅ speed_to_erpm_gain: {GAIN:.0f} -> {GAIN*k:.1f}')
            if abs(k - 1) < 0.02:
                print('  (2% 이내 -> 게인 정상)')
        self.rows.extend(self.seg)
        with open(self.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'v', 'wheel_dist', 'front_range'])
            w.writerows(self.rows)
        print(f'  원시데이터: {self.out}  -> lidar_fit.py 로 전 구간 회귀할 것')
        self.reset()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/lidar_calib.csv'
    rclpy.init()
    try:
        rclpy.spin(Calib(out))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
