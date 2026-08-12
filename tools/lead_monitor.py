#!/usr/bin/env python3
"""lead_monitor.py — 램프 선행(명령 − 실측)을 실시간으로 보는 랩탑 전용 관찰 도구.

`ramp_lead_max`(기본 2.4) 클램프가 실차에서 **실제로 무는지**를 주행 중에 바로 확인한다.
파라미터가 선언돼 있어도(= `ros2 param get`이 값을 돌려줘도) 클램프가 도는지는 별개다 —
0811 bag에서 선행이 2.5~3.8까지 갔던 게 그 예다. 이 도구는 그 숫자를 눈으로 보게 해준다.

발행 없음(순수 구독). 주행 중 켜 둬도 제어에 영향 없다.

사용:
  python3 tools/lead_monitor.py                    # 실차 기본(/pf/pose/odom)
  python3 tools/lead_monitor.py --odom /odom       # VESC 휠 속도로 비교
  python3 tools/lead_monitor.py --limit 2.4        # 컨트롤러 ramp_lead_max와 같은 값

읽는 법:
  · 정지 출발 중 `lead`가 --limit 부근에서 **평평하게 잘리면** 클램프 동작 중이다.
  · limit을 꾸준히(3사이클 이상) 넘으면 클램프가 안 도는 것 → 젯슨 바이너리/런치 확인.
  · 한두 샘플만 넘는 건 정상이다 — 실측이 순간 떨어지면 그만큼 선행이 커 보인다.

⚠️ 컨트롤러가 보는 속도는 `odom_topic`(실차 기본 `/pf/pose/odom`)이다. `/odom`으로 재면
   다른 소스라 값이 어긋날 수 있으니, 클램프 판정은 반드시 `--odom`을 컨트롤러와 맞출 것.
"""
import argparse
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class LeadMonitor(Node):
    def __init__(self, args):
        super().__init__("lead_monitor")
        self.limit = args.limit
        self.v = 0.0
        self.cmd = 0.0
        self.seen_v = False
        self.over = 0          # limit 초과 사이클 누적
        self.over_run = 0      # 연속 초과 (한두 개는 과도값이라 무시)
        self.peak = 0.0
        self.create_subscription(Odometry, args.odom, self._odom, 10)
        self.create_subscription(
            AckermannDriveStamped, args.drive, self._drive, 10)
        self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            f"lead 감시 시작 — 명령 {args.drive} / 실측 {args.odom} / 한계 {self.limit:.2f} m/s")

    def _odom(self, m):
        self.v = m.twist.twist.linear.x
        self.seen_v = True

    def _drive(self, m):
        self.cmd = m.drive.speed

    def _tick(self):
        if not self.seen_v:
            return
        lead = self.cmd - self.v
        if abs(lead) > self.peak:
            self.peak = abs(lead)
        if lead > self.limit + 0.1:
            self.over += 1
            self.over_run += 1
        else:
            self.over_run = 0
        # 막대 = 선행 폭(한계에서 눈금)
        n = max(0, min(40, int(lead / self.limit * 20)))
        bar = "#" * n
        flag = ""
        if self.over_run >= 3:
            flag = f"  🔴 한계 초과 {self.over_run}연속 — 클램프 미동작 의심"
        elif lead > self.limit + 0.1:
            flag = "  ⚠️"
        sys.stdout.write(
            f"\r명령 {self.cmd:5.2f} | 실측 {self.v:5.2f} | lead {lead:+5.2f} "
            f"|{bar:<20}| 최대 {self.peak:4.2f} 초과 {self.over:4d}{flag}   ")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="램프 선행(명령−실측) 실시간 감시")
    ap.add_argument("--odom", default="/pf/pose/odom",
                    help="컨트롤러의 odom_topic과 같게 둘 것 (기본 실차값)")
    ap.add_argument("--drive", default="/drive_autonomous")
    ap.add_argument("--limit", type=float, default=2.4,
                    help="컨트롤러 ramp_lead_max와 같은 값")
    ap.add_argument("--rate", type=float, default=10.0, help="표시 주기 [Hz]")
    args = ap.parse_args()

    rclpy.init()
    node = LeadMonitor(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM으로 끄는 게 정상 종료다 — 트레이스백을 남기지 않는다.
        pass
    finally:
        print(f"\n최대 선행 {node.peak:.2f} m/s / 한계 초과 {node.over} 사이클")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
