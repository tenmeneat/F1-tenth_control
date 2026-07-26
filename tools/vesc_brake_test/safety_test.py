#!/usr/bin/env python3
"""안전 속성: (1) brake_enabled=false면 절대 brake 안 나감 (2) sensors/core 없으면 speed 폴백"""
import rclpy, time, sys
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64

class T(Node):
    def __init__(self):
        super().__init__('safety_test')
        self.cmd = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.core = self.create_publisher(VescStateStamped, 'sensors/core', 10)
        self.speed_rx, self.brake_rx = [], []
        self.create_subscription(Float64, 'commands/motor/speed', lambda m: self.speed_rx.append(m.data), 10)
        self.create_subscription(Float64, 'commands/motor/brake', lambda m: self.brake_rx.append(m.data), 10)
    def cycle(self, n, erpm=None, v=0.0):
        for _ in range(n):
            if erpm is not None:
                s = VescStateStamped(); s.state.speed = float(erpm); self.core.publish(s)
            m = AckermannDriveStamped(); m.drive.speed = float(v); self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=0.03); time.sleep(0.02)
        for _ in range(20): rclpy.spin_once(self, timeout_sec=0.02)

mode = sys.argv[1]
rclpy.init(); t = T(); time.sleep(1.5)
if mode == 'disabled':
    # 감속을 강하게 요구하는 상황인데도 brake가 나가면 안 됨
    t.speed_rx.clear(); t.brake_rx.clear(); t.cycle(15, erpm=8000, v=0.0)
    ok = len(t.brake_rx) == 0 and len(t.speed_rx) > 0
    print(f"  {'✅' if ok else '❌'} brake_enabled=false: brake {len(t.brake_rx)}건(0이어야), speed {len(t.speed_rx)}건(>0이어야)")
else:
    # sensors/core를 아예 안 보냄 → 상태 없음 → speed 전용 폴백
    t.speed_rx.clear(); t.brake_rx.clear(); t.cycle(15, erpm=None, v=0.0)
    ok1 = len(t.brake_rx) == 0 and len(t.speed_rx) > 0
    print(f"  {'✅' if ok1 else '❌'} sensors/core 없음: brake {len(t.brake_rx)}건(0이어야), speed {len(t.speed_rx)}건(>0이어야)")
    # 상태를 줬다가 끊고 timeout(0.2s) 넘긴 뒤
    t.cycle(10, erpm=8000, v=0.0)
    had = len(t.brake_rx) > 0
    time.sleep(0.6)
    t.speed_rx.clear(); t.brake_rx.clear(); t.cycle(15, erpm=None, v=0.0)
    ok2 = had and len(t.brake_rx) == 0
    print(f"  {'✅' if ok2 else '❌'} 상태 끊김 후 timeout: 끊기 전 제동 {had}, 끊긴 뒤 brake {len(t.brake_rx)}건(0이어야)")
t.destroy_node(); rclpy.shutdown()
