#!/usr/bin/env python3
"""차 없이 ackermann_to_vesc의 speed/brake 중재를 검증한다."""
import rclpy, time
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64

# 젯슨 vesc.yaml의 speed_to_erpm_gain과 같아야 한다 (다르면 ERPM 임계 판정이 어긋난다).
# 2026-08-04 세미슬릭 타이어 교체 재보정으로 4232.0 → 4336.0 (08-05 확정).
GAIN = 4336.0

class T(Node):
    def __init__(self):
        super().__init__('brake_test')
        self.cmd = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.core = self.create_publisher(VescStateStamped, 'sensors/core', 10)
        self.speed_rx, self.brake_rx = [], []
        self.create_subscription(Float64, 'commands/motor/speed', lambda m: self.speed_rx.append(m.data), 10)
        self.create_subscription(Float64, 'commands/motor/brake', lambda m: self.brake_rx.append(m.data), 10)

    def pub_state(self, erpm):
        s = VescStateStamped(); s.state.speed = float(erpm); self.core.publish(s)

    def pub_cmd(self, v, steer=0.0):
        m = AckermannDriveStamped(); m.drive.speed = float(v); m.drive.steering_angle = float(steer)
        self.cmd.publish(m)

    def run_case(self, name, erpm_now, v_cmd, expect):
        self.speed_rx.clear(); self.brake_rx.clear()
        for _ in range(12):
            self.pub_state(erpm_now); self.pub_cmd(v_cmd)
            rclpy.spin_once(self, timeout_sec=0.03); time.sleep(0.02)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.02)
        got = 'brake' if self.brake_rx else ('speed' if self.speed_rx else 'none')
        ok = '✅' if got == expect else '❌'
        extra = f" brake={self.brake_rx[-1]:.2f}A" if self.brake_rx else (
                f" erpm={self.speed_rx[-1]:.0f}" if self.speed_rx else "")
        print(f"  {ok} {name}: 기대 {expect:5s} / 실제 {got:5s}{extra}")
        return got == expect

def main():
    rclpy.init(); t = T(); time.sleep(1.5)
    # DDS 매칭 워밍업: 첫 케이스가 discovery 경합으로 실패하는 것을 방지
    for i in range(100):
        t.pub_state(8000); t.pub_cmd(2.5)
        rclpy.spin_once(t, timeout_sec=0.02); time.sleep(0.01)
        if t.speed_rx:
            print(f"  (워밍업 {i+1}회만에 매칭)"); break
    else:
        print("  ⚠️ 워밍업 실패 — 노드가 speed를 전혀 안 냄")
    t.speed_rx.clear(); t.brake_rx.clear()
    print("실측 ERPM 8000 (=1.89 m/s) 고정, 명령만 바꿔가며:")
    r = []
    r.append(t.run_case("명령 2.5 m/s (가속 요구)",        8000, 2.5, 'speed'))
    r.append(t.run_case("명령 1.0 m/s (감속 요구)",        8000, 1.0, 'brake'))
    r.append(t.run_case("명령 1.85 m/s (데드밴드 내)",     8000, 1.85, 'speed'))
    r.append(t.run_case("명령 0.0 m/s (완전 감속)",        8000, 0.0, 'brake'))
    print("실측 ERPM 500 (=0.12 m/s, brake_min_erpm 미만):")
    r.append(t.run_case("저속에선 제동 대신 coast",         500, 0.0, 'speed'))
    print("후진 중 (실측 ERPM -8000 = -1.89 m/s) — 진행방향 투영 검증:")
    r.append(t.run_case("더 세게 후진 (-2.84 m/s)",       -8000, -2.84, 'speed'))
    r.append(t.run_case("후진 감속 (-1.0 m/s)",           -8000, -1.0, 'brake'))
    r.append(t.run_case("후진 중 정지 명령",               -8000,  0.0, 'brake'))
    r.append(t.run_case("후진 중 전진 명령",               -8000,  1.0, 'brake'))
    print("정지 상태에서 후진 명령:")
    r.append(t.run_case("정지→후진은 그냥 통과",              0, -1.5, 'speed'))
    print(f"\n{sum(r)}/{len(r)} 통과")
    t.destroy_node(); rclpy.shutdown()

main()
