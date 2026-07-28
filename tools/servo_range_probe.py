#!/usr/bin/env python3
"""서보 가동범위 실측 프로브 — 좌우 조향 비대칭 해소용 (2026-07-28 신설).

## 왜 필요한가

젯슨 `f1tenth_stack/config/vesc.yaml`의 값들:

    steering_angle_to_servo_gain:   -0.4463
    steering_angle_to_servo_offset:  0.4672
    servo_min: 0.2703      servo_max: 0.6363

servo = gain·δ + offset 이므로 ±0.41 rad을 내려면 servo가
`0.4672 ∓ 0.4463×0.41` = **[0.2842, 0.6502]** 를 커버해야 한다. 그런데:

    좌(δ>0) 최대 : servo_min 0.2703 → δ = +0.441 rad  ← 여유 있음
    우(δ<0) 최대 : servo_max 0.6363 → δ = **-0.379 rad**  ← 0.41을 못 씀 (-7.6%)

`vesc_driver`가 `servo_limit_.clip()`으로 자르므로 **우조향만 8% 손해**다. 07-27 실차
bag(`run_0727_203040`)에서 실제로 servo 0.6502가 발행돼 0.6363으로 잘린 것이 확인됐다.

control_map_node의 `MAX_STEERING_ANGLE`(0.41)과 `steer_authority_ratio` 곡률 속도 캡은
좌우 대칭을 가정하므로, **우선회 코너에서 캡이 낙관적**이 된다(실제보다 8% 더 꺾일 수
있다고 보고 속도를 덜 줄임).

## 무엇을 알아내야 하나

`servo_max = 0.6363`이 **소프트웨어 보수값**인지 **실제 기구 한계**인지. 전자면 0.6502로
올려 대칭을 회복하면 되고, 후자면 `MAX_STEERING_ANGLE`을 0.379로 낮추거나 트림을 재조정해야
한다. 이 스크립트는 그걸 **안전하게** 판별한다(작은 스텝, 매 스텝 확인, 언제든 중단).

## 사용법

    ⚠️ 반드시 **차를 들어 바퀴를 띄운 상태**에서 할 것. 서보가 스톨하면 기어가 상한다.
    ⚠️ f1tenth_stack bringup이 떠 있어야 한다(vesc_driver가 servo 명령을 받는다).
    ⚠️ 자율/수동 어느 쪽도 주행 명령을 보내고 있지 않아야 한다(E-stop 상태 권장).

    # 젯슨에서
    export ROS_DOMAIN_ID=67
    python3 servo_range_probe.py --side right      # 우조향(servo 증가) 한계 탐색
    python3 servo_range_probe.py --side left       # 좌조향(servo 감소) 한계 탐색

각 스텝에서 멈추고 물어본다. **바퀴가 더 안 돌거나 / 서보에서 "웅" 하는 스톨음이 나거나 /
링키지가 튕기면 즉시 n을 입력**한다. 직전 값이 실제 한계다.

⚠️ 이 스크립트는 `commands/servo/position`에 **직접** 발행한다 —
`ackermann_to_vesc`도 같은 토픽에 발행하므로, 그쪽이 조용할 때만(E-stop) 쓸 것.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# 젯슨 vesc.yaml 실값 (2026-07-28 기준). 바뀌면 여기도 맞출 것.
GAIN = -0.4463
OFFSET = 0.4672
SERVO_MIN_CFG = 0.2703
SERVO_MAX_CFG = 0.6363

STEP = 0.005          # 한 스텝 서보 증분 (≈0.011 rad)
SETTLE_S = 0.6        # 스텝 후 안정화 대기
HARD_MIN = 0.15       # 절대 안전 한계 (vesc_driver 기본 servo_limit)
HARD_MAX = 0.85


def servo_to_angle(s):
    return (s - OFFSET) / GAIN


class Probe(Node):
    def __init__(self):
        super().__init__("servo_range_probe")
        self.pub = self.create_publisher(Float64, "commands/servo/position", 10)

    def send(self, s):
        m = Float64()
        m.data = float(s)
        # 몇 번 반복 발행해야 드라이버가 확실히 받는다(신뢰성 낮은 QoS 대비)
        for _ in range(5):
            self.pub.publish(m)
            time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], required=True,
                    help="left = servo 감소 방향(δ>0), right = servo 증가 방향(δ<0)")
    ap.add_argument("--start", type=float, default=None, help="탐색 시작 servo 값")
    ap.add_argument("--limit", type=float, default=None, help="탐색 상/하한 (안전 정지선)")
    args = ap.parse_args()

    direction = -1 if args.side == "left" else +1
    start = args.start if args.start is not None else OFFSET
    if args.limit is not None:
        limit = args.limit
    else:
        limit = HARD_MIN if direction < 0 else HARD_MAX

    print(__doc__.split("## 사용법")[0])
    print(f"\n=== {args.side} 방향 탐색 ===")
    print(f"시작 servo {start:.4f} (δ={servo_to_angle(start):+.4f} rad), "
          f"스텝 {direction*STEP:+.4f}, 안전 정지선 {limit:.4f}")
    print(f"현재 설정 한계: servo_min={SERVO_MIN_CFG} / servo_max={SERVO_MAX_CFG}")
    print(f"±0.41 rad 대칭에 필요한 값: "
          f"[{OFFSET + GAIN*0.41:.4f}, {OFFSET - GAIN*0.41:.4f}]")
    print("\n⚠️ 바퀴가 떠 있는지, E-stop 상태인지 다시 확인하세요.")
    if input("계속하려면 yes 입력: ").strip().lower() != "yes":
        print("중단."); return

    rclpy.init()
    p = Probe()
    s = start
    last_ok = None
    try:
        while (direction < 0 and s > limit) or (direction > 0 and s < limit):
            p.send(s)
            ang = servo_to_angle(s)
            flag = ""
            if s < SERVO_MIN_CFG or s > SERVO_MAX_CFG:
                flag = "  ← 현재 설정 한계 밖(드라이버 servo_limit을 임시로 넓혀야 실제로 나감)"
            print(f"\nservo={s:.4f}  δ={ang:+.4f} rad ({ang*57.3:+.1f}°){flag}")
            r = input("  바퀴가 더 꺾였고 스톨음/걸림 없음? [y/n/q]: ").strip().lower()
            if r == "q":
                break
            if r != "y":
                print(f"  → 한계 도달. 마지막 정상값: {last_ok}")
                break
            last_ok = s
            s += direction * STEP
            time.sleep(SETTLE_S - 0.1)
    finally:
        print("\n중립으로 복귀…")
        p.send(OFFSET)
        time.sleep(0.3)
        p.destroy_node()
        rclpy.shutdown()

    if last_ok is None:
        print("측정값 없음."); return
    ang = servo_to_angle(last_ok)
    print("\n" + "=" * 60)
    print(f"실측 {args.side} 한계: servo={last_ok:.4f}  →  δ={ang:+.4f} rad ({ang*57.3:+.1f}°)")
    if args.side == "right":
        print(f"  현재 vesc.yaml servo_max={SERVO_MAX_CFG} (δ={servo_to_angle(SERVO_MAX_CFG):+.4f})")
        if last_ok > SERVO_MAX_CFG + 1e-6:
            print(f"  → servo_max를 {min(last_ok, OFFSET - GAIN*0.41):.4f}까지 올릴 수 있다. "
                  f"0.6502면 우조향도 -0.41 rad 확보 = 좌우 대칭 회복.")
        else:
            print(f"  → 기구 한계가 맞다. control_map_node의 MAX_STEERING_ANGLE을 "
                  f"{abs(ang):.3f}으로 낮추거나(좌측 권한 손해) 트림 재조정 필요.")
    else:
        print(f"  현재 vesc.yaml servo_min={SERVO_MIN_CFG} (δ={servo_to_angle(SERVO_MIN_CFG):+.4f})")
    print("=" * 60)


if __name__ == "__main__":
    main()
