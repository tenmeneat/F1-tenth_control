#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sector_pub.py — 섹터 스케일 테이블 발행기 (control_map_node의 sector_scale_* 짝)
================================================================================
`analyze_sector_clearance.py`(또는 bag_analyzer 웹앱)가 뽑아준 `sectors.yaml`을 읽어
`/sector_scales`(std_msgs/Float32MultiArray, transient_local)로 발행한다.

메시지 레이아웃: [track_length, s_start, s_end, scale, s_start, s_end, scale, ...]

사용법:
  source /opt/ros/<distro>/setup.bash && source ~/2026_IFAC/install/setup.bash
  python3 tools/sector_pub.py sectors.yaml            # latch 발행 후 대기
  python3 tools/sector_pub.py sectors.yaml --watch    # 파일 저장할 때마다 재발행(라이브 튜닝)
  python3 tools/sector_pub.py sectors.yaml --dry-run  # 검증만 하고 안 쏨

🔑 컨트롤러가 거는 검증을 **여기서 먼저** 똑같이 건다. 차 옆에서 YAML을 고치다 오타를 내면
   컨트롤러는 조용히 테이블 전체를 버리고 1.0으로 돌아가는데(설계상 안전한 동작),
   그걸 "켰는데 왜 안 빨라지지"로 헤매기 쉽다. 발행 전에 여기서 잡아 이유를 찍는다.

⚠️ scale < 1.0은 **금지**다. 전역 max_lateral_accel을 보수값으로 두고 여기서 특정 코너만
   여는 설계라, 1.0 미만을 허용하면 "테이블을 잃으면 위험 코너가 빨라지는" 구조가 된다.
   느리게 하고 싶으면 전역 max_lateral_accel을 낮출 것.

⚠️ 랩을 넘는 구간(s_start > s_end)은 **두 개로 쪼개서** 쓴다. 컨트롤러는 값이 실제로 바뀌는
   전이점에만 블렌딩을 걸므로, 쪼개도 결승선에서 파이지 않는다(2026-08-11 런타임 검증).
"""
import sys, os, time, argparse

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)


def load_and_validate(path: str, scale_max: float):
    """YAML → (track_length, [(s0, s1, scale)]). 컨트롤러와 같은 규칙으로 거른다."""
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}

    track_len = doc.get("track_length")
    if not isinstance(track_len, (int, float)) or track_len <= 1.0:
        raise ValueError(f"track_length가 없거나 비정상: {track_len!r}. "
                         "이 값이 컨트롤러의 라인 동일성 검증 기준이라 반드시 있어야 한다")
    track_len = float(track_len)

    raw = doc.get("sectors") or []
    if not raw:
        raise ValueError("sectors가 비어 있다")

    out = []
    for i, sec in enumerate(raw):
        try:
            s0, s1, sc = float(sec["s_start"]), float(sec["s_end"]), float(sec["scale"])
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"{i}번 항목에 s_start/s_end/scale이 없거나 숫자가 아님: {sec!r}") from e
        if s1 <= s0:
            raise ValueError(
                f"{i}번 항목 s_start {s0:.2f} >= s_end {s1:.2f}. "
                "랩을 넘는 구간이면 두 개로 쪼갤 것 "
                f"(예: {{s_start: {s0:.2f}, s_end: {track_len:.2f}}} 와 {{s_start: 0.0, s_end: {s1:.2f}}})")
        if s0 < 0.0 or s1 > track_len + 1e-6:
            raise ValueError(f"{i}번 항목 s 구간 [{s0:.2f}, {s1:.2f}]이 랩 길이 {track_len:.2f} 밖")
        if sc < 1.0:
            raise ValueError(
                f"{i}번 항목 scale {sc:.2f} < 1.0 — 설계상 금지다. 전역 max_lateral_accel을 "
                "보수값으로 두고 여기서 여는 구조라, 1.0 미만을 쓰면 테이블을 잃었을 때 "
                "위험 코너가 빨라지는 방향이 된다. 느리게 하려면 전역 MLA를 낮출 것")
        if sc > scale_max:
            raise ValueError(f"{i}번 항목 scale {sc:.2f} > 허용 최대 {scale_max:.2f} "
                             "(컨트롤러 sector_scale_max와 맞출 것)")
        out.append((s0, s1, sc))

    # 겹침 검사 — 컨트롤러는 겹치면 큰 쪽을 쓰지만, 의도한 적이 없을 테니 여기서 막는다.
    for a in range(len(out)):
        for b in range(a + 1, len(out)):
            if out[a][0] < out[b][1] and out[b][0] < out[a][1]:
                raise ValueError(f"{a}번과 {b}번 구간이 겹친다: "
                                 f"[{out[a][0]:.2f},{out[a][1]:.2f}] vs [{out[b][0]:.2f},{out[b][1]:.2f}]")

    # 전이점 간격 < 블렌딩 폭이면 램프가 서로 겹쳐 의도한 값에 못 닿는다.
    edges = sorted([s for sec in out for s in sec[:2]])
    tight = [(edges[i], edges[i + 1]) for i in range(len(edges) - 1)
             if 1e-9 < edges[i + 1] - edges[i] < 0.6]
    if tight:
        print(f"[WARN] 전이점 간격이 좁은 곳이 있다 {tight} — 블렌딩 폭(기본 0.5 m)보다 좁으면 "
              "램프가 겹쳐 그 구간이 목표 scale에 못 닿는다. 경계를 벌리거나 "
              "sector_scale_blend를 줄일 것", file=sys.stderr)
    return track_len, out


def to_msg(track_len, sectors):
    m = Float32MultiArray()
    # ⚠️ rclpy에서 .data는 array.array('f')다 — 리스트를 += 하면 TypeError가 난다.
    #    평범한 리스트를 다 만들어서 **한 번에 대입**해야 한다.
    flat = [float(track_len)]
    for s0, s1, sc in sectors:
        flat.extend((float(s0), float(s1), float(sc)))
    m.data = flat
    return m


def resolve_yaml_path(raw_path: str) -> str:
    if raw_path:
        p = os.path.expanduser(raw_path)
        if os.path.isfile(p):
            return p

    candidates = [
        os.path.expanduser("~/2026_IFAC/src/f1tenth_control/config/sectors.yaml"),
        os.path.expanduser("~/F1tenth_control/config/sectors.yaml"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory("f1tenth_control"), "config", "sectors.yaml"))
    except Exception:
        pass

    for c in candidates:
        if os.path.isfile(c):
            return c

    if raw_path:
        return os.path.expanduser(raw_path)
    return candidates[0]


def main() -> int:
    ap = argparse.ArgumentParser(description="섹터 스케일 테이블 발행")
    ap.add_argument("yaml_path", nargs="?", default="", help="sectors.yaml 경로 (생략 시 자동 탐색)")
    ap.add_argument("--topic", default="/sector_scales")
    ap.add_argument("--scale-max", type=float, default=1.5,
                    help="컨트롤러 sector_scale_max와 같은 값 (기본 1.5)")
    ap.add_argument("--watch", action="store_true",
                    help="파일이 바뀌면 자동 재발행 (랩 사이 라이브 튜닝용)")
    ap.add_argument("--dry-run", action="store_true", help="검증만 하고 발행하지 않음")

    # ROS 2 런치에서 띄웠을 때 --ros-args 제거
    clean_argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    args = ap.parse_args(clean_argv)

    path = resolve_yaml_path(args.yaml_path)
    track_len, sectors = load_and_validate(path, args.scale_max)
    print(f"랩 길이 {track_len:.2f} m / 섹터 {len(sectors)}개")
    for s0, s1, sc in sectors:
        print(f"  s {s0:6.2f} ~ {s1:6.2f}  ({s1 - s0:5.2f} m)  ×{sc:.2f}")
    if args.dry_run:
        print("(--dry-run: 발행하지 않음)")
        return 0

    rclpy.init()
    node = Node("sector_pub")
    pub = node.create_publisher(Float32MultiArray, args.topic, LATCHED)
    pub.publish(to_msg(track_len, sectors))
    print(f"→ {args.topic} 발행 (transient_local). "
          f"컨트롤러 로그에서 '섹터 테이블 갱신' 과 '라인 검증 통과'를 확인할 것")

    try:
        if not args.watch:
            # latch라 프로세스가 살아 있어야 늦게 뜬 구독자도 받는다.
            print("Ctrl-C로 종료 (종료하면 latch도 사라진다 — 주행 중엔 켜 둘 것)")
            rclpy.spin(node)
        else:
            mtime = os.path.getmtime(path)
            print(f"--watch: {path} 변경 감시 중")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                try:
                    now = os.path.getmtime(path)
                except OSError:
                    continue
                if now == mtime:
                    continue
                mtime = now
                time.sleep(0.1)   # 에디터가 쓰기를 끝낼 시간
                try:
                    tl, secs = load_and_validate(path, args.scale_max)
                except (ValueError, yaml.YAMLError, OSError) as e:
                    # ⚠️ 실패해도 직전 테이블을 그대로 둔다(재발행 안 함). 컨트롤러 쪽도
                    #    같은 원칙이라, 오타 한 번에 주행 중 스케일이 통째로 날아가지 않는다.
                    print(f"[ERROR] 재로드 실패, 직전 테이블 유지: {e}", file=sys.stderr)
                    continue
                pub.publish(to_msg(tl, secs))
                print(f"[{time.strftime('%H:%M:%S')}] 재발행 — 섹터 {len(secs)}개, "
                      + ", ".join(f"×{s[2]:.2f}" for s in secs))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C / SIGTERM은 정상 종료다 — 스택트레이스를 띄우지 않는다
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, yaml.YAMLError, OSError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
