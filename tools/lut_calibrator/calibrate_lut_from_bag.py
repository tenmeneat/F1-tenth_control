#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_lut_from_bag.py — rosbag → Steering LUT 실측 보정 CSV (오프라인)
================================================================================
실차 rosbag을 넣으면 `lut_calibrator_node`(C++, 온라인 관찰 노드)와 **동일한 수식·
동일한 파일 포맷**으로 보정 LUT CSV를 만들어낸다. ROS 노드를 띄우고 bag을 실시간
재생할 필요가 없다(95초 bag = 95초 대기 → 1초 내 처리).

온라인 노드 대비 이점:
  - bag 전체를 타임스탬프 순으로 결정론적 처리 → DDS 큐 드롭(depth 10) 없음
  - 여러 bag을 한 번에 누적, IMU 토픽명 자동 판별(/imu/data ↔ /sensors/imu/raw)
  - **단위 자동 검증**: IMU deg/s 미보정은 a_lat을 57배로 부풀려 LUT를 조용히
    오염시키는데(주행 중 증상 없음), odom 요레이트와 대조해 사전에 잡아낸다
  - 그리드 커버리지 리포트 → "다음 주행에서 어느 속도/조향 영역을 밟아야 하는지"

`calibration_state.csv` 포맷이 C++ 노드와 100% 호환이라 온라인/오프라인을 섞어
누적해도 된다.

사용법:
  source /opt/ros/<distro>/setup.bash        # 메시지 디시리얼라이즈에 필요 (humble/jazzy 등)
  python3 calibrate_lut_from_bag.py ~/rosbag_log/0725/rosbag_*
  python3 calibrate_lut_from_bag.py <bag> --fresh      # 누적 무시하고 새로 시작

산출물(기본 ~/f1tenth_lut_calibration/):
  NUC6_glc_pacejka_lookup_table_calibrated.csv   ← control_map_node에 넘길 LUT
  calibration_state.csv                          ← 누적 상태(다음 실행에 이어짐)

적용:
  ros2 launch f1tenth_control control_real.launch.py \
      lookup_table_file:=$HOME/f1tenth_lut_calibration/NUC6_glc_pacejka_lookup_table_calibrated.csv
"""
import argparse
import glob
import math
import os
import sqlite3
import sys

DEG2RAD = math.pi / 180.0  # VESC 자이로는 deg/s로 발행(2026-07-19 실차 확인)
WHEELBASE = 0.33           # 단위 검증용 기구학 기대 요레이트 계산에만 쓰임

# 토픽 자동 판별 우선순위. 앞에 있을수록 우선.
IMU_TOPIC_PREFS = ["/imu/data", "/sensors/imu/raw"]
ODOM_TOPIC_PREFS = ["/pf/pose/odom", "/odom"]
DRIVE_TOPIC_PREFS = ["/drive", "/drive_autonomous"]

# 시뮬 bag 판별용. LUT는 실차 sysid 자산이라 시뮬 데이터가 섞이면 안 된다.
SIM_TOPIC_MARKERS = ["/ego_racecar/odom"]


# ══ 1. bag 로딩 ═══════════════════════════════════════════════════════════════
def find_db3(path):
    if path.endswith(".db3"):
        return [path]
    cand = sorted(glob.glob(os.path.join(path, "*.db3")))
    if not cand:
        sys.exit(f"[에러] {path} 에 .db3 파일이 없습니다.")
    return cand


def load_messages(db3_files, wanted_topics):
    """지정 토픽들의 (timestamp, topic_name, deserialized_msg)를 시간순으로 반환.

    분할 bag(여러 .db3)도 하나로 합쳐 정렬한다.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    out = []
    for db3 in db3_files:
        con = sqlite3.connect(db3)
        cur = con.cursor()
        topics = {}
        for tid, name, ttype in cur.execute("SELECT id,name,type FROM topics"):
            if name not in wanted_topics:
                continue
            try:
                topics[tid] = (name, get_message(ttype))
            except Exception as e:
                sys.exit(
                    f"[에러] 메시지 타입 '{ttype}'을 불러올 수 없습니다: {e}\n"
                    "       ROS 2와 워크스페이스를 소싱했는지 확인하세요 "
                    "(source /opt/ros/<distro>/setup.bash; source ~/2026_IFAC/install/setup.bash)"
                )
        if topics:
            placeholders = ",".join("?" * len(topics))
            q = f"SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({placeholders})"
            for tid, ts, data in cur.execute(q, tuple(topics.keys())):
                name, cls = topics[tid]
                out.append((ts, name, deserialize_message(bytes(data), cls)))
        con.close()

    out.sort(key=lambda r: r[0])
    return out


def bag_topic_list(db3_files):
    names = {}
    for db3 in db3_files:
        con = sqlite3.connect(db3)
        for name, ttype in con.execute("SELECT name,type FROM topics"):
            names[name] = ttype
        con.close()
    return names


def pick_topic(available, prefs, kind, override):
    if override:
        if override not in available:
            sys.exit(f"[에러] 지정한 {kind} 토픽 '{override}'이 bag에 없습니다. "
                     f"있는 토픽: {sorted(available)}")
        return override
    for p in prefs:
        if p in available:
            return p
    return None


# ══ 2. 베이스 LUT ═════════════════════════════════════════════════════════════
def _num(cell):
    try:
        return float(cell)
    except ValueError:
        return float("nan")


class BaseLut:
    """steering_lookup_table.hpp와 동일한 CSV 규약: 행=조향각축, 열=속도축.

    축(첫 행·첫 열)은 원본 CSV의 **문자열 그대로** 들고 있다가 그대로 다시 쓴다.
    숫자로 파싱했다가 재포맷하면 0.6015625 같은 정확한 half 값에서 반올림 규약 차이로
    축이 1e-6 흔들린다(C/Python은 half-to-even, JS는 half-up) — 값이 바뀔 이유가
    없는 자리라 아예 원문을 보존한다. 웹앱과 바이트 단위로 같은 파일이 나온다.
    """

    def __init__(self, rows):
        self.corner_raw = rows[0][0]
        self.vel_raw = rows[0][1:]
        self.steer_raw = [r[0] for r in rows[1:]]
        self.vel_axis = [_num(c) for c in self.vel_raw]
        self.steer_axis = [_num(c) for c in self.steer_raw]
        self.grid = [[_num(c) for c in r[1:]] for r in rows[1:]]  # grid[steer_idx][vel_idx]

    @classmethod
    def load(cls, path):
        rows = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(line.split(","))
        if not rows or not rows[0]:
            sys.exit(f"[에러] 베이스 LUT가 비었거나 형식이 잘못됨: {path}")
        return cls(rows)

    def save_blended(self, path, blended):
        """데이터 셀만 %g(= C++ ostream 기본 6자리 유효숫자), 축은 원문 보존."""
        with open(path, "w") as f:
            f.write(",".join([self.corner_raw] + self.vel_raw) + "\n")
            for i, s in enumerate(self.steer_raw):
                f.write(",".join([s] + ["%g" % v for v in blended[i]]) + "\n")


def find_base_lut(explicit):
    if explicit:
        if not os.path.exists(explicit):
            sys.exit(f"[에러] 베이스 LUT를 찾을 수 없음: {explicit}")
        return explicit

    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        # 저장소 원본 (이 도구와 같이 다니는 가장 확실한 경로)
        os.path.join(here, "..", "..", "control_code", "NUC6_glc_pacejka_lookup_table.csv"),
    ]
    # 설치된 ament share (control_map_node의 폴백 순서와 동일)
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        if not prefix:
            continue
        for pkg in ("steering_lookup", "f1tenth_control"):
            cands.append(os.path.join(prefix, "share", pkg, "cfg",
                                      "NUC6_glc_pacejka_lookup_table.csv"))
    for c in cands:
        c = os.path.normpath(c)
        if os.path.exists(c):
            return c
    sys.exit("[에러] 베이스 LUT를 자동으로 찾지 못했습니다. --base-lut 로 지정하세요.")


# ══ 3. 누적 상태 (C++ 노드와 동일 포맷) ═══════════════════════════════════════
def load_state(path, n_steer, n_vel):
    """이전 누적(sum/count)을 읽는다. 없거나 grid 크기가 다르면 0으로 시작."""
    if not os.path.exists(path):
        return None
    sums, counts, in_count = [], [], False
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if "COUNT" in line:
                    in_count = True
                continue
            vals = line.split(",")
            if in_count:
                counts.append([int(float(v)) for v in vals])
            else:
                sums.append([float(v) for v in vals])
    if len(sums) != n_steer or len(counts) != n_steer:
        print(f"  ⚠️  상태 파일 grid 크기가 베이스 LUT와 달라 무시합니다: {path}")
        return None
    if sums and (len(sums[0]) != n_vel or len(counts[0]) != n_vel):
        print(f"  ⚠️  상태 파일 grid 크기가 베이스 LUT와 달라 무시합니다: {path}")
        return None
    return sums, counts


def save_state(path, sums, counts):
    with open(path, "w") as f:
        f.write("# lut_calibration_state v1\n")
        for row in sums:
            f.write(",".join("%.15e" % v for v in row) + "\n")
        f.write("#COUNT\n")
        for row in counts:
            f.write(",".join(str(c) for c in row) + "\n")


# ══ 4. 캘리브레이션 코어 (lut_calibrator_node.cpp와 동일 수식) ════════════════
def nearest_index(axis, val):
    best, best_diff = 0, float("inf")
    for i, a in enumerate(axis):
        d = abs(a - val)
        if d < best_diff:
            best_diff, best = d, i
    return best


def process_bag(msgs, topics, lut, sums, counts, args):
    """IMU 콜백 시점마다 캐시된 speed/steering과 묶어 그리드에 비닝한다.

    C++ 노드의 콜백 구조를 그대로 재현: LPF는 모든 IMU 메시지에 적용되고(속도 게이트
    이전), 샘플 기록만 min_speed 이상에서 일어난다.
    """
    imu_t, odom_t, drive_t = topics["imu"], topics["odom"], topics["drive"]

    speed = 0.0
    steer = 0.0
    yaw_rate = 0.0
    yaw_init = False
    n_samples = 0
    # 단위 검증용: IMU 실측 요레이트 vs 조향 기구학 기대치(v·tanδ/L, rad/s).
    # odom의 angular.z를 쓰지 않는 이유는 /pf/pose/odom엔 그 필드가 0으로 비어 있어서다.
    imu_sq = kin_sq = 0.0
    unit_n = 0

    for _ts, name, msg in msgs:
        if name == odom_t:
            speed = msg.twist.twist.linear.x
        elif name == drive_t:
            steer = msg.drive.steering_angle
        elif name == imu_t:
            raw = msg.angular_velocity.z * args.imu_angular_scale
            if not yaw_init:
                yaw_rate, yaw_init = raw, True
            else:
                yaw_rate = args.alpha * raw + (1.0 - args.alpha) * yaw_rate

            if speed < args.min_speed:
                continue

            lat_accel = speed * yaw_rate
            si = nearest_index(lut.steer_axis, abs(steer))
            vi = nearest_index(lut.vel_axis, speed)
            sums[si][vi] += abs(lat_accel)
            counts[si][vi] += 1
            n_samples += 1

            kin = speed * math.tan(steer) / WHEELBASE
            imu_sq += yaw_rate * yaw_rate
            kin_sq += kin * kin
            unit_n += 1

    imu_rms = math.sqrt(imu_sq / unit_n) if unit_n else 0.0
    kin_rms = math.sqrt(kin_sq / unit_n) if unit_n else 0.0
    return n_samples, imu_rms, kin_rms


def blend(lut, sums, counts, prior_weight):
    """베이지안 블렌딩: 샘플 적은 셀은 원본(prior)에 가깝게 남는다."""
    out = []
    for i, base_row in enumerate(lut.grid):
        row = []
        for j, base_val in enumerate(base_row):
            c = counts[i][j]
            if c <= 0 or math.isnan(base_val):
                row.append(base_val)
            else:
                row.append((base_val * prior_weight + sums[i][j]) / (prior_weight + c))
        out.append(row)
    return out


# ══ 5. 리포트 ═════════════════════════════════════════════════════════════════
def coverage_report(lut, counts):
    n_steer, n_vel = len(lut.steer_axis), len(lut.vel_axis)
    filled = sum(1 for row in counts for c in row if c > 0)
    total = n_steer * n_vel
    print(f"\n  그리드 커버리지: {filled}/{total} 셀 ({100.0 * filled / total:.1f}%)")

    # 샘플이 실제로 들어온 (조향, 속도) 범위 — "다음에 뭘 더 밟아야 하나"의 근거
    s_idx = [i for i in range(n_steer) if any(counts[i][j] > 0 for j in range(n_vel))]
    v_idx = [j for j in range(n_vel) if any(counts[i][j] > 0 for i in range(n_steer))]
    if not s_idx:
        print("  ⚠️  샘플이 하나도 기록되지 않았습니다.")
        return
    print(f"  샘플 조향각 범위: {lut.steer_axis[min(s_idx)]:.3f} ~ "
          f"{lut.steer_axis[max(s_idx)]:.3f} rad (물리한계 0.41)")
    print(f"  샘플 속도 범위  : {lut.vel_axis[min(v_idx)]:.2f} ~ "
          f"{lut.vel_axis[max(v_idx)]:.2f} m/s (축 최대 {lut.vel_axis[-1]:.1f})")

    # ASCII 히트맵 (가로=속도, 세로=조향). 축이 60x65라 그대로 찍으면 안 읽혀서 다운샘플.
    print("\n  커버리지 맵 (가로→속도, 세로↓조향, ' '=없음 .:*# 순으로 샘플 많음)")
    ROWS, COLS = 16, 64
    print("      " + "속도 " + f"{lut.vel_axis[0]:.1f}" + " " * (COLS - 12) + f"{lut.vel_axis[-1]:.1f}")
    for r in range(ROWS):
        i0, i1 = r * n_steer // ROWS, max(r * n_steer // ROWS + 1, (r + 1) * n_steer // ROWS)
        line = ""
        for c in range(COLS):
            j0, j1 = c * n_vel // COLS, max(c * n_vel // COLS + 1, (c + 1) * n_vel // COLS)
            tot = sum(counts[i][j] for i in range(i0, i1) for j in range(j0, j1))
            line += " " if tot == 0 else ("." if tot < 5 else (":" if tot < 20 else ("*" if tot < 100 else "#")))
        print(f"  {lut.steer_axis[i0]:5.3f} |{line}|")


def check_units(imu_rms, kin_rms, scale):
    """IMU 각속도 단위 오보정 사전 감지.

    조향 기구학 기대 요레이트(v·tanδ/L)는 정의상 rad/s다. 실측 IMU와 정확히 같진
    않지만(슬립·지연) 크기 스케일은 같아야 한다. 57배쯤 어긋나면 deg/s→rad/s 보정이
    빠졌거나 이중 적용된 것 — 이 경우 a_lat이 통째로 틀려 LUT가 조용히 오염된다.
    """
    print(f"\n  [단위 검증] IMU 요레이트 RMS {imu_rms:.3f} rad/s vs "
          f"조향 기구학 기대치 RMS {kin_rms:.3f} rad/s")
    if kin_rms < 1e-6 or imu_rms < 1e-6:
        print("  ⚠️  한쪽이 0에 가까워 단위 검증 불가 — imu_angular_scale을 직접 확인하세요.")
        return
    ratio = imu_rms / kin_rms
    if 0.1 <= ratio <= 10.0:
        print(f"  ✅ 비율 {ratio:.2f}배 — 정상 범위(슬립/지연으로 1.0에서 다소 벗어남).")
        return

    # scale/ratio를 그대로 쓰면 "슬립이 0"이라는 가정이 섞여 헛되이 정밀한 값이 나온다.
    # 이 하드웨어에서 물리적으로 가능한 값은 둘뿐이므로 그중 더 잘 맞는 쪽을 제시한다.
    candidates = [(1.0, "이미 rad/s인 데이터"), (DEG2RAD, "π/180 — VESC의 deg/s 발행분 보정")]
    best, label = min(candidates,
                      key=lambda c: abs(math.log(ratio * c[0] / scale)))
    print(f"  ❌ 비율 {ratio:.1f}배 — imu_angular_scale {scale:.7g} 가 잘못됐습니다. "
          f"a_lat이 그만큼 틀어져 LUT가 오염됩니다.")
    print(f"     → --imu-angular-scale {best:.7g} 로 다시 실행하세요 ({label}). (지금 결과는 폐기)")


# ══ 6. main ═══════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(
        description="rosbag에서 Steering LUT를 실측 보정해 CSV로 출력",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", help="rosbag 폴더 또는 .db3 (여러 개 가능 — 순서대로 누적)")
    ap.add_argument("--base-lut", default=None, help="베이스 LUT CSV (기본: 자동탐색)")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/f1tenth_lut_calibration"))
    ap.add_argument("--fresh", action="store_true", help="이전 누적 상태를 무시하고 새로 시작")
    ap.add_argument("--min-speed", type=float, default=1.0, help="샘플 기록 최소 속도 [m/s]")
    ap.add_argument("--prior-weight", type=float, default=3.0, help="원본 LUT 가중(클수록 보수적)")
    ap.add_argument("--alpha", type=float, default=0.3, help="요레이트 LPF 계수")
    ap.add_argument("--imu-angular-scale", type=float, default=DEG2RAD,
                    help=f"IMU 각속도 단위 보정 (기본 {DEG2RAD:.7f} = deg/s→rad/s, 실차 VESC)")
    ap.add_argument("--imu-topic", default=None)
    ap.add_argument("--odom-topic", default=None)
    ap.add_argument("--drive-topic", default=None)
    ap.add_argument("--allow-sim", action="store_true",
                    help="시뮬 bag 차단 해제 (LUT는 실차 sysid 자산이라 기본 차단)")
    args = ap.parse_args()

    lut_path = find_base_lut(args.base_lut)
    lut = BaseLut.load(lut_path)
    n_steer, n_vel = len(lut.steer_axis), len(lut.vel_axis)
    print(f"베이스 LUT: {lut_path}  (grid {n_steer} steer x {n_vel} vel)")

    os.makedirs(args.out_dir, exist_ok=True)
    state_path = os.path.join(args.out_dir, "calibration_state.csv")
    out_path = os.path.join(args.out_dir, "NUC6_glc_pacejka_lookup_table_calibrated.csv")

    prior = None if args.fresh else load_state(state_path, n_steer, n_vel)
    if prior:
        sums, counts = prior
        print(f"🟢 이전 누적 상태 로드: {sum(sum(r) for r in counts)}개 샘플 "
              f"({state_path}) — 이어서 누적합니다.")
    else:
        sums = [[0.0] * n_vel for _ in range(n_steer)]
        counts = [[0] * n_vel for _ in range(n_steer)]

    grand_total = 0
    imu_sq_all = kin_sq_all = 0.0
    n_all = 0

    for bag in args.bags:
        db3s = find_db3(bag)
        available = bag_topic_list(db3s)

        sim_hits = [t for t in SIM_TOPIC_MARKERS if t in available]
        if sim_hits and not args.allow_sim:
            print(f"\n❌ {os.path.basename(bag.rstrip('/'))}: 시뮬 토픽 {sim_hits} 발견 — 건너뜁니다.\n"
                  "   LUT는 실차 sysid 자산이라 시뮬 데이터로 오염되면 안 됩니다 (--allow-sim 으로 강제 가능)")
            continue

        topics = {
            "imu": pick_topic(available, IMU_TOPIC_PREFS, "IMU", args.imu_topic),
            "odom": pick_topic(available, ODOM_TOPIC_PREFS, "odom", args.odom_topic),
            "drive": pick_topic(available, DRIVE_TOPIC_PREFS, "drive", args.drive_topic),
        }
        label = os.path.basename(bag.rstrip("/"))
        missing = [k for k, v in topics.items() if v is None]
        if missing:
            print(f"\n⏭️  {label}: 필수 토픽 없음 {missing} — 건너뜁니다.")
            continue

        print(f"\n▶ {label}")
        print(f"   토픽: imu={topics['imu']}  odom={topics['odom']}  drive={topics['drive']}")

        msgs = load_messages(db3s, set(topics.values()))
        if not msgs:
            print("   ⏭️  해당 토픽 메시지가 0개 — 건너뜁니다.")
            continue

        n, imu_rms, kin_rms = process_bag(msgs, topics, lut, sums, counts, args)
        print(f"   기록 샘플: {n}개 (메시지 {len(msgs)}개 처리)")
        grand_total += n
        if n:
            imu_sq_all += imu_rms * imu_rms * n
            kin_sq_all += kin_rms * kin_rms * n
            n_all += n

    if grand_total == 0:
        print("\n[에러] 기록된 샘플이 없습니다. --min-speed 를 낮추거나 토픽을 확인하세요.")
        sys.exit(1)

    check_units(math.sqrt(imu_sq_all / n_all), math.sqrt(kin_sq_all / n_all),
                args.imu_angular_scale)

    save_state(state_path, sums, counts)
    lut.save_blended(out_path, blend(lut, sums, counts, args.prior_weight))

    total_acc = sum(sum(r) for r in counts)
    print(f"\n✅ 이번 실행 {grand_total}개 / 누적 {total_acc}개 샘플")
    coverage_report(lut, counts)
    print(f"\n  상태 : {state_path}")
    print(f"  LUT  : {out_path}")
    print("\n  적용: ros2 launch f1tenth_control control_real.launch.py \\")
    print(f"           lookup_table_file:={out_path}")


if __name__ == "__main__":
    main()
