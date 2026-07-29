#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_bag.py — F1TENTH 주행 rosbag 통합 분석 도구
================================================================
하나의 rosbag(폴더 또는 .db3)을 넣으면 자체 실행형 report.html 하나를 만든다.
  1) 3D 주행 리플레이 (RViz 스타일, 순수 JS·의존성 0 — 브라우저에서 바로 열림)
  2) 주행상태 분석 그래프 (matplotlib, 팀원 분석 사진과 동일 계열)
  3) 자동 튜닝 가이드 (a_lat 그립초과·감속권한·조향포화 휴리스틱 → 권장 파라미터)

사용법:
  source /opt/ros/<distro>/setup.bash      # rclpy 디시리얼라이즈에 필요 (humble/jazzy 등)
  python3 analyze_bag.py <bag_dir | .db3> [--out report.html]
         [--odom /odom] [--grip 5.0]

전제: ROS 2 소싱, matplotlib, numpy. (bag에 /global_waypoints·/drive 있으면 명령 vs
      실측 비교까지, 없으면 실측 기반 분석만 — 둘 다 자동 판별)
"""
import os, sys, glob, sqlite3, math, json, base64, io, argparse, bisect

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def setup_font():
    """matplotlib 한글 폰트: TTF/TTC 파일을 직접 등록(이름만으론 미인식되는 케이스 회피)."""
    import glob as _glob
    import matplotlib.font_manager as fm
    cands = (["/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
              "/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf"]
             + _glob.glob("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
             + _glob.glob("/usr/share/fonts/**/*Nanum*.ttf", recursive=True))
    for path in cands:
        if not os.path.exists(path):
            continue
        try:
            fm.fontManager.addfont(path)
            name = fm.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False

# ── 차량/물리 상수 (control_map_node와 동일 기준) ────────────────────────────
WHEELBASE = 0.33
MAX_STEER = 0.41
CAR_W, CAR_L = 0.31, 0.58
R_MIN = WHEELBASE / math.tan(MAX_STEER)          # 최소 선회반경 ≈ 0.76 m
DEG2RAD = math.pi / 180.0                          # VESC 자이로 deg/s → rad/s
# 실측 요레이트 출처 라벨 (derive_measured_yaw_rate 참고)
SRC_LABEL = {"imu": "IMU", "mcl": "MCL pose", "odom_kinematic": "⚠️명령값", "none": "없음"}


# ══ 1. bag 로딩 ══════════════════════════════════════════════════════════════
def find_db3(path):
    if path.endswith(".db3"):
        return path
    cand = glob.glob(os.path.join(path, "*.db3"))
    if not cand:
        sys.exit(f"[에러] {path} 에 .db3 파일이 없습니다.")
    return sorted(cand)[0]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_bag(db3):
    con = sqlite3.connect(db3)
    cur = con.cursor()
    topics = {tid: (n, t) for tid, n, t in cur.execute("SELECT id,name,type FROM topics")}
    cls = {}
    for tid, (n, t) in topics.items():
        try:
            cls[tid] = get_message(t)
        except Exception:
            cls[tid] = None
    data = {}
    t0 = None
    for tid, ts, blob in cur.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
        if t0 is None:
            t0 = ts
        name = topics[tid][0]
        if cls[tid] is None:
            continue
        try:
            m = deserialize_message(blob, cls[tid])
        except Exception:
            continue
        data.setdefault(name, []).append(((ts - t0) / 1e9, m))
    con.close()
    return data


# ══ 2. TF 합성 (map→odom→base_link) 으로 map 프레임 pose 복원 ════════════════
class TF:
    def __init__(self, tf_msgs):
        self.seq = {}   # (parent,child) -> [(t,x,y,yaw)]
        for t, m in tf_msgs:
            for tr in m.transforms:
                k = (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))
                tx = tr.transform.translation
                self.seq.setdefault(k, []).append((t, tx.x, tx.y, yaw_of(tr.transform.rotation)))
        self.ts = {k: [s[0] for s in v] for k, v in self.seq.items()}

    def has(self, parent, child):
        return (parent, child) in self.seq

    def _at(self, k, t):
        ts = self.ts[k]
        i = bisect.bisect_right(ts, t) - 1
        if i < 0:
            i = 0
        return self.seq[k][i]

    def map_pose(self, t):
        """map 프레임에서 base_link pose. map→odom, odom→base_link 합성."""
        if self.has("map", "odom") and self.has("odom", "base_link"):
            _, ox, oy, oth = self._at(("map", "odom"), t)
            _, bx, by, bth = self._at(("odom", "base_link"), t)
            x = ox + math.cos(oth) * bx - math.sin(oth) * by
            y = oy + math.sin(oth) * bx + math.cos(oth) * by
            return x, y, oth + bth
        if self.has("odom", "base_link"):
            _, bx, by, bth = self._at(("odom", "base_link"), t)
            return bx, by, bth
        return None


# ══ 3. 시리즈 추출 ═══════════════════════════════════════════════════════════
def build_series(data, odom_topic):
    S = {}
    # odom (속도·요레이트: body frame)
    od = data.get(odom_topic) or data.get("/odom") or data.get("/pf/pose/odom") or []
    S["odom_t"] = np.array([t for t, _ in od])
    S["vx"] = np.array([m.twist.twist.linear.x for _, m in od])
    # ⚠️ 실차 /odom(vesc_to_odom)의 angular.z는 **측정값이 아니라** 조향 명령으로 만든
    #    기구학 합성값 v·tan(δ)/L 이다(2026-07-25 확정: 22_08_50 bag t=9.5에서 합성 5.30 vs
    #    odom 5.29 완전 일치). 따라서 vx·wz는 "명령 횡가속도"이고, 이걸 실측 a_lat으로 쓰면
    #    언더스티어 상황에서 3배까지 과장된다(실측 7 → 표시 21.5). 아래에서 IMU/MCL 기준의
    #    실측 요레이트를 따로 만들고, 이 값은 "명령"으로만 쓴다.
    S["wz_cmd"] = np.array([m.twist.twist.angular.z for _, m in od])
    S["odom_xy"] = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in od]) if od else np.zeros((0, 2))

    # map 프레임 pose (TF 합성). 없으면 odom pose 사용.
    tf = TF(data.get("/tf", []))
    pose = []
    for t, _ in od:
        p = tf.map_pose(t)
        pose.append(p if p else None)
    if any(p is None for p in pose) and len(S["odom_xy"]):
        for i, p in enumerate(pose):
            if p is None:
                x, y = S["odom_xy"][i]
                pose[i] = (x, y, 0.0)
    S["pose"] = np.array([[p[0], p[1], p[2]] for p in pose]) if pose and pose[0] else np.zeros((0, 3))
    S["tf"] = tf
    S["has_map"] = tf.has("map", "odom")

    # 파생: dvx/dt (평활)
    # ⚠️ bag에는 같은 타임스탬프의 odom이 종종 두 번 들어온다(dt=0). 예전엔 dt=0을 1e-3으로
    #    치환했는데, 50Hz(dt=0.02) 데이터에서 이건 20배 뻥튀기라 dvx가 ±900 m/s²까지 튀었다.
    #    그 스파이크가 아래 diagnose()의 사고 onset 판정(|dvx|>15)을 0초에 걸리게 만들어
    #    "감속 권한" 분석 구간이 통째로 비었다. → 중앙값 dt의 절반을 하한으로 클램프한다.
    if len(S["vx"]):
        dt = np.gradient(S["odom_t"]) if len(S["odom_t"]) > 1 else np.array([1.0])
        dt_floor = max(1e-3, 0.5 * float(np.median(dt[dt > 0]))) if np.any(dt > 0) else 1e-3
        dvx = np.gradient(S["vx"]) / np.maximum(dt, dt_floor)
        # 3-tap 평활
        if len(dvx) >= 3:
            dvx = np.convolve(dvx, np.ones(3) / 3, mode="same")
        S["dvx"] = dvx
    else:
        S["dvx"] = np.zeros(0)

    # IMU
    imu = data.get("/imu/data") or data.get("/sensors/imu/raw") or []
    S["imu_t"] = np.array([t for t, _ in imu])
    S["gyro_z_raw"] = np.array([m.angular_velocity.z for _, m in imu])      # 원단위(deg/s로 추정)
    S["gyro_z_rad"] = S["gyro_z_raw"] * DEG2RAD                              # rad/s 환산
    S["acc_x"] = np.array([m.linear_acceleration.x for _, m in imu])
    S["acc_y"] = np.array([m.linear_acceleration.y for _, m in imu])

    # ── 실측 요레이트 & 진짜 a_lat (2026-07-25) ──────────────────────────────
    # 우선순위: ① IMU 자이로(rad/s 환산) ② MCL/TF pose yaw 미분 ③ (최후) odom 기구학값.
    # ①과 ②는 서로 독립인데 07-25 bag에서 1.4~1.9 rad/s로 일치했다 — 교차검증 통과.
    # ③으로 폴백하면 a_lat이 "명령"이므로 리포트에 그대로 경고를 띄운다.
    S["wz_meas"], S["alat_source"] = derive_measured_yaw_rate(S)
    S["alat"] = S["vx"] * S["wz_meas"] if len(S["vx"]) else np.zeros(0)
    S["alat_cmd"] = S["vx"] * S["wz_cmd"] if len(S["vx"]) else np.zeros(0)
    # 요레이트 추종률 = |실측| / |명령|. 1.0이면 차가 명령대로 돎, <1이면 덜 돎(언더스티어).
    # 그립 한참 아래(저a_lat)에서도 1보다 낮으면 슬립이 아니라 **조향 게인 캘리브레이션 오차**다.
    with np.errstate(divide="ignore", invalid="ignore"):
        S["yaw_track"] = np.where(np.abs(S["wz_cmd"]) > 0.3,
                                  np.abs(S["wz_meas"]) / np.maximum(np.abs(S["wz_cmd"]), 1e-6),
                                  np.nan)

    # estop / brake / joy
    S["estop"] = [(t, bool(m.data)) for t, m in data.get("/estop_lock", [])]
    S["brake"] = [(t, float(m.data)) for t, m in data.get("/commands/motor/brake", [])]
    joy = data.get("/joy", [])
    S["joy"] = [(t, list(m.axes), list(m.buttons)) for t, m in joy]

    # (옵션) /drive 명령 — 있으면 명령 조향·속도
    drv = data.get("/drive") or data.get("/drive_autonomous") or []
    if drv:
        S["cmd_t"] = np.array([t for t, _ in drv])
        S["cmd_speed"] = np.array([m.drive.speed for _, m in drv])
        S["cmd_steer"] = np.array([m.drive.steering_angle for _, m in drv])
    else:
        S["cmd_t"] = S["cmd_speed"] = S["cmd_steer"] = np.zeros(0)

    # (옵션) /global_waypoints
    wp = data.get("/global_waypoints") or data.get("/local_waypoints") or []
    if wp:
        arr = wp[-1][1]  # 최신
        pts = getattr(arr, "wpnts", None) or getattr(arr, "waypoints", None) or []
        S["wp"] = np.array([[w.x_m, w.y_m, getattr(w, "vx_mps", 0.0), getattr(w, "kappa_radpm", 0.0)] for w in pts])
    else:
        S["wp"] = np.zeros((0, 4))

    # LiDAR (map 프레임으로 변환한 프레임 목록, 애니메이션용 — 최대 ~140프레임)
    scans = data.get("/scan", [])
    S["scan_frames"] = build_scan_frames(scans, tf, max_frames=140, stride_pts=6)
    return S


def derive_measured_yaw_rate(S):
    """odom 시간축 위의 **실측** 요레이트 [rad/s]와 그 출처 문자열을 돌려준다.

    실차 /odom의 angular.z는 조향 명령의 기구학 합성값이라 실측이 아니다(build_series 주석 참고).
    ① IMU 자이로 z → ② map 프레임 pose yaw 미분(MCL) → ③ odom 기구학값 순으로 폴백한다.
    """
    t = S["odom_t"]
    if not len(t):
        return np.zeros(0), "none"

    # ① IMU 자이로 (가장 직접적인 측정)
    if len(S["imu_t"]) >= 5:
        return np.interp(t, S["imu_t"], S["gyro_z_rad"]), "imu"

    # ② map pose yaw 미분 (MCL 스캔매칭 — 휠/조향과 완전히 독립)
    pose = S.get("pose")
    if pose is not None and len(pose) == len(t) and len(t) >= 5 and S.get("has_map"):
        yaw_unwrapped = np.unwrap(pose[:, 2])
        dt = np.gradient(t)
        wz = np.gradient(yaw_unwrapped) / np.where(dt == 0, 1e-3, dt)
        # MCL 보정 점프가 스파이크로 들어오므로 5-tap 평활
        if len(wz) >= 5:
            wz = np.convolve(wz, np.ones(5) / 5, mode="same")
        return wz, "mcl"

    # ③ 폴백: 명령 기구학값. 실측이 아니므로 호출부가 경고를 띄운다.
    return S["wz_cmd"].copy(), "odom_kinematic"


def build_scan_frames(scans, tf, max_frames=140, stride_pts=6):
    if not scans:
        return []
    step = max(1, len(scans) // max_frames)
    frames = []
    LASER_FWD = 0.27  # base_link→laser 대략 전방 오프셋(정적 TF 미기록 → 근사)
    for idx in range(0, len(scans), step):
        t, m = scans[idx]
        p = tf.map_pose(t)
        if p is None:
            continue
        px, py, pyaw = p
        amin, ainc = m.angle_min, m.angle_increment
        rmax = m.range_max
        pts = []
        rng = m.ranges
        lx0 = px + math.cos(pyaw) * LASER_FWD
        ly0 = py + math.sin(pyaw) * LASER_FWD
        for i in range(0, len(rng), stride_pts):
            r = rng[i]
            if not (0.05 < r < min(rmax, 15.0)) or math.isinf(r) or math.isnan(r):
                continue
            a = amin + i * ainc + pyaw
            pts.append(round(lx0 + r * math.cos(a), 3))
            pts.append(round(ly0 + r * math.sin(a), 3))
        frames.append({"t": round(t, 3), "pts": pts})
    return frames


# ══ 4. 자동 튜닝 휴리스틱 ════════════════════════════════════════════════════
def diagnose(S, grip_target):
    t = S["odom_t"]
    # wz는 **실측**(IMU/MCL). 명령 기구학값은 wz_cmd — 둘을 섞지 말 것(2026-07-25).
    vx, wz, alat, dvx = S["vx"], S["wz_meas"], S["alat"], S["dvx"]
    wz_cmd = S["wz_cmd"]
    findings = []
    rec = {}
    if len(vx) < 5:
        return [{"level": "info", "title": "데이터 부족", "body": "odom 샘플이 너무 적어 분석 불가."}], {}

    # 주행 윈도우: estop=False 구간
    def is_driving(tt):
        st = False
        for et, ev in S["estop"]:
            if et <= tt:
                st = not ev  # estop True면 주행아님
        return st
    drive_mask = np.array([is_driving(tt) and vx[i] > 1.0 for i, tt in enumerate(t)])

    peak_v = float(np.max(vx))
    peak_alat = float(np.max(np.abs(alat)))
    over = int(np.sum(np.abs(alat) > grip_target))
    over_frac = over / len(alat)

    # ── 1) 감속 권한 : 사고(첫 그립붕괴/충돌) '이전' 구간에서만 판정 ──
    # 벽 충돌·E-stop 후의 급감속은 '진짜 브레이크'가 아니므로 제외해야 오판을 막는다.
    # 제어상실 onset: ① 비물리 a_lat/급감속 ② **언더스티어 워시아웃 지속**. ②가 없으면 언더스티어
    # 사고에선 벽에 긁히며 죽는 속도까지 창 안에 들어와 "감속 잘 됨"으로 뒤집힌다(2026-07-25).
    # 위상지연 때문에 코너 진입마다 순간적으로 비율이 무너지므로 0.25초 지속을 요구한다.
    idx = np.arange(len(t))
    incident = (np.abs(alat) > 1.6 * grip_target) | (np.abs(dvx) > 15.0)
    wash = (np.abs(wz_cmd) > 1.5) & (np.abs(alat) > 4.0) & (vx > 1.5) & \
           (np.abs(wz) / np.maximum(np.abs(wz_cmd), 1e-6) < 0.5)
    run_start = -1
    for i in range(len(t)):
        if wash[i]:
            if run_start < 0:
                run_start = i
            elif t[i] - t[run_start] >= 0.25:
                incident[run_start] = True
                break
        else:
            run_start = -1
    onset = int(np.argmax(incident)) if np.any(incident) else len(t)
    pre = idx < onset
    win = pre & drive_mask & (vx > 1.5)          # 사고 이전 · 주행 · 유효속도
    n_pre = int(np.sum(win))
    min_decel = float(np.min(dvx[win])) if n_pre else 0.0
    accel_frac = float(np.mean(dvx[win] > 0)) if n_pre else 0.0
    if n_pre >= 10:
        if min_decel > -0.8:
            findings.append(dict(level="critical", title="감속 권한 없음 (근본 원인)",
                body=(f"그립 붕괴 이전({t[onset] if onset<len(t) else t[-1]:.1f}s)까지 최저 종가속도 "
                      f"{min_decel:+.2f} m/s² — 감속 국면이 사실상 없음(구간의 {accel_frac*100:.0f}%가 가속). "
                      f"실제 감속은 벽/E-stop 이후에만 나타남. 속도명령을 낮춰도 차가 안 서므로 코너 "
                      f"사전감속(프로파일러 √(a_lat/κ) 캡)이 무효. → 브레이크 채널/VESC 회생제동 점검이 "
                      f"근본 해결. 그전까지는 max_speed를 코너 안전속도로 낮추는 게 유일한 레버.")))
        elif min_decel > -2.5:
            findings.append(dict(level="warn", title="감속 권한 약함",
                body=f"그립 붕괴 이전 최저 종가속도 {min_decel:.2f} m/s². 감속이 약해 코너 사전감속이 부족할 수 있음."))

    # ── 1-b) a_lat 출처 경고 ──
    if S.get("alat_source") == "odom_kinematic":
        findings.append(dict(level="warn", title="a_lat이 실측이 아님 (IMU/TF 없음)",
            body=("bag에 IMU도 map TF도 없어 a_lat을 /odom의 angular.z로 계산했다. 실차 "
                  "vesc_to_odom의 angular.z는 조향 명령의 기구학 합성값(v·tanδ/L)이라 "
                  "언더스티어 구간에서 최대 3배까지 과장된다. 아래 그립 판정은 상한으로만 볼 것 — "
                  "다음 녹화 땐 /sensors/imu/raw 또는 /tf를 반드시 포함할 것.")))

    # ── 2) 그립 초과 → 스핀인가 언더스티어인가 ──
    # 실측 요레이트로 판정한다. 명령(wz_cmd)으로 하면 조향 포화가 전부 '스핀'으로 오분류된다
    # (2026-07-25: 07-23 '풀락 스핀' 진단이 실제로는 언더스티어였음).
    R = np.where(np.abs(wz) > 1e-3, vx / np.abs(wz), 1e9)
    spin = (R < 1.15 * R_MIN) & (vx > 2.0)
    n_spin = int(np.sum(spin))
    # 언더스티어: 조향은 꺾여 있는데(명령 요레이트 큼) 차가 그만큼 안 돌아감
    us_mask = (np.abs(wz_cmd) > 1.0) & (vx > 1.5)
    us_ratio = float(np.median(np.abs(wz[us_mask]) / np.abs(wz_cmd[us_mask]))) if np.sum(us_mask) >= 8 else None
    # 스핀 판정에는 최소 지속시간을 요구한다. 벽 충돌 순간의 1~2 샘플(요레이트 임펄스)만으로
    # "풀락 스핀"이라 부르면 언더스티어 사고를 매번 스핀으로 오분류한다(2026-07-25).
    real_spin = n_spin >= 5
    if peak_alat > 1.6 * grip_target:
        if us_ratio is not None and us_ratio < 0.6 and not real_spin:
            mode = (f"실측 요레이트가 명령의 {us_ratio*100:.0f}%뿐 → **스핀이 아니라 언더스티어**"
                    f"(차가 안 돌고 밀림). 조향을 더 꺾어도 안 돌므로 진입속도를 낮추는 것 외엔 답이 없다. "
                    f"yaw_rate_gain 카운터스티어는 이 상황에서 조향을 더 밀어 악화시키니 0으로 A/B할 것.")
        elif real_spin:
            mode = (f"선회반경이 최소반경({R_MIN:.2f}m)에 닿는 샘플 {n_spin}개 = 실제 풀락 스핀.")
        else:
            mode = "조향 포화까지는 안 갔으나 그립 한계 초과."
        findings.append(dict(level="critical", title="그립 초과 (코너 진입 과속)",
            body=(f"최대 |a_lat| = {peak_alat:.1f} m/s² (목표 그립 {grip_target:.1f}의 "
                  f"{peak_alat/grip_target:.1f}배). 전체의 {over_frac*100:.0f}%가 그립 초과. " + mode)))
    elif over_frac > 0.05:
        findings.append(dict(level="warn", title="간헐적 그립 초과",
            body=f"최대 |a_lat| = {peak_alat:.1f}, 그립 초과 {over_frac*100:.0f}%. 마진 부족."))
    else:
        findings.append(dict(level="ok", title="그립 마진 양호",
            body=f"최대 |a_lat| = {peak_alat:.1f} m/s² < 목표 {grip_target:.1f}. 코너 그립 마진 확보."))

    # ── 3) 권장 max_speed : '사고 이전' 그립초과 코너들이 요구한 안전속도의 최솟값 ──
    # 스핀(사고) 이후의 비현실적 a_lat(20+)을 넣으면 권장속도가 과도히 낮아지므로 pre 구간만.
    over_idx = np.where((np.abs(alat) > grip_target) & pre)[0]
    v_safe = None
    for i in over_idx:
        if vx[i] > 0.5 and abs(alat[i]) > 1e-3:
            vs = vx[i] * math.sqrt(grip_target / abs(alat[i]))
            v_safe = vs if v_safe is None else min(v_safe, vs)
    if v_safe is not None:
        rec["max_speed"] = max(1.5, math.floor(v_safe * 2) / 2)  # 0.5 내림, 하한 1.5

    # ── 4) 권장 MLA : 스핀 있으면 5.0로 보수화 ──
    if peak_alat > 1.6 * grip_target:
        rec["max_lateral_accel"] = 5.0
        rec["min_speed"] = 1.5
        rec["note_recovery"] = True

    # ── 4-b) 조향 결손의 원인 분리: 서보 게인 오차 vs 언더스티어 그래디언트 ──
    # ⚠️ "요레이트 추종률이 낮다 → 서보 게인이 틀렸다"로 바로 결론내면 안 된다(2026-07-25에
    #    실제로 그렇게 오진했다). 정상상태 자전거 모델로 두 원인은 분리된다:
    #        delta = A·(L·κ) + K_us·a_lat + offset
    #      A ≈ 1  → 기하 게인 정상. 결손은 전부 타이어 언더스티어 그래디언트(K_us) = **물성**.
    #               서보를 건드리면 안 되고, 그 노면에서 LUT를 재보정하는 게 정답.
    #      A > 1  → 명령 대비 실제 조향각이 작다 = 진짜 서보 게인/링키지 문제.
    #    포화 샘플은 δ-κ 관계가 성립하지 않으므로 회귀에서 제외한다.
    gain_A = k_us = steer_offset = None
    if len(S["cmd_t"]) and S.get("alat_source") != "odom_kinematic" and len(t) > 30:
        st_i = np.interp(t, S["cmd_t"], S["cmd_steer"])
        dst = np.abs(np.gradient(st_i) / np.maximum(np.gradient(t), 1e-3))
        k_act = wz / np.maximum(vx, 0.1)
        m = (vx > 1.2) & (dst < 1.5) & (np.abs(st_i) < 0.40) & (np.abs(k_act) > 0.08)
        if m.sum() >= 50:
            X = np.column_stack([WHEELBASE * k_act[m], alat[m], np.ones(int(m.sum()))])
            coef, *_ = np.linalg.lstsq(X, st_i[m], rcond=None)
            gain_A, k_us, steer_offset = (float(c) for c in coef)
    # ⚠️ 회귀가 물리적으로 말이 되는 범위 안에 있을 때만 해석한다. 짧은 런·단일 코너·조향
    #    포화가 많은 런에서는 L·κ와 a_lat이 공선성을 띠어 A가 음수까지 나온다(2026-07-26
    #    run_0726_181747에서 A=-1.10, K_us=0.103). 그걸 "게인 정상"으로 보고하면 오진이다.
    if gain_A is not None and not (0.5 <= gain_A <= 2.0 and 0.0 <= k_us <= 0.08):
        findings.append(dict(level="info", title="조향 게인/언더스티어 회귀 신뢰 불가",
            body=(f"자전거모델 회귀가 비물리적 해로 수렴했다(A={gain_A:.2f}, K_us={k_us:+.4f}). "
                  f"코너가 하나뿐이거나 조향 포화 구간이 많아 L·κ와 a_lat이 공선성을 띤 경우다. "
                  f"게인/언더스티어 판정은 이 bag으로 할 수 없다 — 여러 코너를 포화 없이 도는 "
                  f"주행으로 재측정할 것.")))
        gain_A = k_us = None
    if gain_A is not None and gain_A > 1.25:
        findings.append(dict(level="critical", title="서보 조향 게인 부족",
            body=(f"자전거모델 회귀: 기하학이 요구하는 조향의 {gain_A:.2f}배를 명령해야 그 곡률이 나온다"
                  f"(정상 1.0). 실제 조향각이 명령의 {1/gain_A*100:.0f}%뿐이라는 뜻 — 젯슨 f1tenth_stack "
                  f"vesc.yaml의 steering_angle_to_servo_gain/offset과 조향 링키지 가동범위 확인.")))
    elif gain_A is not None:
        findings.append(dict(level="info", title=f"조향 기하 게인 정상 (A={gain_A:.2f})",
            body=(f"자전거모델 회귀 A={gain_A:.2f}(정상 1.0), 중립 오프셋 {steer_offset:+.4f} rad. "
                  f"**서보 게인은 원인이 아니다.** 요레이트 추종률이 낮은 건 언더스티어 그래디언트 "
                  f"K_us={k_us:+.4f} rad/(m/s²) 때문 — 타이어·노면 물성이라 서보로 못 고친다. "
                  f"a_lat {grip_target:.1f}에서 추가로 먹는 조향 {k_us*grip_target:+.3f} rad"
                  f"(한계 0.41의 {abs(k_us*grip_target)/0.41*100:.0f}%). 대응은 ① 이 노면에서 LUT 재보정"
                  f"(lut_calibration.launch.py) ② max_lateral_accel을 실측 그립 이내로.")))
    if k_us is not None and k_us > 0.008:
        v_st = math.sqrt(max(0.0, (0.41 - WHEELBASE * 0.7) / (k_us * 0.7)))
        findings.append(dict(level="info", title="조향 포화 한계 속도",
            body=(f"K_us={k_us:.4f}이면 κ=0.7(R=1.43m) 코너에서 필요 조향이 ±0.41에 닿는 속도가 "
                  f"**{v_st:.2f} m/s**. 그립 한계 √(그립/κ)={math.sqrt(grip_target/0.7):.2f} m/s와 비교해 "
                  f"낮은 쪽이 실제 구속조건이다.")))

    # ── 5) IMU 카운터스티어 유의 ──
    if len(S["gyro_z_raw"]) and np.max(np.abs(S["gyro_z_raw"])) > 30:
        findings.append(dict(level="info", title="IMU 자이로 = deg/s 확인",
            body=(f"자이로 z 최대 {np.max(np.abs(S['gyro_z_raw'])):.0f} (deg/s 스케일). "
                  f"IMU_ANGULAR_SCALE(π/180) 보정 전제. 언더스티어 시 요레이트 카운터스티어가 조향을 "
                  f"더 밀어 포화를 앞당길 수 있으니 use_imu:=false A/B 권장.")))

    stats = dict(peak_v=peak_v, peak_alat=peak_alat, over_frac=over_frac,
                 min_decel=min_decel, accel_frac=accel_frac, n_spin=n_spin,
                 peak_alat_cmd=float(np.max(np.abs(S["alat_cmd"]))) if len(S["alat_cmd"]) else 0.0,
                 alat_source=S.get("alat_source", "none"),
                 us_ratio=us_ratio, gain_A=gain_A, k_us=k_us,
                 duration=float(t[-1]) if len(t) else 0.0, grip_target=grip_target)
    return findings, {"rec": rec, "stats": stats}


# ══ 5. matplotlib 분석 그래프 → base64 PNG ══════════════════════════════════
def make_figure(S, grip_target):
    setup_font()
    t = S["odom_t"]
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle("주행상태 분석 (실측 기반)", fontsize=15, weight="bold")

    # (0,0) 궤적
    a = ax[0, 0]
    if len(S["pose"]):
        xy = S["pose"]
        sc = a.scatter(xy[:, 0], xy[:, 1], c=S["vx"], cmap="viridis", s=6)
        over = np.abs(S["alat"]) > grip_target
        a.scatter(xy[over, 0], xy[over, 1], c="red", s=14, label=f"|a_lat|>{grip_target}", zorder=3)
        fig.colorbar(sc, ax=a, label="speed [m/s]")
        a.set_aspect("equal"); a.legend(loc="best", fontsize=8)
    a.set_title("궤적 (map, 속도색)"); a.set_xlabel("x [m]"); a.set_ylabel("y [m]")

    # (0,1) 속도
    a = ax[0, 1]
    a.plot(t, S["vx"], label="/odom vx (실측)", lw=1.5)
    if len(S["cmd_speed"]):
        a.plot(S["cmd_t"], S["cmd_speed"], "--k", label="명령 speed", lw=1)
    a.set_title("속도"); a.set_xlabel("t [s]"); a.set_ylabel("m/s"); a.legend(fontsize=8); a.grid(alpha=.3)

    # (0,2) 요레이트 — 명령(기구학) vs 실측. 둘의 간격이 곧 언더스티어량이다.
    a = ax[0, 2]
    a.plot(t, S["wz_cmd"], label="명령 wz (/odom = v·tanδ/L)", lw=1.0, ls="--", color="gray")
    a.plot(t, S["wz_meas"], label=f"실측 wz ({SRC_LABEL.get(S['alat_source'], '?')})", lw=1.3, color="tab:blue")
    a.set_title("요레이트: 명령 vs 실측"); a.set_xlabel("t [s]"); a.set_ylabel("rad/s")
    a.legend(fontsize=8); a.grid(alpha=.3)

    # (1,0) 횡가속도 — 실측이 기본. 명령값은 참고선으로만.
    a = ax[1, 0]
    a.plot(t, S["alat_cmd"], label="명령 a_lat (기구학, 참고)", lw=.9, ls="--", color="gray")
    a.plot(t, S["alat"], label=f"실측 a_lat = vx·wz({SRC_LABEL.get(S['alat_source'], '?')})", lw=1.3, color="crimson")
    a.axhline(grip_target, ls=":", c="orange"); a.axhline(-grip_target, ls=":", c="orange", label=f"±grip {grip_target}")
    a.set_title("횡가속도 (그립 한계)"); a.set_xlabel("t [s]"); a.set_ylabel("m/s²"); a.legend(fontsize=8); a.grid(alpha=.3)

    # (1,1) 종가속도 (감속권한)
    a = ax[1, 1]
    a.plot(t, S["dvx"], label="dvx/dt", lw=1.2, color="teal")
    a.axhline(0, ls="-", c="k", lw=.6)
    a.fill_between(t, 0, S["dvx"], where=(S["dvx"] < 0), color="teal", alpha=.25, label="감속 구간")
    a.set_title("종가속도 (감속 권한)"); a.set_xlabel("t [s]"); a.set_ylabel("m/s²"); a.legend(fontsize=8); a.grid(alpha=.3)

    # (1,2) IMU raw
    a = ax[1, 2]
    if len(S["imu_t"]):
        a.plot(S["imu_t"], S["gyro_z_raw"], label="gyro z (deg/s)", lw=.8)
        a.plot(S["imu_t"], S["acc_x"], label="acc x (g)", lw=.8, alpha=.7)
        a.plot(S["imu_t"], S["acc_y"], label="acc y (g)", lw=.8, alpha=.7)
    a.set_title("IMU raw"); a.set_xlabel("t [s]"); a.legend(fontsize=8); a.grid(alpha=.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=90)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


# ══ 6. 리플레이용 JSON 데이터 ════════════════════════════════════════════════
def build_replay_json(S):
    # pose 트랙 (t,x,y,yaw,vx,alat) — 다운샘플 없이 (odom rate면 충분)
    t = S["odom_t"]
    track = []
    for i in range(len(t)):
        x, y, yaw = (S["pose"][i] if len(S["pose"]) else (0, 0, 0))
        track.append([round(float(t[i]), 3), round(float(x), 3), round(float(y), 3),
                      round(float(yaw), 4), round(float(S["vx"][i]), 3), round(float(S["alat"][i]), 2),
                      round(float(S["dvx"][i]), 2), round(float(S["wz_meas"][i]), 3)])
    wp = [[round(float(x), 3), round(float(y), 3)] for x, y, _, _ in S["wp"]] if len(S["wp"]) else []
    return {"track": track, "scans": S["scan_frames"], "wp": wp,
            "R_min": R_MIN, "car_w": CAR_W, "car_l": CAR_L}


# ══ 7. 튜닝 가이드 HTML ══════════════════════════════════════════════════════
def render_guide(findings, out, meta):
    rec = out.get("rec", {})
    st = out.get("stats", {})
    order = {"critical": 0, "warn": 1, "info": 2, "ok": 3}
    findings = sorted(findings, key=lambda f: order.get(f["level"], 9))
    colors = {"critical": "#e5484d", "warn": "#f5a623", "info": "#4c8dff", "ok": "#30a46c"}
    labels = {"critical": "치명", "warn": "주의", "info": "참고", "ok": "양호"}
    cards = []
    for f in findings:
        c = colors.get(f["level"], "#888")
        cards.append(
            f'<div class="finding" style="border-left:4px solid {c}">'
            f'<span class="badge" style="background:{c}">{labels.get(f["level"],"")}</span>'
            f'<b>{f["title"]}</b><p>{f["body"]}</p></div>')
    # 권장 런치 명령
    args = []
    if "max_speed" in rec: args.append(f'max_speed:={rec["max_speed"]:g}')
    if "max_lateral_accel" in rec: args.append(f'max_lateral_accel:={rec["max_lateral_accel"]:g}')
    if "min_speed" in rec: args.append(f'min_speed:={rec["min_speed"]:g}')
    cmd = "ros2 launch f1tenth_control control_real.launch.py"
    if args:
        cmd += " \\\n    " + " \\\n    ".join(args)
    rec_html = f'<pre class="cmd">{cmd}</pre>'
    if rec.get("note_recovery"):
        rec_html += ('<p class="note">＋ 코드 안전망 권장: 측정 a_lat이 그립 초과하거나 조향이 ±0.41에 '
                     '포화되면 target_speed를 하드컷(스핀/limit-cycle 차단), recovery_lat_error 1.0→0.5.</p>')
    return "".join(cards), rec_html


# ══ 8. HTML 조립 ═════════════════════════════════════════════════════════════
def _fragment_from_template(tpl):
    """아티팩트용: <style>+본문만 남기고 doctype/html/head/body 스캐폴딩 제거."""
    import re
    style = re.search(r"<style>.*?</style>", tpl, re.S).group(0)
    body = re.search(r"<body>(.*)</body>", tpl, re.S).group(1)
    return style + "\n" + body


def write_html(path, data_json, png_b64, guide_cards, rec_html, stats, meta, fragment=False):
    st = stats
    tiles = [
        ("주행 시간", f'{st.get("duration",0):.1f} s'),
        ("최고 속도", f'{st.get("peak_v",0):.2f} m/s'),
        (f'최대 |a_lat| ({SRC_LABEL.get(st.get("alat_source"), "?")})', f'{st.get("peak_alat",0):.1f} m/s²'),
        ("그립 초과", f'{st.get("over_frac",0)*100:.0f} %'),
        ("최저 종가속도", f'{st.get("min_decel",0):+.2f} m/s²'),
        ("요레이트 추종률",
         "—" if st.get("us_ratio") is None else f'{st["us_ratio"]*100:.0f} %'),
        ("언더스티어 K_us",
         "—" if st.get("k_us") is None else f'{st["k_us"]:+.3f}'),
        ("스핀 샘플", f'{st.get("n_spin",0)}'),
    ]
    tiles_html = "".join(f'<div class="tile"><div class="tv">{v}</div><div class="tl">{k}</div></div>' for k, v in tiles)
    html = _fragment_from_template(PAGE_TEMPLATE) if fragment else PAGE_TEMPLATE
    html = html.replace("__TITLE__", meta["title"])
    html = html.replace("__TILES__", tiles_html)
    html = html.replace("__GUIDE__", guide_cards)
    html = html.replace("__RECCMD__", rec_html)
    html = html.replace("__PNG__", png_b64)
    html = html.replace("__GRIP__", f'{stats.get("grip_target",5.0):g}')
    html = html.replace("__DATA__", data_json)
    with open(path, "w") as f:
        f.write(html)


PAGE_TEMPLATE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<style>
:root{--bg:#0e1116;--panel:#161b22;--line:#2a313c;--fg:#e6edf3;--dim:#9aa4b2;--accent:#4c8dff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,'Segoe UI',sans-serif}
h1{font-size:18px;margin:0}h2{font-size:15px;margin:22px 0 10px;color:var(--dim);font-weight:600}
header{padding:14px 20px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.wrap{max-width:1200px;margin:0 auto;padding:0 20px 60px}
.tiles{display:flex;gap:10px;flex-wrap:wrap;margin:16px 0}
.tile{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 16px;min-width:110px}
.tv{font-size:20px;font-weight:700}.tl{font-size:12px;color:var(--dim)}
.viewer{background:#0a0d12;border:1px solid var(--line);border-radius:10px;overflow:hidden}
#gl{width:100%;height:460px;display:block;cursor:grab}
#gl:active{cursor:grabbing}
.ctl{display:flex;align-items:center;gap:12px;padding:10px 14px;border-top:1px solid var(--line);background:var(--panel);flex-wrap:wrap}
.ctl button{background:#21262d;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 12px;cursor:pointer;font-size:14px}
.ctl button:hover{border-color:var(--accent)}
#seek{flex:1;min-width:180px}
.tval{font-variant-numeric:tabular-nums;color:var(--dim);min-width:120px}
.charts{display:grid;grid-template-columns:1fr;gap:8px;margin-top:10px}
.chart{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:6px 8px}
.chart canvas{width:100%;height:120px;display:block;cursor:crosshair}
.chart .clab{font-size:12px;color:var(--dim);padding:2px 4px}
.hint{font-size:12px;color:var(--dim);margin:6px 2px}
.finding{background:var(--panel);border-radius:8px;padding:10px 14px;margin:8px 0}
.finding b{font-size:14px}.finding p{margin:4px 0 0;color:var(--dim);font-size:13px}
.badge{color:#fff;font-size:11px;padding:1px 7px;border-radius:10px;margin-right:8px;font-weight:600}
pre.cmd{background:#0a0d12;border:1px solid var(--line);border-radius:8px;padding:12px;overflow-x:auto;color:#7ee787;font-size:13px}
.note{color:#f5a623;font-size:13px}
img.fig{width:100%;border:1px solid var(--line);border-radius:8px;background:#fff}
.legend{display:flex;gap:16px;font-size:12px;color:var(--dim);padding:4px 2px;flex-wrap:wrap}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;vertical-align:middle;margin-right:4px}
</style></head>
<body>
<header><h1>🏎️ __TITLE__</h1><span class="hint">F1TENTH 주행 rosbag 통합 분석</span></header>
<div class="wrap">
  <div class="tiles">__TILES__</div>

  <h2>3D 주행 리플레이 <span class="hint">— 드래그: 회전 · 휠: 줌 · Shift+드래그: 이동</span></h2>
  <div class="viewer">
    <canvas id="gl"></canvas>
    <div class="ctl">
      <button id="play">▶︎ 재생</button>
      <input id="seek" type="range" min="0" max="1000" value="0">
      <span class="tval" id="tval">0.00 s</span>
      <label class="hint">속도 <select id="rate"><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select>×</label>
      <button id="reset">시점 리셋</button>
    </div>
  </div>
  <div class="legend">
    <span><span class="dot" style="background:#39d353"></span>저속</span>
    <span><span class="dot" style="background:#e3b341"></span>중속</span>
    <span><span class="dot" style="background:#e5484d"></span>고속</span>
    <span><span class="dot" style="background:#58a6ff"></span>LiDAR</span>
    <span><span class="dot" style="background:#bc8cff"></span>글로벌 경로</span>
  </div>

  <div class="charts">
    <div class="chart"><div class="clab">속도 vx [m/s]</div><canvas id="c_v"></canvas></div>
    <div class="chart"><div class="clab">횡가속도 a_lat = vx·wz(실측 요레이트) [m/s²] (주황선 = 그립 ±__GRIP__)</div><canvas id="c_a"></canvas></div>
    <div class="chart"><div class="clab">종가속도 dvx/dt [m/s²] (0 아래 = 감속)</div><canvas id="c_d"></canvas></div>
  </div>

  <h2>자동 튜닝 가이드</h2>
  __GUIDE__
  <h3 style="margin:16px 0 6px">권장 다음 셰이크다운</h3>
  __RECCMD__

  <h2>주행상태 분석 그래프</h2>
  <img class="fig" src="data:image/png;base64,__PNG__">
</div>

<script>
const DATA = __DATA__;
const GRIP = parseFloat("__GRIP__");
const track = DATA.track;   // [t,x,y,yaw,vx,alat,dvx,wz]
const T0 = track[0][0], T1 = track[track.length-1][0], DUR = T1-T0;
const scans = DATA.scans, wp = DATA.wp;

// ── 색상: 속도 → 색 ──
const VMAX = Math.max(1, ...track.map(p=>p[4]));
function speedColor(v){const x=Math.min(1,Math.max(0,v/VMAX));
  const r=Math.round(57+x*(229-57)),g=Math.round(211-x*(211-72)),b=Math.round(83-x*(83-77));
  return `rgb(${r},${g},${b})`;}

// ── 카메라 ──
let bounds=(()=>{let xs=track.map(p=>p[1]),ys=track.map(p=>p[2]);
  let x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys);
  return {cx:(x0+x1)/2,cy:(y0+y1)/2,r:Math.max(4,Math.hypot(x1-x0,y1-y0)/2)};})();
let cam={az:-0.9,el:0.62,dist:bounds.r*2.6,tx:bounds.cx,ty:bounds.cy,tz:0};
function defcam(){cam={az:-0.9,el:0.62,dist:bounds.r*2.6,tx:bounds.cx,ty:bounds.cy,tz:0};}

const gl=document.getElementById('gl');const ctx=gl.getContext('2d');
function resize(){const w=gl.clientWidth,h=gl.clientHeight;const dpr=window.devicePixelRatio||1;
  gl.width=w*dpr;gl.height=h*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);}
window.addEventListener('resize',()=>{resize();draw();});

function basis(){
  const cp=[cam.tx+cam.dist*Math.cos(cam.el)*Math.cos(cam.az),
            cam.ty+cam.dist*Math.cos(cam.el)*Math.sin(cam.az),
            cam.tz+cam.dist*Math.sin(cam.el)];
  let f=[cam.tx-cp[0],cam.ty-cp[1],cam.tz-cp[2]];const fl=Math.hypot(...f);f=f.map(v=>v/fl);
  let up=[0,0,1];
  let r=[f[1]*up[2]-f[2]*up[1],f[2]*up[0]-f[0]*up[2],f[0]*up[1]-f[1]*up[0]];
  const rl=Math.hypot(...r);r=r.map(v=>v/rl);
  let u=[r[1]*f[2]-r[2]*f[1],r[2]*f[0]-r[0]*f[2],r[0]*f[1]-r[1]*f[0]];
  return {cp,f,r,u};
}
let B=basis();
function proj(p){ // world [x,y,z] -> screen or null
  const d=[p[0]-B.cp[0],p[1]-B.cp[1],p[2]-B.cp[2]];
  const zc=d[0]*B.f[0]+d[1]*B.f[1]+d[2]*B.f[2];
  if(zc<0.05)return null;
  const xc=d[0]*B.r[0]+d[1]*B.r[1]+d[2]*B.r[2];
  const yc=d[0]*B.u[0]+d[1]*B.u[1]+d[2]*B.u[2];
  const W=gl.clientWidth,H=gl.clientHeight,foc=H*0.9;
  return [W/2+foc*xc/zc, H/2-foc*yc/zc, zc];
}
function line(a,b,col,w){const p=proj(a),q=proj(b);if(!p||!q)return;
  ctx.strokeStyle=col;ctx.lineWidth=w||1;ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();}

// ── pose 보간 ──
function idxAt(t){let lo=0,hi=track.length-1;if(t<=track[0][0])return 0;if(t>=track[hi][0])return hi;
  while(lo+1<hi){const m=(lo+hi)>>1;if(track[m][0]<=t)lo=m;else hi=m;}return lo;}
function poseAt(t){const i=idxAt(t),a=track[i],b=track[Math.min(i+1,track.length-1)];
  const d=b[0]-a[0],s=d>1e-6?(t-a[0])/d:0;
  let dy=b[3]-a[3];while(dy>Math.PI)dy-=2*Math.PI;while(dy<-Math.PI)dy+=2*Math.PI;
  return {x:a[1]+s*(b[1]-a[1]),y:a[2]+s*(b[2]-a[2]),yaw:a[3]+s*dy,vx:a[4]+s*(b[4]-a[4])};}
function scanAt(t){let s=null;for(const f of scans){if(f.t<=t)s=f;else break;}return s;}

function drawGround(){
  const g=Math.max(1,Math.round(bounds.r/6));const R=Math.ceil(bounds.r*1.6);
  const x0=Math.floor(bounds.cx-R),x1=Math.ceil(bounds.cx+R),y0=Math.floor(bounds.cy-R),y1=Math.ceil(bounds.cy+R);
  for(let x=x0;x<=x1;x+=g)line([x,y0,0],[x,y1,0],'#1c2230',1);
  for(let y=y0;y<=y1;y+=g)line([x0,y,0],[x1,y,0],'#1c2230',1);
}
function drawCar(px,py,yaw){
  const hl=DATA.car_l/2,hw=DATA.car_w/2,hz=0.16;const c=Math.cos(yaw),s=Math.sin(yaw);
  function corner(dx,dy,dz){return [px+dx*c-dy*s,py+dx*s+dy*c,dz];}
  const bot=[corner(hl,hw,0),corner(hl,-hw,0),corner(-hl,-hw,0),corner(-hl,hw,0)];
  const top=bot.map(p=>[p[0],p[1],hz]);
  // top face fill
  const tp=top.map(proj);
  if(tp.every(p=>p)){ctx.fillStyle='rgba(88,166,255,.5)';ctx.beginPath();ctx.moveTo(tp[0][0],tp[0][1]);
    for(let i=1;i<4;i++)ctx.lineTo(tp[i][0],tp[i][1]);ctx.closePath();ctx.fill();}
  for(let i=0;i<4;i++){line(bot[i],bot[(i+1)%4],'#58a6ff',1.5);line(top[i],top[(i+1)%4],'#79c0ff',1.5);line(bot[i],top[i],'#58a6ff',1.5);}
  // heading arrow
  line(corner(hl,0,hz),corner(hl+0.35,0,hz),'#ffdd55',2.5);
}

function draw(){
  B=basis();
  const W=gl.clientWidth,H=gl.clientHeight;
  ctx.fillStyle='#0a0d12';ctx.fillRect(0,0,W,H);
  drawGround();
  // waypoints
  if(wp.length){for(let i=0;i+1<wp.length;i++)line([wp[i][0],wp[i][1],0.01],[wp[i+1][0],wp[i+1][1],0.01],'#bc8cff',1.2);}
  // trajectory (speed colored)
  for(let i=0;i+1<track.length;i++)line([track[i][1],track[i][2],0.02],[track[i+1][1],track[i+1][2],0.02],speedColor(track[i][4]),2);
  // current scan
  const sc=scanAt(cur);
  if(sc){ctx.fillStyle='#58a6ff';for(let i=0;i<sc.pts.length;i+=2){const p=proj([sc.pts[i],sc.pts[i+1],0.03]);
    if(p){ctx.beginPath();ctx.arc(p[0],p[1],1.6,0,7);ctx.fill();}}}
  // car
  const ps=poseAt(cur);drawCar(ps.x,ps.y,ps.yaw);
  drawCharts();
}

// ── 차트 ──
function setupChart(id){const cv=document.getElementById(id);const c=cv.getContext('2d');
  function rs(){const dpr=window.devicePixelRatio||1;cv.width=cv.clientWidth*dpr;cv.height=cv.clientHeight*dpr;c.setTransform(dpr,0,0,dpr,0,0);}
  rs();window.addEventListener('resize',rs);
  cv.addEventListener('click',e=>{const x=e.offsetX/cv.clientWidth;cur=T0+x*DUR;syncSeek();draw();});
  return {cv,c};}
const chV=setupChart('c_v'),chA=setupChart('c_a'),chD=setupChart('c_d');
function chart(ch,val,col,zeroCenter,grip){
  const {cv,c}=ch,W=cv.clientWidth,Ht=cv.clientHeight;c.clearRect(0,0,W,Ht);
  let mn=Math.min(...val),mx=Math.max(...val);if(zeroCenter){const a=Math.max(Math.abs(mn),Math.abs(mx),grip||1);mn=-a;mx=a;}
  if(mx-mn<1e-6)mx=mn+1;const pad=6;
  const X=t=>pad+(t-T0)/DUR*(W-2*pad),Y=v=>Ht-pad-(v-mn)/(mx-mn)*(Ht-2*pad);
  // zero / grip lines
  c.strokeStyle='#2a313c';c.lineWidth=1;
  if(zeroCenter){c.beginPath();c.moveTo(pad,Y(0));c.lineTo(W-pad,Y(0));c.stroke();}
  if(grip){c.strokeStyle='#f5a623';c.setLineDash([4,3]);
    [grip,-grip].forEach(g=>{c.beginPath();c.moveTo(pad,Y(g));c.lineTo(W-pad,Y(g));c.stroke();});c.setLineDash([]);}
  // series
  c.strokeStyle=col;c.lineWidth=1.4;c.beginPath();
  for(let i=0;i<track.length;i++){const x=X(track[i][0]),y=Y(val[i]);i?c.lineTo(x,y):c.moveTo(x,y);}c.stroke();
  // playhead
  const px=X(cur);c.strokeStyle='#e6edf3';c.lineWidth=1;c.beginPath();c.moveTo(px,pad);c.lineTo(px,Ht-pad);c.stroke();
  const ps=poseAt(cur);
}
function drawCharts(){
  chart(chV,track.map(p=>p[4]),'#39d353',false,null);
  chart(chA,track.map(p=>p[5]),'#e5484d',true,GRIP);
  chart(chD,track.map(p=>p[6]),'#2dd4bf',true,null);
}

// ── 재생 ──
let cur=T0,playing=false,rate=1,lastTs=0;
const seek=document.getElementById('seek'),tval=document.getElementById('tval'),playBtn=document.getElementById('play');
function syncSeek(){seek.value=((cur-T0)/DUR*1000)|0;const ps=poseAt(cur);
  tval.textContent=`${(cur-T0).toFixed(2)} s | ${ps.vx.toFixed(2)} m/s`;}
seek.addEventListener('input',()=>{cur=T0+seek.value/1000*DUR;syncSeek();draw();});
document.getElementById('rate').addEventListener('change',e=>rate=parseFloat(e.target.value));
playBtn.addEventListener('click',()=>{playing=!playing;playBtn.textContent=playing?'❚❚ 일시정지':'▶︎ 재생';lastTs=0;if(playing)requestAnimationFrame(loop);});
document.getElementById('reset').addEventListener('click',()=>{defcam();draw();});
function loop(ts){if(!playing)return;if(lastTs){cur+=(ts-lastTs)/1000*rate;if(cur>=T1){cur=T0;}}lastTs=ts;syncSeek();draw();requestAnimationFrame(loop);}

// ── 마우스 조작 ──
let drag=null;
gl.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,shift:e.shiftKey,btn:e.button};});
window.addEventListener('mouseup',()=>drag=null);
window.addEventListener('mousemove',e=>{if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;
  if(drag.shift||drag.btn===2){const k=cam.dist*0.0015;const ca=Math.cos(cam.az),sa=Math.sin(cam.az);
    cam.tx-=(-sa*dx)*k;cam.ty-=(ca*dx)*k;cam.tx+= (ca*dy)*k*0.6;cam.ty+=(sa*dy)*k*0.6;}
  else{cam.az-=dx*0.008;cam.el=Math.max(0.08,Math.min(1.5,cam.el+dy*0.006));}draw();});
gl.addEventListener('wheel',e=>{e.preventDefault();cam.dist*=Math.exp(e.deltaY*0.0011);cam.dist=Math.max(1,Math.min(bounds.r*8,cam.dist));draw();},{passive:false});
gl.addEventListener('contextmenu',e=>e.preventDefault());

resize();defcam();syncSeek();draw();
</script>
</body></html>
"""


# ══ 9. main ══════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="F1TENTH 주행 rosbag 통합 분석 → report.html")
    ap.add_argument("bag", help="bag 폴더 또는 .db3 경로")
    ap.add_argument("--out", default=None, help="출력 html (기본: <bag>_report.html)")
    ap.add_argument("--odom", default="/odom", help="odom 토픽 (기본 /odom)")
    ap.add_argument("--grip", type=float, default=5.0, help="목표 그립 a_lat [m/s²] (기본 5.0)")
    ap.add_argument("--fragment", action="store_true",
                    help="아티팩트/임베드용 본문 조각(HTML) 출력 (doctype/head 없음)")
    args = ap.parse_args()

    db3 = find_db3(args.bag)
    name = os.path.basename(os.path.dirname(db3) if db3.endswith(".db3") else args.bag) or os.path.basename(db3)
    out = args.out or os.path.join(os.getcwd(), f"{name}_report.html")

    print(f"[1/5] bag 로딩: {db3}")
    data = load_bag(db3)
    print("      토픽:", ", ".join(f"{k}({len(v)})" for k, v in sorted(data.items())))

    print("[2/5] 시리즈/TF 추출")
    S = build_series(data, args.odom)
    if not len(S["vx"]):
        sys.exit("[에러] odom 데이터가 없습니다. --odom 으로 토픽을 지정하세요.")

    print("[3/5] 자동 진단")
    findings, diag = diagnose(S, args.grip)
    stats = diag.get("stats", {})

    print("[4/5] 그래프 렌더")
    png = make_figure(S, args.grip)

    print("[5/5] 리플레이 데이터/HTML 조립")
    rjson = json.dumps(build_replay_json(S), separators=(",", ":"))
    cards, rec_html = render_guide(findings, diag, {"title": name})
    write_html(out, rjson, png, cards, rec_html, stats, {"title": name}, fragment=args.fragment)

    print(f"\n✅ 완료 → {out}")
    print(f"   브라우저에서 열기:  xdg-open '{out}'")
    for f in sorted(findings, key=lambda x: {"critical":0,"warn":1,"info":2,"ok":3}.get(x["level"],9)):
        print(f"   [{f['level'].upper():8}] {f['title']}")


if __name__ == "__main__":
    main()
