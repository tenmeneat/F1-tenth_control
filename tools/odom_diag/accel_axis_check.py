#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
accel_axis_check.py — VESC IMU 가속도계 축·부호·단위 확정 (관찰 전용, 차 불필요)
================================================================================
횡슬립각 β 추정기(= odom 강건화 + 카운터스티어의 공통 전제)는 a_y 하나에 걸려 있는데,
VESC 가속도계의 **축 매핑·부호·단위는 검증된 적이 없다**(자이로 z만 검증됨).
이 도구는 이미 있는 주행 bag만으로 그걸 확정한다. 원돌이 테스트가 필요 없다.

기준자 두 개 — 둘 다 캘리브레이션 대상과 독립이라 공짜다:

  ① 중력 (정지 구간)  : 차가 서 있으면 가속도 벡터의 크기는 정확히 1g다.
                        크기 → 단위(g vs m/s²), 방향 → 수직축과 부호.
  ② a_y ≡ v·ψ̇ (선회)  : 슬립이 없으면 이건 항등식이다. v(ERPM, ±0.3% 검증)와
                        ψ̇(자이로, +0.06% 검증)는 이미 결백이 확정됐으므로
                        그 곱이 곧 a_y의 정답지다.
  ③ a_x ≡ dv/dt (가감속): 종축 확인용. 단, 피치(노즈다이브)가 중력을 섞으므로 ②보다 약하다.

  ⚠️ ②의 잔차는 그냥 노이즈가 아니라 **v̇_y = a_y − v·ψ̇ 그 자체**, 즉 우리가 찾는
     횡슬립 신호다. 그래서 이 스크립트는 축 검증기이면서 동시에 β 추정기의 1차 타당성
     점검이기도 하다(--beta 로 적분해 본다).

사용법:
  source /opt/ros/<distro>/setup.bash
  python3 accel_axis_check.py <bag_dir | .db3> [--imu /sensors/imu/raw] [--odom /odom] [--beta]
  python3 accel_axis_check.py <bag1> <bag2> ...       # 여러 bag 교차확인(권장)
"""
import os, sys, glob, sqlite3, math, argparse
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DEG2RAD = math.pi / 180.0
G = 9.80665

# 정지 판정 / 회귀 게이트
STOP_SPEED = 0.05        # [m/s] 이 미만이면 정지 후보
STOP_GYRO = 2.0          # [deg/s] 자이로도 조용해야 진짜 정지(사람이 차를 들면 제외)
STOP_MIN_DUR = 0.5       # [s] 이 이상 연속돼야 정지 구간으로 채택
FIT_MIN_SPEED = 1.0      # [m/s] 저속은 v·ψ̇가 작아 SNR이 나쁨 → 제외
FIT_MIN_YAWRATE = 0.15   # [rad/s] 직진 구간은 양변이 모두 0이라 기울기 정보가 없음


# ══ bag 로딩 ═════════════════════════════════════════════════════════════════
def open_bag(path):
    if os.path.isdir(path):
        cands = sorted(glob.glob(os.path.join(path, "*.db3")))
        if not cands:
            raise FileNotFoundError(f"{path} 안에 .db3 없음")
        path = cands[0]
    return sqlite3.connect(path)


def read_topic(con, topic):
    """(t[s], msg) 리스트. 토픽 없으면 빈 리스트."""
    cur = con.cursor()
    row = cur.execute("SELECT id, type FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        return []
    tid, tname = row
    try:
        msgtype = get_message(tname)
    except Exception:
        return []
    out = []
    for ts, data in cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        try:
            out.append((ts * 1e-9, deserialize_message(bytes(data), msgtype)))
        except Exception:
            continue
    return out


def load(path, imu_topic, odom_topic):
    con = open_bag(path)
    imu = read_topic(con, imu_topic)
    if not imu:
        for alt in ("/sensors/imu/raw", "/sensors/imu", "/imu/data"):
            imu = read_topic(con, alt)
            if imu:
                imu_topic = alt
                break
    odom = read_topic(con, odom_topic)
    if not odom:
        for alt in ("/odom", "/pf/pose/odom"):
            odom = read_topic(con, alt)
            if odom:
                odom_topic = alt
                break
    con.close()
    if not imu or not odom:
        raise RuntimeError(f"IMU({imu_topic}) 또는 odom({odom_topic}) 토픽 없음")

    S = {}
    S["it"] = np.array([t for t, _ in imu])
    S["a"] = np.array([[m.linear_acceleration.x,
                        m.linear_acceleration.y,
                        m.linear_acceleration.z] for _, m in imu])   # 원시 단위 그대로
    S["gz_raw"] = np.array([m.angular_velocity.z for _, m in imu])   # 원시(=deg/s 추정)
    S["ot"] = np.array([t for t, _ in odom])
    S["ov"] = np.array([m.twist.twist.linear.x for _, m in odom])
    S["imu_topic"], S["odom_topic"] = imu_topic, odom_topic
    return S


# ══ ① 중력 = 단위·수직축 ══════════════════════════════════════════════════════
def gravity_check(S):
    """정지 구간의 평균 가속도 벡터 → 단위(g/SI)와 수직축·부호."""
    v = np.interp(S["it"], S["ot"], S["ov"])
    still = (np.abs(v) < STOP_SPEED) & (np.abs(S["gz_raw"]) < STOP_GYRO)

    # 연속 구간만 채택 (순간적으로 스치는 0속도 제외)
    dt = np.median(np.diff(S["it"])) if len(S["it"]) > 1 else 0.02
    need = max(2, int(STOP_MIN_DUR / dt))
    keep = np.zeros_like(still)
    i = 0
    segs = 0
    while i < len(still):
        if still[i]:
            j = i
            while j < len(still) and still[j]:
                j += 1
            if j - i >= need:
                keep[i:j] = True
                segs += 1
            i = j
        else:
            i += 1

    if keep.sum() < need:
        return None
    a = S["a"][keep]
    mean = a.mean(axis=0)
    norm = float(np.linalg.norm(mean))
    return dict(n=int(keep.sum()), segs=segs, dur=float(keep.sum() * dt),
                mean=mean, std=a.std(axis=0), norm=norm)


# ══ ②③ 동적 회귀 ═══════════════════════════════════════════════════════════════
def robust_fit(x, y):
    """y ≈ s·x 의 기울기. 슬립 구간이 소수 섞이므로 2단계 트림(잔차 상위 20% 제거)."""
    if len(x) < 20:
        return float("nan"), float("nan"), 0
    s = float(np.sum(x * y) / np.sum(x * x))
    for _ in range(2):
        r = np.abs(y - s * x)
        thr = np.quantile(r, 0.80)
        m = r <= thr
        if m.sum() < 20:
            break
        s = float(np.sum(x[m] * y[m]) / np.sum(x[m] * x[m]))
    pred = s * x
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return s, r2, int(len(x))


def dynamic_check(S, unit_scale, vert_axis):
    """각 후보 축을 v·ψ̇(횡) / dv/dt(종)에 회귀. 수직축은 후보에서 제외."""
    v = np.interp(S["it"], S["ot"], S["ov"])
    gz = S["gz_raw"] * DEG2RAD
    a = S["a"] * unit_scale            # → m/s²

    truth_lat = v * gz
    # 종방향 진실값은 odom 속도의 미분 (IMU와 독립)
    ov_s = np.interp(S["it"], S["ot"], S["ov"])
    dt = np.gradient(S["it"])
    truth_lon = np.gradient(ov_s) / np.where(dt > 1e-6, dt, np.nan)
    truth_lon = np.nan_to_num(truth_lon)

    gate_lat = (np.abs(v) > FIT_MIN_SPEED) & (np.abs(gz) > FIT_MIN_YAWRATE)
    gate_lon = (np.abs(v) > FIT_MIN_SPEED) & (np.abs(truth_lon) > 0.5)

    names = ["x", "y", "z"]
    res = {"lat": [], "lon": []}
    for i, nm in enumerate(names):
        if i == vert_axis:
            continue
        col = a[:, i] - a[:, i].mean()      # 상수 오프셋(바이어스·중력 잔여) 제거
        s, r2, n = robust_fit(truth_lat[gate_lat], col[gate_lat])
        res["lat"].append((nm, s, r2, n))
        s2, r22, n2 = robust_fit(truth_lon[gate_lon], col[gate_lon])
        res["lon"].append((nm, s2, r22, n2))
    return res, dict(v=v, gz=gz, a=a, truth_lat=truth_lat, gate_lat=gate_lat)


# ══ 출력 ═════════════════════════════════════════════════════════════════════
def analyze(path, args):
    print("=" * 78)
    print(f"■ {os.path.basename(path.rstrip('/'))}")
    try:
        S = load(path, args.imu, args.odom)
    except Exception as e:
        print(f"  건너뜀: {e}")
        return None
    print(f"  IMU {S['imu_topic']} {len(S['it'])}개 / odom {S['odom_topic']} {len(S['ot'])}개 "
          f"/ {S['it'][-1]-S['it'][0]:.1f}s, 최고속 {S['ov'].max():.2f} m/s")

    # ── ① 중력 ──
    g = gravity_check(S)
    if g is None:
        print("  ① 중력: 정지 구간 부족 → 단위 판정 불가")
        unit_scale, vert_axis = None, None
    else:
        n = g["norm"]
        if abs(n - 1.0) < 0.25:
            unit, unit_scale = "g", G
        elif abs(n - G) < 2.0:
            unit, unit_scale = "m/s² (SI)", 1.0
        else:
            unit, unit_scale = f"불명(크기 {n:.3f})", None
        vert_axis = int(np.argmax(np.abs(g["mean"])))
        vs = "+" if g["mean"][vert_axis] > 0 else "−"
        print(f"  ① 중력 [{g['segs']}구간 {g['dur']:.1f}s, {g['n']}샘플]")
        print(f"     평균 a = ({g['mean'][0]:+.4f}, {g['mean'][1]:+.4f}, {g['mean'][2]:+.4f})"
              f"  |a| = {n:.4f}")
        print(f"     노이즈 σ = ({g['std'][0]:.4f}, {g['std'][1]:.4f}, {g['std'][2]:.4f})")
        print(f"     → 단위 = {unit}   수직축 = {vs}{'xyz'[vert_axis]}")

    if unit_scale is None:
        print("     단위 미확정 → 동적 검증은 배율 1.0 가정으로 진행(기울기만 참고)")
        unit_scale = 1.0
    if vert_axis is None:
        vert_axis = -1

    # ── ②③ 동적 ──
    res, D = dynamic_check(S, unit_scale, vert_axis)
    print(f"  ② 횡: 후보축 vs v·ψ̇   (기울기 ±1.0·R² 높음 = 이 축이 횡방향)")
    for nm, s, r2, n in res["lat"]:
        mark = "  ←★" if (not math.isnan(r2) and r2 > 0.7 and abs(abs(s) - 1.0) < 0.35) else ""
        print(f"     a_{nm}: 기울기 {s:+.3f}  R² {r2:+.3f}  n={n}{mark}")
    print(f"  ③ 종: 후보축 vs dv/dt  (피치 결합으로 ②보다 약함 — 참고용)")
    for nm, s, r2, n in res["lon"]:
        mark = "  ←★" if (not math.isnan(r2) and r2 > 0.5 and abs(abs(s) - 1.0) < 0.5) else ""
        print(f"     a_{nm}: 기울기 {s:+.3f}  R² {r2:+.3f}  n={n}{mark}")

    # ── 잔차 = 횡슬립 신호 ──
    best = max((r for r in res["lat"] if not math.isnan(r[2])), key=lambda r: r[2], default=None)
    if best and args.beta:
        nm, s, r2, _ = best
        i = "xyz".index(nm)
        ay = (D["a"][:, i] - D["a"][:, i].mean()) / s       # 부호·배율 정규화한 a_y
        vdot_y = ay - D["truth_lat"]                        # = v̇_y (횡슬립 신호)
        m = D["gate_lat"]
        print(f"  ④ 잔차 v̇_y = a_{nm}/s − v·ψ̇  (β 추정기의 입력)")
        print(f"     RMS {np.sqrt(np.mean(vdot_y[m]**2)):.3f} m/s²  "
              f"p95 |·| {np.quantile(np.abs(vdot_y[m]),0.95):.3f}  "
              f"최대 {np.max(np.abs(vdot_y[m])):.3f}")
        print(f"     ⚠️ 이 값에는 진짜 슬립 + 가속도계 노이즈/바이어스 + 축 미정렬이 섞여 있다.")
    return dict(path=path, g=g, res=res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--imu", default="/sensors/imu/raw")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--beta", action="store_true", help="잔차(횡슬립 신호) 통계도 출력")
    args = ap.parse_args()

    outs = [o for o in (analyze(p, args) for p in args.bags) if o]

    if len(outs) > 1:
        print("=" * 78)
        print("■ 교차 확인 (bag 간 일치해야 진짜)")
        for key, label in (("lat", "횡 vs v·ψ̇"), ("lon", "종 vs dv/dt")):
            print(f"  [{label}]")
            axes = {}
            for o in outs:
                for nm, s, r2, n in o["res"][key]:
                    axes.setdefault(nm, []).append((s, r2))
            for nm, vals in axes.items():
                ss = np.array([v[0] for v in vals]); rr = np.array([v[1] for v in vals])
                print(f"     a_{nm}: 기울기 {np.nanmean(ss):+.3f} ± {np.nanstd(ss):.3f}   "
                      f"R² {np.nanmean(rr):+.3f} ± {np.nanstd(rr):.3f}")
        norms = [o["g"]["norm"] for o in outs if o["g"]]
        if norms:
            print(f"  [정지 |a|] {np.mean(norms):.4f} ± {np.std(norms):.4f}  (1.0=g, 9.81=SI)")


if __name__ == "__main__":
    main()
