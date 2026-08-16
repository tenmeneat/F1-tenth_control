#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compare_steering_model.py — LUT 역조회 vs 자전거 역모델 오프라인 대조  [②-p 검증]
================================================================================
`steering_model:=bicycle`로 실차에 태우기 **전에**, 기존 bag으로 두 모델이 같은 상황에서
각각 어떤 조향을 냈을지 재구성해 비교한다. 차 시간 0.

무엇을 재구성하나
  bag의 pose(/pf/pose/odom) + 경로(/local_waypoints)로 control_map_node의 조향 경로를
  그대로 다시 계산한다: L1 거리 → 목표점(호 길이 전진) → sin_eta → a_lat →
    (A) LUT 역조회          = 지금 실차에 올라가 있는 것
    (B) 자전거 역모델 + FF/FB = steering_model:=bicycle
  그리고 실제 발행된 조향(/drive_autonomous)과 대조해 재구성 자체가 맞는지 검증한다.

⚠️ 이건 **개루프 비교**다. B로 달렸다면 차가 다른 라인을 그렸을 테니 랩타임·벽 여유를
   예측하지 못한다. 여기서 답할 수 있는 건 "같은 순간에 두 모델의 조향 명령이 얼마나
   다른가"와 "LUT 천장이 언제 구속했나"뿐이다. 폐루프 효과는 실차 저속 셰이크다운으로만.

사용법:
  source /opt/ros/<distro>/setup.bash && source ~/2026_IFAC/install/setup.bash
  python3 compare_steering_model.py <bag폴더|.db3> [--fb-gain 1.0 0.8 0.6] [--kus 0.014]
"""
import sys, os, glob, sqlite3, math, argparse
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── control_map_node / 런치 기본값과 같은 기준 ──────────────────────────────
WHEELBASE = 0.33
L1_OFFSET = 0.6
L1_SPEED_GAIN = 0.4
T_CLIP_MIN, T_CLIP_MAX = 0.9, 5.0
L1_MIN_DENOM = 0.9
REACH_RATIO = 0.85
STEER_LIMIT = 0.410
DEFAULT_KUS = 0.014
MLA = 7.0


def load_lut(path=None):
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, "..", "..", "control_code", "LUT_calibrated.csv")
    raw = np.genfromtxt(path, delimiter=",")
    return raw[0, 1:], raw[1:, 0], raw[1:, 1:]


class Lut:
    """steering_lookup_table.hpp의 build_columns + column_lookup 재현."""

    def __init__(self, path=None):
        self.vs, self.steers, self.table = load_lut(path)
        self.cols = []
        for j in range(len(self.vs)):
            col = self.table[:, j]
            if np.all(np.isnan(col)):
                self.cols.append((np.array([]), np.array([])))
                continue
            peak = int(np.nanargmax(col))
            acc, st = [], []
            for i in range(peak + 1):
                if np.isnan(col[i]):
                    break          # NaN에서 끊긴다 = 이 속도의 조향 천장
                acc.append(col[i]); st.append(self.steers[i])
            self.cols.append((np.array(acc), np.array(st)))

    def ceiling(self, v):
        j = int(np.argmin(np.abs(self.vs - np.clip(v, self.vs.min(), self.vs.max()))))
        _, st = self.cols[j]
        return st[-1] if len(st) else 0.0

    def lookup(self, a_lat, v):
        sign = 1.0 if a_lat >= 0 else -1.0
        a = abs(a_lat)
        j = int(np.argmin(np.abs(self.vs - np.clip(v, self.vs.min(), self.vs.max()))))
        acc, st = self.cols[j]
        if len(acc) == 0:
            return 0.0, True
        sat = a >= acc[-1]
        if sat:
            return st[-1] * sign, True
        return float(np.interp(a, acc, st)) * sign, False


def bicycle(a_lat, v, kus):
    return a_lat * (WHEELBASE / max(v * v, 1e-4) + kus)


def read_topic(cur, topics, typemap, name):
    tid = topics[name]
    mt = get_message(typemap[name])
    rows = cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,)).fetchall()
    return [(ts, deserialize_message(d, mt)) for ts, d in rows]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def walk_forward(pts, start, dist):
    """호 길이 dist만큼 전진한 인덱스 (control_map_node.walk_forward와 같은 규칙)."""
    acc, i, n = 0.0, start, len(pts)
    while acc < dist and i + 1 < n:
        acc += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        i += 1
    return i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--fb-gain", type=float, nargs="+", default=[1.0, 0.8, 0.6])
    ap.add_argument("--kus", type=float, default=DEFAULT_KUS)
    ap.add_argument("--pose", default="/pf/pose/odom")
    ap.add_argument("--lut", default=None)
    args = ap.parse_args()

    dbs = glob.glob(os.path.join(args.bag, "*.db3")) if os.path.isdir(args.bag) else [args.bag]
    if not dbs:
        print("bag을 찾을 수 없다:", args.bag); sys.exit(1)

    lut = Lut(args.lut)
    con = sqlite3.connect(dbs[0])
    cur = con.cursor()
    topics = dict(cur.execute("SELECT name,id FROM topics").fetchall())
    typemap = dict(cur.execute("SELECT name,type FROM topics").fetchall())
    for need in (args.pose, "/drive_autonomous", "/local_waypoints"):
        if need not in topics:
            print(f"토픽 없음: {need}"); sys.exit(1)

    odom = read_topic(cur, topics, typemap, args.pose)
    drive = read_topic(cur, topics, typemap, "/drive_autonomous")
    local = read_topic(cur, topics, typemap, "/local_waypoints")
    con.close()

    ots = np.array([t for t, _ in odom], dtype=float)
    lts = np.array([t for t, _ in local], dtype=float)

    rows = []
    for ts, dm in drive:
        if abs(dm.drive.speed) < 0.5:
            continue
        o = odom[int(np.argmin(np.abs(ots - ts)))][1]
        wpm = local[int(np.argmin(np.abs(lts - ts)))][1]
        if len(wpm.wpnts) < 3:
            continue
        px, py = o.pose.pose.position.x, o.pose.pose.position.y
        v = o.twist.twist.linear.x
        if v < 1.0:
            continue
        yaw = yaw_of(o.pose.pose.orientation)

        pts = [(w.x_m, w.y_m) for w in wpm.wpnts]
        d2 = [(px - x) ** 2 + (py - y) ** 2 for x, y in pts]
        ci = int(np.argmin(d2))
        lat_err = math.sqrt(d2[ci])

        kappa = wpm.wpnts[ci].kappa_radpm
        L1 = L1_OFFSET + v * L1_SPEED_GAIN
        if abs(kappa) > 0.3:
            L1 *= (1.0 - 0.25 * min(1.0, (abs(kappa) - 0.3) / 1.0))
        L1 = max(max(T_CLIP_MIN, math.sqrt(2.0) * lat_err), min(L1, T_CLIP_MAX))

        ia = walk_forward(pts, ci, L1)
        tx, ty = pts[ia]
        vx, vy = tx - px, ty - py
        norm = math.hypot(vx, vy)
        if norm < 1e-5:
            continue
        sin_eta = max(-1.0, min(1.0, (-math.sin(yaw) * vx + math.cos(yaw) * vy) / norm))

        # steering_speed_cap_measured=true → 조향용 속도는 실측으로 상한
        v_lu = min(wpm.wpnts[ci].vx_mps, v)
        denom = max(norm, L1_MIN_DENOM)
        a_lat = 2.0 * v_lu * v_lu / denom * sin_eta

        d_lut, sat = lut.lookup(a_lat, v_lu)
        a_ff = kappa * v_lu * v_lu
        row = dict(v=v, v_lu=v_lu, kappa=kappa, a_lat=a_lat, a_ff=a_ff,
                   d_lut=d_lut, sat=sat, pub=dm.drive.steering_angle,
                   ceil=lut.ceiling(v_lu), lat_err=lat_err)
        for g in args.fb_gain:
            a_cmd = max(-MLA, min(MLA, a_ff + g * (a_lat - a_ff)))
            row[f"d_bic_{g}"] = bicycle(a_cmd, v_lu, args.kus)
        rows.append(row)

    if not rows:
        print("유효 샘플 없음"); sys.exit(1)

    n = len(rows)
    pub = np.array([r["pub"] for r in rows])
    d_lut = np.array([r["d_lut"] for r in rows])
    # 발행값은 LUT 출력에 스케일러·1/reach·trim이 더 붙는다. 재구성 신뢰도는 상관으로 본다.
    recon = d_lut / REACH_RATIO
    corr = np.corrcoef(np.abs(recon), np.abs(pub))[0, 1]

    print(f"\n=== {os.path.basename(os.path.normpath(args.bag))} — 샘플 {n} ===")
    print(f"재구성 검증: |LUT/reach| vs |실제발행| 상관 r = {corr:.3f}  "
          f"(높을수록 재구성이 컨트롤러를 잘 따라간 것)")
    sat_n = sum(1 for r in rows if r["sat"])
    ceil_n = sum(1 for r in rows if r["ceil"] > 0 and abs(r["d_lut"]) >= 0.95 * r["ceil"])
    print(f"LUT 포화 {sat_n/n*100:.1f}%  |  LUT 조향천장 구속 {ceil_n/n*100:.1f}%")

    print(f"\n{'fb_gain':>8} {'|δ| 중앙':>9} {'LUT 대비':>9} {'p95':>8} {'풀락초과':>9}")
    med_lut = float(np.median(np.abs(d_lut)))
    print(f"{'(LUT)':>8} {med_lut:9.4f} {'—':>9} "
          f"{float(np.percentile(np.abs(d_lut),95)):8.4f} "
          f"{sum(1 for x in d_lut if abs(x)>STEER_LIMIT)/n*100:8.1f}%")
    for g in args.fb_gain:
        arr = np.abs(np.array([r[f"d_bic_{g}"] for r in rows]))
        med = float(np.median(arr))
        print(f"{g:8.2f} {med:9.4f} {med/med_lut*100-100:+8.1f}% "
              f"{float(np.percentile(arr,95)):8.4f} "
              f"{float((arr>STEER_LIMIT).mean()*100):8.1f}%")

    # 속도대별 — 고속에서 벌어지는지 확인(LUT 천장 붕괴의 직접 증거)
    print(f"\n{'속도대':>10} {'n':>6} {'LUT중앙':>9} {'bicycle(1.0)':>13} {'차이':>8} {'천장구속':>9}")
    for lo, hi in [(1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 9)]:
        sel = [r for r in rows if lo <= r["v_lu"] < hi]
        if len(sel) < 10:
            continue
        a = np.median([abs(r["d_lut"]) for r in sel])
        b = np.median([abs(r["d_bic_1.0"]) for r in sel])
        cn = sum(1 for r in sel if r["ceil"] > 0 and abs(r["d_lut"]) >= 0.95 * r["ceil"])
        print(f"{lo}~{hi:<8} {len(sel):6d} {a:9.4f} {b:13.4f} "
              f"{(b/a*100-100 if a>1e-6 else 0):+7.1f}% {cn/len(sel)*100:8.1f}%")

    # FF가 실제로 얼마나 일하는지 — a_ff/a_lat 비중
    ratio = [abs(r["a_ff"]) / max(abs(r["a_lat"]), 1e-6) for r in rows if abs(r["a_lat"]) > 0.5]
    if ratio:
        print(f"\nFF 기여도 a_ff/a_lat: 중앙 {np.median(ratio):.2f} / "
              f"p10 {np.percentile(ratio,10):.2f} / p90 {np.percentile(ratio,90):.2f}")
        print("  (1.0 근처 = 경로 곡률이 명령의 대부분을 설명 → FF가 그 몫을 지연 없이 담당)")


if __name__ == "__main__":
    main()
