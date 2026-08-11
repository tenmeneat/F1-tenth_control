#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_sector_clearance.py — 섹터별 벽 여유·그립·추종오차 실측  [섹터 레이어 게이트]
================================================================================
"이 코너의 a_lat 권한(max_lateral_accel)을 올려도 되는가"를 bag 하나로 판정한다.

2026-08-11 배경: 랩타임 레버가 사실상 `max_lateral_accel` 하나뿐인데(현 트랙 스윕:
6.0→7.0이 −0.505 s, understeer_gradient·max_speed·steering_reach_ratio는 전부 0.000 s),
전역 상향은 0807 전복 코너 때문에 막혀 있다. 섹터별로 열려면 **섹터별 여유**를 재야 한다.

🔴 플래너가 주는 `d_left`/`d_right`로 판정하면 안 된다 — 0807 실측에서 0.16~0.23 m
   낙관적이었다(같은 웨이포인트를 3랩 통과하며 전부 음수 오차). 이 스크립트는 `/scan`
   실측으로 재고, 주장값과의 차이도 같이 뽑는다.

내는 것 (섹터 × 랩):
  1) `/scan` 실측 벽 여유          — 차체 옆면 ~ 벽. **랩 간 단조감소** 검출이 핵심
  2) 실측 a_lat (자이로 v·ψ̇)      — 지금 실제로 쓰고 있는 그립
  3) 라인 대비 횡오차              — 여유를 먹는 다른 축
  4) 주장 d_left/d_right vs 실측   — 경계 데이터 품질
  5) 섹터 경계 s 구간              — sector_tuner YAML 초안

사용법:
  source /opt/ros/<distro>/setup.bash && source ~/2026_IFAC/install/setup.bash
  python3 analyze_sector_clearance.py <bag폴더 | .db3>
        [--pose /pf/pose/odom] [--scan /scan] [--imu /sensors/imu/raw]
        [--corner-kappa 0.30] [--min-sector-len 0.6] [--yaml sectors.yaml]

⚠️ 트랙에 장애물(콘·상대차)이 있으면 `/scan`은 그걸 벽으로 본다. 여유 판정 관점에선
   그게 **맞는** 거동이지만(피해야 할 물체인 건 같다), 경계 데이터 품질 비교(4번)는
   오염된다. 장애물 없는 클린 랩 bag으로 재는 것이 원칙이고, 4번 표의 랩별 산포가
   갑자기 커지면 장애물 혼입을 의심할 것.
"""
import sys, os, glob, sqlite3, math, argparse
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── 차량 상수 (control_map_node / CLAUDE.md와 동일 기준) ──────────────────────
CAR_HALF_WIDTH = 0.155        # 차체 폭 0.31 m의 절반
LASER_DX = 0.27               # base_link → laser (tf_static 실측, 회전 없음)
DEG2RAD = math.pi / 180.0     # VESC IMU는 deg/s로 발행(sensor_msgs 규약 위반)

# 이 s 창 안의 빔만 "지금 차 옆의 벽"으로 본다. 0.3 m면 차체 길이(0.58)의 절반 수준이라
# 코너에서 앞뒤 벽이 섞여 들어오지 않으면서도 빔이 충분히 잡힌다.
LATERAL_S_WINDOW = 0.30


# ══ 1. bag 로딩 ══════════════════════════════════════════════════════════════
def read_topic(bag: str, topic: str):
    """(t[s], msg) 리스트. 없으면 빈 리스트."""
    dbs = sorted(glob.glob(os.path.join(bag, "*.db3")))
    if not dbs and bag.endswith(".db3"):
        dbs = [bag]
    out = []
    for db in dbs:
        con = sqlite3.connect(db)
        try:
            rows = con.execute("SELECT id, name, type FROM topics").fetchall()
        except sqlite3.DatabaseError:
            con.close()
            continue
        for tid, name, ttype in rows:
            if name != topic:
                continue
            try:
                msg_t = get_message(ttype)
            except Exception:
                continue
            for ts, data in con.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
            ):
                out.append((ts * 1e-9, deserialize_message(bytes(data), msg_t)))
        con.close()
    return out


def yaw_of(msg) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ══ 2. 라인 Frenet 투영 ══════════════════════════════════════════════════════
class Raceline:
    """글로벌 라인을 촘촘히 재샘플해 (s, d) 투영을 제공한다.

    웨이포인트 간격(0.25 m)에서 바로 최근접을 잡으면 d 오차가 최대 간격/2만큼 생겨
    벽 여유 판정에 그대로 실린다 → 0.05 m로 선형 보간해서 쓴다.
    """

    def __init__(self, wpnts, dense_step: float = 0.05):
        self.xy = np.array([[w.x_m, w.y_m] for w in wpnts], dtype=np.float64)
        self.kappa = np.abs(np.array([w.kappa_radpm for w in wpnts], dtype=np.float64))
        self.vx = np.array([w.vx_mps for w in wpnts], dtype=np.float64)
        self.d_left_claim = np.array([w.d_left for w in wpnts], dtype=np.float64)
        self.d_right_claim = np.array([w.d_right for w in wpnts], dtype=np.float64)

        closed = np.vstack([self.xy, self.xy[0]])
        seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        self.s_wp = np.r_[0.0, np.cumsum(seg)[:-1]]
        self.length = float(seg.sum())

        # 촘촘한 라인 + 접선
        s_dense = np.arange(0.0, self.length, dense_step)
        s_closed = np.r_[self.s_wp, self.length]
        self.dense = np.column_stack([
            np.interp(s_dense, s_closed, np.r_[self.xy[:, 0], self.xy[0, 0]]),
            np.interp(s_dense, s_closed, np.r_[self.xy[:, 1], self.xy[0, 1]]),
        ])
        self.s_dense = s_dense
        tang = np.roll(self.dense, -1, axis=0) - np.roll(self.dense, 1, axis=0)
        self.tang = tang / np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)

        try:
            from scipy.spatial import cKDTree
            self._tree = cKDTree(self.dense)
        except ImportError:
            self._tree = None

    def project(self, pts: np.ndarray):
        """(N,2) → (s, d). d는 진행방향 기준 **왼쪽이 양수**."""
        if self._tree is not None:
            _, idx = self._tree.query(pts)
        else:                                            # scipy 없으면 브루트포스
            d2 = ((pts[:, None, :] - self.dense[None, :, :]) ** 2).sum(axis=2)
            idx = np.argmin(d2, axis=1)
        rel = pts - self.dense[idx]
        t = self.tang[idx]
        d = t[:, 0] * rel[:, 1] - t[:, 1] * rel[:, 0]    # cross(tangent, rel) = 왼쪽 양수
        s = self.s_dense[idx] + (t[:, 0] * rel[:, 0] + t[:, 1] * rel[:, 1])
        return np.mod(s, self.length), d

    def wp_index(self, s: np.ndarray) -> np.ndarray:
        return np.clip(np.searchsorted(self.s_wp, np.mod(s, self.length), side="right") - 1,
                       0, len(self.s_wp) - 1)


# ══ 3. 섹터 분할 ═════════════════════════════════════════════════════════════
def split_sectors(line: Raceline, corner_kappa: float, min_len: float):
    """|κ| > corner_kappa 연속구간 = 코너. 가장 완만한 점에서 원을 끊어 직선에서 시작한다."""
    n = len(line.kappa)
    is_corner = line.kappa > corner_kappa
    start = int(np.argmin(line.kappa))
    order = [(start + j) % n for j in range(n)]

    raw, cur, begin = [], is_corner[order[0]], 0
    for j in range(1, n + 1):
        nxt = is_corner[order[j % n]] if j < n else None
        if j == n or nxt != cur:
            raw.append((cur, np.array([order[q] for q in range(begin, j)])))
            begin, cur = j, nxt

    seg_len = np.r_[np.diff(line.s_wp), line.length - line.s_wp[-1]]
    # 너무 짧은 조각은 앞 섹터에 흡수 — 안 그러면 노이즈 한 점이 섹터를 쪼갠다
    sectors = []
    for corner, idx in raw:
        if sectors and seg_len[idx].sum() < min_len:
            sectors[-1] = (sectors[-1][0], np.r_[sectors[-1][1], idx])
        else:
            sectors.append((corner, idx))
    if len(sectors) > 1 and seg_len[sectors[0][1]].sum() < min_len:
        sectors[-1] = (sectors[-1][0], np.r_[sectors[-1][1], sectors[0][1]])
        sectors.pop(0)
    return sectors, seg_len


# ══ 4. 메인 ══════════════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(description="섹터별 벽 여유·그립·추종오차 실측")
    ap.add_argument("bag")
    ap.add_argument("--pose", default="/pf/pose/odom", help="위치추정 odom (기본 MCL)")
    ap.add_argument("--odom", default="/odom", help="속도 소스 (VESC 휠)")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--imu", default="", help="비우면 /sensors/imu/raw → /imu/data 순으로 자동")
    ap.add_argument("--corner-kappa", type=float, default=0.30, help="코너 판정 |κ| [rad/m]")
    ap.add_argument("--min-sector-len", type=float, default=0.6, help="섹터 최소 길이 [m]")
    ap.add_argument("--min-speed", type=float, default=1.5, help="이 속도 미만 샘플 제외 [m/s]")
    ap.add_argument("--scan-max-range", type=float, default=6.0, help="이보다 먼 빔 무시 [m]")
    ap.add_argument("--yaml", default="", help="섹터 초안 YAML 출력 경로")
    args = ap.parse_args()

    bag = os.path.expanduser(args.bag)

    # ── 4.1 로딩 ─────────────────────────────────────────────────────────────
    gw = read_topic(bag, "/global_waypoints")
    if not gw:
        print("[ERROR] /global_waypoints가 없다 — 라인 기준 없이는 섹터를 못 자른다.")
        return 1
    line = Raceline(gw[-1][1].wpnts)

    pose = read_topic(bag, args.pose)
    if not pose:
        print(f"[ERROR] {args.pose} 없음")
        return 1
    scans = read_topic(bag, args.scan)
    odom = read_topic(bag, args.odom) or pose

    imu_topic = args.imu
    imu = read_topic(bag, imu_topic) if imu_topic else []
    if not imu:
        for cand in ("/sensors/imu/raw", "/imu/data"):
            imu = read_topic(bag, cand)
            if imu:
                imu_topic = cand
                break

    t_p = np.array([t for t, _ in pose])
    x_p = np.array([m.pose.pose.position.x for _, m in pose])
    y_p = np.array([m.pose.pose.position.y for _, m in pose])
    yaw_p = np.unwrap(np.array([yaw_of(m) for _, m in pose]))

    t_v = np.array([t for t, _ in odom])
    v_v = np.array([m.twist.twist.linear.x for _, m in odom])

    dm = read_topic(bag, "/drive_mode")
    if dm:
        t_a = np.array([t for t, _ in dm])
        f_a = np.array([1.0 if m.data == "autonomous" else 0.0 for _, m in dm])
    else:
        t_a, f_a = np.array([t_p[0], t_p[-1]]), np.array([1.0, 1.0])

    print("=" * 96)
    print(f"bag           : {bag}")
    print(f"라인          : {len(line.xy)}점 / {line.length:.2f} m / 최대|κ| {line.kappa.max():.3f}")
    print(f"토픽          : pose {args.pose} {len(pose)} | scan {len(scans)} | imu {imu_topic or '없음'} {len(imu)}")
    print("=" * 96)
    if not scans:
        print("[WARN] /scan이 없다 — 벽 여유(1·4번)를 못 낸다. f1rec TOPICS에 /scan을 넣을 것.")

    # ── 4.2 차량 궤적 → Frenet + 랩 분할 ────────────────────────────────────
    s_car, d_car = line.project(np.column_stack([x_p, y_p]))
    v_car = np.interp(t_p, t_v, v_v)
    auto = np.interp(t_p, t_a, f_a) > 0.5
    valid = auto & (v_car > args.min_speed)

    # 랩 번호: s가 뒤로 크게 점프하면 한 바퀴 넘긴 것 (MCL 지터로 인한 잔진동은 무시)
    lap = np.zeros(len(s_car), dtype=int)
    cur = 0
    for i in range(1, len(s_car)):
        if s_car[i - 1] - s_car[i] > line.length * 0.5:
            cur += 1
        lap[i] = cur
    n_lap = int(lap[valid].max() - lap[valid].min() + 1) if valid.any() else 0

    # ── 4.3 요레이트 → 실측 a_lat ───────────────────────────────────────────
    if imu:
        t_i = np.array([t for t, _ in imu])
        wz = np.array([m.angular_velocity.z for _, m in imu])
        # VESC는 deg/s로 쏜다. |값|이 rad/s 스케일을 크게 넘으면 deg/s로 판정.
        scale = DEG2RAD if np.percentile(np.abs(wz), 99) > 15.0 else 1.0
        wz_car = np.interp(t_p, t_i, wz) * scale
    else:
        wz_car = np.full(len(t_p), np.nan)
    alat_car = np.abs(v_car * wz_car)

    # ── 4.4 스캔 → 벽 여유 ──────────────────────────────────────────────────
    #   각 스캔마다 차 옆(|Δs| < LATERAL_S_WINDOW)의 빔만 골라 좌/우 최근접을 잡는다.
    #   차의 실제 위치 기준이라 "라인이 좋은가"가 아니라 "지금 안 부딪히는가"를 잰다.
    scan_s = np.full(len(scans), np.nan)
    scan_lap = np.zeros(len(scans), dtype=int)
    scan_clr_l = np.full(len(scans), np.nan)
    scan_clr_r = np.full(len(scans), np.nan)
    scan_ok = np.zeros(len(scans), dtype=bool)
    # 라인 기준 경계(주장값 비교용). ⚠️ 전 스캔·전 빔의 **최소값**으로 모으면 안 된다 —
    #    이상 빔(바닥 반사·트랙 밖 물체) 하나가 그 칸을 영구히 끌어내린다. 2026-08-11에
    #    실제로 그렇게 짜서 폭을 0.4~1.2 m 과소평가했고, 포즈·라인을 안 쓰는 독립 추정
    #    (차 좌표계 ±90° 빔)과 대조해서야 발견했다. 스캔마다 한 개씩만 뽑아 **중앙값**으로 모은다.
    bnd_samples_l = [[] for _ in range(len(line.xy))]
    bnd_samples_r = [[] for _ in range(len(line.xy))]

    for si, (ts, sc) in enumerate(scans):
        if ts < t_p[0] or ts > t_p[-1]:
            continue
        j = int(np.searchsorted(t_p, ts))
        j = min(max(j, 0), len(t_p) - 1)
        if not valid[j]:
            continue
        cx, cy, cyaw = np.interp(ts, t_p, x_p), np.interp(ts, t_p, y_p), np.interp(ts, t_p, yaw_p)

        r = np.asarray(sc.ranges, dtype=np.float64)
        ang = sc.angle_min + np.arange(len(r)) * sc.angle_increment
        good = np.isfinite(r) & (r > sc.range_min) & (r < min(sc.range_max, args.scan_max_range))
        if good.sum() < 20:
            continue
        r, ang = r[good], ang[good]
        # laser frame → base_link → map
        bx = LASER_DX + r * np.cos(ang)
        by = r * np.sin(ang)
        cs, sn = math.cos(cyaw), math.sin(cyaw)
        pts = np.column_stack([cx + cs * bx - sn * by, cy + sn * bx + cs * by])

        s_b, d_b = line.project(pts)
        scan_s[si] = s_car[j]
        scan_lap[si] = lap[j]

        # 차 옆(|Δs| < 창)의 빔만 = "지금 이 위치에서 좌/우로 얼마 남았나"
        ds = np.abs(np.mod(s_b - s_car[j] + line.length * 0.5, line.length) - line.length * 0.5)
        near = ds < LATERAL_S_WINDOW
        if near.sum() < 3:
            continue
        dn = d_b[near]
        left = dn[dn > d_car[j]]
        right = dn[dn < d_car[j]]
        wi_car = int(line.wp_index(np.array([s_car[j]]))[0])
        if left.size:
            nearest = float(np.min(left))                       # 라인 기준 좌측 경계 위치
            scan_clr_l[si] = (nearest - d_car[j]) - CAR_HALF_WIDTH
            bnd_samples_l[wi_car].append(nearest)
        if right.size:
            nearest = float(np.max(right))                      # 라인 기준 우측 경계 위치(음수)
            scan_clr_r[si] = (d_car[j] - nearest) - CAR_HALF_WIDTH
            bnd_samples_r[wi_car].append(-nearest)
        scan_ok[si] = True

    bnd_l = np.array([np.median(v) if len(v) >= 3 else np.nan for v in bnd_samples_l])
    bnd_r = np.array([np.median(v) if len(v) >= 3 else np.nan for v in bnd_samples_r])

    # ── 4.5 섹터 분할 ───────────────────────────────────────────────────────
    sectors, seg_len = split_sectors(line, args.corner_kappa, args.min_sector_len)
    wp_of_car = line.wp_index(s_car)
    wp_of_scan = line.wp_index(np.nan_to_num(scan_s))

    print(f"\n자율주행 샘플 {int(valid.sum())} / 랩 {n_lap} / 섹터 {len(sectors)}\n")

    # ── 4.6 섹터 요약 ───────────────────────────────────────────────────────
    hdr = (f"{'섹':>2} {'종류':4} {'s구간[m]':>13} {'길이':>5} {'κp90':>5} "
           f"{'실측a_lat p50/p95':>17} {'횡오차 p50/p95':>14} {'벽여유 p5/min':>14} {'주장−실측폭':>11}")
    print(hdr)
    print("-" * len(hdr))
    rows = []
    for si, (is_corner, idx) in enumerate(sectors):
        m_car = valid & np.isin(wp_of_car, idx)
        if m_car.sum() < 10:
            continue
        m_scan = scan_ok & np.isin(wp_of_scan, idx)
        clr = np.concatenate([scan_clr_l[m_scan], scan_clr_r[m_scan]])
        clr = clr[np.isfinite(clr)]
        err = np.abs(d_car[m_car])
        al = alat_car[m_car]
        al = al[np.isfinite(al)]

        claim = line.d_left_claim[idx] + line.d_right_claim[idx]
        meas = bnd_l[idx] + bnd_r[idx]
        both = np.isfinite(meas)
        gap = float(np.median(claim[both] - meas[both])) if both.any() else float("nan")

        s0, s1 = line.s_wp[idx[0]], line.s_wp[idx[-1]] + seg_len[idx[-1]]
        rows.append(dict(i=si, corner=is_corner, s0=s0, s1=s1, idx=idx,
                         length=seg_len[idx].sum(), k90=float(np.percentile(line.kappa[idx], 90)),
                         alat95=float(np.percentile(al, 95)) if al.size else float("nan"),
                         clr_p5=float(np.percentile(clr, 5)) if clr.size else float("nan"),
                         clr_min=float(clr.min()) if clr.size else float("nan"),
                         err95=float(np.percentile(err, 95))))
        print(f"{si:>2} {'코너' if is_corner else '직선':4} {s0:6.1f}~{s1:6.1f} "
              f"{seg_len[idx].sum():5.2f} {np.percentile(line.kappa[idx], 90):5.3f} "
              f"{(np.percentile(al, 50) if al.size else float('nan')):7.2f}/"
              f"{(np.percentile(al, 95) if al.size else float('nan')):5.2f}  "
              f"{np.percentile(err, 50):6.3f}/{np.percentile(err, 95):5.3f}  "
              f"{(np.percentile(clr, 5) if clr.size else float('nan')):6.3f}/"
              f"{(clr.min() if clr.size else float('nan')):5.3f}  "
              f"{gap:+10.3f}")

    print("\n※ 벽여유 = 차체 옆면 ~ 벽 [m] (차체 반폭 0.155 제외 후). p5를 판정에, min은 참고로 쓴다"
          " — min은 단일 샘플이라 이상 빔 하나에 흔들린다.")
    print("※ 주장−실측폭 = 플래너 `d_left+d_right`가 라이다 실측 폭보다 이만큼 낙관적(+) [m]")

    # ── 4.7 랩별 벽 여유 — 🔴 단조감소가 가장 중요한 신호 ────────────────────
    if scan_ok.any():
        laps = sorted(set(scan_lap[scan_ok].tolist()))
        print(f"\n랩별 벽 여유 p5 [m] — 🔴 랩마다 **단조 감소**면 라인이 아니라 드리프트/누적 오차다")
        print("   (0807 전복이 정확히 이 패턴이었다: 0.381 → 0.187 → 0.117, 3랩째 접촉)")
        head = f"{'섹':>2} {'종류':4} " + "".join(f"{'랩'+str(l):>8}" for l in laps) + f"{'추세':>9}"
        print(head)
        print("-" * len(head))
        for r in rows:
            cells, series = "", []
            for l in laps:
                m = scan_ok & np.isin(wp_of_scan, r["idx"]) & (scan_lap == l)
                c = np.concatenate([scan_clr_l[m], scan_clr_r[m]])
                c = c[np.isfinite(c)]
                if c.size >= 5:
                    val = float(np.percentile(c, 5))
                    cells += f"{val:8.3f}"
                    series.append((len(series), val))
                else:
                    cells += f"{'-':>8}"
            # 랩당 추세 [m/랩] — 음수가 크면 매 랩 여유가 깎이고 있다는 뜻
            if len(series) >= 4:
                a = np.array(series, dtype=float)
                slope = float(np.polyfit(a[:, 0], a[:, 1], 1)[0])
                r["trend"] = slope
                cells += f"{slope:+9.4f}"
            else:
                r["trend"] = float("nan")
                cells += f"{'-':>9}"
            print(f"{r['i']:>2} {'코너' if r['corner'] else '직선':4} {cells}")

    # ── 4.8 판정 ────────────────────────────────────────────────────────────
    # 임계값 근거: 0807 전복은 라이다 실측 여유 **0.117 m**에서 벽에 닿았고, 그 직전 랩들이
    # 0.381 → 0.187로 이미 깎이고 있었다. 그래서 p5 0.20은 "접촉 지점의 1.7배", min 0.12는
    # "접촉 지점 바로 위"를 뜻한다. 근거 있는 하한이지 넉넉한 값이 아니다 — 올릴 땐 한 섹터씩.
    CLR_P5_MIN, CLR_ABS_MIN, ERR_P95_MAX, TREND_MIN = 0.20, 0.12, 0.35, -0.010
    print("\n" + "=" * 96)
    print("판정 — max_lateral_accel 섹터 스케일 상향 가능 여부")
    print(f"  기준: 벽여유 p5 ≥ {CLR_P5_MIN} m  AND  min ≥ {CLR_ABS_MIN} m  AND  "
          f"횡오차 p95 ≤ {ERR_P95_MAX} m  AND  랩 추세 ≥ {TREND_MIN} m/랩")
    print("  (0807 전복 접촉 지점 0.117 m 기준으로 잡은 값 — 통과해도 한 섹터씩 +0.1만)")
    print("=" * 96)
    for r in rows:
        reasons = []
        if not np.isfinite(r["clr_p5"]):
            reasons.append("벽여유 미측정(/scan 없음)")
        else:
            if r["clr_p5"] < CLR_P5_MIN:
                reasons.append(f"벽여유 p5 {r['clr_p5']:.3f} < {CLR_P5_MIN}")
            if r["clr_min"] < CLR_ABS_MIN:
                reasons.append(f"벽여유 min {r['clr_min']:.3f} < {CLR_ABS_MIN}")
        if r["err95"] > ERR_P95_MAX:
            reasons.append(f"횡오차 p95 {r['err95']:.3f} > {ERR_P95_MAX}")
        if np.isfinite(r.get("trend", float("nan"))) and r["trend"] < TREND_MIN:
            reasons.append(f"랩마다 여유 {r['trend']*100:.1f} cm씩 감소")
        if not r["corner"]:
            reasons.append("직선 — 그립 캡이 비활성이라 올려도 이득 0")
        verdict = "🟢 상향 가능" if not reasons else "🔴 보류"
        extra = ""
        if r["corner"] and np.isfinite(r["alat95"]):
            extra = f" (실측 a_lat p95 {r['alat95']:.2f})"
        print(f"  섹터 {r['i']} ({'코너' if r['corner'] else '직선'}, "
              f"s {r['s0']:.1f}~{r['s1']:.1f}): {verdict}{extra}"
              + (f" — {', '.join(reasons)}" if reasons else ""))

    # ── 4.9 sector_tuner YAML 초안 ──────────────────────────────────────────
    if args.yaml:
        # 🔑 분석 섹터 경계(|κ|=corner_kappa 교차점)를 그대로 쓰면 안 된다 — 거기는 그립 캡이
        #    이미 활성인 지점이라, MCL s 지터가 경계를 넘나들 때마다 캡이 계단으로 토글한다.
        #    경계를 인접 ±snap 안의 **|κ| 최소점**으로 옮기면 캡이 비활성인 곳에서 전환되므로
        #    지터가 속도 명령에 도달하지 못한다. 필터링보다 경계 배치가 근본 대책이다.
        snap = 1.5

        def snap_to_flat(s: float) -> float:
            """s를 ±snap 안에서 |κ|가 가장 작은 웨이포인트로 옮긴다."""
            off = np.abs((line.s_wp - s + line.length * 0.5) % line.length - line.length * 0.5)
            cand = np.flatnonzero(off <= snap)
            if cand.size == 0:
                return s
            return float(line.s_wp[cand[int(np.argmin(line.kappa[cand]))]])

        # scale을 거는 대상은 코너뿐이다(직선은 그립 캡이 비활성이라 이득 0). 코너만 내보내고
        # 나머지 구간은 컨트롤러에서 암묵적 1.0으로 둔다 — 항목 수가 줄어 룩업도 튜닝도 단순해진다.
        corners = [r for r in rows if r["corner"]]
        with open(args.yaml, "w", encoding="utf-8") as f:
            f.write("# analyze_sector_clearance.py 자동 생성 — 검토 후 scale 조정\n")
            f.write("# scale은 max_lateral_accel에 곱해진다. 1.0 미만은 컨트롤러가 clamp로 막는다\n")
            f.write("# (전역 MLA를 보수값으로 두고 여기서만 여는 설계 — 미수신 시 항상 느려지는 쪽)\n")
            f.write("# 여기 없는 구간은 자동으로 scale 1.0. 직선은 그립 캡이 비활성이라 넣어도 이득 0.\n")
            f.write(f"track_length: {line.length:.3f}   # 컨트롤러가 이 값으로 라인 동일성을 검증한다\n")
            f.write("blend_len: 0.5                     # 경계 선형 블렌딩 폭 [m]\n")
            f.write("sectors:\n")
            for r in corners:
                s0, s1 = snap_to_flat(r["s0"]), snap_to_flat(r["s1"])
                k0 = line.kappa[line.wp_index(np.array([s0]))[0]]
                k1 = line.kappa[line.wp_index(np.array([s1]))[0]]
                ok = (np.isfinite(r["clr_p5"]) and r["clr_p5"] >= CLR_P5_MIN
                      and r["clr_min"] >= CLR_ABS_MIN and r["err95"] <= ERR_P95_MAX)
                tag = (f"섹터{r['i']} κp90 {r['k90']:.3f}, 경계κ {k0:.3f}/{k1:.3f}, "
                       f"{'🟢 상향 가능' if ok else '🔴 보류'}")
                # 🔴 랩을 넘는 구간(s0 > s1)은 **두 줄로 쪼개서** 쓴다. 컨트롤러도
                #    scripts/sector_pub.py도 s_start < s_end만 받는다 — 안 쪼개면 발행 단계에서
                #    통째로 거부된다. 쪼개도 결승선에서 파이지 않는다(컨트롤러가 값이 실제로
                #    바뀌는 전이점에만 블렌딩을 걸기 때문, 2026-08-11 런타임 검증).
                spans = ([(s0, s1, "")] if s0 <= s1
                         else [(s0, line.length, " (랩 넘어감 1/2)"), (0.0, s1, " (랩 넘어감 2/2)")])
                for a, b, note in spans:
                    f.write(f"  - {{ s_start: {a:6.2f}, s_end: {b:6.2f}, scale: 1.0 }}"
                            f"   # {tag}{note}\n")
        print(f"\n섹터 초안 저장: {args.yaml}  (코너 {len(corners)}개, 경계를 ±{snap} m 안 κ 최소점으로 스냅)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
