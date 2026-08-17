"""MCL 정확도 구간별 측정 v2 — v1의 세 결함 수정.

v1 결함
  1) 0816 bag 3개 중 2개는 **대부분 정지**다(frenet v 중앙 0.00). 통계가 정지 구간에
     지배됐고, 랩 카운트도 그래서 망가졌다 → 주행 구간만 본다.
  2) MCL 헤딩을 np.gradient로 미분해 자이로와 비교했는데, ~25 Hz 불규칙 샘플의 미분은
     양자화 노이즈를 증폭한다(σ 0.5 rad/s는 MCL 오차가 아니라 미분 노이즈).
     → **0.5 s 창의 헤딩 변화량**을 자이로 적분과 비교한다(미분 대신 적분).
  3) 랩 번호를 interp+round로 스캔에 옮겼다 → 경계에서 뭉개진다. 스캔 시각의 s를 직접 보고
     랩을 센다.
"""
import sqlite3, glob, os, math, sys
import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DEG2RAD = math.pi / 180.0
WIN = 0.5          # 헤딩 비교 창 [s]


def read_topic(cur, topics, typemap, name):
    mt = get_message(typemap[name])
    rows = cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                       (topics[name],)).fetchall()
    return [(ts, deserialize_message(d, mt)) for ts, d in rows]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


_DEFAULT = [("0816", "run_0816_230340"), ("0816", "run_0816_225916"), ("0816", "run_0816_230107"),
            ("0815", "run_0815_202844"), ("0815", "run_0815_205059")]
# 인자로 bag 디렉터리를 주면 그것만 본다: analyze_mcl_quality.py ~/rosbag_log/0817/run_*
_ARGS = [os.path.expanduser(p).rstrip("/") for p in sys.argv[1:]]
BAGS = [(None, p) for p in _ARGS] or _DEFAULT

for day, name in BAGS:
    dbs = glob.glob((name if day is None else
                     os.path.expanduser(f"~/rosbag_log/{day}/{name}")) + "/*.db3")
    name = os.path.basename(name)
    if not dbs:
        continue
    con = sqlite3.connect(dbs[0]); cur = con.cursor()
    topics = dict(cur.execute("SELECT name,id FROM topics").fetchall())
    typemap = dict(cur.execute("SELECT name,type FROM topics").fetchall())
    need = ["/pf/pose/odom", "/car_state/frenet/odom", "/sensors/imu/raw", "/scan",
            "/global_waypoints"]
    if any(n not in topics for n in need):
        print(f"{name}: 토픽 부족 — 건너뜀"); con.close(); continue
    pf = read_topic(cur, topics, typemap, "/pf/pose/odom")
    fr = read_topic(cur, topics, typemap, "/car_state/frenet/odom")
    imu = read_topic(cur, topics, typemap, "/sensors/imu/raw")
    scan = read_topic(cur, topics, typemap, "/scan")
    gw = read_topic(cur, topics, typemap, "/global_waypoints")
    con.close()

    track_len = max(w.s_m for w in gw[-1][1].wpnts)

    pt = np.array([t for t, _ in pf], float) / 1e9
    yaw = np.unwrap(np.array([yaw_of(m.pose.pose.orientation) for _, m in pf]))
    px = np.array([m.pose.pose.position.x for _, m in pf])
    py = np.array([m.pose.pose.position.y for _, m in pf])
    pv = np.array([m.twist.twist.linear.x for _, m in pf])

    ft = np.array([t for t, _ in fr], float) / 1e9
    s = np.array([m.pose.pose.position.x for _, m in fr])
    d = np.array([m.pose.pose.position.y for _, m in fr])

    it = np.array([t for t, _ in imu], float) / 1e9
    iwz = np.array([m.angular_velocity.z * DEG2RAD for _, m in imu])

    v_at_f = np.interp(ft, pt, pv)
    moving_f = v_at_f > 1.0
    drive_frac = moving_f.mean()

    print("=" * 76)
    print(f"  {name}   트랙 {track_len:.2f} m   주행(v>1) 비율 {drive_frac*100:.0f}%")
    print("=" * 76)
    if moving_f.sum() < 100:
        print("  주행 샘플 부족 — 건너뜀\n"); continue

    # ── A) Frenet s 연속성 (주행 중만) ────────────────────────────────────
    ds = np.diff(s); dtf = np.diff(ft)
    ds = np.where(ds < -track_len * 0.5, ds + track_len, ds)
    ds = np.where(ds > track_len * 0.5, ds - track_len, ds)
    ok = (dtf > 5e-3) & moving_f[:-1] & moving_f[1:]
    sdot = ds[ok] / dtf[ok]
    vref = v_at_f[:-1][ok]
    bad = np.abs(sdot) > vref * 1.5 + 1.5
    print(f"[A] Frenet s 연속성 (로컬플래닝이 이 s로 경로 윈도우를 자른다)  n={ok.sum()}")
    print(f"    ds/dt 중앙 {np.median(sdot):+.2f} vs 실측 v 중앙 {np.median(vref):.2f} m/s")
    print(f"    물리 불가 점프 {bad.sum()}건 ({bad.mean()*100:.2f}%)"
          + (f"   최대 {np.abs(sdot)[bad].max():.0f} m/s" if bad.sum() else "   ✅ 없음"))
    dm = np.abs(d[:-1][ok])
    print(f"    |d|(라인 이탈) 중앙 {np.median(dm):.3f} / p95 {np.percentile(dm,95):.3f} m")

    # ── B) MCL 헤딩 vs 자이로 — 창 적분 비교 (미분 X) ─────────────────────
    errs, s_err = [], []
    j = 0
    for i in range(len(pt)):
        t0, t1 = pt[i], pt[i] + WIN
        if t1 > pt[-1]:
            break
        k = np.searchsorted(pt, t1)
        if k >= len(pt) or pv[i] < 1.0 or pv[min(k, len(pv) - 1)] < 1.0:
            continue
        dyaw_mcl = yaw[k] - yaw[i]
        m = (it >= t0) & (it <= pt[k])
        if m.sum() < 5:
            continue
        dyaw_gyro = np.trapz(iwz[m], it[m])
        errs.append(dyaw_mcl - dyaw_gyro)
        s_err.append(np.interp(t0, ft, s))
    errs = np.array(errs); s_err = np.array(s_err)
    if len(errs) > 50:
        print(f"\n[B] MCL 헤딩 vs 자이로 — {WIN}s 창 누적 비교  n={len(errs)}")
        print(f"    오차 중앙 {np.median(errs)*180/math.pi:+.2f}° / "
              f"σ {np.std(errs)*180/math.pi:.2f}° / p95 {np.percentile(np.abs(errs),95)*180/math.pi:.2f}° "
              f"/ max {np.abs(errs).max()*180/math.pi:.1f}°")
        big = np.abs(errs) > 10 * DEG2RAD
        print(f"    10° 초과(= MCL 헤딩 점프): {big.sum()}건 ({big.mean()*100:.2f}%)")

    # ── C) 스캔 재투영 일관성 — 랩 교차 ───────────────────────────────────
    st = np.array([t for t, _ in scan], float) / 1e9
    s_at_scan = np.interp(st, ft, s)
    v_at_scan = np.interp(st, pt, pv)
    lap_s = np.zeros(len(st), int)
    L = 0
    for i in range(1, len(st)):
        if s_at_scan[i] < s_at_scan[i - 1] - track_len * 0.5:
            L += 1
        lap_s[i] = L
    nlaps = lap_s.max() + 1

    buckets = {}
    for k, (ts, m) in enumerate(scan):
        if v_at_scan[k] < 1.0:
            continue
        i = int(np.argmin(np.abs(pt - ts)))
        r = np.asarray(m.ranges, float)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        g = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 5.0))
        if g.sum() < 50:
            continue
        rr, aa = r[g], ang[g]
        P = np.column_stack([px[i] + rr * np.cos(yaw[i] + aa),
                             py[i] + rr * np.sin(yaw[i] + aa)])
        buckets.setdefault(lap_s[k] % 2, []).append((P, s_at_scan[k]))

    if nlaps >= 2 and 0 in buckets and 1 in buckets:
        E = np.vstack([p for p, _ in buckets[0]])
        SE = np.concatenate([np.full(len(p), sv) for p, sv in buckets[0]])
        O = np.vstack([p for p, _ in buckets[1]])
        dist, _ = cKDTree(O).query(E, k=1)
        keep = dist < 1.0
        print(f"\n[C] 스캔 재투영 일관성 — 랩 {nlaps}개 (짝 {len(E)}점 vs 홀 {len(O)}점)")
        print(f"    전체 중앙 {np.median(dist[keep])*100:.1f} cm / "
              f"p90 {np.percentile(dist[keep],90)*100:.1f} / p99 {np.percentile(dist[keep],99)*100:.1f}")
        NB = 10
        edges = np.linspace(0, track_len, NB + 1)
        print(f"    {'s 구간 [m]':>15} {'n':>7} {'중앙':>7} {'p90':>7} {'p99[cm]':>8}")
        for i in range(NB):
            sel = keep & (SE >= edges[i]) & (SE < edges[i + 1])
            if sel.sum() < 800:
                continue
            print(f"    {edges[i]:6.1f}~{edges[i+1]:6.1f} {sel.sum():7d} "
                  f"{np.median(dist[sel])*100:7.1f} {np.percentile(dist[sel],90)*100:7.1f} "
                  f"{np.percentile(dist[sel],99)*100:8.1f}")
    else:
        print(f"\n[C] 랩 {nlaps}개 — 교차 비교 불가")
    print()
