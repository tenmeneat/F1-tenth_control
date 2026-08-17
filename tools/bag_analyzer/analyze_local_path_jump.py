"""Frenet s 글리치가 **실제로 /local_waypoints를 흔드는가** — 0807 벽충돌(②-h)의 재발 검사.

s 자체가 튀는 것보다 중요한 건 그게 컨트롤러가 따라가는 경로를 바꾸느냐다.
state_machine이 s로 윈도우를 자르므로, s가 튀면 경로 전체가 다른 곳으로 옮겨간다.

측정
  1) s 점프의 **지속성** — 1샘플 글리치(자기수복)인가, 여러 샘플 지속인가
  2) /local_waypoints 첫 점의 이동량 vs 차량 이동량 — 경로가 차보다 훨씬 많이 움직이면 점프
  3) 경로 **헤딩 반전** — local[0]의 psi가 직전 대비 90° 이상 뒤집혔나 (0807의 결정적 증상)
  4) 구간별 분포 — "전구간에서 정확한가"
"""
import sqlite3, glob, math, os, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topic(cur, topics, typemap, name):
    mt = get_message(typemap[name])
    rows = cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                       (topics[name],)).fetchall()
    return [(ts, deserialize_message(d, mt)) for ts, d in rows]


_DEFAULT = [("0815", "run_0815_202844"), ("0815", "run_0815_205059"),
            ("0816", "run_0816_230340"), ("0816", "run_0816_225916"), ("0816", "run_0816_230107")]
# 인자로 bag 디렉터리를 주면 그것만 본다: analyze_local_path_jump.py ~/rosbag_log/0817/run_*
_ARGS = [os.path.expanduser(p).rstrip("/") for p in sys.argv[1:]]
BAGS = [(None, p) for p in _ARGS] or _DEFAULT

for day, name in BAGS:
    dbs = glob.glob((name if day is None else
                     f"/home/tenmeneat/rosbag_log/{day}/{name}") + "/*.db3")
    name = os.path.basename(name)
    if not dbs:
        continue
    con = sqlite3.connect(dbs[0]); cur = con.cursor()
    topics = dict(cur.execute("SELECT name,id FROM topics").fetchall())
    typemap = dict(cur.execute("SELECT name,type FROM topics").fetchall())
    fr = read_topic(cur, topics, typemap, "/car_state/frenet/odom")
    lw = read_topic(cur, topics, typemap, "/local_waypoints")
    pf = read_topic(cur, topics, typemap, "/pf/pose/odom")
    gw = read_topic(cur, topics, typemap, "/global_waypoints")
    con.close()

    track_len = max(w.s_m for w in gw[-1][1].wpnts)
    ft = np.array([t for t, _ in fr], float) / 1e9
    s = np.array([m.pose.pose.position.x for _, m in fr])
    pt = np.array([t for t, _ in pf], float) / 1e9
    pv = np.array([m.twist.twist.linear.x for _, m in pf])
    px = np.array([m.pose.pose.position.x for _, m in pf])
    py = np.array([m.pose.pose.position.y for _, m in pf])
    v_at_f = np.interp(ft, pt, pv)

    print("=" * 74)
    print(f"  {name}")
    print("=" * 74)

    # 1) s 점프 지속성
    ds = np.diff(s); dtf = np.diff(ft)
    ds = np.where(ds < -track_len * .5, ds + track_len, ds)
    ds = np.where(ds > track_len * .5, ds - track_len, ds)
    ok = (dtf > 5e-3) & (v_at_f[:-1] > 1.0)
    sdot = np.where(ok, ds / np.maximum(dtf, 1e-6), 0.0)
    bad = ok & (np.abs(sdot) > v_at_f[:-1] * 1.5 + 1.5)
    runs, i = [], 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j < len(bad) and bad[j]:
                j += 1
            runs.append(j - i); i = j
        else:
            i += 1
    if runs:
        runs = np.array(runs)
        print(f"[1] s 점프 {bad.sum()}건 / 연속덩어리 {len(runs)}개  "
              f"길이 중앙 {np.median(runs):.0f}샘플 / 최대 {runs.max()}샘플")
        print(f"    1샘플 자기수복 {(runs==1).sum()}개 ({(runs==1).mean()*100:.0f}%), "
              f"3샘플 이상 지속 {(runs>=3).sum()}개")
        # 점프 크기(변위)
        mag = np.abs(ds[bad])
        print(f"    점프 변위 중앙 {np.median(mag):.2f} m / p90 {np.percentile(mag,90):.2f} / "
              f"최대 {mag.max():.2f} m")
    else:
        print("[1] s 점프 없음")

    # 2·3) local_waypoints 불연속
    lt = np.array([t for t, _ in lw], float) / 1e9
    prev0 = None; prev_t = None
    lp_jump, lp_flip, cnt = 0, 0, 0
    jump_s = []
    for k, (ts, m) in enumerate(lw):
        if not m.wpnts:
            continue
        tsec = ts / 1e9
        v = np.interp(tsec, pt, pv)
        if v < 1.0:
            prev0 = None; continue
        w0 = m.wpnts[0]
        cur0 = (w0.x_m, w0.y_m, w0.psi_rad)
        if prev0 is not None and prev_t is not None:
            dtl = tsec - prev_t
            if 0 < dtl < 0.5:
                cnt += 1
                move = math.hypot(cur0[0] - prev0[0], cur0[1] - prev0[1])
                veh = v * dtl
                # 경로 시작점이 차량 이동량보다 1.0 m 이상 더 움직임 = 윈도우가 튐
                if move > veh + 1.0:
                    lp_jump += 1
                    jump_s.append(np.interp(tsec, ft, s))
                dpsi = abs(math.atan2(math.sin(cur0[2] - prev0[2]),
                                      math.cos(cur0[2] - prev0[2])))
                if dpsi > math.pi / 2:
                    lp_flip += 1
        prev0 = cur0; prev_t = tsec
    print(f"[2] /local_waypoints 시작점 점프(>차량이동+1m): {lp_jump}/{cnt} "
          f"({lp_jump/max(cnt,1)*100:.2f}%)")
    print(f"[3] 경로 헤딩 반전(>90°, 0807 크래시의 결정적 증상): {lp_flip}/{cnt} "
          f"({lp_flip/max(cnt,1)*100:.2f}%)"
          + ("   ✅ 없음" if lp_flip == 0 else "   🔴"))
    if jump_s:
        js = np.array(jump_s)
        NB = 8
        edges = np.linspace(0, track_len, NB + 1)
        hist = [((js >= edges[i]) & (js < edges[i + 1])).sum() for i in range(NB)]
        loc = "  ".join(f"{edges[i]:.0f}-{edges[i+1]:.0f}m:{h}" for i, h in enumerate(hist) if h)
        print(f"    점프 위치: {loc}")
    print()
