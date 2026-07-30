#!/usr/bin/env python3
"""bag의 토픽별 달성 레이트 검사. ROS 소싱 불필요(sqlite3만 읽는다).

랩탑에서 wifi 너머로 녹화할 때 "DDS가 메시지를 흘렸는지"를 매번 숫자로 답하기 위한 도구.
기준선(2026-07-29 젯슨 로컬 녹화 run_0729_210346): 50 Hz 토픽 전부 100.0%, /scan 99.8%.
그보다 눈에 띄게 낮으면 무선 구간에서 유실된 것이다.

주의: 아래 '기대 Hz'는 발행자 쪽 실제 주기다. /pf/pose/odom·/local_waypoints·/tf 처럼
원래 50 Hz가 아닌 토픽을 50으로 잡으면 멀쩡한 녹화가 유실로 오판된다(07-29에 실제로 겪음).
"""
import sqlite3, sys, glob, os
import numpy as np

# 발행자 실측 주기 (run_0729_210346 = 젯슨 로컬 녹화, 유실 없음으로 간주한 기준)
EXPECT = {
    "/scan": 40, "/odom": 50, "/sensors/imu/raw": 50, "/imu/data": 50,
    "/drive": 50, "/drive_autonomous": 50, "/drive_mppi": 50,
    "/commands/motor/speed": 46, "/commands/servo/position": 46,
    "/pf/pose/odom": 30, "/local_waypoints": 30, "/tf": 80,
    "/joy": 15, "/drive_mode": 20, "/estop_lock": 20, "/sensors/core": 50,
    "/car_state/frenet/odom": 30, "/state": 20, "/debug/l1_lookahead": 50,
}
# 이 토픽들은 이벤트성이라 레이트 판정 대상이 아니다
SKIP = {"/tf_static", "/global_waypoints", "/mppi_active",
        "/commands/motor/brake", "/avoid_waypoints", "/overtake_waypoints",
        "/perception/detection/raw_obstacles"}
THRESH = 0.90


def check(bag):
    dbs = sorted(glob.glob(os.path.join(bag, "*.db3"))) if os.path.isdir(bag) else [bag]
    if not dbs:
        print(f"❌ .db3 없음: {bag}"); return 1
    con = sqlite3.connect(dbs[0])
    rows = con.execute("""select t.id,t.name,count(m.id),min(m.timestamp),max(m.timestamp)
                          from topics t left join messages m on m.topic_id=t.id
                          group by t.id""").fetchall()
    rows = [r for r in rows if r[2]]
    if not rows:
        print("❌ 메시지가 없습니다 (토픽이 하나도 안 잡혔음)"); return 1
    size = sum(os.path.getsize(f) for f in dbs)
    span = (max(r[4] for r in rows) - min(r[3] for r in rows)) / 1e9

    print(f"\n{'='*76}")
    print(f"{os.path.basename(bag.rstrip('/'))}   {span:.1f}s   {size/1e6:.1f} MB   "
          f"평균 {size/span/1e3:.0f} KB/s")
    print(f"{'='*76}")
    print(f"{'토픽':28s} {'개수':>7} {'실제Hz':>7} {'기대':>5} {'달성률':>8} {'최대공백ms':>10}")

    bad, gappy = [], []
    for tid, name, cnt, a, b in sorted(rows, key=lambda r: r[1]):
        ts = np.array([x[0] for x in con.execute(
            "select timestamp from messages where topic_id=? order by timestamp", (tid,))])
        if len(ts) < 3:
            continue
        dur = (ts[-1] - ts[0]) / 1e9
        hz = (len(ts) - 1) / dur if dur > 0 else 0.0
        gap = np.diff(ts).max() / 1e6
        exp = EXPECT.get(name)
        if name in SKIP or not exp:
            print(f"{name:28s} {len(ts):7d} {hz:7.1f} {'-':>5} {'-':>8} {gap:10.0f}")
            continue
        ratio = hz / exp
        mark = ""
        if ratio < THRESH:
            mark = " ⚠ 유실 의심"; bad.append((name, ratio))
        if gap > 1000:
            mark += " ⏸ 발행 중단"; gappy.append((name, gap))
        print(f"{name:28s} {len(ts):7d} {hz:7.1f} {exp:5d} {100*ratio:7.1f}% {gap:10.0f}{mark}")

    print()
    if not bad and not gappy:
        print("✅ 유실 없음 — 모든 토픽이 기대 주기의 90% 이상")
    if bad:
        print(f"⚠️  달성률 미달 {len(bad)}건: " + ", ".join(f"{n}({100*r:.0f}%)" for n, r in bad))
        print("    무선 녹화라면 DDS 유실일 수 있다 → 젯슨 로컬 녹화(f1rec --remote)와 비교해볼 것.")
    if gappy:
        print(f"⏸  1초 이상 발행 중단 {len(gappy)}건: " + ", ".join(f"{n}({g/1000:.0f}s)" for n, g in gappy))
        print("    이건 녹화가 아니라 '발행 노드가 죽었다'는 뜻일 가능성이 높다(07-29 f110 브링업 사례).")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    sys.exit(max(check(b) for b in sys.argv[1:]))
