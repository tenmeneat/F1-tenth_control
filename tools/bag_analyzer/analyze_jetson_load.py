#!/usr/bin/env python3
"""bag의 토픽 공백 ↔ 젯슨 부하 로그를 같은 시계로 붙여 "왜 멈췄나"를 가른다.

사용법:
    python3 analyze_jetson_load.py <bag폴더> <부하폴더>   # 정식: 공백 시점의 실제 부하
    python3 analyze_jetson_load.py <부하폴더>            # 부하만 요약
    python3 analyze_jetson_load.py <bag폴더>             # bag만 → **대리지표**(아래)

🔑 bag만 있을 때(부하 측정을 같이 안 돌린 주행): CPU%·메모리는 원리적으로 못 나온다.
   대신 스케줄링 압박이 셋으로 드러난다 —
     (a) 발행 지터: 타이머 노드가 규정 주기를 못 지키는 정도. 순수 타이머인
         `/drive_autonomous`(50 Hz)의 지터가 가장 깨끗한 압박 지표다.
     (b) 발행 지연: header.stamp(생성) → bag 기록(수신). **`/pf/pose/odom`의 이 값이
         곧 MCL 1사이클 연산시간**이라 25 ms 예산 대비 여유를 바로 읽을 수 있다.
     (c) 레이트 드룹: 실측 Hz가 공칭보다 낮은가.
   ⚠️ bag이 젯슨 녹화여야 의미가 있다(랩탑 녹화면 wifi 지연이 섞인다).

전제: bag과 부하 로그가 **둘 다 젯슨에서** 기록됐고(jetson_rec.sh / jetson_load.sh)
      따라서 같은 시계다. bag 타임스탬프는 epoch ns라 그대로 붙는다.
      ⚠️ 랩탑에서 녹화한 bag은 붙이지 말 것 — 시계가 다르면 정렬이 조용히 틀린다.

판정 규칙 (2026-08-18 `run_0818_134408` 사건 기준):
    psi_cpu 급등    → 런큐 대기 = CPU 경합. 범인은 proc.csv 상위 프로세스
    psi_io 급등     → 디스크 대기. 녹화 flush·로그 쓰기 의심(bag을 젯슨에 쓰는 중이다)
    psi_mem 급등    → 페이지 회수. majflt_delta 동반이면 확정
    tick_slip 급등  → 샘플러 자신도 밀렸다 = 특정 노드가 아니라 **시스템 전체** 스톨
    셋 다 조용함    → CPU/메모리/IO가 아니다. 드라이버 내부(USB 리셋 등) → dmesg를 볼 것
"""
import sys, os, sqlite3, csv
import numpy as np

TOPICS = ["/pf/pose/odom", "/scan", "/odom", "/drive_autonomous", "/sensors/core"]


def bag_stamps(bag):
    db = bag if bag.endswith(".db3") else [
        os.path.join(bag, f) for f in sorted(os.listdir(bag)) if f.endswith(".db3")][0]
    con = sqlite3.connect(db); out = {}
    for tid, name in con.execute("select id,name from topics"):
        out[name] = np.array([r[0] / 1e9 for r in con.execute(
            "select timestamp from messages where topic_id=? order by timestamp", (tid,))])
    con.close(); return out


def load_csv(path):
    if not os.path.exists(path): return None
    with open(path) as f:
        r = list(csv.DictReader(f))
    return r if r else None


def num(rows, key):
    out = []
    for r in rows:
        try: out.append(float(r[key]))
        except Exception: out.append(float("nan"))
    return np.array(out)


SPEC = {"/pf/pose/odom": 40.0, "/scan": 40.0, "/odom": 50.0, "/drive_autonomous": 50.0,
        "/sensors/imu/raw": 50.0, "/car_state/frenet/odom": 40.0, "/local_waypoints": 40.0}


def proxy(bag):
    """부하 로그 없이 bag만으로 내는 대리지표."""
    import sqlite3
    db = bag if bag.endswith(".db3") else [
        os.path.join(bag, f) for f in sorted(os.listdir(bag)) if f.endswith(".db3")][0]
    con = sqlite3.connect(db)
    tmap = {n: (i, t) for i, n, t in con.execute("select id,name,type from topics")}
    print(f"=== 젯슨 부하 **대리지표** (bag만, CPU%는 원리적으로 안 나옴)  {os.path.basename(bag)}")
    print(f"  {'토픽':>24} {'공칭':>5} {'실측Hz':>7} {'주기':>9} {'지터p99':>8} {'지터max':>9} "
          f"{'지연중앙':>9} {'지연p99':>8}")
    worst = []
    for tp, hz in SPEC.items():
        if tp not in tmap: continue
        tid, typ = tmap[tp]
        rows = con.execute("select timestamp,data from messages where topic_id=? order by timestamp",
                           (tid,)).fetchall()
        if len(rows) < 10: continue
        rec = np.array([r[0] / 1e9 for r in rows])
        ls = l9 = float("nan")
        try:
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
            cls = get_message(typ)
            hdr = np.array([(lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)(
                deserialize_message(r[1], cls)) for r in rows])
            lat = (rec - hdr) * 1000.0
            lat = lat[np.isfinite(lat)]
            if len(lat): ls, l9 = np.median(lat), np.percentile(lat, 99)
        except Exception:
            pass
        g = np.diff(rec) * 1000.0
        per = np.median(g); jit = np.abs(g - per)
        print(f"  {tp:>24} {hz:5.0f} {1000/per:7.1f} {per:8.2f}ms {np.percentile(jit,99):7.1f}ms "
              f"{jit.max():8.1f}ms {ls:8.1f}ms {l9:7.1f}ms")
        worst.append((tp, np.percentile(jit, 99), jit.max(), ls, per))
    con.close()
    print()
    ctl = [w for w in worst if w[0] == "/drive_autonomous"]
    mcl = [w for w in worst if w[0] == "/pf/pose/odom"]
    if ctl:
        j = ctl[0][1]
        verdict = "여유 충분" if j < 3 else ("압박 있음" if j < 10 else "🔴 심한 스케줄링 압박")
        print(f"  컨트롤러(순수 50 Hz 타이머) 지터 p99 {j:.1f} ms → {verdict}")
    if mcl and mcl[0][3] == mcl[0][3]:
        lat, per = mcl[0][3], mcl[0][4]
        print(f"  MCL 1사이클 연산 ≈ {lat:.1f} ms / 예산 {per:.1f} ms = 활용률 {100*lat/per:.0f}%"
              f"  (p99가 예산을 넘으면 여유가 없다)")
    print("\n  ⚠️ 이건 대리지표다. CPU·메모리·I/O 중 무엇인지 가르려면 주행 중")
    print("     tools/jetson_load.sh 를 jrec과 같이 돌릴 것.")


def main():
    args = [a for a in sys.argv[1:]]
    if not args:
        print(__doc__); sys.exit(1)
    for a in args:
        if not os.path.exists(a):
            print(f"❌ 경로가 없습니다: {a}"); sys.exit(1)

    def is_bag(p):
        if p.endswith(".db3"): return True
        return os.path.isdir(p) and any(f.endswith(".db3") for f in os.listdir(p))

    if len(args) == 1 and is_bag(args[0]):
        proxy(args[0]); return
    bag, loaddir = (None, args[0]) if len(args) == 1 else (args[0], args[1])

    meta = {}
    mp = os.path.join(loaddir, "meta.txt")
    if os.path.exists(mp):
        for line in open(mp):
            if "\t" in line:
                k, v = line.rstrip("\n").split("\t", 1); meta[k] = v
    sysr = load_csv(os.path.join(loaddir, "sys.csv"))
    if not sysr: print(f"❌ sys.csv 없음/빔: {loaddir}"); sys.exit(1)
    t = num(sysr, "t")
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.05

    print(f"=== 젯슨 부하 요약  ({os.path.basename(loaddir)})")
    print(f"  호스트 {meta.get('hostname','?')} / CPU {meta.get('ncpu','?')}코어 / PSI {meta.get('psi','?')}")
    print(f"  샘플 {len(t)}개, {t[-1]-t[0]:.1f}초, 주기 {dt*1000:.0f} ms")
    cpu = num(sysr, "cpu_pct")
    print(f"  CPU 사용률   중앙 {np.nanmedian(cpu):5.1f}%  p95 {np.nanpercentile(cpu,95):5.1f}%  최대 {np.nanmax(cpu):5.1f}%")
    for key, lbl in [("psi_cpu_some_us", "PSI cpu "), ("psi_io_some_us", "PSI io  "),
                     ("psi_mem_some_us", "PSI mem ")]:
        v = num(sysr, key)
        if np.all(v < 0): print(f"  {lbl} — 커널에 PSI 없음"); continue
        pct = 100.0 * v / (dt * 1e6)      # 구간 대비 몇 %를 기다렸나
        print(f"  {lbl}대기  중앙 {np.nanmedian(pct):5.2f}%  p99 {np.nanpercentile(pct,99):6.2f}%  최대 {np.nanmax(pct):6.2f}%")
    slip = num(sysr, "tick_slip_ms")
    print(f"  샘플러 지연  중앙 {np.nanmedian(slip):5.1f} ms  p99 {np.nanpercentile(slip,99):6.1f}  최대 {np.nanmax(slip):6.1f}"
          f"   ← 크면 시스템 전체 스톨")
    ma = num(sysr, "mem_avail_kb")
    print(f"  가용 메모리  최소 {np.nanmin(ma)/1024:.0f} MB")

    # 🔑 이더넷 링크 — 라이다가 이더넷이라(192.168.0.10:10940) 링크가 내려가면
    #    urg_node가 TCP 재연결에 약 8초를 쓴다(0819 실측). 이게 찍히면 원인 확정이다.
    link_keys = [k for k in sysr[0].keys() if k.startswith("link_")]
    if link_keys:
        for k in link_keys:
            v = num(sysr, k)
            down = np.sum(v == 0); unread = np.sum(v < 0)
            if down == 0 and unread == 0:
                print(f"  이더넷 {k[5:]:<8} 링크 유지 (끊김 0회)")
                continue
            # 전이 구간을 찾아 길이와 시점을 찍는다
            segs, st = [], None
            for i, x in enumerate(v):
                if x == 0 and st is None: st = i
                elif x != 0 and st is not None: segs.append((t[st], t[i] - t[st])); st = None
            if st is not None: segs.append((t[st], t[-1] - t[st]))
            print(f"  🔴 이더넷 {k[5:]:<8} 링크 끊김 {len(segs)}회 "
                  f"(총 {np.sum(v == 0) * dt:.2f} s, 읽기실패 {unread})")
            for t0_, dur in segs[:10]:
                print(f"       t={t0_ - t[0]:7.2f}s 부터 {dur * 1000:8.0f} ms")

    procr = load_csv(os.path.join(loaddir, "proc.csv"))
    if procr:
        names = {}
        for r in procr:
            try: c = float(r["cpu_pct"])
            except Exception: continue
            names.setdefault(r["name"], []).append(c)
        print(f"\n  프로세스별 CPU (상위 10)")
        print(f"    {'이름':<26} {'샘플':>5} {'중앙':>7} {'p99':>7} {'최대':>7}")
        for n, v in sorted(names.items(), key=lambda x: -np.percentile(x[1], 99))[:10]:
            a = np.array(v)
            print(f"    {n:<26} {len(a):5d} {np.median(a):6.1f}% {np.percentile(a,99):6.1f}% {a.max():6.1f}%")

    if bag is None:
        print("\n(bag을 같이 주면 토픽 공백 시점의 부하를 붙여 보여줍니다)")
        return

    # ── bag 공백 ↔ 부하 정렬 ────────────────────────────────────────────────
    st = bag_stamps(bag)
    print(f"\n=== bag 정렬  ({os.path.basename(bag)})")
    overlap_lo, overlap_hi = max(t[0], min(v[0] for v in st.values() if len(v))), \
                             min(t[-1], max(v[-1] for v in st.values() if len(v)))
    if overlap_hi <= overlap_lo:
        print("  🔴 bag과 부하 로그의 시간대가 겹치지 않는다 — 같은 주행이 맞는지,")
        print("     그리고 bag이 **젯슨에서** 녹화됐는지(랩탑 녹화면 시계가 다르다) 확인할 것.")
        print(f"     부하 {t[0]:.0f}~{t[-1]:.0f} / bag {min(v[0] for v in st.values() if len(v)):.0f}~"
              f"{max(v[-1] for v in st.values() if len(v)):.0f}")
        return
    print(f"  겹치는 구간 {overlap_hi-overlap_lo:.1f}초")

    events = []
    for tp in TOPICS:
        v = st.get(tp)
        if v is None or len(v) < 3: continue
        g = np.diff(v)
        thr = max(5 * np.median(g), 0.2)
        for i in np.where(g > thr)[0]:
            if overlap_lo <= v[i] <= overlap_hi:
                events.append((v[i], g[i], tp))
    events.sort()
    if not events:
        print("  ✅ 겹치는 구간에 200 ms 이상 토픽 공백이 없다 — 이 주행은 깨끗하다.")
        return

    print(f"\n  토픽 공백 {len(events)}건 — 각 시점의 부하 (공백 직전 0.5초 창)")
    print(f"    {'시각':>12} {'토픽':>18} {'공백':>8} {'CPU':>6} {'PSIcpu':>7} {'PSIio':>7} {'PSImem':>7} {'slip':>7}")
    for ts, gap, tp in events[:25]:
        m = (t > ts - 0.5) & (t <= ts + gap)
        if m.sum() == 0:
            print(f"    {ts:12.2f} {tp:>18} {gap*1000:7.0f}ms   (부하 샘플 없음)")
            continue
        f = lambda k: 100.0 * np.nanmax(num(sysr, k)[m]) / (dt * 1e6)
        print(f"    {ts:12.2f} {tp:>18} {gap*1000:7.0f}ms {np.nanmax(cpu[m]):5.0f}% "
              f"{f('psi_cpu_some_us'):6.1f}% {f('psi_io_some_us'):6.1f}% {f('psi_mem_some_us'):6.1f}% "
              f"{np.nanmax(slip[m]):6.0f}ms")
    if len(events) > 25: print(f"    ... 외 {len(events)-25}건")

    print("\n  판정 가이드: PSIcpu↑=CPU경합(proc.csv 상위 확인) / PSIio↑=디스크 / "
          "PSImem↑=메모리\n              slip↑=시스템 전체 스톨 / 전부 조용=드라이버 내부 → dmesg_after.txt")


if __name__ == "__main__":
    main()
