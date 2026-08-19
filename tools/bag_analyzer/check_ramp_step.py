#!/usr/bin/env python3
"""s_pid_ramp_erpms_s 상향 후 감속 경로 충돌 검증.

`vesc_mcconf.xml`의 `s_pid_ramp_erpms_s`를 올리면 가속만 풀리는 게 아니라
**속도PID의 감속 setpoint도 같이 풀린다**. 실차 감속 경로가 셋(속도PID 회생 /
ackermann_to_vesc 브레이크 패치 / drive_mode_manager E-stop)이라 상호작용을
매 스텝 확인해야 한다. bag 하나 던지면 아래 7개를 판정한다.

    source /opt/ros/<distro>/setup.bash
    python3 check_ramp_step.py <bag폴더|.db3> [--ramp 21160]

vesc_msgs 없이 동작한다(/sensors/core를 CDR 직접 디코드) — 랩탑에서 바로 돌아간다.

베이스라인은 2026-07-26 `run_0726_204116` (ramp=2000, 7랩 클린) 실측이다.
"""
import os, sys, glob, sqlite3, struct, argparse
import numpy as np

# speed_to_erpm_gain. 2026-08-04 세미슬릭 타이어 교체 재보정으로 4232.0 → 4336.0 (08-05 확정).
# 2026-08-19 타이어 마모 진행으로 4336.0 → 4420.0 (+1.94%).
# 젯슨 vesc.yaml과 반드시 같아야 한다 — 다르면 여기서 나오는 m/s² 가 통째로 어긋난다.
G = 4420.0
DMAX = 0.41         # 조향 물리한계 [rad]

# ── ramp=2000 베이스라인 (run_0726_204116) ─────────────────────────────
# ⚠️ 이 값들은 **오프로드 타이어 + G=4232** 시절에 뽑은 것이다. 세미슬릭으로 바뀌면서
#    그립과 거리 스케일이 둘 다 달라졌으므로, 절대 비교가 아니라 **자릿수 감각**으로만 쓸 것.
#    엄밀한 A/B를 하려면 세미슬릭으로 ramp를 한 번 되돌려 새 베이스라인을 떠야 한다.
BASE = dict(
    ret_cur_p50=8.7, ret_cur_p90=15.9,      # brake→speed 복귀 0.3s 내 최대 전류 [A]
    ret_acc_p50=0.08, ret_acc_p90=0.38,     # 복귀 0.3s 내 최대 가속 [m/s²]
    brake_dec_mean=-1.06, brake_dec_min=-1.57,
    switch_hz=1.20, blip_pct=8.0,           # 채널 전환률 / 20ms 단발 비율
    brake_cur_mean=2.14,
)


def find_db3(p):
    if os.path.isdir(p):
        c = sorted(glob.glob(os.path.join(p, "*.db3")))
        if not c:
            sys.exit(f"❌ {p} 안에 .db3 없음")
        return c[0]
    return p


def load(con, topics, name, field=None):
    """표준 메시지 로드. field 없으면 .data (Float64)."""
    if name not in topics:
        return None
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    tid, typ = topics[name]
    cls = get_message(typ)
    out = []
    for t, d in con.execute(
            "select timestamp,data from messages where topic_id=? order by timestamp", (tid,)):
        m = deserialize_message(bytes(d), cls)
        v = m
        for f in (field or "data").split('.'):
            v = getattr(v, f)
        out.append([t * 1e-9, v])
    return np.array(out) if out else None


def load_core(con, topics):
    """vesc_msgs 없이 VescStateStamped를 CDR 직접 디코드.
    header(stamp+frame_id) 뒤 8바이트 정렬 → float64 22개. idx는 실측으로 확인한 것."""
    if '/sensors/core' not in topics:
        return None, None
    tid, _ = topics['/sensors/core']
    T, V = [], []
    for t, d in con.execute(
            "select timestamp,data from messages where topic_id=? order by timestamp", (tid,)):
        b = bytes(d)
        off = 4 + 8                                   # 캡슐화 헤더 + stamp
        sl = struct.unpack_from('<I', b, off)[0]
        off += 4 + sl
        off += (-(off - 4)) % 8                       # 8바이트 정렬
        n = (len(b) - off) // 8
        if n < 8:
            continue
        V.append(struct.unpack_from('<%dd' % n, b, off)[:22])
        T.append(t * 1e-9)
    if not T:
        return None, None
    return np.array(T), np.array(V)


def verdict(ok, warn=False):
    return "\033[92m✅ PASS\033[0m" if ok else ("\033[93m⚠️  WARN\033[0m" if warn else "\033[91m❌ FAIL\033[0m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--ramp", type=float, default=21160.0, help="VESC에 써넣은 ramp [ERPM/s]")
    ap.add_argument("--odom", default="/pf/pose/odom")
    ap.add_argument("--grip", type=float, default=6.5, help="목표 a_lat [m/s²]")
    a = ap.parse_args()

    con = sqlite3.connect(find_db3(a.bag))
    topics = {n: (i, t) for i, n, t in con.execute("select id,name,type from topics")}

    dv = load(con, topics, '/drive', 'drive.speed')
    dst = load(con, topics, '/drive', 'drive.steering_angle')
    sp = load(con, topics, '/commands/motor/speed')
    bk = load(con, topics, '/commands/motor/brake')
    sv = load(con, topics, '/commands/servo/position')
    od = load(con, topics, a.odom, 'twist.twist.linear.x')
    imu = load(con, topics, '/sensors/imu/raw', 'angular_velocity.z')
    VT, V = load_core(con, topics)

    if dv is None or VT is None:
        sys.exit("❌ /drive 또는 /sensors/core 없음 — 검증 불가")

    t0 = dv[0, 0]
    for arr in (dv, dst, sp, bk, sv, od, imu):
        if arr is not None:
            arr[:, 0] -= t0
    VT = VT - t0
    erpm, cur, duty = V[:, 7], V[:, 2], V[:, 6]

    # 20ms 균일 그리드
    g = np.arange(VT[0], VT[-1], 0.02)
    e = np.interp(g, VT, erpm)
    c = np.interp(g, VT, cur)
    v = e / G
    acc = np.gradient(np.convolve(v, np.ones(3) / 3, 'same'), g)
    cmd = np.interp(g, dv[:, 0], dv[:, 1])

    print(f"\n\033[1mbag {os.path.basename(a.bag)}  ({VT[-1]-VT[0]:.0f}s, 설정 ramp={a.ramp:.0f})\033[0m")
    print("=" * 74)
    fails, warns = 0, 0

    # ── ① ramp가 실제로 반영됐나 ────────────────────────────────────
    run = (e[:-10] > 3000) & (e[:-10] < 0.9 * e.max())
    d10 = ((e[10:] - e[:-10]) / (g[10:] - g[:-10]))[run]
    p99, pmax = np.percentile(d10, 99), d10.max()
    ok = pmax > a.ramp * 0.6
    fails += not ok
    print(f"\n\033[1m① ramp 반영 (Write 성공 여부)\033[0m   {verdict(ok)}")
    print(f"   dERPM/dt  p50={np.percentile(d10,50):7.0f}  p99={p99:7.0f}  max={pmax:7.0f} ERPM/s")
    print(f"   실측 가속 상한 {pmax/G:.2f} m/s²  (설정 {a.ramp/G:.2f}, 이전 0.47)")
    if not ok:
        print("   → \033[91mWrite가 안 먹었다. VESC Tool limited mode 확인.\033[0m")

    # ── ② 배타성: speed/brake 동시 발행 없나 ────────────────────────
    if bk is not None and sp is not None:
        both = sum(1 for t in bk[:, 0] if np.min(np.abs(sp[:, 0] - t)) < 0.002)
        nsv = len(sv) if sv is not None else len(sp) + len(bk)
        ok = both == 0 and abs(nsv - len(sp) - len(bk)) < 30
        fails += not ok
        print(f"\n\033[1m② 배타적 중재 (speed XOR brake)\033[0m   {verdict(ok)}")
        print(f"   servo {nsv} vs speed {len(sp)} + brake {len(bk)} = {len(sp)+len(bk)}")
        print(f"   ±2ms 내 동시 발행 {both}건 (0이어야 함)")

        # ── ③ 채널 전환 채터링 ──────────────────────────────────────
        ev = np.r_[np.c_[sp[:, 0], np.zeros(len(sp))], np.c_[bk[:, 0], np.ones(len(bk))]]
        ev = ev[np.argsort(ev[:, 0])]
        sw = int(np.sum(np.diff(ev[:, 1]) != 0))
        hz = sw / (ev[-1, 0] - ev[0, 0])
        lab, tt, eps, i = ev[:, 1], ev[:, 0], [], 0
        while i < len(lab):
            if lab[i] == 1:
                j = i
                while j < len(lab) and lab[j] == 1:
                    j += 1
                eps.append(tt[min(j, len(tt) - 1)] - tt[i])
                i = j
            else:
                i += 1
        eps = np.array(eps)
        blip = 100 * np.mean(eps < 0.03) if len(eps) else 0
        ok = hz < BASE['switch_hz'] * 3 and blip < 30
        warns += not ok
        print(f"\n\033[1m③ 채널 전환 채터링\033[0m   {verdict(ok, warn=True)}")
        print(f"   전환 {hz:.2f} Hz (기준 {BASE['switch_hz']:.2f})   "
              f"brake 에피소드 {len(eps)}회 p50={np.median(eps)*1000 if len(eps) else 0:.0f}ms")
        print(f"   20ms 단발 비율 {blip:.0f}% (기준 {BASE['blip_pct']:.0f}%, 30% 넘으면 채터링)")

        # ── ④ 핵심: brake→speed 복귀 전이 스파이크 ──────────────────
        ends = np.array([ev[i, 0] for i in range(1, len(ev))
                         if ev[i - 1, 1] == 1 and ev[i, 1] == 0])
        ends = ends[ends + 0.3 < g[-1]]
        if len(ends):
            ii = np.searchsorted(g, ends)
            mc = np.array([c[k:k + 15].max() for k in ii])
            ma = np.array([acc[k:k + 15].max() for k in ii])
            cur_p90, acc_p90 = np.percentile(mc, 90), np.percentile(ma, 90)
            # 램프가 풀렸으니 상승 자체는 정상. 위험은 '그립 초과 급가속'.
            ok = acc_p90 < a.grip
            hard = acc_p90 > a.grip * 1.3
            fails += hard
            warns += (not ok) and (not hard)
            print(f"\n\033[1m④ brake→speed 복귀 전이 (setpoint 이월 충돌)\033[0m   {verdict(ok, warn=not hard)}")
            print(f"   복귀 {len(ends)}회. 이후 0.3s 내 최대")
            print(f"     전류  p50={np.median(mc):5.1f}A  p90={cur_p90:5.1f}A   (기준 {BASE['ret_cur_p50']:.1f}/{BASE['ret_cur_p90']:.1f})")
            print(f"     가속  p50={np.median(ma):5.2f}   p90={acc_p90:5.2f} m/s²  (기준 {BASE['ret_acc_p50']:.2f}/{BASE['ret_acc_p90']:.2f})")
            if hard:
                print(f"   → \033[91m복귀 급가속이 그립({a.grip})의 1.3배 초과. setpoint 이월 의심 —\033[0m")
                print( "     코너 탈출 휠스핀 위험. s_pid_ramp를 한 단계 내리거나 s_pid_allow_braking=0 시험.")
            elif not ok:
                print(f"   → \033[93m복귀 가속이 그립 근처. 코너 탈출 거동을 영상으로 확인할 것.\033[0m")

        # ── ⑤ brake 구간 실효 감속 ──────────────────────────────────
        starts = np.array([ev[i, 0] for i in range(1, len(ev))
                           if ev[i - 1, 1] == 0 and ev[i, 1] == 1])
        n = min(len(starts), len(ends))
        decs = []
        for s_, e_ in zip(starts[:n], ends[:n]):
            if e_ - s_ < 0.1:
                continue
            i0, i1 = np.searchsorted(g, s_), np.searchsorted(g, e_)
            if i1 - i0 > 3:
                decs.append((v[i1] - v[i0]) / (g[i1] - g[i0]))
        if decs:
            decs = np.array(decs)
            ok = decs.mean() < BASE['brake_dec_mean']
            print(f"\n\033[1m⑤ brake 구간 실효 감속\033[0m   {verdict(ok, warn=True)}")
            print(f"   n={len(decs)}  mean={decs.mean():+.2f}  p10={np.percentile(decs,10):+.2f} m/s²"
                  f"   (기준 {BASE['brake_dec_mean']:+.2f})")
            print(f"   brake 전류 mean={bk[:,1].mean():.2f}A max={bk[:,1].max():.2f}A  "
                  f"20A 도달 {100*np.mean(bk[:,1]>=19.9):.1f}%")
            if np.mean(bk[:, 1] >= 19.9) < 5:
                print("   → 상한이 아니라 \033[93m요구 전류 자체가 작다\033[0m. 더 세게 세우려면 패치의 제동 게인을 올릴 것.")

    # ── ⑥ 코너 안전 마진 (가속이 풀리면 여기가 다음 한계) ────────────
    if imu is not None and od is not None:
        wz = np.interp(g, imu[:, 0], imu[:, 1]) * np.pi / 180.0
        alat = np.abs(v * wz)
        m = v > 1.0
        p99a = np.percentile(alat[m], 99)
        ok = p99a < a.grip
        fails += p99a > a.grip * 1.15
        warns += not ok
        print(f"\n\033[1m⑥ 코너 그립 마진\033[0m   {verdict(ok, warn=p99a < a.grip*1.15)}")
        print(f"   |a_lat| p99={p99a:.2f}  max={alat[m].max():.2f} m/s²  (그립 가정 {a.grip})")
        if dst is not None:
            st = np.abs(np.interp(g, dst[:, 0], dst[:, 1]))
            print(f"   조향 포화(>0.40rad) {100*np.mean(st[m]>0.40):.2f}%   max={st[m].max():.3f}")

    # ── ⑦ 속도/랩 요약 ──────────────────────────────────────────────
    print(f"\n\033[1m⑦ 성능\033[0m")
    print(f"   실측 최고속 {v.max():.2f} m/s (ERPM {e.max():.0f})   명령 최고 {cmd.max():.2f} m/s")
    print(f"   명령-실측 갭 mean={np.mean(cmd[v>1]-v[v>1]):+.2f} m/s   duty max={duty.max():.3f}")
    print(f"   모터전류 p99={np.percentile(cur,99):.1f}A max={cur.max():.1f}A (l_current_max 60)")

    print("\n" + "=" * 74)
    if fails:
        print(f"\033[91m❌ FAIL {fails}건 — 위 항목 조치 후 재주행\033[0m")
    elif warns:
        print(f"\033[93m⚠️  WARN {warns}건 — 진행 가능하나 다음 스텝 전 확인\033[0m")
    else:
        print("\033[92m✅ 전 항목 통과 — 다음 스텝 진행 가능\033[0m")
    print()


if __name__ == "__main__":
    main()
