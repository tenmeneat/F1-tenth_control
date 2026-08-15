#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bench_damping.py — 눌렀다 놓기(pluck) 시험으로 서스펜션 f0 · ζ 를 시간영역에서 측정
================================================================================
왜 시간영역인가: 주행 bag의 스펙트럼으로 ζ 를 추정하려면 **노면 가진이 백색잡음**이라는
가정이 들어간다(반전력 대역폭법). 벤치에서 한 번 눌렀다 놓으면 그 가정이 필요 없다 —
자유진동의 로그 감쇠율(log decrement)이 ζ 를 직접 준다.

왜 손으로 보면 안 되는가: 댐퍼 힘은 **속도 비례**다. 손으로 누르는 속도(1~3 Hz)에서는
거의 저항하지 않으므로 "통통 안 튄다"가 10 Hz 에서의 감쇠를 전혀 보장하지 않는다.
2026-08-14 실제로 "투잉하고 한 번에 튀어나온다"는 관찰과 실측 ζ 0.056 이 공존했다.

측정 방법 (차량 정차, 시동/자율 무관):
    ros2 bag record -o ~/bench_damp /sensors/imu/raw
    # 앞범퍼를 꾹 눌렀다가 탁 놓기 3~4회 (사이 2~3초)
    # 그다음 뒤쪽도 3~4회
    # Ctrl-C
    python3 tools/bench_damping.py ~/bench_damp

읽는 법:
    f0  차체(스프렁) 모드 고유진동수. 1/10 레이싱 통상 3~5 Hz.
        10 Hz 대면 k/m 이 크다 = 스프링이 세다 (ζ = c/(2√(km)) 라 ζ 도 같이 깎인다)
    ζ   감쇠비. RC 레이싱 목표 0.30~0.50.
        0.1 미만 = 사실상 감쇠 없음 → 오일 누유 / 에어 물림 / 너무 묽은 오일 / 스틱션

⚠️ 가속도계 LPF 가 15 Hz 다(`vesc_appconf.xml` imu_conf.accel_lowpass_filter_*).
   그 위의 모드(단단한 리어 등)는 **원리적으로 안 보인다** — 이 도구의 사각지대다.
   f0 가 12 Hz 를 넘게 나오면 필터에 눌린 값이므로 그대로 믿지 말 것.
"""
import sys, os, glob, sqlite3, math, argparse
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

G = 9.80665
DEG2RAD = math.pi / 180.0


def read_topic(bag, topic):
    dbs = sorted(glob.glob(os.path.join(bag, "*.db3")))
    if not dbs and bag.endswith(".db3"):
        dbs = [bag]
    out = []
    for db in dbs:
        con = sqlite3.connect(db)
        tid = typ = None
        for i, n, t in con.execute("SELECT id,name,type FROM topics").fetchall():
            if n == topic:
                tid, typ = i, t
        if tid is None:
            con.close(); continue
        cls = get_message(typ)
        for ts, data in con.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
            out.append((ts / 1e9, deserialize_message(data, cls)))
        con.close()
    return out


def find_plucks(a, fs, min_gap_s=1.0, thresh_k=4.0):
    """놓는 순간 = 정지 상태 대비 큰 이탈. 이후 min_gap 동안은 같은 이벤트로 본다."""
    x = a - np.median(a)
    # 조용한 구간의 산포로 임계를 잡는다(사람이 만지는 동안은 시끄럽다)
    q = np.percentile(np.abs(x), 40)
    thr = max(thresh_k * q, 0.8)
    idx = np.where(np.abs(x) > thr)[0]
    ev, last = [], -1e9
    for i in idx:
        if i / fs - last > min_gap_s:
            ev.append(i); last = i / fs
    return ev, thr


def _envelope(x):
    """해석신호 |x + jH{x}| — 힐베르트 변환을 FFT 로 직접 (scipy 의존 없이)."""
    n = len(x)
    X = np.fft.fft(x)
    h = np.zeros(n)
    h[0] = 1
    if n % 2 == 0:
        h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[1:(n + 1) // 2] = 2
    return np.abs(np.fft.ifft(X * h))


def analyze_ring(seg, fs):
    """자유진동 구간에서 f0 와 ζ 를 뽑는다.

    ⚠️ 극값(peak) 세는 방식은 쓰지 않는다 — 링잉이 노이즈 바닥에 닿는 순간 잡음이
       극값을 무한정 만들어 **f0 가 fs/4 에 고착**되고 ζ 가 최대 −86% 틀어진다
       (2026-08-14 합성신호 검증에서 실제로 그랬다). 대신:
         f0 : 제로패딩 FFT 피크 + 포물선 보간   (10 Hz/50 Hz = 주기당 4.8 샘플이라
                                              샘플 격자에 의존하는 방법은 못 쓴다)
         ζ  : 힐베르트 포락선의 ln 기울기 회귀. 포락선이 최대의 15% 아래로 내려가면
              거기서 자른다(그 아래는 노이즈라 회귀를 오염시킨다)
    """
    x = np.asarray(seg, dtype=float)
    x = x - x.mean()
    if len(x) < 12:
        return None

    # ── f0: 제로패딩 FFT + 포물선 보간 ─────────────────────────────────────
    nfft = 1 << (int(np.log2(len(x))) + 5)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)), nfft)) ** 2
    f = np.fft.rfftfreq(nfft, 1 / fs)
    lo = np.searchsorted(f, 1.0)          # 1 Hz 미만은 드리프트
    k = lo + int(np.argmax(P[lo:]))
    if 0 < k < len(P) - 1:
        a, b, c = P[k - 1], P[k], P[k + 1]
        denom = (a - 2 * b + c)
        d = 0.5 * (a - c) / denom if abs(denom) > 1e-30 else 0.0
        f0 = f[k] + d * (f[1] - f[0])
    else:
        f0 = f[k]
    if not (0.5 < f0 < fs / 2):
        return None

    # ── ζ: 포락선 ln 기울기 ────────────────────────────────────────────────
    env = _envelope(x)
    # 시작 과도(놓는 순간의 충격)를 피해 포락선 최대 지점부터
    i0 = int(np.argmax(env))
    env = env[i0:]
    if len(env) < 8:
        return None
    thr = 0.15 * env[0]
    end = np.argmax(env < thr) if (env < thr).any() else len(env)
    end = max(end, 6)
    e = env[:end]
    t = np.arange(len(e)) / fs
    good = e > 1e-9
    if good.sum() < 6:
        return None
    slope = np.polyfit(t[good], np.log(e[good]), 1)[0]     # = -ζ·ω_n
    if slope >= 0:
        return f0, 0.0, len(e), True
    zeta = (-slope) / (2 * math.pi * f0)
    zeta = min(zeta, 0.99)

    # ── 신뢰도 검사 ────────────────────────────────────────────────────────
    # 감쇠가 크고 f0 가 높으면 50 Hz 로는 몇 주기 못 담아 FFT 피크가 엉뚱한 데로 간다
    # (합성 검증: f0 10.5 Hz·ζ 0.35 에서 f0 가 −85% 틀어졌다). 조용히 틀리느니 표시한다.
    # 교차검증: 유효 구간의 영교차율로 센 주파수와 FFT f0 가 크게 다르면 못 믿는다.
    xs = x[i0:i0 + end]
    zc = np.count_nonzero(np.diff(np.signbit(xs)))
    dur = len(xs) / fs
    f_zc = zc / (2 * dur) if dur > 0 else 0.0
    # 주 판정은 **영교차 일치**다. 주기수는 하한만 본다 — ζ 가 크면 포락선이 일찍
    # 잘려 주기수가 적어지는데, 그건 추정이 틀렸다는 뜻이 아니다(합성 검증에서
    # 주기 1.9회여도 오차 1.4% 였다). 주기수를 3회로 잡으면 정확한 결과까지 버린다.
    cycles = f0 * dur
    ok = (cycles >= 1.5) and (f_zc > 0) and (abs(f0 - f_zc) <= 0.35 * f_zc)
    return f0, zeta, len(e), ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/sensors/imu/raw")
    ap.add_argument("--ring", type=float, default=1.0, help="놓은 뒤 분석할 길이 [s]")
    args = ap.parse_args()

    msgs = read_topic(args.bag, args.topic)
    if not msgs:
        print(f"🔴 {args.topic} 이 bag 에 없다: {args.bag}"); sys.exit(1)
    t = np.array([x[0] for x in msgs])
    # CLAUDE.md 축: 전방 = -a_x, 좌 = -a_y, 위 = +z. 단위는 g.
    az = np.array([m.linear_acceleration.z for _, m in msgs]) * G
    wy = np.array([m.angular_velocity.y for _, m in msgs]) * DEG2RAD   # 피치레이트
    fs = 1.0 / np.median(np.diff(t))
    print(f"샘플 {len(t)}개 / {t[-1]-t[0]:.1f} s / {fs:.1f} Hz  (가속도계 LPF 15 Hz → 12 Hz 초과는 신뢰 금지)")

    ev, thr = find_plucks(az, fs)
    print(f"검출된 pluck {len(ev)}회 (임계 {thr:.2f} m/s²)\n")
    if not ev:
        print("🔴 이벤트를 못 찾았다. 더 세게 누르고 확실히 '탁' 놓을 것. 또는 --ring 조정.")
        sys.exit(1)

    print("  #   시각[s]  f0[Hz]    ζ      피크수  판정")
    rows = []
    for k, i in enumerate(ev, 1):
        j = min(len(az), i + int(args.ring * fs))
        r = analyze_ring(az[i:j], fs)
        if r is None:
            print(f" {k:2d}  {t[i]-t[0]:7.2f}      —       —        —    (링잉 부족)")
            continue
        f0, z, npk, ok = r
        verdict = ("🔴 감쇠 거의 없음" if z < 0.10 else
                   "🟡 부족" if z < 0.25 else
                   "🟢 정상" if z <= 0.55 else "🟡 과감쇠")
        if f0 > 12:
            verdict += " ⚠️f0>12Hz: LPF에 눌린 값"
        if not ok:
            verdict = "⚠️ 신뢰불가(주기수 부족/영교차 불일치) — " + verdict
        print(f" {k:2d}  {t[i]-t[0]:7.2f}  {f0:6.2f}  {z:6.3f}  {npk:5d}    {verdict}")
        rows.append((f0, z))

    if rows:
        A = np.array(rows)
        print(f"\n  중앙값:  f0 {np.median(A[:,0]):.2f} Hz   ζ {np.median(A[:,1]):.3f}")
        print(f"  목표  :  ζ 0.30~0.50   (1/10 레이싱 차체 모드 f0 는 통상 3~5 Hz)")
        z = np.median(A[:, 1]); f0 = np.median(A[:, 0])
        print("\n  해석:")
        if z < 0.10:
            print("   · ζ<0.10 = 사실상 감쇠 없음 → 오일 누유 / 에어 물림 / 너무 묽은 오일 / 스틱션")
        if f0 > 8:
            print(f"   · f0 {f0:.1f} Hz 는 차체 모드치고 높다 = k/m 이 크다(스프링이 세다).")
            print("     ζ = c/(2√(km)) 이므로 스프링을 무르게 하면 ζ 와 f0 가 **동시에** 개선된다.")
        print("   · 앞/뒤를 따로 쳤다면 시각 순서로 두 그룹이 나뉜다 — 위 표의 시각으로 갈라 볼 것.")


if __name__ == "__main__":
    main()
