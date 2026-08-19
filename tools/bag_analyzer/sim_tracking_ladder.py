"""조향 추종오차 파라미터 사다리 — 단계별 예상 횡오차 (CLAUDE.md 2-v).

  # 1) bag에서 글로벌 라인 + 실주행을 뽑는다 (run.npz 생성)
  python3 extract_line_and_run.py <bag>/<bag>_0.db3

  # 2) 사다리 실행 (같은 디렉터리의 run.npz를 읽는다)
  SIM_DATA_DIR=. PYTHONPATH=. python3 sim_tracking_ladder.py

절대값은 보정되지 않았다 - **상대 비교용**이다(타이어 과도응답/노면 미포함).
실측 대비 공칭 오차를 약 2배 낙관한다. 판단은 max와 omega_n*tau의 **순위**로 할 것.
"""
import numpy as np
import sim_tracking_sweep as S

NOISE, LAPS = 0.025, 5
P60 = S.build_path(mla=6.0)      # 지금 라인 (a_lat 6.0 요구)
P55 = S.build_path(mla=5.5)      # 내일 재생성할 라인


def run(P, mode='lag', kt=(0.011, 0.024), **kw):
    p = dict(S.BASE); p.update(kw)
    e, v, k, dl = S.simulate_x(P, p, laps=LAPS, pose_noise=NOISE, seed=3,
                               kus_true=kt, delay_mode=mode)
    a = np.abs(e)
    return np.median(a), np.percentile(a, 95), a.max(), 100 * np.mean(a > 0.20)


LADDER = [
    ("0. 현재                        ", dict(fb_gain=1.00, l1_speed_gain=0.40, ff_preview=0.0)),
    ("1. decel_scaler 1.0           ", dict(fb_gain=1.00, l1_speed_gain=0.40, ff_preview=0.0,
                                            decel_scaler=1.0)),
    ("2. + fb_gain 0.80             ", dict(fb_gain=0.80, l1_speed_gain=0.40, ff_preview=0.0,
                                            decel_scaler=1.0)),
    ("3. + l1_speed_gain 0.35       ", dict(fb_gain=0.80, l1_speed_gain=0.35, ff_preview=0.0,
                                            decel_scaler=1.0)),
    ("4. + ff_preview 0.3 (A/B)     ", dict(fb_gain=0.80, l1_speed_gain=0.35, ff_preview=0.3,
                                            decel_scaler=1.0)),
    ("(참고) 스케일러 둔 채 2~3만        ", dict(fb_gain=0.80, l1_speed_gain=0.35, ff_preview=0.0)),
]

for name, P in [("지금 라인 (a_lat 6.0)", P60), ("재생성 라인 (a_lat 5.5)", P55)]:
    print("=== %s | 프로파일 중앙 %.2f m/s, 최고 %.2f ===" % (name, np.median(P['v']), P['v'].max()))
    print("   %-30s %-8s %-8s %-8s %-8s %-8s" %
          ("단계", "p50", "p95", "max", ">0.2m", "K+25%max"))
    for lbl, kw in LADDER:
        a = run(P, **kw)
        b = run(P, kt=(0.011 * 1.25, 0.024 * 1.25), **kw)
        # 지연 모델 민감도: 순수 수송지연에서의 max도 같이
        c = run(P, mode='delay', **kw)
        print("   %-30s %-8.3f %-8.3f %-8.3f %-8.1f %-8.3f  (수송지연 max %.3f)"
              % (lbl, a[0], a[1], a[2], a[3], b[2], c[2]))
    print()

print("=== 감쇠 여유 ω_n·τ = √2·v/L1·√fb·τ  (건전 대역 0.29~0.44, ②-f) ===")
for name, P in [("지금 라인", P60), ("재생성 라인", P55)]:
    vm = P['v'].max()
    print("  %s (최고 %.2f m/s)" % (name, vm))
    for lbl, kw in LADDER:
        L1 = 0.6 + vm * kw['l1_speed_gain']
        wt = np.sqrt(2) * vm / L1 * np.sqrt(kw['fb_gain']) * 0.14
        print("    %-30s L1 %.2f m  ω_n·τ %.3f %s" %
              (lbl, L1, wt, "← 초과" if wt > 0.44 else ""))
