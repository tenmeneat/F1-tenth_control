#!/usr/bin/env python3
"""lidar_odom_calib.py 원시데이터 정밀 분석 — 끝점 2개만 쓰면 안 되는 이유.

끝점만 빼면 그 순간의 노이즈·빔 튐이 통째로 결과가 된다. 전 구간에서
front_range 대 wheel_dist 를 회귀하면:
  * 기울기 = -(휠/참) 비율 (전 구간 평균이라 훨씬 강건)
  * R^2 / 잔차 = 벽이 평평했는지, 빔이 다른 면으로 옮겨갔는지
  * R^2 > 0.999 인 구간만 신뢰

2026-07-28 실측: 9구간 중 R²>0.999 는 2개뿐이었고 그 둘은 0.9968/1.0028 로 일치
(중앙 0.9998 -> gain 4231, 즉 **당시** 게인 4232 가 정확). 나머지 7구간의 끝점 계산은
+17%/-4%/+2153% 같은 값을 냈다 — 걸러내지 않으면 전부 오독한다.

⚠️ 위 4232 는 오프로드 타이어 시절의 기록이다. 2026-08-04 세미슬릭 교체 후 재보정으로
   그리고 2026-08-19 마모 진행으로 4336 → 4420 이 됐다(아래 GAIN). 예전 로그를 다시
   돌릴 땐 그 시점 게인으로 바꿀 것(07-28 이전 4232 / 08-04~08-18 4336).
"""
import sys

import numpy as np

# 현재 젯슨 vesc.yaml의 speed_to_erpm_gain (2026-08-04 세미슬릭 재보정, 08-05 확정).
GAIN = 4420.0


def main():
    d = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
    t, wd, fr = d['t'], d['wheel_dist'], d['front_range']

    # wheel_dist 가 0 으로 리셋되는 지점에서 구간을 자른다
    starts = [0] + list(np.nonzero(np.diff(wd) < -0.05)[0] + 1)
    segs = [(a, b) for a, b in zip(starts, starts[1:] + [len(t)]) if b - a > 40]
    print(f'구간 {len(segs)}개 검출\n')

    good = []
    for i, (a, b) in enumerate(segs, 1):
        w, f = wd[a:b], fr[a:b]
        ok = np.isfinite(f) & (f > 0.1)
        w, f = w[ok], f[ok]
        if len(w) < 40:
            print(f'#{i}: 유효 표본 부족')
            continue
        A = np.vstack([w, np.ones(len(w))]).T
        (s, f0), *_ = np.linalg.lstsq(A, f, rcond=None)
        pred = s * w + f0
        r2 = 1 - np.sum((f - pred) ** 2) / max(np.sum((f - f.mean()) ** 2), 1e-12)
        ratio = -1.0 / s if s != 0 else float('nan')
        ep, el = w[-1] - w[0], abs(f[0] - f[-1])
        print(f'#{i}  n={len(w):4d}  휠 {ep:.3f} m')
        print(f'    회귀 -> 휠/참 = {ratio:.4f} ({100*(ratio-1):+.1f}%)   R² = {r2:.5f}   '
              f'잔차 sigma = {1000*np.std(f-pred):.0f} mm')
        print(f'    끝점 방식 = {ep/el if el>0 else float("nan"):.4f}  '
              f'(회귀와 차이 {100*(ep/max(el,1e-9) - ratio):+.1f}%p)')
        if r2 < 0.999:
            print('    ⚠️ 직선성 나쁨 — 벽이 평평하지 않거나 빔이 다른 면으로 옮겨갔다. 버릴 것')
        else:
            good.append(ratio)
        print()

    print('=== 종합 ===')
    print(f'  전체 {len(segs)}구간, R²>0.999 인 신뢰 구간 {len(good)}개')
    if good:
        rs = np.array(good)
        print(f'  휠/참 비율: {np.round(rs,4).tolist()}   중앙 {np.median(rs):.4f} '
              f'({100*(np.median(rs)-1):+.1f}%)')
        print(f'  -> speed_to_erpm_gain: {GAIN:.0f} -> {GAIN*np.median(rs):.0f}')
        if rs.max() - rs.min() > 0.05:
            print('  ⚠️ 신뢰 구간끼리도 5%p 넘게 벌어짐 — 측정 조건 재검토')


if __name__ == '__main__':
    main()
