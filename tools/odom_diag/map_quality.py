#!/usr/bin/env python3
"""맵 품질 비교 — 루프가 벌어지면 벽이 두 겹으로 그려진다.

map->odom 표류가 0 이라고 맵이 좋은 건 아니다(스캔매칭을 꺼도 0 이 나온다).
실제 산출물인 점유격자를 직접 재야 한다.

  * 점유 픽셀 수  : 벽이 이중으로 찍히면 늘어난다 (같은 트랙이므로 적을수록 선명)
  * 벽 두께       : 점유 픽셀의 8방향 점유 이웃 수 평균
  * 바운딩박스    : 루프가 안 닫히면 트랙이 늘어져 커진다

⚠️ 한 바퀴짜리 bag 에서는 갭이 국소적이라 전역 픽셀 수로 잘 안 잡힌다
   (2026-07-28: 표류를 0 으로 만들어도 점유 픽셀은 -1.8% 뿐이었다).
   최종 판정은 사람 눈으로 맵을 볼 것.

사용: map_quality.py /tmp/slamrun/<라벨> [...]
"""
import sys

import numpy as np
from PIL import Image


def analyze(path, res=0.05):
    a = np.array(Image.open(path).convert('L'))
    occ, free = a < 100, a > 200
    ys, xs = np.nonzero(occ)
    if len(xs) == 0:
        return None
    p = np.pad(occ, 1)
    nb = sum(p[1 + dy:p.shape[0] - 1 + dy, 1 + dx:p.shape[1] - 1 + dx]
             for dy in (-1, 0, 1) for dx in (-1, 0, 1)
             if not (dx == 0 and dy == 0)).astype(float)
    return dict(occ=int(occ.sum()), free=int(free.sum()),
                w=(xs.max() - xs.min() + 1) * res, h=(ys.max() - ys.min() + 1) * res,
                thick=float(nb[occ].mean()))


if __name__ == '__main__':
    print(f'{"실행":<14} {"점유px":>8} {"벽두께":>7} {"자유px":>8} '
          f'{"가로":>7} {"세로":>7}')
    base = None
    for p in sys.argv[1:]:
        label = p.rstrip('/').split('/')[-1]
        try:
            m = analyze(p.rstrip('/') + '/map.pgm')
        except FileNotFoundError:
            print(f'{label:<14} map.pgm 없음 (저장 실패 — save.log 확인)')
            continue
        if m is None:
            print(f'{label:<14} 빈 맵')
            continue
        base = base or m
        d = 100 * (m['occ'] / base['occ'] - 1)
        print(f'{label:<14} {m["occ"]:8d} {m["thick"]:7.2f} {m["free"]:8d} '
              f'{m["w"]:6.2f}m {m["h"]:6.2f}m'
              f'{"" if base is m else f"   점유 {d:+.1f}%"}')
    print('\n(같은 트랙 같은 주행이므로, 점유 픽셀이 적고 벽이 얇을수록 이중선이 없는 것)')
