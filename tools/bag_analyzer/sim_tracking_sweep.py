"""control_map_node 조향 법칙 폐루프 시뮬 + steering_fb_gain / curvature_ff_preview /
l1_speed_gain 스윕.

모델이 잡는 것: L1 룩어헤드가 만드는 코너 안쪽 파고듦(기하), FF/FB 분리, 조향 지연,
rate limit, 좌/우 비대칭 K_us, 그립 클램프.
모델이 못 잡는 것: 타이어 과도응답, 노면 변화, 위치추정 오차(별도로 더한다).
"""
import numpy as np

import os
SC = os.environ.get('SIM_DATA_DIR', '.') + '/'
L_WB = 0.33
DT = 1.0 / 50.0
STEP = 0.25            # 실차 웨이포인트 간격


def build_path(mla=6.0, max_speed=8.0, min_speed=2.0, prebrake=2.6, accel=3.5,
               sector_scale=1.0):
    """글로벌 라인을 0.25 m 격자로 리샘플하고, 컨트롤러와 같은 규칙으로 속도 프로파일을 만든다."""
    d = np.load(SC + 'run.npz')
    W = d['W']                                   # x y s kappa vx psi
    s = W[:, 2]
    L = s[-1] + np.hypot(W[0, 0] - W[-1, 0], W[0, 1] - W[-1, 1])
    ns = np.arange(0, L, STEP)
    x = np.interp(ns, s, W[:, 0])
    y = np.interp(ns, s, W[:, 1])
    k = np.interp(ns, s, W[:, 3])
    vx = np.interp(ns, s, W[:, 4])

    # 컨트롤러의 smooth_curvature (window_half_m 0.3 → 0.25 격자에서 ±1점)
    ks = (np.roll(k, 1) + k + np.roll(k, -1)) / 3.0
    ka = (np.roll(np.abs(k), 1) + np.abs(k) + np.roll(np.abs(k), -1)) / 3.0

    # 지점별 상한: 프로파일 ∧ 그립캡 ∧ max_speed  (조향 권한 캡은 이 트랙에서 0점 구속 — ②-j)
    cap = np.minimum(vx, max_speed)
    m = ka > 0.01
    # 섹터 스케일은 **코너에만** 걸린다(직선은 그립 캡 비활성 → 이득 0)
    mla_pt = np.where(ka > 0.20, mla * sector_scale, mla)
    cap[m] = np.minimum(cap[m], np.sqrt(mla_pt[m] / ka[m]))
    mla_field = mla_pt

    # 전방-후방 패스 (감속 권한 prebrake, 가속 권한 accel), 랩을 도는 폐곡선이라 2회 순회
    v = cap.copy()
    n = len(v)
    for _ in range(3):
        for i in range(n - 1, -1, -1):          # backward: 다음 지점까지 감속 가능한가
            j = (i + 1) % n
            v[i] = min(v[i], np.sqrt(max(v[j] ** 2 + 2 * prebrake * STEP, 0.0)))
        for i in range(n):                      # forward: 가속 권한
            j = (i + 1) % n
            v[j] = min(v[j], np.sqrt(max(v[i] ** 2 + 2 * accel * STEP, 0.0)))
    v = np.maximum(v, min_speed)
    # 종가속 a_x = v·dv/ds (컨트롤러의 acc_mean 대용 — IMU 실측과 부호·크기 규약 동일)
    dv = (np.roll(v, -1) - np.roll(v, 1)) / (2 * STEP)
    ax = v * dv
    return dict(x=x, y=y, s=ns, k=k, ks=ks, ka=ka, v=v, ax=ax, mla=mla_field,
                L=L, n=len(ns))


def walk_forward(P, i, dist):
    """호 길이 dist만큼 전진한 인덱스 (컨트롤러 walk_forward와 같은 취지)."""
    return int((i + int(round(dist / STEP))) % P['n'])


def simulate(P, p, laps=4, kus_true=(0.011, 0.024), pose_noise=0.0, seed=1):
    return simulate_x(P, p, laps, kus_true, pose_noise, seed, 'lag')


def simulate_x(P, p, laps=4, kus_true=(0.011, 0.024), pose_noise=0.0, seed=1,
               delay_mode='lag'):
    n, X, Y = P['n'], P['x'], P['y']
    rng = np.random.default_rng(seed)
    i = 0
    x, y = X[0], Y[0]
    yaw = np.arctan2(Y[1] - Y[0], X[1] - X[0])
    delta = 0.0
    delta_prev_cmd = 0.0
    ndelay = max(1, int(round(p['lag'] / DT)))
    dq = [0.0] * ndelay
    dist = 0.0
    warm = P['L'] * 1.0          # 첫 한 바퀴는 과도구간이라 버린다
    rec_e, rec_v, rec_k, rec_d = [], [], [], []

    while dist < laps * P['L']:
        # --- 최근접 (컨트롤러: 이전 인덱스 주변 윈도우) ---
        rng_idx = (np.arange(i - 4, i + 12) % n)
        dd = np.hypot(X[rng_idx] - x, Y[rng_idx] - y)
        i = int(rng_idx[np.argmin(dd)])

        # 부호 있는 횡오차 (좌 +)
        tx = np.arctan2(Y[(i + 1) % n] - Y[i], X[(i + 1) % n] - X[i])
        e_true = -np.sin(tx) * (x - X[i]) + np.cos(tx) * (y - Y[i])

        # 컨트롤러가 보는 포즈 (위치추정 잡음)
        if pose_noise > 0:
            xm, ym = x + rng.normal(0, pose_noise), y + rng.normal(0, pose_noise)
            rng_idx = (np.arange(i - 4, i + 12) % n)
            dd = np.hypot(X[rng_idx] - xm, Y[rng_idx] - ym)
            im = int(rng_idx[np.argmin(dd)])
            lat_err = dd.min()
        else:
            xm, ym, im, lat_err = x, y, i, abs(e_true)

        v = P['v'][im]
        v = min(v, max(0.5, v))                      # 실측 상한(정상주행에선 프로파일 추종)
        v = max(v, p['speed_floor'])

        # --- L1 ---
        L1 = p['l1_offset'] + v * p['l1_speed_gain']
        kc = P['ka'][im]
        if kc > 0.3:
            L1 *= (1.0 - 0.25 * min(1.0, (kc - 0.3) / 1.0))
        L1 = max(max(p['t_clip_min'], np.sqrt(2.0) * lat_err), min(L1, p['t_clip_max']))
        ia = walk_forward(P, im, L1)
        dx, dy = X[ia] - xm, Y[ia] - ym
        Ln = np.hypot(dx, dy)
        sin_eta = np.clip((-np.sin(yaw) * dx + np.cos(yaw) * dy) / max(Ln, 1e-5), -1, 1)
        lat_acc = 2.0 * v * v / max(Ln, p['l1_min_denom']) * sin_eta

        # --- FF/FB 분리 + 자전거 역모델 ---
        iff = walk_forward(P, im, p['ff_preview']) if p['ff_preview'] > 1e-6 else im
        a_ff = P['ks'][iff] * v * v
        a_cmd = a_ff + p['fb_gain'] * (lat_acc - a_ff)
        a_max = P['mla'][im] * p['margin']
        a_cmd = float(np.clip(a_cmd, -a_max, a_max))
        kus_c = p['kus_l'] if a_cmd > 0 else p['kus_r']
        d_cmd = a_cmd * (L_WB / max(v * v, 1e-4) + kus_c)

        # 가감속 조향 스케일러 (ref 1.0 / accel 1.0 / decel 0.95)
        ax = P['ax'][im]
        w = min(abs(ax) / p['scaler_ref'], 1.0)
        d_cmd *= (1.0 - w) + w * (p['accel_scaler'] if ax >= 0 else p['decel_scaler'])
        # 속도 비례 다운스케일 (start 7.0 / end 8.0 / factor 0.10)
        d_cmd *= (1.0 - np.clip((v - 7.0) / 1.0, 0, 1) * p['downscale']) 

        # rate limit + 물리 한계
        lim = p['max_rate'] * DT
        d_cmd = float(np.clip(d_cmd, delta_prev_cmd - lim, delta_prev_cmd + lim))
        d_cmd = float(np.clip(d_cmd, -0.410, 0.410))
        delta_prev_cmd = d_cmd

        # --- 차량 ---
        if delay_mode == 'delay':
            dq.append(d_cmd); delta = dq.pop(0)
        else:
            delta += (d_cmd - delta) * (DT / p['lag'])
        kus_t = kus_true[0] if delta > 0 else kus_true[1]
        yaw_rate = v * delta / (L_WB + kus_t * v * v)
        yaw += yaw_rate * DT
        x += v * np.cos(yaw) * DT
        y += v * np.sin(yaw) * DT
        dist += v * DT
        if dist > warm:
            rec_e.append(e_true); rec_v.append(v); rec_k.append(P['ks'][i]); rec_d.append(delta)
    return (np.array(rec_e), np.array(rec_v), np.array(rec_k), np.array(rec_d))


BASE = dict(l1_offset=0.6, l1_speed_gain=0.4, t_clip_min=0.9, t_clip_max=5.0,
            l1_min_denom=0.9, fb_gain=1.0, ff_preview=0.0,
            kus_l=0.011, kus_r=0.024, mla=6.0, max_rate=20.0, lag=0.14,
            speed_floor=0.5, margin=1.0, accel_scaler=1.0, decel_scaler=1.0, scaler_ref=1.0,
            downscale=0.10)


def summarize(e, v, k, tag):
    a = np.abs(e)
    corner = np.abs(k) > 0.20
    # 조향 진동 지표: 발행 조향의 2차 미분 RMS 대용 = 부호교대율
    return dict(tag=tag, p50=np.median(a), p95=np.percentile(a, 95), mx=a.max(),
                cp50=np.median(a[corner]) if corner.any() else 0,
                cp95=np.percentile(a[corner], 95) if corner.any() else 0,
                over02=100.0 * np.mean(a > 0.20))
