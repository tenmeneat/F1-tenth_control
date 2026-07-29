#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sideslip_estimator.py — 차체 횡슬립각 β 추정기 (오프라인 레퍼런스 + 검증)
================================================================================
odom 강건화(슬립 중 위치 누출 차단)와 카운터스티어(자세 복구)는 **같은 상태 β 하나**를
쓴다. 이 파일은 C++ 포팅 전에 거동을 확정하기 위한 레퍼런스 구현이고, 동시에 기존
주행 bag에 돌려 "크래시가 정말 β로 설명되는가"를 판정하는 검증 도구다.

원리 — 새 센서 0개:
    a_y = v̇_y + v_x·ψ̇        (차체 좌표 횡방향 운동방정식)
  ⇒ v̇_y = a_y − v_x·ψ̇        (우변은 전부 이미 검증된 값)

  v_x  : ERPM 유래 (07-28 라이다 대조 ±0.3%)
  ψ̇    : VESC 자이로 (07-28 닫힌루프 +0.06%)
  a_y  : VESC 가속도계 = −a_y_raw (07-29 축 확정: 전방=−a_x, 좌측=−a_y, 위=+z)

  ⚠️ 순수 적분은 가속도계 바이어스로 발산한다 → **누설 적분기**(τ). 실제 슬립은 1초
     미만이라 손실이 없고, 직진에서 자동으로 0으로 되돌아온다.

부호 규약 (REP-103): ψ̇>0 = 반시계 = 좌선회. v_y>0 = 좌측. β>0 = 속도벡터가 헤딩보다 좌측.
  오버스티어 좌선회 → 뒤가 우측으로 흐름 → 속도벡터가 헤딩보다 우측 → β<0 → 우조향으로
  카운터 → δ_corr = k_β·β (k_β>0) 가 자동으로 맞는 부호를 낸다.

사용법:
  source /opt/ros/<distro>/setup.bash
  python3 sideslip_estimator.py <bag...> [--out slip.png] [--tau 1.5] [--scale-y 0.95]
"""
import os, sys, glob, sqlite3, math, argparse
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

G = 9.80665
D2R = math.pi / 180.0
WHEELBASE = 0.33
STEER_REACH = 0.74      # 실측 도달각/명령각 (07-28, 3회 재현). 분류용 기준에만 쓴다.
GRIP_ALAT = 6.7         # LUT 실그립 피크 [m/s²] — 기구학 기준의 상한

# ── 추정기 튜너블 (C++ 포팅 시 파라미터가 될 값들) ──
TAU_LEAK = 1.0          # [s] 누설 시정수. 슬립 이벤트(<1s)는 살리고 바이어스는 죽인다
V_MIN_BETA = 1.0        # [m/s] β 분모 하한 (저속 특이점)
V_GATE = 0.5            # [m/s] 이 미만이면 추정기 리셋
SCALE_Y = 1.03          # a_y→a_lat 배율. 07-29 실측(0726/0728 bag 회귀 1.00~1.09)
# 바이어스 온라인 학습 게이트 — "지금 횡가속이 0인 게 확실한" 조건에서만 배운다
TAU_BIAS = 5.0          # [s] 바이어스 학습 시정수 (느리게)
BIAS_YAW_MAX = 0.10     # [rad/s] 이보다 조용해야 직진으로 인정
BIAS_V_MIN = 1.0        # [m/s] 정지 말고 **주행 중** 직진에서 배운다


class SideslipEstimator:
    """C++ 포팅 대상. 상태는 v_y와 bias 두 개뿐이다.

    ⚠️ 바이어스를 **정지**가 아니라 **직진 주행 중**에 배우는 게 핵심이다.
       07-29 실측: 정지 +0.32 vs 직진 +0.51 m/s²로 서로 다르다(0726_200957).
       정지 기준으로 보정하면 잔차 0.2 m/s²가 남고, τ=1s 적분에서 유령 횡속도
       0.2 m/s → 3 m/s 주행에서 β 오차 4°가 된다. 진짜 슬립과 구분이 안 된다.
    """

    def __init__(self, tau=TAU_LEAK, scale_y=SCALE_Y, bias_y=0.0, adapt=True):
        self.tau = tau
        self.scale_y = scale_y
        self.bias_y = bias_y        # [m/s²] 초기값(정지 구간 등). 이후 온라인 갱신
        self.adapt = adapt
        self.vy = 0.0

    def reset(self):
        self.vy = 0.0               # 바이어스는 유지 — 재획득 비용이 크다

    def update(self, dt, ay_raw_g, yaw_rate, vx):
        """ay_raw_g: IMU 원시 y [g] / yaw_rate: [rad/s] / vx: [m/s]"""
        if dt <= 0.0 or dt > 0.5:
            return 0.0, 0.0, 0.0, self.bias_y

        a_lat_raw = -ay_raw_g * G / self.scale_y     # 축·배율 보정 (좌측 +)

        # ① 바이어스 온라인 학습 — 직진 주행 중에는 a_lat이 0이어야 한다
        if self.adapt and vx > BIAS_V_MIN and abs(yaw_rate) < BIAS_YAW_MAX:
            k = 1.0 - math.exp(-dt / TAU_BIAS)
            self.bias_y += k * (a_lat_raw - self.bias_y)

        if vx < V_GATE:
            self.reset()
            return 0.0, 0.0, 0.0, self.bias_y

        # ② 운동방정식에서 횡속도 변화율 분리
        a_lat = a_lat_raw - self.bias_y
        vdot_y = a_lat - vx * yaw_rate
        # ③ 누설 적분 (해석적 leak — dt 변동에 안전)
        self.vy = (self.vy + vdot_y * dt) * math.exp(-dt / self.tau)
        beta = math.atan2(self.vy, max(vx, V_MIN_BETA))
        return beta, self.vy, vdot_y, self.bias_y


# ══ bag 로딩 ═════════════════════════════════════════════════════════════════
def read_topic(con, topic):
    cur = con.cursor()
    r = cur.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    if r is None:
        return []
    M = get_message(r[1])
    out = []
    for ts, d in cur.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],)):
        try:
            out.append((ts * 1e-9, deserialize_message(bytes(d), M)))
        except Exception:
            continue
    return out


def load(path):
    p = path
    if os.path.isdir(p):
        c = sorted(glob.glob(os.path.join(p, "*.db3")))
        if not c:
            raise FileNotFoundError(f"{p}: .db3 없음")
        p = c[0]
    con = sqlite3.connect(p)
    imu = read_topic(con, "/sensors/imu/raw") or read_topic(con, "/imu/data")
    odom = read_topic(con, "/odom") or read_topic(con, "/pf/pose/odom")
    drive = read_topic(con, "/drive")
    con.close()
    if not imu or not odom:
        raise RuntimeError("IMU 또는 odom 없음")

    S = {}
    S["t"] = np.array([t for t, _ in imu])
    S["ay"] = np.array([m.linear_acceleration.y for _, m in imu])      # 원시 [g]
    S["ax"] = np.array([m.linear_acceleration.x for _, m in imu])
    S["gz"] = np.array([m.angular_velocity.z for _, m in imu]) * D2R   # → rad/s
    ot = np.array([t for t, _ in odom])
    ov = np.array([m.twist.twist.linear.x for _, m in odom])
    S["v"] = np.interp(S["t"], ot, ov)
    if drive:
        dt_ = np.array([t for t, _ in drive])
        S["delta"] = np.interp(S["t"], dt_, [m.drive.steering_angle for _, m in drive])
        S["vcmd"] = np.interp(S["t"], dt_, [m.drive.speed for _, m in drive])
    else:
        S["delta"] = np.zeros_like(S["t"])
        S["vcmd"] = np.full_like(S["t"], np.nan)
    return S


def learn_bias(S):
    """정지 구간(속도≈0, 자이로 조용)의 평균 a_lat → 바이어스 [m/s²]."""
    still = (np.abs(S["v"]) < 0.05) & (np.abs(S["gz"]) < 2.0 * D2R)
    if still.sum() < 25:
        return 0.0, 0
    return float(np.mean(-S["ay"][still] * G)), int(still.sum())


# ══ 실행 ═════════════════════════════════════════════════════════════════════
def run(S, tau, scale_y, bias_y):
    est = SideslipEstimator(tau=tau, scale_y=scale_y, bias_y=bias_y)
    n = len(S["t"])
    beta = np.zeros(n); vy = np.zeros(n); vdy = np.zeros(n); bias = np.zeros(n)
    for i in range(1, n):
        dt = S["t"][i] - S["t"][i - 1]
        beta[i], vy[i], vdy[i], bias[i] = est.update(dt, S["ay"][i], S["gz"][i], S["v"][i])
    return beta, vy, vdy, bias


def sanity(S):
    """이 bag이 β 추정에 쓸 수 있는지 — a_y가 v·ψ̇와 상관이 있어야 한다.
    07-24 IMU 필터 수정 이전 bag은 여기서 걸러진다(0723: 스케일 0.052)."""
    turn = (S["v"] > 1.0) & (np.abs(S["gz"]) > 0.15)
    if turn.sum() < 30:
        return None, None
    x = (S["v"] * S["gz"])[turn]; y = (-S["ay"] * G)[turn]
    s = float(np.sum(x * y) / np.sum(x * x))
    r = float(np.corrcoef(x, y)[0, 1])
    return s, r


def classify(S):
    """언더/오버 판정 신호. 기준 요레이트는 실도달각(74%) + 그립 상한으로 이중 제한."""
    v = np.maximum(S["v"], 0.1)
    psi_kin = v * np.tan(S["delta"] * STEER_REACH) / WHEELBASE
    psi_grip = GRIP_ALAT / v
    psi_ref = np.sign(psi_kin) * np.minimum(np.abs(psi_kin), psi_grip)
    # 언더스티어 = 기준만큼 안 돌아감 (양수)
    under = np.abs(psi_ref) - np.abs(S["gz"])
    return psi_ref, under


def report(name, S, beta, vy, vdy, psi_ref, under, args):
    mv = S["v"] > 1.0
    if mv.sum() < 20:
        print(f"  주행 구간 부족 — 생략")
        return
    bd = np.degrees(beta)
    print(f"  β [deg]  RMS {np.sqrt(np.mean(bd[mv]**2)):5.2f}   "
          f"p95 {np.quantile(np.abs(bd[mv]),0.95):5.2f}   최대 {np.max(np.abs(bd[mv])):5.2f}")
    print(f"  |a_lat| = v·ψ̇  p95 {np.quantile(np.abs((S['v']*S['gz'])[mv]),0.95):5.2f} m/s²")

    # 풀락 에피소드 (|δ| ≥ 0.9·0.41 가 0.2초 이상 지속)
    lock = (np.abs(S["delta"]) >= 0.9 * 0.41) & mv
    ep = []
    i = 0
    while i < len(lock):
        if lock[i]:
            j = i
            while j < len(lock) and lock[j]:
                j += 1
            if S["t"][j - 1] - S["t"][i] >= 0.2:
                ep.append((i, j))
            i = j
        else:
            i += 1
    print(f"  풀락 에피소드 {len(ep)}건")
    for i, j in ep[:6]:
        t0 = S["t"][i] - S["t"][0]
        sl = slice(i, j)
        ratio = np.mean(np.abs(S["gz"][sl])) / max(np.mean(np.abs(psi_ref[sl])), 1e-6)
        print(f"    t={t0:6.2f}s  {S['t'][j-1]-S['t'][i]:.2f}s  v {S['v'][sl].mean():4.2f}  "
              f"ψ̇/ψ̇ref {ratio:5.2f}  β {np.degrees(beta[sl]).mean():+6.2f}° "
              f"(|max| {np.max(np.abs(np.degrees(beta[sl]))):5.2f}°)  "
              f"언더 {under[sl].mean():+5.2f} rad/s")
    return ep


def plot(name, S, beta, vy, vdy, psi_ref, ep, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = S["t"] - S["t"][0]
    fig, ax = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    ax[0].plot(t, S["v"], lw=.9, label="v (odom)")
    if not np.all(np.isnan(S["vcmd"])):
        ax[0].plot(t, S["vcmd"], lw=.7, alpha=.6, label="v cmd")
    ax[0].set_ylabel("speed [m/s]"); ax[0].legend(fontsize=7)
    ax[1].plot(t, S["delta"], lw=.9, label="steer cmd")
    ax[1].axhline(0.41, color="r", ls=":", lw=.7); ax[1].axhline(-0.379, color="r", ls=":", lw=.7)
    ax[1].set_ylabel("steer [rad]"); ax[1].legend(fontsize=7)
    ax[2].plot(t, S["gz"], lw=.9, label="yaw rate (gyro)")
    ax[2].plot(t, psi_ref, lw=.8, alpha=.7, label="ref (74% reach, grip-capped)")
    ax[2].set_ylabel("yaw rate [rad/s]"); ax[2].legend(fontsize=7)
    ax[3].plot(t, np.degrees(beta), lw=1.0, color="crimson", label="beta")
    ax[3].axhline(0, color="k", lw=.5)
    ax[3].set_ylabel("sideslip [deg]"); ax[3].set_xlabel("t [s]"); ax[3].legend(fontsize=7)
    for i, j in (ep or []):
        for a in ax:
            a.axvspan(t[i], t[j - 1], color="orange", alpha=.18)
    fig.suptitle(name)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  → {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--tau", type=float, default=TAU_LEAK)
    ap.add_argument("--scale-y", type=float, default=0.95)
    ap.add_argument("--outdir", default="/tmp")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    for p in args.bags:
        name = os.path.basename(p.rstrip("/"))
        print("=" * 78); print(f"■ {name}")
        try:
            S = load(p)
        except Exception as e:
            print(f"  건너뜀: {e}"); continue
        bias0, nb = learn_bias(S)
        s, r = sanity(S)
        print(f"  {len(S['t'])}샘플 {S['t'][-1]-S['t'][0]:.1f}s  최고속 {S['v'].max():.2f} m/s"
              f"  |  정지바이어스 {bias0:+.3f} ({nb}샘플)"
              + (f"  |  a_y↔v·ψ̇ 스케일 {s:.3f} r={r:+.3f}" if s is not None else ""))
        if s is not None and (r < 0.7 or not (0.5 < s < 1.6)):
            print(f"  ⛔ 이 bag은 a_y가 v·ψ̇와 안 맞는다 → β 추정 불가 (07-24 IMU 수정 이전?)")
            continue
        beta, vy, vdy, bias = run(S, args.tau, args.scale_y, bias0)
        print(f"  학습된 바이어스 {bias[-1]:+.3f} m/s² (정지값 대비 {bias[-1]-bias0:+.3f})")
        psi_ref, under = classify(S)
        ep = report(name, S, beta, vy, vdy, psi_ref, under, args)
        if not args.no_plot:
            plot(name, S, beta, vy, vdy, psi_ref, ep,
                 os.path.join(args.outdir, f"slip_{name}.png"))


if __name__ == "__main__":
    main()
