#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_longitudinal.py — 종방향 권한(가속·감속) 실측 분석  [게이트 1 전용]
================================================================================
"왜 2.5 m/s 위에서 가속이 죽는가 / 감속이 왜 0인가"를 bag 하나로 판정한다.

2026-07-26 배경: 07-25 실차 bag에서 가속 권한이 속도대별로
  1.0~1.5 m/s → 4.93 m/s²   /   2.5~3.0 m/s → 0.34 m/s²
로 붕괴하고 감속은 전 구간 ~0인 것이 확인됐다. 최고속 6.0 m/s는 직선 13.6m에서
a·D ≥ v²−v_corner² 를 만족해야 하므로 가감속 2.23 m/s² 이상이 필요하다 →
종방향 권한이 유일한 병목. 이 스크립트가 그 값을 재고 원인을 좁힌다.

의존성 주의: `/sensors/core`(vesc_msgs/VescStateStamped)는 **랩탑에 vesc_msgs가 없어도**
읽을 수 있도록 자체 CDR 파서를 쓴다. 필드 레이아웃은 vesc_msgs 버전마다 다르고
**voltage_input이 선두일 거라는 흔한 가정이 틀리다** — 실차(Humble) 실제 순서는
temp_fet, temp_motor, current_motor, current_input, avg_id, avg_iq, duty_cycle, speed,
voltage_input 이다(2026-07-26 젯슨 echo로 확인·검증). 여러 후보를 시도해 전압/듀티
타당성과 **odom 속도 대비 ERPM 비**(speed_to_erpm_gain 4420 부근)로 자동 판별한다.

⚠️ 게인은 2026-08-04 세미슬릭 타이어 교체로 4232 → 4336, 2026-08-19 마모 진행으로
   4336 → 4420이 됐다. **그 이전에 녹화한 bag은 `--erpm-gain`으로 그 시점 값을 명시**해야
   레이아웃 자동 판별이 맞는다(08-04 이전 4232 / 08-04~08-18 4336).

사용법:
  source /opt/ros/<distro>/setup.bash
  python3 analyze_longitudinal.py <bag폴더 | .db3> [--odom /odom] [--erpm-gain 4420]
"""
import sys, os, glob, sqlite3, struct, math, argparse
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WHEELBASE = 0.33


# ══ 자체 CDR 리더 (vesc_msgs 미설치 대응) ═══════════════════════════════════
class CDR:
    """ROS2 CDR(리틀엔디안 가정) 최소 리더. 정렬 규칙을 지켜야 필드가 안 밀린다."""
    def __init__(self, buf):
        self.b = buf
        self.p = 4          # 4바이트 encapsulation 헤더 건너뜀

    def _align(self, n):
        rem = (self.p - 4) % n
        if rem:
            self.p += n - rem

    def u32(self):
        self._align(4); v = struct.unpack_from('<I', self.b, self.p)[0]; self.p += 4; return v

    def i32(self):
        self._align(4); v = struct.unpack_from('<i', self.b, self.p)[0]; self.p += 4; return v

    def f64(self):
        self._align(8); v = struct.unpack_from('<d', self.b, self.p)[0]; self.p += 8; return v

    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode('utf-8', 'replace')
        self.p += n
        return s

    def header(self):
        """std_msgs/Header = builtin_interfaces/Time(sec:int32, nanosec:uint32) + string"""
        self.i32(); self.u32(); self.string()


# 젯슨 실차에서 `ros2 topic echo /sensors/core`로 직접 확인한 실제 필드 순서(2026-07-26).
# ⚠️ voltage_input이 첫 번째가 아니라 9번째고, duty_cycle이 speed보다 앞이다 — 흔한 오해라
#    처음엔 (volt, temp, cur_motor, cur_in, speed, duty)로 잘못 가정했었다.
VESC_LAYOUTS = {
    # 이름: (필드 순서 리스트) — 전부 float64, 헤더 직후부터 순서대로
    "humble_f1tenth": ["temp_fet", "temp_motor", "cur_motor", "cur_in", "avg_id", "avg_iq",
                       "duty", "erpm", "volt"],
    # 구버전(온도 1개, voltage 선두) 호환용 — 자동 판별이 알아서 고른다
    "legacy_volt_first": ["volt", "temp_pcb", "cur_motor", "cur_in", "erpm", "duty"],
}


def parse_vesc(buf, layout):
    """VescStateStamped를 vesc_msgs 없이 파싱. layout = VESC_LAYOUTS의 필드 순서 리스트."""
    try:
        c = CDR(buf)
        c.header()
        vals = {name: c.f64() for name in layout}
        return dict(volt=vals.get("volt", float("nan")),
                    cur_motor=vals.get("cur_motor", float("nan")),
                    cur_in=vals.get("cur_in", float("nan")),
                    erpm=vals.get("erpm", float("nan")),
                    duty=vals.get("duty", float("nan")))
    except Exception:
        return None


def load(db3):
    con = sqlite3.connect(db3)
    topics = {tid: (n, t) for tid, n, t in con.execute("SELECT id,name,type FROM topics")}
    out, raw, t0 = {}, {}, None
    for tid, ts, blob in con.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
        if t0 is None:
            t0 = ts
        name, typ = topics[tid]
        rt = (ts - t0) / 1e9
        if 'VescState' in typ:                       # vesc_msgs 없이 원본 바이트 보관
            raw.setdefault(name, []).append((rt, blob))
            continue
        try:
            cls = get_message(typ)
        except Exception:
            continue
        try:
            out.setdefault(name, []).append((rt, deserialize_message(blob, cls)))
        except Exception:
            pass
    con.close()
    return out, raw


def resolve_vesc(raw_list, t_odom, vx_odom, erpm_gain):
    """알려진 레이아웃을 모두 시도해 ERPM/vx 비가 erpm_gain에 가장 가까운 쪽을 채택한다.

    ⚠️ vesc_msgs 버전마다 필드 순서가 달라 하드코딩은 조용히 틀린다. 실측 vx와의
       ERPM 비(=speed_to_erpm_gain, 4420 근처여야 함)를 지문으로 써서 자동 판별한다.
    """
    best = None
    for name, layout in VESC_LAYOUTS.items():
        rows = [(t, parse_vesc(b, layout)) for t, b in raw_list]
        rows = [(t, d) for t, d in rows if d]
        if len(rows) < 20:
            continue
        t = np.array([r[0] for r in rows])
        erpm = np.array([r[1]['erpm'] for r in rows])
        volt = np.array([r[1]['volt'] for r in rows])
        duty = np.array([r[1]['duty'] for r in rows])
        v_at = np.interp(t, t_odom, vx_odom)
        m = v_at > 0.8
        if m.sum() < 10:
            continue
        gain = float(np.median(erpm[m] / v_at[m]))
        # 타당성: 전압 6~30V, 듀티 |.|<=1.05
        plaus = (6 < np.median(volt) < 30) and (np.nanmax(np.abs(duty)) < 1.05)
        score = abs(gain - erpm_gain) / erpm_gain + (0 if plaus else 10)
        cand = dict(layout=name, gain=gain, plaus=plaus, score=score, rows=rows)
        if best is None or score < best['score']:
            best = cand
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--erpm-gain", type=float, default=4420.0,
                    help="speed_to_erpm_gain. 2026-08-19 마모 재보정 후 4420. "
                         "그 이전 bag은 4232를 넘길 것")
    a = ap.parse_args()

    db3 = a.bag if a.bag.endswith(".db3") else (sorted(glob.glob(os.path.join(a.bag, "*.db3"))) or [None])[0]
    if not db3:
        sys.exit(f"[에러] {a.bag} 에 .db3 없음")
    print(f"bag: {db3}\n")
    data, raw = load(db3)

    od = data.get(a.odom) or data.get("/odom") or data.get("/pf/pose/odom")
    if not od:
        sys.exit("[에러] odom 토픽 없음")
    t = np.array([x[0] for x in od])
    vx = np.array([m.twist.twist.linear.x for _, m in od])
    dt = np.gradient(t)
    dt = np.maximum(dt, 0.5 * float(np.median(dt[dt > 0])))
    dv = np.convolve(np.gradient(vx) / dt, np.ones(7) / 7, mode="same")

    drv = data.get("/drive") or data.get("/drive_autonomous") or []
    if drv:
        cmd = np.interp(t, [x[0] for x in drv], [m.drive.speed for _, m in drv])
    else:
        cmd = np.full_like(vx, np.nan)
        print("⚠️ /drive·/drive_autonomous 없음 — 명령 기준 구간 분리 불가, 실측만 사용\n")

    brake = data.get("/commands/motor/brake") or []
    print(f"주행 {t[-1]:.1f}s | vx 최대 {vx.max():.2f} m/s | "
          f"브레이크 메시지 {len(brake)}건" + (f" (최대 {max(m.data for _, m in brake):.1f}A)" if brake else ""))

    # ── VESC 텔레메트리 ──────────────────────────────────────────────────
    V = None
    for nm, rl in raw.items():
        got = resolve_vesc(rl, t, vx, a.erpm_gain)
        if got:
            V = got
            print(f"\n{nm}: 레이아웃 자동판별 = {got['layout']}, "
                  f"ERPM/vx = {got['gain']:.0f} (기대 {a.erpm_gain:.0f}) "
                  f"{'✅' if abs(got['gain']-a.erpm_gain)/a.erpm_gain < 0.25 else '⚠️ 불일치 — 레이아웃 의심'}")
            break
    if V is None:
        print("\n⚠️ /sensors/core 없음 또는 파싱 실패 → 가속 붕괴 '원인'은 판정 불가(현상만 측정).")

    # ── ① 가속 권한: 명령이 실측보다 충분히 앞선 = 전개도 100% 요구 구간 ──
    print("\n" + "═" * 74)
    print("① 가속 권한 — 속도대별 실측 dvx/dt")
    print("═" * 74)
    acc_m = (cmd - vx > 0.5) if drv else (dv > 0)
    hdr = f"{'vx 구간':>11s} {'n':>5s} {'중앙':>7s} {'p90':>7s}"
    if V:
        hdr += f" | {'듀티':>6s} {'모터A':>7s} {'전압V':>7s}"
    print(hdr)
    tv = np.array([r[0] for r in V['rows']]) if V else None
    prof = {}
    for k in ('duty', 'cur_motor', 'volt'):
        if V:
            prof[k] = np.interp(t, tv, np.array([r[1][k] for r in V['rows']]))
    bands = [(0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0),
             (3.0, 3.5), (3.5, 4.0), (4.0, 5.0), (5.0, 7.0)]
    acc_curve = []
    for lo, hi in bands:
        s = acc_m & (vx >= lo) & (vx < hi)
        if s.sum() < 3:
            continue
        med = float(np.median(dv[s]))
        acc_curve.append(((lo + hi) / 2, med, int(s.sum())))
        line = f"{lo:5.1f}~{hi:4.1f} {int(s.sum()):5d} {med:7.2f} {np.percentile(dv[s],90):7.2f}"
        if V:
            line += (f" | {np.median(prof['duty'][s]):6.2f} {np.median(prof['cur_motor'][s]):7.1f} "
                     f"{np.median(prof['volt'][s]):7.2f}")
        print(line)

    # ── ② 감속 권한: 명령 < 실측 (줄이라고 한 구간) ──────────────────────
    print("\n" + "═" * 74)
    print("② 감속 권한 — 명령이 실측보다 낮은 구간(=감속 요구)의 실측 dvx/dt")
    print("═" * 74)
    if drv:
        dec_m = (vx - cmd > 0.3) & (vx > 1.0)
        if dec_m.sum() >= 5:
            x = dv[dec_m]
            print(f"n={int(dec_m.sum())}  중앙 {np.median(x):+.2f}  p10 {np.percentile(x,10):+.2f}  "
                  f"최소 {x.min():+.2f} [m/s²]")
            print(f"→ 서비스 감속 권한 ≈ {abs(min(np.percentile(x,10),0)):.2f} m/s²")
            if V:
                print(f"   그 구간 듀티 중앙 {np.median(prof['duty'][dec_m]):+.2f}, "
                      f"모터전류 중앙 {np.median(prof['cur_motor'][dec_m]):+.1f}A "
                      f"(음수여야 회생제동)")
        else:
            print(f"감속 요구 구간 표본 {int(dec_m.sum())}개 — 테스트 B(스틱 급격히 놓기)를 안 했거나 너무 짧음")
    if brake:
        bt = np.array([x[0] for x in brake]); bv = np.array([m.data for _, m in brake])
        on = bv > 0.5
        if on.any():
            b0 = bt[on][0]
            w = (t >= b0) & (t <= b0 + 1.5) & (vx > 0.3)
            print(f"\n[참고] 브레이크 채널 작동 구간(t={b0:.2f}s~, {bv[on].max():.0f}A): "
                  f"실측 감속 {np.min(dv[w]) if w.any() else float('nan'):+.2f} m/s²")
            print("       ← 이 값이 크면 **하드웨어는 제동 가능**하고 명령 경로만 문제라는 뜻")

    # ── ③ 제약 판정 ──────────────────────────────────────────────────────
    print("\n" + "═" * 74)
    print("③ 가속을 무엇이 막고 있나")
    print("═" * 74)
    if not V:
        print("  /sensors/core가 없어 판정 불가. f1rec으로 재녹화할 것(이미 토픽 목록에 포함돼 있음).")
    else:
        hi = acc_m & (vx > max(2.0, vx.max() * 0.6))
        if hi.sum() < 5:
            print("  고속 가속 표본 부족 — 더 긴 직선에서 재측정")
        else:
            d, c, v_ = (float(np.median(prof[k][hi])) for k in ('duty', 'cur_motor', 'volt'))
            v_rest = float(np.median(prof['volt'][vx < 0.3])) if (vx < 0.3).any() else float('nan')
            print(f"  고속(vx>{max(2.0, vx.max()*0.6):.1f}) 가속 중: 듀티 {d:.2f}  모터전류 {c:.1f}A  전압 {v_:.2f}V"
                  f"  (정지 시 전압 {v_rest:.2f}V)")
            if abs(d) > 0.90:
                print("  ▶ 듀티 포화 → **전압 한계**. 배터리 셀수↑ 또는 기어비를 짧게(피니언↓).")
                print("    파라미터로는 못 고침. 이 상태에서 max_speed를 올려도 차는 안 빨라진다.")
            elif c > 55:
                print("  ▶ 전류 제한 도달 → vesc_mcconf의 Motor Current Max(현재 60A) 상향 검토.")
                print("    ⚠️ 모터·ESC 발열/정격 확인 후에만. 배터리 전류 제한도 같이 봐야 함.")
            elif not math.isnan(v_rest) and v_rest - v_ > 2.0:
                print(f"  ▶ 부하 시 전압 새그 {v_rest - v_:.1f}V → **배터리 내부저항/방전**. 셀 교체·완충 확인.")
            else:
                print("  ▶ 듀티·전류·전압 모두 여유 있음 → 전기 쪽이 아님.")
                print("    남은 후보: 기어/드라이브트레인 마찰, 타이어 구름저항, 센서리스 FOC 토크 손실.")

    # ── ④ 다음 단계 파라미터 ─────────────────────────────────────────────
    print("\n" + "═" * 74)
    print("④ 이 측정으로 정해지는 다음 단계 값")
    print("═" * 74)
    # 가속 권한은 **고속대의 최저값**을 쓴다 — 저속에서 잘 나와도 직선 끝까지 못 밀면 의미 없음
    hi_band = [med for center, med, _ in acc_curve if center >= 2.0]
    a_acc = min(hi_band) if hi_band else None
    a_dec = None
    if drv:
        dm = (vx - cmd > 0.3) & (vx > 1.0)
        if dm.sum() >= 5:
            a_dec = abs(min(float(np.percentile(dv[dm], 10)), 0.0))
    D = 13.6      # ifac_track_v2에서 v_cap>=6.0인 최장 직선
    v_c = 2.4
    print(f"  실측 가속(2 m/s 이상 구간 최저) = {a_acc if a_acc is None else round(a_acc,2)} m/s²")
    print(f"  실측 감속                      = {a_dec if a_dec is None else round(a_dec,2)} m/s²")
    if a_dec:
        print(f"\n  → prebrake_decel := {a_dec*0.8:.2f}   (실측 × 0.8)")
    if a_acc is not None and a_dec is not None:
        a_eff = min(a_acc, a_dec)
        v_peak = math.sqrt(v_c ** 2 + a_eff * D)
        print(f"  → 직선 {D}m에서 도달 가능한 피크 = √({v_c}² + {a_eff:.2f}×{D}) = {v_peak:.2f} m/s")
        print(f"  → max_speed := {math.floor(v_peak*10)/10:.1f}  (이보다 높게 줘도 차가 못 감)")
        need = (6.0 ** 2 - v_c ** 2) / D
        print(f"\n  목표 6.0 m/s에 필요한 가감속 = {need:.2f} m/s²  "
              f"→ 현재 대비 {need/max(a_eff,1e-6):.1f}배 필요")
    print()


if __name__ == "__main__":
    main()
