"""자이로 그립 분석 v3 — 준정상상태 게이트 + reach 스윕.

v2의 두 결함
  A) δ_cmd 를 [κ, a_lat] 두 열로 회귀했는데 κ=ψ̇/v, a_lat=v·ψ̇ 라 **공선성**이 심해
     r 이 28.9, −1.83 같은 쓰레기가 나왔다. → r 은 회귀로 뽑지 말고 **스윕**한다.
     (②-k 지뢰 2번: "K_us 절대값은 못 믿는다, 추세만 쓸 것"이 정확히 이 얘기다)
  B) a_lat = v·ψ̇ 는 **정상상태에서만** 참이다. 과도구간에선 a_y = v·(ψ̇+β̇) 이라
     요레이트가 빠르게 쌓이는 순간 v·ψ̇ 가 실제 횡가속을 과대평가한다.
     → |dψ̇/dt| 게이트로 준정상상태만 남긴다.
"""
import sqlite3, glob, os, math, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

G = 9.80665
DEG2RAD = math.pi / 180.0
L = 0.33
LAG = 0.14
YAW_ACC_MAX = 3.0     # [rad/s²] 준정상상태 게이트

_DEFAULT = [
    ("신", "/home/tenmeneat/rosbag_log/0816/run_0816_225916"),
    ("신", "/home/tenmeneat/rosbag_log/0816/run_0816_230107"),
    ("신", "/home/tenmeneat/rosbag_log/0816/run_0816_230340"),
    ("구", "/home/tenmeneat/rosbag_log/0814/run_0814_214351"),
    ("구", "/home/tenmeneat/rosbag_log/0814/run_0814_214021"),
    ("구", "/home/tenmeneat/rosbag_log/0813/run_0813_154525"),
]
# 인자로 bag 디렉터리를 주면 그것들만 본다:  analyze_grip_envelope.py ~/rosbag_log/0817/run_*
BAGS = [("신", os.path.expanduser(p)) for p in sys.argv[1:]] or _DEFAULT


def read_topic(cur, topics, typemap, name):
    mt = get_message(typemap[name])
    rows = cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                       (topics[name],)).fetchall()
    return [(ts, deserialize_message(d, mt)) for ts, d in rows]


def lpf(x, a):
    y = np.empty_like(x); acc = x[0]
    for i, v in enumerate(x):
        acc += a * (v - acc); y[i] = acc
    return y


def load(bagdir):
    dbs = glob.glob(bagdir + "/*.db3")
    if not dbs:
        return None
    con = sqlite3.connect(dbs[0]); cur = con.cursor()
    topics = dict(cur.execute("SELECT name,id FROM topics").fetchall())
    typemap = dict(cur.execute("SELECT name,type FROM topics").fetchall())
    if any(n not in topics for n in ("/sensors/imu/raw", "/pf/pose/odom", "/drive_autonomous")):
        con.close(); return None
    imu = read_topic(cur, topics, typemap, "/sensors/imu/raw")
    odom = read_topic(cur, topics, typemap, "/pf/pose/odom")
    drv = read_topic(cur, topics, typemap, "/drive_autonomous")
    con.close()

    t = np.array([x[0] for x in imu], float) / 1e9
    wz = lpf(np.array([m.angular_velocity.z * DEG2RAD for _, m in imu]), 0.25)
    ay = lpf(np.array([-m.linear_acceleration.y * G for _, m in imu]), 0.25)
    ot = np.array([x[0] for x in odom], float) / 1e9
    v = np.interp(t, ot, np.array([m.twist.twist.linear.x for _, m in odom]))
    dt_ = np.array([x[0] for x in drv], float) / 1e9
    ds = np.array([m.drive.steering_angle for _, m in drv])
    steer = np.interp(t - LAG, dt_, ds)
    wdot = np.gradient(wz, t)
    live = (t > dt_[0] + 1.0) & (t < dt_[-1] - 0.2)
    return dict(t=t, wz=wz, wdot=wdot, ay=ay, v=v, steer=steer, live=live)


def masks(d):
    """base=자율주행중 / trk=조향을 추종중 / ss=추종 + 준정상상태"""
    v, wz, st = d["v"], d["wz"], d["steer"]
    ref = v * st / (L + 0.014 * v * v)
    base = d["live"] & (v > 2.0)
    trk = base & (np.sign(ref) * np.sign(wz) > 0) & (np.abs(wz) < 1.6 * np.abs(ref) + 0.3)
    ss = trk & (np.abs(d["wdot"]) < YAW_ACC_MAX)
    return base, trk, ss


data = {}
for tag, b in BAGS:
    d = load(b)
    if d:
        data[os.path.basename(b)] = (tag, d)

print("=" * 86)
print("1) 그립 포락선 — 추종 + 준정상상태(|dψ̇/dt|<3)에서 실제 도달 a_lat")
print("   ★ 검증: 자이로 v·ψ̇ 와 가속도계 a_y 가 이 게이트 안에서 일치해야 한다")
print("=" * 86)
print(f"{'bag':>20} {'hw':>3} {'n':>5} {'남은%':>6} | {'p50':>5} {'p90':>5} {'p95':>5} "
      f"{'p99':>5} {'max':>5} {'>7':>6} | {'자이로vs가속 스케일':>18} {'r':>5}")
for name, (tag, d) in data.items():
    base, trk, ss = masks(d)
    if ss.sum() < 120:
        print(f"{name:>20} {tag:>3} {ss.sum():5d}  (부족)")
        continue
    a = d["v"][ss] * d["wz"][ss]
    aa = d["ay"][ss]
    sc = float(np.sum(a * aa) / np.sum(a * a))
    rr = float(np.corrcoef(a, aa)[0, 1])
    A = np.abs(a)
    print(f"{name:>20} {tag:>3} {ss.sum():5d} {ss.sum()/max(base.sum(),1)*100:5.1f}% | "
          f"{np.percentile(A,50):5.2f} {np.percentile(A,90):5.2f} {np.percentile(A,95):5.2f} "
          f"{np.percentile(A,99):5.2f} {A.max():5.2f} {(A>7).mean()*100:5.1f}% | "
          f"{sc:18.3f} {rr:5.3f}")

print()
print("=" * 86)
print("2) reach ratio 스윕 — r 마다 K_us(a_lat) 곡선이 어떻게 보이나")
print("   ★ 절대값은 r 가정에 비례해 움직이지만 **모양(상승/하강)은 불변**이어야 한다")
print("=" * 86)
for hw in ["신", "구"]:
    sets = [(n, d) for n, (t, d) in data.items() if t == hw]
    if not sets:
        continue
    A_, K_raw, V_ = [], [], []
    for n, d in sets:
        _, _, ss = masks(d)
        v, wz, st = d["v"][ss], d["wz"][ss], d["steer"][ss]
        a_lat = v * wz
        ok = np.abs(a_lat) > 2.0        # 작은 a_lat 으로 나누면 노이즈 폭발
        A_.append(np.abs(a_lat[ok]))
        K_raw.append((st[ok], wz[ok] / v[ok], a_lat[ok]))
        V_.append(v[ok])
    A = np.concatenate(A_); V = np.concatenate(V_)
    ST = np.concatenate([x[0] for x in K_raw])
    KA = np.concatenate([x[1] for x in K_raw])
    AL = np.concatenate([x[2] for x in K_raw])
    print(f"\n  [{hw} 프런트]  n={len(A)}")
    hdr = f"  {'a_lat':>9} {'n':>6} {'v':>5}"
    for r in (0.85, 1.0, 1.15):
        hdr += f" {'K_us@r=' + str(r):>12}"
    print(hdr)
    for lo, hi in [(2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 10), (10, 14)]:
        s = (A >= lo) & (A < hi)
        if s.sum() < 40:
            continue
        line = f"  {lo:>3}~{hi:<5} {s.sum():6d} {np.median(V[s]):5.2f}"
        for r in (0.85, 1.0, 1.15):
            k = (ST[s] * r - L * KA[s]) / AL[s]
            line += f" {np.median(k):12.4f}"
        print(line)

print()
print("=" * 86)
print("3) 요레이트 결손비 = |ψ̇_meas| / |ψ̇_ref| , ψ̇_ref = v·(r·δ)/(L+K_us v²)")
print("   ★ 이게 조향각과 함께 **떨어지는 지점**이 그립 한계다 (LUT 없이 비선형성 관측)")
print("=" * 86)
for r_try, kus_try in [(1.0, 0.014)]:
    print(f"\n  [r={r_try}, K_us={kus_try}]")
    print(f"  {'bag':>20} {'hw':>3} " + " ".join(f"{f'δ{lo:.2f}-{hi:.2f}':>11}"
          for lo, hi in [(0.05, 0.12), (0.12, 0.20), (0.20, 0.28), (0.28, 0.36), (0.36, 0.45)]))
    for name, (tag, d) in data.items():
        _, _, ss = masks(d)
        ad = np.abs(d["steer"])
        row = f"  {name:>20} {tag:>3} "
        for lo, hi in [(0.05, 0.12), (0.12, 0.20), (0.20, 0.28), (0.28, 0.36), (0.36, 0.45)]:
            m = ss & (ad >= lo) & (ad < hi)
            if m.sum() < 25:
                row += f" {'-':>11}"
                continue
            v, wz, st = d["v"][m], d["wz"][m], d["steer"][m] * r_try
            ref = np.abs(v * st / (L + kus_try * v * v))
            row += f" {np.median(np.abs(wz)/np.maximum(ref,1e-3)):11.3f}"
        print(row)

print()
print("=" * 86)
print("4) 슬립 잔차 v̇_y = a_y − v·ψ̇  (준정상 vs 과도 vs 스핀)")
print("=" * 86)
print(f"{'bag':>20} {'hw':>3} {'준정상 σ':>9} {'과도 σ':>8} {'스핀 σ':>8} {'스핀/준정상':>11}")
for name, (tag, d) in data.items():
    base, trk, ss = masks(d)
    res = d["ay"] - d["v"] * d["wz"]
    tr = trk & ~ss
    sp = base & ~trk
    if ss.sum() < 80 or sp.sum() < 25:
        continue
    a, b_, c = np.std(res[ss]), np.std(res[tr]) if tr.sum() > 25 else float("nan"), np.std(res[sp])
    print(f"{name:>20} {tag:>3} {a:9.2f} {b_:8.2f} {c:8.2f} {c/max(a,1e-6):10.2f}x")
