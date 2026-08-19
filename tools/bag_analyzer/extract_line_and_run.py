"""bag에서 글로벌 라인(κ·vx 포함)과 실주행 궤적을 뽑아 npz로 저장."""
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import os
SC = os.environ.get('SIM_DATA_DIR', '.') + '/'
BAG = sys.argv[1]   # <bag>/<bag>_0.db3

con = sqlite3.connect(BAG)
tm = {n: (i, t) for i, n, t in con.execute("select id,name,type from topics")}


def msgs(topic, limit=None):
    tid, typ = tm[topic]
    M = get_message(typ)
    q = f"select timestamp,data from messages where topic_id={tid} order by timestamp"
    if limit:
        q += f" limit {limit}"
    for ts, d in con.execute(q):
        yield ts * 1e-9, deserialize_message(bytes(d), M)


# 글로벌 라인
_, m = next(msgs('/global_waypoints', 1))
W = np.array([[w.x_m, w.y_m, w.s_m, w.kappa_radpm, w.vx_mps, w.psi_rad] for w in m.wpnts])

# 실주행 포즈 + 속도
P = []
for t, m in msgs('/pf/pose/odom'):
    q = m.pose.pose.orientation
    P.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
              np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
              m.twist.twist.linear.x))
P = np.array(P)

# 발행 조향/속도 명령
D = np.array([(t, m.drive.steering_angle, m.drive.speed) for t, m in msgs('/drive_autonomous')])

# 자율 구간
DM = np.array([(t, 1.0 if m.data == 'autonomous' else 0.0) for t, m in msgs('/drive_mode')])

np.savez(SC + 'run.npz', W=W, P=P, D=D, DM=DM)
print("W", W.shape, "P", P.shape, "D", D.shape, "DM", DM.shape)
print("track L=%.2f m  |kappa|max=%.3f  vx[%.2f,%.2f]" %
      (W[-1, 2], np.abs(W[:, 3]).max(), W[:, 4].min(), W[:, 4].max()))
print("autonomous frac %.2f" % DM[:, 1].mean())
