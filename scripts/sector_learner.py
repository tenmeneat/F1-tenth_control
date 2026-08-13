#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sector_learner.py — 섹터 MLA 스케일 온라인 학습기 (AIMD)
================================================================================
`/sector_scales`를 발행하는 **유일한** 노드다 (2026-08-12에 `sector_pub.py`를 흡수하고
은퇴시켰다 — 구 코드는 git 이력에 있다). 기본 모드 `static`은 YAML을 그대로 쏘기만 하고,
`guard`/`explore`에서는 **주행하면서 랩마다 게이트를 재고 scale을 스스로 올리고 내린다**.

    /pf/pose/odom, /sensors/imu/raw, /scan, /drive_autonomous, /state
            ↓
      sector_learner ──/sector_scales──→ control_map_node (수정 불필요)
            ↓
      학습된 sectors.yaml (종료 시 저장)

🔑 **컨트롤러를 안 건드린다.** 발행 토픽·레이아웃이 구 `sector_pub.py`와 동일하므로
   컨트롤러의 검증(scale ≥ 1.0 clamp / track_length 대조 / `/state` 회피 게이팅)이
   학습기 출력에도 **그대로** 걸린다. 학습기를 안 띄우면 정확히 기존 거동이다.

🔑 **AIMD — 올릴 땐 조금씩(+0.05, 연속 2랩 통과), 내릴 땐 왕창(−0.15, 즉시).**
   너무 높을 때의 비용은 벽이고 너무 낮을 때의 비용은 랩당 0.05 s다. 대칭 탐색은 틀렸다.

⚠️ **모드를 반드시 의식할 것.** "얼마나 자율적으로 둘 것인가"의 3단계다.
   `--mode static`       : scale을 **전혀 안 바꾼다.** 표를 읽어 1 Hz로 쏘기만 한다
                          (= 구 `sector_pub.py` 거동) + 랩마다 게이트를 재서 로그로만 알려준다.
                          "오프라인으로 검증한 표를 아무도 안 건드리게" 쓰고 싶을 때.
   `--mode guard` (기본) : 하향만 한다. **결선용.** 연습에서 수렴시킨 표를 싣고 들어가
                          위험이 감지될 때만 되돌린다 → 모든 고장이 느려지는 쪽.
   `--mode explore`      : 상향+하향. **연습 전용.** 차가 스스로 코너 속도를 올린다.

`--watch`를 주면 YAML 저장 시 표를 다시 읽는다(차 옆에서 라이브 튜닝).
⚠️ 재로드는 **학습 상태를 초기화**한다 — 경계가 바뀌면 K_us 기준선·잠금이 다른 구간의
   통계가 되어버리기 때문이다. 재로드 실패 시에는 직전 표를 그대로 유지한다(오타 한 번에
   주행 중 스케일이 통째로 날아가지 않게).

⚠️ scale < 1.0은 발행하지 않는다. 컨트롤러가 테이블을 통째로 버리기 때문이다
   (설계상 안전 동작이지만 "켰는데 왜 안 빨라지지"로 헤매게 된다). 하한은 항상 1.0.

게이트 근거는 `tools/bag_analyzer/analyze_sector_clearance.py`와 같다 — 0807 전복이
라이다 실측 여유 0.117 m에서 접촉했고 직전 랩들이 0.381 → 0.187로 깎이고 있었다.

사용법:
  source /opt/ros/<distro>/setup.bash && source ~/2026_IFAC/install/setup.bash
  python3 scripts/sector_learner.py config/sectors.yaml --mode explore   # 연습
  python3 scripts/sector_learner.py config/sectors.yaml                  # 결선(하향만)
  python3 scripts/sector_learner.py config/sectors.yaml --mode explore --out learned.yaml
"""
import sys, os, math, argparse, time, signal

import numpy as np
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

try:
    from f110_msgs.msg import WpntArray, StateMachine
except ImportError:  # 시뮬 등 f110_msgs 없는 환경
    WpntArray = StateMachine = None

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)

# ⚠️ 센서 스트림은 **BEST_EFFORT로 구독**해야 한다. 라이다·IMU·odom 발행자가
#    BEST_EFFORT면 RELIABLE 구독자는 QoS 비호환으로 **한 건도 못 받는다**(조용히 0 Hz가
#    되고 노드는 정상처럼 보인다). 반대로 발행자가 RELIABLE이어도 BEST_EFFORT 구독자는
#    정상 수신하므로 이쪽이 항상 안전한 선택이다. rosbag 재생에서 실제로 걸렸다.
SENSOR = QoSProfile(depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST)

# ── 차량·컨트롤러 상수 (CLAUDE.md / control_map_node와 동일 기준) ────────────
WHEELBASE       = 0.33
MAX_STEER       = 0.410     # 좌우 공통 실제 바퀴각 [rad]
STEERING_REACH  = 0.85      # steering_reach_ratio
CAR_HALF_WIDTH  = 0.155     # 차폭 0.31의 절반
LASER_DX        = 0.27      # base_link → laser (tf_static 실측)
DEG2RAD         = math.pi / 180.0   # ⚠️ VESC IMU는 deg/s로 발행(sensor_msgs 규약 위반)

# ── 게이트 임계 (analyze_sector_clearance.py와 동일 근거) ────────────────────
CLR_P5_MIN   = 0.20    # 벽 여유 p5 [m]
CLR_ABS_MIN  = 0.12    # 벽 여유 최소 [m]
ERR_P95_MAX  = 0.35    # 횡오차 p95 [m]
SAT_FRAC_MAX = 0.02    # 조향 포화 허용 비율

# K_us 게이트 — ⚠️ 랩별·섹터별 K_us 추정은 **매우 시끄럽다**(0811 실측: 같은 섹터에서
#   랩마다 0.0022~0.0096, 4배). 그래서 세 가지를 지킨다:
#   ① 기준선은 초기 KUS_BASE_LAPS 랩의 중앙값으로 **한 번 정하고 고정**한다.
#      (러닝 최소값으로 잡으면 기준선이 랩마다 내려가 게이트가 계속 엄격해지고
#       결국 모든 섹터를 거부한다 — 0811 통합시험에서 실제로 이렇게 됐다)
#   ② 판정은 단일 랩이 아니라 **최근 KUS_WIN 랩의 중앙값**으로 한다
#   ③ 허용폭은 30% — 절대값이 아니라 추세만 본다. 절대값은 steering_reach_ratio
#      가정에 비례해 0.0037~0.0301까지 흔들려서 못 믿는다(0811 reach 스윕).
KUS_BASE_LAPS = 3
KUS_WIN       = 3
KUS_RISE_TOL  = 0.30

# ── AIMD ─────────────────────────────────────────────────────────────────────
STEP_UP      = 0.05
STEP_DOWN    = 0.15
CONFIRM_LAPS = 2
SCALE_MIN    = 1.0
SCALE_MAX    = 1.5

# ── 신호 처리 ────────────────────────────────────────────────────────────────
STEER_LAG    = 0.14    # 조향→요레이트 지연 [s]. 0811 bag 스윕에서 R² 최대(0.887).
                       # 미반영 시 자전거모델 회귀의 L이 0.33 대신 0.16으로 나온다.
MIN_SPEED    = 1.5     # 이 속도 미만 샘플은 게이트에서 제외
SIDE_BEAM_HALF = 15.0 * DEG2RAD   # 차 옆 ±90°에서 이 폭의 빔만 벽으로 본다
SIDE_X_MAX   = 0.60    # |x| 이 범위 안의 빔만 (앞뒤 벽 혼입 차단)
MIN_SAMPLES  = 20      # 섹터·랩당 최소 샘플 수


# ══ 라인 ════════════════════════════════════════════════════════════════════
class Raceline:
    """글로벌 라인의 (s, d) 투영. 창 탐색이라 O(1)에 가깝다."""

    def __init__(self, wpnts, dense_step=0.05):
        xy = np.array([[w.x_m, w.y_m] for w in wpnts], dtype=float)
        seg = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
        s_wp = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
        self.length = float(seg.sum())
        # 경계 자동 재생성(autobuild_sectors)이 쓰는 웨이포인트 단위 κ·s.
        # 촘촘한 self.s(0.05 m 재샘플)와 다르다 — κ는 웨이포인트에만 실려 온다.
        self.s_wp = s_wp
        self.kappa = np.abs(np.array([w.kappa_radpm for w in wpnts], dtype=float))
        n = max(2, int(self.length / dense_step))
        s_d = np.linspace(0.0, self.length, n, endpoint=False)
        closed_xy = np.vstack([xy, xy[:1]])
        closed_s = np.concatenate([s_wp, [self.length]])
        self.s = s_d
        self.xy = np.column_stack([np.interp(s_d, closed_s, closed_xy[:, 0]),
                                   np.interp(s_d, closed_s, closed_xy[:, 1])])
        tang = np.gradient(self.xy, axis=0)
        self.yaw = np.arctan2(tang[:, 1], tang[:, 0])
        self.step = self.length / n
        self._last = 0

    def project(self, x, y, window_m=4.0):
        """(s, d). d는 진행방향 기준 왼쪽이 양수."""
        n = len(self.s)
        w = max(4, int(window_m / self.step))
        idx = (self._last + np.arange(-w, w + 1)) % n
        dx = x - self.xy[idx, 0]
        dy = y - self.xy[idx, 1]
        k = int(np.argmin(dx * dx + dy * dy))
        # 창 밖으로 벗어났으면(초기·pose 점프) 전역 재탐색
        if k in (0, len(idx) - 1):
            adx = x - self.xy[:, 0]
            ady = y - self.xy[:, 1]
            j = int(np.argmin(adx * adx + ady * ady))
        else:
            j = int(idx[k])
        self._last = j
        yaw = self.yaw[j]
        d = -math.sin(yaw) * (x - self.xy[j, 0]) + math.cos(yaw) * (y - self.xy[j, 1])
        return float(self.s[j]), float(d)


# ══ 섹터 ════════════════════════════════════════════════════════════════════
class Sector:
    __slots__ = ("i", "s0", "s1", "scale", "streak", "locked",
                 "kus_base", "kus_laps", "acc", "note")

    def __init__(self, i, s0, s1, scale, note=""):
        self.i, self.s0, self.s1 = i, s0, s1
        self.scale = scale
        self.streak = 0
        self.locked = False
        self.kus_base = None      # 초기 랩들로 한 번 정하고 고정하는 기준선
        self.kus_laps = []        # 랩별 고하중 K_us 중앙값 이력
        self.note = note
        self.reset()

    def reset(self):
        self.acc = {"err": [], "clr": [], "kus": [], "sat": 0, "n": 0}

    def contains(self, s):
        if self.s0 <= self.s1:
            return self.s0 <= s <= self.s1
        return s >= self.s0 or s <= self.s1   # 랩을 넘는 구간


def autobuild_sectors(line, corner_kappa=0.30, min_len=0.6, snap=1.5):
    """라인의 κ만으로 코너 섹터를 만든다 — `analyze_sector_clearance.py`와 **같은 규칙**.

    🔴 2026-08-13 신설. 예전엔 라인을 재생성하면 학습기가 발행을 중단했고, 그때마다
    bag 녹화 → 오프라인 분석 → YAML 수동 편집 → 재기동을 돌아야 했다. 라인이 바뀌는 건
    흔한 일이라(0813 하루에만 33.62 → 33.87) 그 왕복이 자동화의 실질적 장벽이었다.
    학습기는 이미 `/global_waypoints`로 κ를 받고 있으므로 그 자리에서 만들 수 있다.

    🔑 **안전은 scale 1.0에서 온다.** 새로 만든 섹터는 전부 1.0으로 시작하므로 전역 MLA와
       동일 = "표가 없는 것과 같다". 옛 표를 새 라인에 **재해석하지 않고 폐기**하는 것도
       그대로다 — 기존 track_length 검증이 막으려던 위험("같은 s가 다른 코너를 가리킨다")은
       조금도 완화되지 않는다.

    경계는 ±snap 안의 |κ| 최소점으로 옮긴다. 거기는 그립 캡이 비활성인 지점이라 MCL의 s
    지터가 경계를 넘나들어도 속도 명령에 도달하지 못한다(필터링보다 경계 배치가 근본 대책).
    ②-k 지뢰 4번 — 경계 품질이 학습 수익을 지배한다 — 이 오프라인과 같은 코드로 지켜진다.

    직선은 내보내지 않는다(그립 캡이 비활성이라 scale을 걸어도 이득 0).
    """
    k = line.kappa
    n = len(k)
    if n < 4:
        return []
    is_corner = k > corner_kappa
    # 가장 완만한 점에서 원을 끊는다 → 섹터가 직선에서 시작한다
    start = int(np.argmin(k))
    order = [(start + j) % n for j in range(n)]

    raw, cur, begin = [], is_corner[order[0]], 0
    for j in range(1, n + 1):
        nxt = is_corner[order[j % n]] if j < n else None
        if j == n or nxt != cur:
            raw.append((cur, np.array([order[q] for q in range(begin, j)])))
            begin, cur = j, nxt

    seg_len = np.r_[np.diff(line.s_wp), line.length - line.s_wp[-1]]
    # 너무 짧은 조각은 앞 섹터에 흡수 — 안 그러면 노이즈 한 점이 섹터를 쪼갠다
    merged = []
    for corner, idx in raw:
        if merged and seg_len[idx].sum() < min_len:
            merged[-1] = (merged[-1][0], np.r_[merged[-1][1], idx])
        else:
            merged.append((corner, idx))
    if len(merged) > 1 and seg_len[merged[0][1]].sum() < min_len:
        merged[-1] = (merged[-1][0], np.r_[merged[-1][1], merged[0][1]])
        merged.pop(0)

    def snap_to_flat(s):
        off = np.abs((line.s_wp - s + line.length * 0.5) % line.length - line.length * 0.5)
        cand = np.flatnonzero(off <= snap)
        if cand.size == 0:
            return float(s)
        return float(line.s_wp[cand[int(np.argmin(k[cand]))]])

    out, i = [], 0
    for corner, idx in merged:
        if not corner:
            continue
        s0 = snap_to_flat(float(line.s_wp[idx[0]]))
        s1 = snap_to_flat(float(line.s_wp[idx[-1]]))
        note = f"자동생성 κp90 {np.percentile(k[idx], 90):.3f}"
        # 🔴 랩을 넘는 구간(s0 > s1)은 두 줄로 쪼갠다 — 컨트롤러도 load_sectors도
        #    s_start < s_end만 받는다. 컨트롤러가 값이 실제로 바뀌는 전이점에만 블렌딩을
        #    걸므로 쪼개도 결승선에서 파이지 않는다(2026-08-11 런타임 검증).
        spans = ([(s0, s1, "")] if s0 <= s1
                 else [(s0, line.length, " 랩넘김1/2"), (0.0, s1, " 랩넘김2/2")])
        for a, b, tag in spans:
            if b - a < 1e-3:          # 스냅이 두 경계를 같은 점으로 모은 경우
                continue
            out.append(Sector(i, a, b, 1.0, note + tag))
            i += 1
    return out


def resolve_out_dir(yaml_path):
    """자동 저장 위치 = **패키지 소스 폴더의 `learned/`** (2026-08-13).

    `--symlink-install`로 빌드하면 share의 `config/sectors.yaml`이 소스를 가리키는 심볼릭
    링크라, realpath로 소스 트리(`.../src/f1tenth_control/`)를 되찾을 수 있다. 복사본
    빌드거나 사용자가 임의 경로의 YAML을 넘긴 경우엔 홈으로 물러선다.

    🔑 dev repo(`~/F1tenth_control`)가 있으면 **그쪽을 먼저 쓴다.** 거기가 `f1up`이 개인
    저장소로 올리는 정본이라, 학습 결과가 자동으로 버전 관리·백업된다. 젯슨엔 dev repo가
    없으므로(워크스페이스만 있다) 자연히 패키지 소스로 떨어진다 — 그건 `f1learn`이 랩탑으로
    회수한다.

    🔴 하위폴더 `learned/`인 이유: `f1up`·`f1down`·`jetsonup`이 이 패키지를 **`rsync
    --delete`로 통째 덮어쓴다.** 패키지 루트에 저장하면 다음 배포에서 지워진다 — 그래서 세
    함수의 exclude 목록에 `learned/`를 같이 넣어 뒀다. 넷은 한 묶음이다.
    """
    dev = os.path.expanduser("~/F1tenth_control")
    if os.path.isfile(os.path.join(dev, "package.xml")):
        return os.path.join(dev, "learned")
    try:
        root = os.path.dirname(os.path.dirname(os.path.realpath(yaml_path)))
        if os.path.isfile(os.path.join(root, "package.xml")):
            return os.path.join(root, "learned")
    except OSError:
        pass
    return os.path.expanduser("~")


def resolve_yaml(raw_path):
    """구 sector_pub.py와 **같은 규칙**으로 sectors.yaml을 찾는다. 런치가 빈 문자열을
    넘길 수 있으므로(sector_scale_file 기본값이 '') 여기서 자동 탐색해야 한다."""
    if raw_path:
        p = os.path.expanduser(raw_path)
        if os.path.isfile(p):
            return p
    candidates = [
        os.path.expanduser("~/2026_IFAC/src/f1tenth_control/config/sectors.yaml"),
        os.path.expanduser("~/F1tenth_control/config/sectors.yaml"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("f1tenth_control"), "config", "sectors.yaml"))
    except Exception:
        pass
    for c in candidates:
        if os.path.isfile(c):
            return c
    if raw_path:
        return os.path.expanduser(raw_path)
    return candidates[0]


def load_sectors(path, scale_max=SCALE_MAX, warn=print):
    """YAML → (track_length, blend_len, [Sector]).

    🔑 컨트롤러가 거는 검증을 **여기서 먼저** 똑같이 건다. 컨트롤러는 이상하면 조용히
       테이블 전체를 버리고 1.0으로 돌아가는데(설계상 안전한 동작), 그걸 "켰는데 왜 안
       빨라지지"로 헤매기 쉽다. 발행 전에 여기서 잡아 이유를 찍는다.
       (2026-08-12 sector_pub.py 은퇴로 그쪽 검증을 통째로 이관받았다)
    """
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}

    track_len = doc.get("track_length")
    if not isinstance(track_len, (int, float)) or track_len <= 1.0:
        raise ValueError(f"track_length가 없거나 비정상: {track_len!r}. "
                         "이 값이 컨트롤러의 라인 동일성 검증 기준이라 반드시 있어야 한다")
    track_len = float(track_len)

    raw = doc.get("sectors") or []
    if not raw:
        raise ValueError("sectors가 비어 있다")

    out = []
    for i, sec in enumerate(raw):
        try:
            s0, s1, sc = float(sec["s_start"]), float(sec["s_end"]), float(sec["scale"])
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"{i}번 항목에 s_start/s_end/scale이 없거나 숫자가 아님: {sec!r}") from e
        if s1 <= s0:
            raise ValueError(
                f"{i}번 항목 s_start {s0:.2f} >= s_end {s1:.2f}. "
                "랩을 넘는 구간이면 두 개로 쪼갤 것 "
                f"(예: {{s_start: {s0:.2f}, s_end: {track_len:.2f}}} 와 {{s_start: 0.0, s_end: {s1:.2f}}})")
        if s0 < 0.0 or s1 > track_len + 1e-6:
            raise ValueError(f"{i}번 항목 s 구간 [{s0:.2f}, {s1:.2f}]이 랩 길이 {track_len:.2f} 밖")
        if sc < SCALE_MIN:
            raise ValueError(
                f"{i}번 항목 scale {sc:.2f} < 1.0 — 설계상 금지다. 전역 max_lateral_accel을 "
                "보수값으로 두고 여기서 여는 구조라, 1.0 미만을 쓰면 테이블을 잃었을 때 "
                "위험 코너가 빨라지는 방향이 된다. 느리게 하려면 전역 MLA를 낮출 것")
        if sc > scale_max:
            raise ValueError(f"{i}번 항목 scale {sc:.2f} > 허용 최대 {scale_max:.2f} "
                             "(컨트롤러 sector_scale_max와 맞출 것)")
        out.append(Sector(i, s0, s1, sc, str(sec.get("note", ""))))

    # 겹침 검사 — 컨트롤러는 겹치면 큰 쪽을 쓰지만, 의도한 적이 없을 테니 여기서 막는다.
    for a in range(len(out)):
        for b in range(a + 1, len(out)):
            if out[a].s0 < out[b].s1 and out[b].s0 < out[a].s1:
                raise ValueError(f"{a}번과 {b}번 구간이 겹친다: "
                                 f"[{out[a].s0:.2f},{out[a].s1:.2f}] vs [{out[b].s0:.2f},{out[b].s1:.2f}]")

    # 전이점 간격 < 블렌딩 폭이면 램프가 서로 겹쳐 의도한 값에 못 닿는다.
    blend = float(doc.get("blend_len", 0.5))
    edges = sorted([s for sec in out for s in (sec.s0, sec.s1)])
    tight = [(edges[i], edges[i + 1]) for i in range(len(edges) - 1)
             if 1e-9 < edges[i + 1] - edges[i] < blend]
    if tight and warn:
        warn(f"[WARN] 전이점 간격이 좁은 곳이 있다 {tight} — 블렌딩 폭({blend} m)보다 좁으면 "
             "램프가 겹쳐 그 구간이 목표 scale에 못 닿는다. 경계를 벌리거나 "
             "sector_scale_blend를 줄일 것")
    return track_len, blend, out


# ══ 노드 ════════════════════════════════════════════════════════════════════
class SectorLearner(Node):

    def __init__(self, args):
        super().__init__("sector_learner")
        self.args = args
        self.yaml_path = resolve_yaml(args.yaml)
        self.track_len, self.blend, self.sectors = load_sectors(self.yaml_path)
        self.mode = args.mode
        self.explore = (args.mode == "explore")
        self.static = (args.mode == "static")
        self.auto_sectors = args.auto_sectors
        # 🔴 2026-08-13: --out 미지정이면 자동 경로로 저장한다. 예전엔 그냥 안 저장했고,
        #    인자를 깜빡한 주행의 학습 결과가 통째로 사라졌다. 자동 재생성과 짝이다 —
        #    라인이 바뀌어도 주행만 하면 그 라인용 유효한 표가 항상 파일로 남는다.
        self.out_path = os.path.expanduser(args.out) if args.out else (
            os.path.join(resolve_out_dir(self.yaml_path),
                         f"learned_sectors_{time.strftime('%m%d_%H%M%S')}.yaml")
            if args.auto_save else "")
        try:
            self.yaml_mtime = os.path.getmtime(self.yaml_path)
        except OSError:
            self.yaml_mtime = 0.0

        self.line = None
        self.lap = 0
        self.prev_s = None
        self.laps_done = 0

        # 최신 상태
        self.v = 0.0
        self.wz = 0.0
        self.pose = None          # (x, y)
        self.s = None
        self.d = 0.0
        self.cur_sec = None
        self.engaged = False
        self.on_global = True
        self.state_seen = False
        self.state_t = 0.0
        self.steer_buf = []       # (t, delta) — 지연 보상용 링버퍼

        self.pub = self.create_publisher(Float32MultiArray, args.topic, LATCHED)

        self.create_subscription(Odometry, args.odom, self.cb_odom, SENSOR)
        self.create_subscription(Imu, args.imu, self.cb_imu, SENSOR)
        self.create_subscription(LaserScan, args.scan, self.cb_scan, SENSOR)
        self.create_subscription(AckermannDriveStamped, args.drive, self.cb_drive, SENSOR)
        self.create_subscription(String, "/drive_mode", self.cb_mode, SENSOR)
        if WpntArray is not None:
            self.create_subscription(WpntArray, "/global_waypoints", self.cb_wpnts, LATCHED)
        if StateMachine is not None:
            self.create_subscription(StateMachine, "/state", self.cb_state, SENSOR)

        # 데드맨 하트비트 — 컨트롤러의 `sector_scale_timeout`(기본 3.0s)이 이걸로
        # "발행자가 아직 살아있다"를 판정한다. latch는 늦게 뜬 구독자를 위한 것이지
        # 발행자 생존을 말해주지 않는다 — 둘은 다르다.
        self.create_timer(1.0, self.publish_table)

        self.watch = str(args.watch).lower() == "true"
        if self.watch:
            self.create_timer(0.5, self.check_reload)

        mode_desc = {"static": "정적(scale 고정, 게이트는 로그만)",
                     "guard": "보호(하향만)",
                     "explore": "탐색(상향+하향)"}[self.mode]
        self.get_logger().info(
            f"🟢 sector_learner 시작 — 모드 {mode_desc} | "
            f"섹터 {len(self.sectors)}개 | 랩길이 {self.track_len:.3f} m | 발행 {args.topic}\n"
            f"   표: {self.yaml_path}" + ("  (--watch: 저장 시 재로드)" if str(args.watch).lower() == "true" else "")
            + (f"\n   저장: {self.out_path}" + ("" if args.out else "  (자동 경로 — --out 으로 지정 가능)")
               if self.out_path else "\n   ⚠️ 저장 안 함(--no-auto-save)")
            + (f"\n   라인이 바뀌면 섹터 경계를 κ>{args.corner_kappa}로 자동 재생성한다 (scale 1.0 시작)"
               if self.auto_sectors else "\n   ⚠️ 자동 재생성 꺼짐(--no-auto-sectors) — 라인이 바뀌면 발행을 멈춘다"))
        if self.explore:
            self.get_logger().warn(
                "⚠️ 탐색 모드다 — 차가 스스로 코너 속도를 올린다. 연습 주행에서만 쓸 것. "
                "E-stop에 손을 올려 둘 것.")

    # ── 콜백 ────────────────────────────────────────────────────────────────
    def cb_wpnts(self, msg):
        if self.line is not None:
            return
        self.line = Raceline(msg.wpnts)
        err = abs(self.line.length - self.track_len)
        if err <= 0.02:
            self.get_logger().info(f"글로벌 라인 수신: {self.line.length:.3f} m")
            return

        # ── 라인이 바뀌었다 ────────────────────────────────────────────────
        # 옛 표는 **어느 경우에도 재사용하지 않는다** — 같은 s가 다른 코너를 가리키고,
        # scale ≥ 1.0이라 그 오적용은 "빨라지는" 쪽이다.
        if not self.auto_sectors:
            self.get_logger().error(
                f"🔴 랩길이 불일치: 테이블 {self.track_len:.3f} vs 실제 경로 {self.line.length:.3f} m "
                f"(오차 {err:.3f}). 라인이 바뀌었다 — 같은 s가 다른 코너를 가리킨다. "
                f"섹터 표를 새 bag으로 다시 뽑을 것. 학습기는 발행을 중단한다. "
                f"(--no-auto-sectors 로 자동 재생성을 껐다)")
            self.line = None
            self.sectors = []
            return

        secs = autobuild_sectors(self.line, self.args.corner_kappa, self.args.min_sector_len)
        if not secs:
            self.get_logger().error(
                f"🔴 랩길이 불일치({self.track_len:.3f} vs {self.line.length:.3f} m)인데 "
                f"κ > {self.args.corner_kappa}인 코너를 하나도 못 찾았다 — 발행 중단. "
                f"--corner-kappa 를 낮춰 볼 것.")
            self.line = None
            self.sectors = []
            return

        self.track_len = self.line.length
        self.sectors = secs                # 새 Sector 객체 = 학습 상태도 자동 초기화
        self.publish_table()
        self.get_logger().warn(
            f"🔄 라인이 바뀌었다({err:.3f} m 차이) → 섹터 경계를 κ로 자동 재생성했다. "
            f"코너 {len(secs)}개 / 랩길이 {self.track_len:.3f} m / **scale 전부 1.0**"
            f"(= 전역 MLA와 동일, 표가 없는 것과 같다).\n"
            + "\n".join(f"      s {s.s0:6.2f} ~ {s.s1:6.2f}   {s.note}" for s in secs)
            + f"\n   ⚠️ 옛 표({self.yaml_path})의 scale은 버렸다 — 다른 라인 기준이라 재사용 불가.")

    def cb_mode(self, msg):
        self.engaged = (msg.data == "autonomous")

    def cb_state(self, msg):
        self.on_global = (msg.state == StateMachine.STATE_GLOBAL)
        self.state_seen = True
        self.state_t = time.monotonic()

    def cb_imu(self, msg):
        # ⚠️ VESC는 deg/s로 발행한다 — 빠뜨리면 요레이트가 57.3배가 된다.
        self.wz = msg.angular_velocity.z * DEG2RAD

    def cb_drive(self, msg):
        t = time.monotonic()
        self.steer_buf.append((t, msg.drive.steering_angle))
        cutoff = t - 1.0
        while self.steer_buf and self.steer_buf[0][0] < cutoff:
            self.steer_buf.pop(0)

    def steer_delayed(self):
        """STEER_LAG 전의 조향 명령 — 지금의 요레이트를 만든 그 명령."""
        if not self.steer_buf:
            return None
        target = time.monotonic() - STEER_LAG
        best = min(self.steer_buf, key=lambda p: abs(p[0] - target))
        return best[1] if abs(best[0] - target) < 0.10 else None

    def collecting(self):
        """게이트 표본을 모아도 되는 상태인가."""
        if self.line is None or not self.sectors:
            return False
        if not self.engaged or self.v < MIN_SPEED:
            return False
        # 회피 중에는 안 모은다 — scale의 근거인 벽 여유는 '차가 라인 위에 있을 때'의 값이다.
        if self.args.global_only:
            if not self.state_seen or (time.monotonic() - self.state_t) > 1.0:
                return False      # /state 미수신이면 모르니 끈다
            if not self.on_global:
                return False
        return True

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            return
        self.v = float(msg.twist.twist.linear.x)
        self.pose = (p.x, p.y)
        if self.line is None:
            return
        s, d = self.line.project(p.x, p.y)
        self.s, self.d = s, d

        # 랩 검출
        if self.prev_s is not None and s < self.prev_s - self.line.length * 0.5:
            self.on_lap_end()
        self.prev_s = s

        self.cur_sec = next((sec for sec in self.sectors if sec.contains(s)), None)
        if not self.collecting() or self.cur_sec is None:
            return

        a = self.cur_sec.acc
        a["n"] += 1
        a["err"].append(abs(d))

        delta = self.steer_delayed()
        if delta is not None:
            if abs(delta) > 0.95 * MAX_STEER:
                a["sat"] += 1
            # K_us = (δ_도달 − L·κ_실측) / a_lat   — 고하중 표본만 의미가 있다
            a_lat = self.v * self.wz
            if abs(a_lat) > 3.0 and abs(self.v) > 0.5:
                kap = self.wz / self.v
                kus = (abs(delta) * STEERING_REACH - WHEELBASE * abs(kap)) / abs(a_lat)
                a["kus"].append((abs(a_lat), kus))

    def cb_scan(self, msg):
        if not self.collecting() or self.cur_sec is None:
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        if r.size < 20:
            return
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        good = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, 6.0))
        if good.sum() < 20:
            return
        r, ang = r[good], ang[good]
        # 차 좌표계에서 옆(±90°)을 보는 빔만. 라인 투영이 필요 없어 O(빔)이다.
        side = np.abs(np.abs(ang) - math.pi / 2) < SIDE_BEAM_HALF
        if side.sum() < 3:
            return
        bx = LASER_DX + r[side] * np.cos(ang[side])
        by = r[side] * np.sin(ang[side])
        near = np.abs(bx) < SIDE_X_MAX
        if near.sum() < 3:
            return
        by = by[near]
        cands = []
        if (by > 0).any():
            cands.append(float(np.min(by[by > 0])))
        if (by < 0).any():
            cands.append(float(np.min(-by[by < 0])))
        if cands:
            self.cur_sec.acc["clr"].append(min(cands) - CAR_HALF_WIDTH)

    # ── 랩 종료: 게이트 판정 + AIMD ─────────────────────────────────────────
    def on_lap_end(self):
        self.laps_done += 1
        changed = False
        lines = []
        for sec in self.sectors:
            a = sec.acc
            if a["n"] < MIN_SAMPLES:
                sec.reset()
                continue
            err95 = float(np.percentile(a["err"], 95))
            clr = np.array(a["clr"], dtype=float)
            clr = clr[np.isfinite(clr)]
            cp5 = float(np.percentile(clr, 5)) if clr.size >= 5 else float("nan")
            cmn = float(np.min(clr)) if clr.size >= 5 else float("nan")
            satf = a["sat"] / max(1, a["n"])

            # K_us: 이 섹터 고하중 표본의 중앙값. 같은 섹터의 기준선과 비교하므로
            # steering_reach_ratio 가정의 상수 편향이 상쇄된다(절대값은 못 믿는다).
            kus_hi = float("nan")
            if len(a["kus"]) >= 10:
                arr = np.array(a["kus"])
                thr = np.percentile(arr[:, 0], 75)
                sel = arr[arr[:, 0] >= thr, 1]
                if sel.size >= 5:
                    kus_hi = float(np.median(sel))

            breach = []
            if np.isfinite(cp5) and cp5 < CLR_P5_MIN:
                breach.append(f"여유p5 {cp5:.3f}")
            if np.isfinite(cmn) and cmn < CLR_ABS_MIN:
                breach.append(f"여유min {cmn:.3f}")
            if err95 > ERR_P95_MAX:
                breach.append(f"횡오차 {err95:.3f}")
            if satf > SAT_FRAC_MAX:
                breach.append(f"조향포화 {100*satf:.1f}%")
            # K_us: 기준선은 초기 몇 랩으로 고정, 판정은 최근 몇 랩의 중앙값으로.
            # 단일 랩 값끼리 비교하면 노이즈만 잡는다(위 상수 주석 참고).
            if np.isfinite(kus_hi):
                sec.kus_laps.append(kus_hi)
                if sec.kus_base is None and len(sec.kus_laps) >= KUS_BASE_LAPS:
                    sec.kus_base = float(np.median(sec.kus_laps[:KUS_BASE_LAPS]))
            kus_now = (float(np.median(sec.kus_laps[-KUS_WIN:]))
                       if len(sec.kus_laps) >= KUS_WIN else float("nan"))
            if (np.isfinite(kus_now) and sec.kus_base is not None
                    and kus_now > sec.kus_base * (1.0 + KUS_RISE_TOL)):
                breach.append(f"Kus상승 {kus_now:.4f}>{sec.kus_base:.4f}")

            old = sec.scale
            if self.static:
                # 게이트는 재되 **조치는 안 한다** — 관측 전용. sector_pub과 같은 거동에
                # 랩별 진단만 얹은 것이라, 표를 손대지 않는다는 보장이 필요할 때 쓴다.
                verdict = "정적" + (f"(경고: {','.join(breach)})" if breach else "")
                breach = []
            elif breach:
                sec.scale = max(SCALE_MIN, sec.scale - STEP_DOWN)
                sec.streak = 0
                sec.locked = True
                verdict = "↓하향"
            elif not self.explore:
                verdict = "유지"
            elif sec.locked:
                verdict = "잠금"
            else:
                sec.streak += 1
                if sec.streak >= CONFIRM_LAPS:
                    sec.scale = min(SCALE_MAX, sec.scale + STEP_UP)
                    sec.streak = 0
                    verdict = "↑상향"
                else:
                    verdict = f"대기 {sec.streak}/{CONFIRM_LAPS}"
            if abs(sec.scale - old) > 1e-9:
                changed = True

            lines.append(
                f"  섹{sec.i} s{sec.s0:>5.1f}~{sec.s1:<5.1f} | 횡p95 {err95:.3f} | "
                f"여유p5 {cp5:.3f}/min {cmn:.3f} | Kus {kus_hi:.4f}"
                f"(최근{kus_now:.4f}/기준{sec.kus_base if sec.kus_base else float('nan'):.4f}) | "
                f"포화 {100*satf:.1f}% | "
                f"{verdict} scale {old:.2f}→{sec.scale:.2f}"
                + ("  ← " + ", ".join(breach) if breach else ""))
            sec.reset()

        if lines:
            self.get_logger().info(f"[랩 {self.laps_done} 종료]\n" + "\n".join(lines))
        if changed:
            self.publish_table()

    # ── 라이브 재로드 (--watch) ─────────────────────────────────────────────
    def check_reload(self):
        try:
            m = os.path.getmtime(self.yaml_path)
        except OSError:
            return
        if m == self.yaml_mtime:
            return
        self.yaml_mtime = m
        time.sleep(0.1)          # 에디터가 쓰기를 끝낼 시간
        try:
            track_len, blend, sectors = load_sectors(self.yaml_path)
        except (ValueError, yaml.YAMLError, OSError) as e:
            # ⚠️ 실패하면 직전 표를 그대로 둔다. 차 옆에서 오타 한 번에 주행 중 스케일이
            #    통째로 날아가지 않게 — 구 sector_pub.py에서 이어받은 원칙이다.
            self.get_logger().error(f"재로드 실패, 직전 표 유지: {e}")
            return
        if self.line is not None and abs(self.line.length - track_len) > 0.02:
            self.get_logger().error(
                f"재로드 거부 — 랩길이 {track_len:.3f}가 실제 경로 {self.line.length:.3f}와 다르다")
            return

        # 🔑 학습 상태를 초기화한다. 경계가 바뀌면 기존 K_us 기준선·잠금은 **다른 구간의
        #    통계**가 되어버린다. 경계가 그대로여도 사용자가 scale을 손댔다는 건 판단을
        #    덮어썼다는 뜻이라, 거기서부터 다시 배우는 게 맞다.
        self.track_len, self.blend, self.sectors = track_len, blend, sectors
        self.publish_table()
        self.get_logger().info(
            f"🔄 표 재로드 — 섹터 {len(sectors)}개 ("
            + ", ".join(f"×{s.scale:.2f}" for s in sectors)
            + "), 학습 상태 초기화")

    # ── 발행 ────────────────────────────────────────────────────────────────
    def publish_table(self):
        if not self.sectors:
            return
        flat = [float(self.track_len)]
        for sec in self.sectors:
            sc = min(SCALE_MAX, max(SCALE_MIN, sec.scale))
            flat += [float(sec.s0), float(sec.s1), float(sc)]
        m = Float32MultiArray()
        m.data = flat          # ⚠️ rclpy의 .data는 array.array('f') — 리스트 += 는 TypeError
        self.pub.publish(m)

    def save(self):
        path = self.out_path
        if not path or not self.sectors:
            return
        # ⚠️ 라인을 한 번도 못 받았으면 자동 저장은 건너뛴다. 그 표는 실제 경로와 대조된 적이
        #    없어서(랩길이 검증 미실행) 다음 세션에 그대로 실으면 조용히 폐기된다 — 파일만
        #    그럴듯하게 남는 게 더 나쁘다. --out 을 **명시**했으면 사용자 의도로 보고 저장한다.
        if self.line is None and not self.args.out:
            self.get_logger().warn(
                "자동 저장 건너뜀 — /global_waypoints를 못 받아 표가 실제 라인과 대조되지 않았다")
            return
        doc = {
            "track_length": round(self.track_len, 3),
            "blend_len": self.blend,
            "sectors": [
                {"s_start": round(s.s0, 2), "s_end": round(s.s1, 2),
                 "scale": round(min(SCALE_MAX, max(SCALE_MIN, s.scale)), 3),
                 "note": (s.note + (" [학습:잠금]" if s.locked else " [학습]")).strip()}
                for s in self.sectors],
        }
        header = (
            "# sector_learner.py 자동 생성 — 학습 결과\n"
            f"# 모드 {self.args.mode} / 완주 {self.laps_done}랩 / "
            f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            "# ⚠️ 결선에 쓰기 전 analyze_sector_clearance.py로 같은 bag을 재검증할 것.\n"
            "# ⚠️ 라인(글로벌 경로)을 재생성했으면 이 파일은 폐기하고 새 bag으로 다시 뽑는다.\n")
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(header)
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(f"학습 결과 저장: {path}")


def main():
    ap = argparse.ArgumentParser(description="섹터 MLA 스케일 온라인 학습기 (AIMD)")
    ap.add_argument("yaml", nargs="?", default="",
                    help="섹터 경계·초기 scale이 든 sectors.yaml (생략 시 자동 탐색)")
    ap.add_argument("--mode", choices=["static", "guard", "explore"], default="guard",
                    help="static=안 바꿈(구 sector_pub 거동, 게이트는 로그로만) / "
                         "guard=하향만(결선) / explore=상향+하향(연습). 기본 guard")
    # ⚠️ store_true가 아니라 값을 받는다 — 런치는 불리언을 문자열로 넘기므로
    #    (`--watch false`) store_true면 끌 방법이 없다. 맨손 CLI에서는 `--watch`만 써도 켜진다.
    ap.add_argument("--watch", nargs="?", const="true", default="false",
                    help="YAML 저장 시 표를 다시 읽는다(라이브 튜닝). "
                         "재로드는 학습 상태를 초기화하고, 실패 시 직전 표를 유지한다")
    ap.add_argument("--out", default="",
                    help="학습 결과를 저장할 YAML 경로 "
                         "(생략 시 <패키지 소스>/learned/learned_sectors_<시각>.yaml)")
    ap.add_argument("--no-auto-save", dest="auto_save", action="store_false",
                    help="--out 미지정 시의 자동 저장을 끈다")
    ap.set_defaults(auto_save=True)
    # ── 경계 자동 재생성 (2026-08-13) ──────────────────────────────────────
    ap.add_argument("--no-auto-sectors", dest="auto_sectors", action="store_false",
                    help="라인이 바뀌었을 때 섹터 경계를 κ로 자동 재생성하지 않고 발행을 멈춘다(구 거동)")
    ap.set_defaults(auto_sectors=True)
    ap.add_argument("--corner-kappa", type=float, default=0.30,
                    help="자동 재생성의 코너 판정 |κ| [rad/m] (analyze_sector_clearance.py와 동일)")
    ap.add_argument("--min-sector-len", type=float, default=0.6,
                    help="자동 재생성의 섹터 최소 길이 [m]")
    ap.add_argument("--dry-run", action="store_true",
                    help="YAML 검증만 하고 ROS를 안 띄운다 (주행 전 사전 점검)")
    ap.add_argument("--topic", default="/sector_scales")
    ap.add_argument("--odom", default="/pf/pose/odom")
    ap.add_argument("--imu", default="/sensors/imu/raw")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--drive", default="/drive_autonomous")
    ap.add_argument("--no-global-only", dest="global_only", action="store_false",
                    help="회피 중에도 표본을 모은다 (권장하지 않음)")
    ap.set_defaults(global_only=True)
    # ⚠️ 런치가 `--ros-args -r __node:=...`를 뒤에 붙인다. 그대로 parse_args를 하면
    #    "unrecognized arguments"로 죽는다 — rclpy 유틸로 걷어낸다.
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if args.dry_run:
        path = resolve_yaml(args.yaml)
        try:
            tl, blend, secs = load_sectors(path, warn=lambda m: print(m, file=sys.stderr))
        except (ValueError, yaml.YAMLError, OSError) as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            return 1
        print(f"{path}\n랩 길이 {tl:.2f} m / 블렌딩 {blend:.2f} m / 섹터 {len(secs)}개")
        for s in secs:
            print(f"  s {s.s0:>6.2f} ~ {s.s1:>6.2f}  ({s.s1 - s.s0:>5.2f} m)  ×{s.scale:.2f}"
                  + (f"   {s.note}" if s.note else ""))
        print("(--dry-run: 발행하지 않음)")
        return 0

    # ⚠️ SIGTERM에서도 학습 결과를 저장해야 한다. Ctrl-C(SIGINT)만 처리하면
    #    `ros2 launch` 종료나 `timeout`으로 죽을 때 한 세션치 학습이 통째로 날아간다.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    rclpy.init()
    node = None
    try:
        node = SectorLearner(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.save()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
