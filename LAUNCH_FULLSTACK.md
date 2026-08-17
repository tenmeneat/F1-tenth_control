# 실차 풀스택 실행 가이드 (ROS 2 Jazzy)

**최종 개정 2026-08-17** — `~/2026_IFAC` 실제 워크스페이스 상태 기준으로 재검증했다.
아래 코드블록은 전부 **터미널에 그대로 붙여넣으면 되는 형태**다(source·cd 포함).

| | 값 |
|---|---|
| 젯슨 | **`jetson`** (= `miru@10.1.1.1`, wifi `wlP1p1s0`), Ubuntu 24.04 / **ROS 2 Jazzy** |
| 본체 PC | `10.1.1.24` (wifi `wlo1`), **ROS 2 Jazzy** |
| 워크스페이스 | 젯슨 `~/f1tenth_ws`(하드웨어) + `~/2026_IFAC`(제어·플래닝·MCL) |
| 지도 이름 | **`map`** (slam_toolbox 기본 저장명 그대로) |
| `ROS_DOMAIN_ID` | **70** |

> 💡 IP를 직접 쓰지 말고 **ssh 별칭 `jetson`**을 쓸 것. `~/.ssh/config`에 wifi 단절 대응
> 설정(`ServerAliveInterval` 등)이 들어 있고, IP가 바뀌어도 거기 한 줄만 고치면 된다.
> (구 문서의 `10.1.1.3`은 폐기 — 현재 `10.1.1.1`이다.)

---

## 0. 반드시 알고 있어야 할 함정

| # | 함정 | 대응 |
|---|---|---|
| 1 | **지도를 넣었으면 재빌드** — MCL은 `src/`가 아니라 설치된 share에서 읽고, 지도는 심볼릭 링크가 아닌 **실제 복사본** | [§2](#2-지도-넣었으면-재빌드-젯슨) |
| 2 | 지도 yaml의 `image:`가 `.pgm`으로 써질 때가 있다 | 저장 시 `--fmt png` ([§1](#1-지도-만들기--옮기기-새-트랙일-때만)) |
| 3 | `odom→base_link` TF 이중 발행 | `mcl_launch.py`의 `publish_odom_base_tf` 기본 false ([§6](#6-tf-책임-구조)) |
| 4 | **랩탑 ufw가 DDS 디스커버리를 막는다** — ping·ssh는 되는데 토픽만 0개 | [§7](#7-통신이-안-될-때) |
| 5 | **`global_planning`은 반드시 `~/2026_IFAC`에서 실행** — `output_base_dir`이 상대경로 | [T3](#t3-젯슨--global-planning) |

### 0-1. ✅ 지도 이름 불일치 함정은 해소됐다 (2026-08-17 확인)

구 문서는 "세 패키지가 지도 이름을 각각 다른 기본값으로 찾는다"고 크게 경고했는데,
**지금은 셋 다 기본값이 `map`이다.** 확인한 실제 코드:

| 패키지 | 어디서 읽나 | 기본값 |
|---|---|---|
| **MCL** (`particle_filter_cpp`) | `map_name:=` 인자 (**`F1_MAP`을 안 읽는다**) | `map` ✅ |
| **global_planning** | `F1_MAP` 환경변수 | `map` ✅ |
| **local_planning** | `F1_MAP` 환경변수 | `map` ✅ ← 구 `ifac_track`에서 수정됨 |

`state_machine`은 지도 이름을 안 쓴다.

> 🔴 **다만 젯슨 `~/.zshrc`의 `F1_MAP` 값은 여전히 확인해야 한다.** 환경변수가 `ifac_track`
> 같은 옛 값으로 남아 있으면 기본값이 아니라 그 값이 이긴다. 그리고 **이게 조용히 실패한다** —
> 노드는 뜨는데 경로가 안 나오거나 엉뚱한 지도로 돈다("스캔은 벽에 맞는데 경로만 어긋남", §8).

```bash
ssh jetson 'echo "F1_MAP=$F1_MAP"'          # 비어 있거나 map 이면 OK
# 옛 값이 박혀 있으면:
ssh jetson "sed -i 's/^export F1_MAP=.*/export F1_MAP=map/' ~/.zshrc"
# 이미 열어둔 터미널엔 반영 안 된다 → 새 터미널 또는 그 창에서: export F1_MAP=map
```

MCL은 어느 경우에도 `F1_MAP`을 안 읽으므로 **`map_name:=map`을 항상 명시**한다.

### 0-2. 현재 워크스페이스 패키지 (2026-08-17)

```
~/2026_IFAC/src/
├── f1tenth_control/          ← 이 저장소 (제어)
├── monte_carlo_localization/ ← 패키지명은 particle_filter_cpp
├── global_planning/          ← 글로벌 라인 발행 + frenet_odom_node
├── local_planning/           ← 회피 경로 생성
├── state_machine/            ← /state 판정 + /local_waypoints 선택 발행
├── obstacle_detector/        ← 라이다 장애물 검출 (구 opponent_detector)
├── static_obstacle_map/      ← 정적 장애물 맵
├── lap_timer/ , lap_referee/ ← 랩타임 계측 (주행 필수 아님)
└── cma_gt_localization/      ← 시뮬 GT 브리지 (CMA 튜닝 전용, 실차 미사용)
```

- ✅ 구 `wpnt_publisher` 유령 패키지는 **이미 정리돼 있다**(`build/`·`install/` 없음).
- ✅ `f110_msgs` / `steering_lookup`도 워크스페이스에 있다.

---

## 1. 지도 만들기 / 옮기기 (새 트랙일 때만)

### 매핑 (**랩탑**에서 실행. 젯슨은 f110 bringup만 떠 있으면 됨)
```bash
cd ~/slam_toolbox && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false \
  slam_params_file:=$HOME/F1tenth_control/tools/odom_diag/slam_toolbox_params.yaml
```

⚠️ **`slam_params_file`을 꼭 넘길 것.** 안 주면 패키지 기본값(`map_update_interval: 5.0`)이라
`/map`이 5초에 한 번만 나온다 — 짧게 돌리면 맵이 거의 안 그려져 "안 되는 것처럼" 보인다
(2026-07-31 실제로 헷갈렸던 지점). 우리 파일은 트랙에 맞춰 조정돼 있다:
`map_update_interval 1.0` / `minimum_travel_distance 0.3` / `minimum_travel_heading 0.2` /
`max_laser_range 10.0`.

ℹ️ 로그의 `Message Filter dropping message ... queue is full`은 **정상**이다(40 Hz 스캔 중
`minimum_time_interval: 0.5`라 2 Hz만 쓴다). 진짜로 TF가 없으면 `Registering sensor`가 안 뜬다.

매핑 요령:
- **천천히**(0.5~1 m/s). 2 m/s면 처리 스캔 간격이 1 m라 스캔매칭이 성겨진다
- 한 바퀴 이상 (10초로는 아무것도 안 나온다)
- **중간에 몇 초씩 정지** — odom 헤딩이 자이로 순수 적분이고 바이어스는 정지 중에만
  학습된다(τ=10 s). 계속 굴러다니면 10분에 ~19°까지 쌓인다
- 진행 확인: `ros2 topic hz /map` / `ros2 run tf2_ros tf2_echo map odom`

### 저장 (다른 터미널 — 🔴 **slam_toolbox를 끄기 전에**)
```bash
cd ~/slam_toolbox && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 run nav2_map_server map_saver_cli -f ~/slam_toolbox/map --fmt png \
  --ros-args -p save_map_timeout:=30.0 -p map_subscribe_transient_local:=true
```

🔴 **순서가 전부다: 저장 → 확인 → 그다음에 slam_toolbox에 Ctrl+C.**
`map_saver_cli`는 `/map`을 **구독해서** 받아 적으므로 발행자가 살아 있어야 한다. 먼저 끄면
30초 뒤 `Failed to spin map subscription`으로 죽고 **맵이 그대로 날아간다**
(2026-07-31에 매핑 주행을 통째로 다시 했다).

```bash
ls -l --time-style=+%H:%M:%S ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml
```

⚠️ **`-f`는 디렉터리가 아니라 "파일 이름 접두사"다.** `-f ~/slam_toolbox`로 주면 폴더 안이
아니라 홈에 `~/slam_toolbox.pgm`/`.yaml`이 생긴다(같은 이름 폴더가 있어 헷갈린다).

### 글로벌 패스 생성 (**랩탑**에서. 젯슨은 CPU가 좁다)

**① 새 지도를 로컬 워크스페이스에**
```bash
cp ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml \
   ~/2026_IFAC/src/monte_carlo_localization/maps/
```

**② GUI로 라인 확인하며 생성**
```bash
cd ~/2026_IFAC
python3 offline_trajectory_generator/trajectory_gui.py \
  --map-yaml src/monte_carlo_localization/maps/map.yaml
```
- 파라미터는 `offline_trajectory_generator/gui_params.yaml`에 **자동 저장**
- `Save` → `output/map/`에 `global_waypoints.json`/`.csv`, `centerline.csv`, `metadata.json`

**③ 헤드리스로 생성** (재현·스윕용)
```bash
cd ~/2026_IFAC
python3 offline_trajectory_generator/generate_global_trajectory.py \
  --map-yaml src/monte_carlo_localization/maps/map.yaml \
  --output-dir offline_trajectory_generator/output/map \
  --optimizer mincurv --width-mode hybrid \
  --waypoint-step 0.25 --optimizer-step 0.46 \
  --safety-width 0.5 --boundary-margin 0.5 --max-width-distance 2.0 \
  --max-speed 7.0 --min-speed 4.0 \
  --max-lateral-accel 7.0 --max-accel 3.5 --max-decel 2.6 \
  --max-curvature 1.18 \
  --smooth-sigma 3.7 --raceline-smooth-sigma 1.0
```

#### ⚠️ 생성기 인자를 **차량 실측 한계**에 맞출 것 (2026-08-17 갱신)

생성기가 차보다 낙관적이면 **컨트롤러가 못 따라가는 프로파일**이 나오고, 곡률 사전감속이
매 코너에서 그걸 깎느라 싸운다.

| 생성기 인자 | 차량/컨트롤러 실제 한계 | 권장 | 근거 |
|---|---|---|---|
| `--max-curvature` | **1.317** (조향 0.410 rad, R 0.759 m) | **1.18** (R 0.85 m, 여유 15%) | CLAUDE.md ②-d |
| `--max-lateral-accel` | 컨트롤러 캡 **7.0** | **7.0** | 🔴 구 문서 6.0은 stale. 08-12 정정 |
| `--max-accel` | VESC 램프 `s_pid_ramp_erpms_s` 21160 = **4.88** | **3.5** | 🔴 구 문서 15600=3.69는 stale (08-05 상향) |
| `--max-decel` | 브레이크 하드웨어 4.8 / 현 튜닝 **2.6** | **2.6** | `prebrake_decel`과 맞춤 |
| `--max-speed` | 컨트롤러 `max_speed` **8.0**, 실제 도달 7.4 | 7.0 | 직선 13.3 m라 8.0엔 못 닿음 |

> 🔑 **`--max-lateral-accel`은 컨트롤러 `max_lateral_accel`과 반드시 같게.** 어긋나면 한쪽이
> 다른 쪽을 `min()`으로 덮으면서 이득이 0이 된다(CLAUDE.md 세미슬릭 절).

🔴 **`--max-curvature`가 실제로 지켜졌는지 반드시 확인할 것.** 구 `ifac_track_v2`는 이 값이
1.2인데도 결과 κ가 **1.485**까지 나와 187점 중 2점이 차량 최소 선회반경보다 급했다
(= 어떤 제어 튜닝으로도 못 도는 코너. 07-25 시케인·07-26 헤어핀 크래시의 근본 원인).

```bash
cd ~/2026_IFAC && python3 -c "
import csv
r=[x for x in list(csv.reader(open('offline_trajectory_generator/output/map/global_waypoints.csv')))[1:] if x]
k=[abs(float(x[5])) for x in r]
print('최대 kappa %.3f -> R_min %.3f m  (차량 한계 R 0.759 m = kappa 1.317)' % (max(k), 1/max(k)))
print('차량 한계 초과 점: %d / %d' % (sum(1 for v in k if v > 1.317), len(k)))
"
```
초과 점이 0이 아니면 `--max-curvature`를 낮추거나 `--safety-width`를 키워 다시 뽑는다.

### 본체 → 젯슨 전송
```bash
# 지도 (MCL용) — 재빌드 필요
scp ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml \
    jetson:~/2026_IFAC/src/monte_carlo_localization/maps/

# 글로벌 패스 — 재빌드 불필요
ssh jetson 'mkdir -p ~/2026_IFAC/offline_trajectory_generator/output/map'
scp -r ~/2026_IFAC/offline_trajectory_generator/output/map \
    jetson:~/2026_IFAC/offline_trajectory_generator/output/
```

ℹ️ **둘의 재빌드 여부가 다르다.**
- **지도**는 MCL이 `install/.../share`의 **실제 복사본**에서 읽으므로 §2 재빌드 필요
- **글로벌 패스**는 `output_base_dir`이 **런치 cwd 기준 상대경로**라 소스를 직접 읽는다
  → 재빌드 없이 즉시 반영 (단 `~/2026_IFAC`에서 launch)

### yaml 확인 — `image:`를 `.png`로
```yaml
image: map.png            # ← .pgm 이면 반드시 고친다
mode: trinary
resolution: 0.025
origin: [-20.3043, -1.4273, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```
`origin`/`resolution`이 틀리면 **스캔은 벽에 맞는데 경로만 어긋난다.**

### global path가 이 지도로 만들어졌는지 확인 (젯슨에 jq 없음)
```bash
cd ~/2026_IFAC && python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['map_info_str']['data'])" \
  offline_trajectory_generator/output/map/global_waypoints.json
```

---

## 2. 지도 넣었으면 재빌드 (젯슨)

```bash
cd ~/2026_IFAC && source /opt/ros/jazzy/setup.zsh
MAKEFLAGS="-j4" colcon build --symlink-install --executor sequential \
  --packages-select particle_filter_cpp global_planning local_planning
source install/setup.zsh
```
⚠️ Orin Nano는 메모리가 좁다 — `-j4 --executor sequential` 빼면 OOM으로 죽는다(`cb` alias가 이 설정).

확인:
```bash
ls -lh "$(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/maps/map."{yaml,png}
```

---

## 3. 실행 순서 (각 단계 검증 통과 후 다음으로. 제어기가 마지막)

```
젯슨 T1  f110 bringup     → /scan 40Hz, /odom 50Hz, /joy 20Hz
젯슨 T2  MCL              → /map, /pf/pose/odom
본체     RViz2            → 2D Pose Estimate → 스캔·벽 정합 확인
젯슨 T3  global_planning  → 경로·지도 정합 확인
젯슨 T4  local_planning
젯슨 T5  state_machine    → /state, /local_waypoints
젯슨 T6  control_real     → 전부 통과한 뒤에만
(선택)   obstacle_detector / lap_timer
```

### T1 (젯슨) — 하드웨어 bringup
```bash
f110
```
(= `cd ~/f1tenth_ws && source install/setup.zsh && ros2 launch f1tenth_stack bringup_launch.py`.
다른 launch를 쓰면 VESC 파라미터가 달라진다)

떠야 하는 노드: `/urg_node` `/vesc_driver_node` `/vesc_to_odom_node` `/ackermann_to_vesc_node`
`/ackermann_mux` `/drive_mode_manager` `/joy` `/static_baselink_to_laser`

```bash
ros2 topic hz /scan     # ≈40Hz, publisher 1개
ros2 topic hz /odom     # ≈50Hz, publisher 1개
ros2 topic hz /joy      # ≈20Hz
ros2 topic echo /drive_mode --field data     # estop → (X) manual
```
- 기동 로그에 `yaw rate from MEASURED gyro`가 보여야 정상(07-28부터 실측 자이로 기반).
  `IMU stale` 반복이면 IMU가 끊긴 것.
- 조이스틱 F710은 **X 모드**(뒷면 스위치). 절전되면 로지텍 버튼으로 깨운다.

### T2 (젯슨) — MCL
```bash
cd ~/2026_IFAC && source install/setup.zsh
echo "F1_MAP=$F1_MAP"     # 비었거나 map 이어야 한다 (§0-1)
ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map use_rviz:=false
```
`map_name:=map` — MCL은 `F1_MAP`을 안 읽으므로 **항상 명시**한다.
`use_rviz:=false` — RViz는 본체에서 띄운다(젯슨 렌더 부하 0).

```bash
ros2 param get /particle_filter_map_server yaml_filename   # .../maps/map.yaml
ros2 topic hz /pf/pose/odom
ros2 topic info /map --verbose                             # publisher = /particle_filter 1개
```
`/map` publisher가 2개면 중복 map_server를 내린다:
```bash
ros2 service call /lifecycle_manager_particle_filter/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"
```

### 본체 PC — RViz2
```bash
source /opt/ros/jazzy/setup.zsh
source ~/2026_IFAC/install/setup.zsh
export ROS_DOMAIN_ID=70
ros2 daemon stop && ros2 daemon start
rviz2 -d "$(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/rviz/particle_filter.rviz"
```
Fixed Frame `map` / Map `/map` / LaserScan `/scan` / Odometry `/pf/pose/odom` /
PoseArray `/pf/viz/particles` / TF.

**초기 위치**: 차량 정지 → `2D Pose Estimate` → 실제 위치 클릭 → 드래그로 방향 → 스캔이 벽과
겹치는지 확인.
- 스캔이 벽과 **안 맞음** → 지도 / initialpose / laser TF / MCL
- 스캔은 맞는데 **경로만 어긋남** → `global_waypoints.json` 또는 지도 `origin`/`resolution`

### T3 (젯슨) — global planning
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```
⚠️ **반드시 `~/2026_IFAC`에서** — `output_base_dir`이 상대경로다.

```bash
ros2 param get /global_trajectory_publisher_node map_name   # 'map' 이어야 한다
ros2 topic info /global_waypoints --verbose        # publisher 1개
ros2 topic info /car_state/frenet/odom --verbose   # /frenet_odom_node
```
RViz MarkerArray: `/global_waypoints/markers`, `/trackbounds/markers`,
`/centerline_waypoints/markers` → 지도와 겹쳐야 한다.

### T4 (젯슨) — local planning
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch local_planning local_planning.launch.py simulator:=false
```
`reference_map`은 `F1_MAP`에서 `<이름>.yaml`로 풀린다(기본 `map`).

### T5 (젯슨) — state machine (+ 로컬 웨이포인트 선택 발행)
`/state` 판정과 `/local_waypoints`·`/local_waypoints/path` 발행을 한 노드가 겸한다
(2026-08-02에 구 `wpnt_publisher` 흡수).
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```
```bash
ros2 topic echo /state
ros2 topic hz /local_waypoints          # publisher = /state_machine_node 하나
```

> 🔑 **`/local_waypoints`의 발행자는 `local_planner_node`가 아니라 `state_machine_node`다.**
> 로컬 플래너를 안 켜도 이 토픽은 나온다. `ros2 topic info --verbose`로 확인할 것.

**안 나올 때** — 세 입력이 다 있어야 GLOBAL 상태에서 발행된다:
```bash
ros2 topic echo /global_waypoints --once
ros2 topic echo /car_state/frenet/odom --once
ros2 topic echo /state --once
ros2 topic echo /car_state/frenet/odom --field child_frame_id   # '0','125' 같은 정수 문자열
```
`child_frame_id`가 `''`/`'base_link'`면 `Invalid child_frame_id for index` 경고와 함께 멈춘다.

### T6 (젯슨) — control (마지막)

**실행 전**: 바퀴 들거나 스탠드 / E-stop 준비 / `/scan`·`/odom`·`/pf/pose/odom` 정상 /
경로·지도 정합 / TF 정상(§6) / `/state` 정상.

```bash
source ~/f1tenth_ws/install/setup.zsh
source ~/2026_IFAC/install/setup.zsh
cd ~/2026_IFAC
ros2 launch f1tenth_control control_real.launch.py
```

#### 🔴 2026-08-17 조향 메커니즘이 바뀌었다 — 첫 주행은 반드시 사다리로

LUT를 걷어내고 **자전거 역모델 + FF/FB 분리**로 교체했다. 상세는 [`README.md`](README.md).
**구 LUT 대비 조향이 17~67% 커진다(속도에 비례해 증가).**

```bash
# ① 저속 — 순 증가 ≈ +4%라 사실상 구 거동 확인 + MCL 4랩 이상 데이터
ros2 launch f1tenth_control control_real.launch.py \
    max_speed:=3.5 max_lateral_accel:=5 min_speed:=1.5

# ② 중속 — 순 증가 +15~26%. **횡진동이 처음 보일 구간이라 여기가 진짜 관문이다**
ros2 launch f1tenth_control control_real.launch.py max_speed:=5.0 max_lateral_accel:=6

# ③ 정상
ros2 launch f1tenth_control control_real.launch.py
```
각 단계마다 `python3 tools/bag_analyzer/analyze_bag.py <bag>` **CRITICAL 0건** 확인 후 다음으로.

**진동이 나면 원인별로 노브가 다르다**:

| 증상 | 원인 | 노브 |
|---|---|---|
| 전 속도에서 진동 | `steering_reach_ratio` 이중보정 (+17.6%) | `steering_reach_ratio:=1.0` ⚠️ δ_avail도 같이 움직임 |
| 고속에서만 진동 | 루프게인 초과 | **`steering_fb_gain:=0.8`** ← FF는 그대로 두고 FB만 줄인다 |
| 저속 서행에서 진동 | L1 감쇠 여유 | `l1_offset` ↑ |

#### 상태 로그 읽는 법

```
| FF κ +0.412 a_ff +5.83 / a_cmd +5.83 | K_us 0.01423(관측)(n=812)
| 곡선(관측)[0.0140/0.0152/0.0171/0.0189 n=210/420/380/95] | 슬립 0.84(peak 2.10)
```
| 항목 | 보는 법 |
|---|---|
| `FF κ` | **좌우코너에 따라 부호가 바뀌어야 한다.** 항상 +면 버그 재발 |
| `a_cmd` | `steering_fb_gain=1.0`이면 L1 값과 같다 |
| `K_us …(관측)` | 수렴하면 0.0135~0.0153 부근 |
| `곡선[…]` | 4개 빈이 **오름차순**이면 포화 관측됨. `n`이 300 넘어야 실제로 쓸 수 있다 |
| `슬립` | 자이로↔가속도계 잔차. 준정상 0.4~1.5 / 스핀 1.1~4.3 (**진단 전용**) |

#### 같이 뜨는 노드

실차 런치는 `sector_scale_enable:=true`가 기본이라 **`sector_learner` 노드가 자동 기동**된다
(기본 모드 `static` = `config/sectors.yaml`을 그대로 발행, 값 안 바꿈).
```bash
ros2 topic hz /sector_scales     # 1 Hz 하트비트 (컨트롤러 데드맨용)
```

```bash
ros2 topic echo /drive
```
조이스틱: **A**=자율, **B**=E-stop, **X**=수동.
수동/자율/E-stop Mux는 `f1tenth_stack`의 `drive_mode_manager`+`ackermann_mux` 담당.
우리 `drive_source_selector`는 `/drive_autonomous`를 `/drive`로 포워딩만 한다.

### (선택) 장애물 검출 / 랩타임
```bash
ros2 launch obstacle_detector obstacle_detector.launch.py     # 회피 테스트할 때
ros2 launch lap_timer lap_timer.launch.py                     # 랩타임 계측
```

---

## 4. rosbag 녹화

```bash
f1rec [태그]            # 랩탑에서 녹화 → ~/rosbag_log/MMDD/run_MMDD_HHMMSS[_태그]/
f1rec --remote [태그]   # 젯슨에서 녹화 후 자동 회수 (무선 유실 없음)
f1rate                  # 최신 bag 달성률 재검사
```
젯슨 직접 녹화는 `tools/jetson_rec.sh`가 담당하며 **tmux 세션 안에서 돈다** — 주행 중 wifi가
끊겨도 녹화는 계속되고 `jetson_rec.sh --attach`로 다시 붙어 정상 종료(=회수)할 수 있다.

⚠️ `-s sqlite3` 고정(`tools/bag_analyzer`는 `.db3`만 파싱) — `f1rec`이 알아서 준다.

🔴 **`ROS_DISCOVERY_SERVER`를 export 하지 말 것.** 젯슨은 디스커버리 서버를 안 쓴다
(도메인 **70** 멀티캐스트로 동작). 이 변수가 있으면 없는 서버한테만 물어보게 돼서 **토픽이
하나도 안 잡힌다.** 실수로 export 했으면 `unset ROS_DISCOVERY_SERVER`.

### 시작 시점 — "control 전"이 아니라 "**A(자율) 누르기 전**"이다

`/global_waypoints`·`/tf_static`은 **transient_local(latched)**이고 rosbag2가 발행자 QoS에
맞춰 구독하므로 **늦게 붙어도 과거 발행분을 받아온다**(2026-07-31 실측 확인).
진짜로 놓치면 안 되는 건 자율 진입 순간의 과도구간(engage 게이트·런치 킥)이다.

---

## 5. 분석 도구 (랩탑, 오프라인)

```bash
cd ~/F1tenth_control
python3 tools/bag_analyzer/analyze_bag.py             <bag>      # 종합 리포트 (CRITICAL 확인)
python3 tools/bag_analyzer/analyze_grip_envelope.py   <bag...>   # 그립 한계 + K_us(a_lat) 곡선
python3 tools/bag_analyzer/analyze_mcl_quality.py     <bag...>   # MCL 구간별 정확도
python3 tools/bag_analyzer/analyze_local_path_jump.py <bag...>   # Frenet s → 경로 점프 검사
python3 tools/bag_analyzer/analyze_sector_clearance.py <bag>     # 섹터별 벽 여유
python3 tools/bag_analyzer/compare_steering_model.py  <bag...>   # LUT vs 자전거모델
bash   tools/f1net_client.sh                                     # 네트워크 자가진단 (§7)
```

---

## 6. 종료

```bash
ros2 node list | sort
ps -ef | grep '[r]os2 launch'
kill -SIGINT <정확한_PID>
```
⚠️ `pkill ros2` / `killall` / 광범위한 `pkill -f` 금지 — 필요한 노드까지 죽인다.

**하드웨어 bringup 내릴 때 순서**: 차량 고정 → E-stop(B) → 자율 명령 중단 →
`ros2 topic echo /commands/motor/speed --field data`로 0 확인 → bringup에 Ctrl+C.

---

## 7. TF 책임 구조

```
particle_filter     → map → odom
vesc_to_odom_node   → odom → base_link
static TF           → base_link → laser
```
`odom→base_link`를 MCL과 `vesc_to_odom`이 동시에 내던 것을 07-29에 해소
(`publish_odom_base_tf` 런치 인자, 기본 false). odom을 믿는 근거는 07-28 검증
(34 m 폐합 15.6 cm, 2바퀴 헤딩 +717.6°/720°, 자이로 스케일 오차 +0.06%).

⚠️ `odom→base_link`를 내는 쪽이 **반드시 살아 있어야 한다** — 실차는 f110 bringup
(`vesc.yaml` `publish_tf: true`), 시뮬은 gym_bridge. 둘 다 없는 bag 재생에서만 켠다:
```bash
ros2 launch particle_filter_cpp mcl_launch.py mod:=bag publish_odom_base_tf:=true
```

```bash
ros2 topic info /tf --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

---

## 8. 통신이 안 될 때

### 먼저 이걸 돌린다 — 원인을 갈라준다
```bash
bash ~/F1tenth_control/tools/f1net_client.sh
```
젯슨 ssh도 sudo도 필요 없다. **핵심은 [4]단계** — 젯슨 스택이 떠 있으면 DDS가 자기 존재를
`239.255.0.1`의 **`7400 + 250×도메인` = 24900**(도메인 70)으로 방송하므로, 그 SPDP 패킷을
직접 받아보면 "패킷이 내 랩탑까지 오는가 + 도메인이 맞는가"가 한 번에 판정된다.

⚠️ **`ros2 topic list`로는 안 갈린다.** 방화벽이든 도메인 불일치든 스택 미기동이든 **전부
똑같이 "토픽 0개"**로 보인다 — 2026-08-10에 AP 멀티캐스트를 헛짚은 원인이 이거였다.

### 🔴 실제 원인이었던 것: **랩탑의 ufw**
`DEFAULT_INPUT_POLICY="DROP"`이라 DDS 디스커버리(상대가 먼저 쏘는 unsolicited inbound UDP)만
막혔다. ping·ssh는 conntrack이 통과시켜서 "네트워크 정상"으로 보였다.
```bash
sudo ufw allow from 10.1.1.0/24 comment 'jetson AP (ROS2 DDS)'
sudo ufw reload && ros2 daemon stop     # ← daemon stop 빼면 빈 캐시가 그대로 보인다
```
`ufw disable`은 쓰지 말 것(대회장 공용 WiFi에서 랩탑이 통째로 열린다).

### 기본 확인
```bash
printenv ROS_DOMAIN_ID          # 양쪽 70
printenv RMW_IMPLEMENTATION     # rmw_fastrtps_cpp
date                            # 시각 차이 크면 TF가 안 보인다
```
⚠️ **`ssh jetson 'cmd'`로 젯슨 상태를 재면 안 된다** — 비대화형이라 ROS를 안 소싱해서
`ros2: not found`가 "토픽 0개"로 둔갑한다. `bash -lc`도 젯슨 유저가 zsh면 `~/.zshrc`
(= `ROS_DOMAIN_ID` 정의처)를 안 읽는다. 노드의 진짜 도메인은 `/proc/<pid>/environ`으로 볼 것.

ℹ️ 젯슨은 멀티홈(`wlP1p1s0` 통신 / `enP8p1s0` 192.168.0.15 **라이다 전용** / `l4tbr0` USB).
AP(HY_MIRU)의 멀티캐스트 전달 자체는 정상임이 확인됐으므로 `ROS_STATIC_PEERS`·랜선은 불필요.

---

## 9. 증상별 첫 확인 지점

| 증상 | 먼저 볼 것 |
|---|---|
| 본체에서 젯슨 토픽이 안 보임 | **`f1net_client.sh` 먼저** (§8). ufw / `ROS_DOMAIN_ID` 양쪽 70 / `ros2 daemon stop && start` |
| 스캔이 벽과 안 맞음 | 지도, `2D Pose Estimate`, `base_link→laser` static TF, MCL |
| 스캔은 맞는데 경로만 어긋남 | `global_waypoints.json`의 `map_info_str`, 지도 `origin`/`resolution` |
| MCL만 다른 지도를 봄 | `map_name:=map`을 안 넘겼다 — MCL은 `F1_MAP`을 안 읽는다 (§0-1) |
| global/local이 경로를 못 찾음 | 젯슨 `.zshrc`에 옛 `F1_MAP`이 박혀 있는지 (§0-1). 기본값은 셋 다 `map` |
| 지도를 넣었는데 없다고 함 | 재빌드 안 함 (§2) |
| `/local_waypoints` 무발행 | `child_frame_id`가 정수 문자열인지 (T5) |
| `/joy` 무발행 | F710 절전 — 로지텍 버튼. `input` 그룹은 새 로그인 세션에만 적용 |
| 자율 진입 시 급발진 | `engage_gate_enable`(기본 true), `/drive_mode` 발행 여부, `ramp_lead_max` |
| **코너에서 밀림 / 횡진동** | **T6 사다리를 건너뛰었는지** — 조향이 17~67% 커졌다 (T6·README.md) |
| odom 헤딩 이상 | 기동 로그 `yaw rate from MEASURED gyro` 유무, `IMU stale` 반복 여부 |
| 한쪽 풀조향이 걸림 | 젯슨 `vesc.yaml`의 `steering_angle_to_servo_offset`이 서보창(0.23~0.66) 밖인지 |
