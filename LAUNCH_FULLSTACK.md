# 실차 풀스택 실행 가이드 (ROS 2 Jazzy)

2026-07-29 젯슨 재설치 이후 **실제로 띄워보면서** 확인/수정한 절차.
아래 코드블록은 전부 **터미널에 그대로 붙여넣으면 되는 형태**다(source·cd 포함).

| | 값 |
|---|---|
| 젯슨 | `miru@10.1.1.3` (wifi `wlP1p1s0`), Ubuntu 24.04 / **ROS 2 Jazzy** |
| 본체 PC | `10.1.1.24` (wifi `wlo1`), **ROS 2 Jazzy** |
| 워크스페이스 | 젯슨 `~/f1tenth_ws`(하드웨어) + `~/2026_IFAC`(제어·플래닝·MCL) |
| 지도 이름 | **`map`** (slam_toolbox 기본 저장명 그대로 씀) |

젯슨 `~/.zshrc`에 `ROS_DOMAIN_ID=67`, `F1_MAP`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`가
이미 있으므로 **젯슨에서는 export가 필요 없다.** 본체에서만 맞춰준다.

---

## 0. 반드시 알고 있어야 할 함정 4개

| # | 함정 | 대응 |
|---|---|---|
| 1 | **MCL만 `F1_MAP`을 안 읽는다** (기본값 `map` 고정) | MCL 런치에 `map_name:=` 명시. 안 하면 "경로 ≠ 위치추정" |
| 2 | **지도를 넣었으면 재빌드** — MCL은 `src/`가 아니라 설치된 share에서 읽고, 지도는 심볼릭 링크가 아닌 실제 복사본 | §2 재빌드 |
| 3 | 지도 yaml의 `image:`가 `.pgm`으로 써질 때가 있다 | `.png`로 고칠 것 |
| 4 | `odom→base_link` TF 이중 발행 | `mcl_launch.py`의 `publish_odom_base_tf` 기본 false로 분리 (§6) |

`global_planning`/`local_planning`은 `F1_MAP`을 자동으로 읽으므로 yaml 수정도 인자 전달도 불필요.

### 2026-07-29 젯슨에서 바꾼 것
- `vesc.yaml` `speed_min/max`: ±33856 → **±40000** (= 9.45 m/s). 허용 상한을 푼 것일 뿐
  실제 속도는 컨트롤러 `max_speed`(실차 기본 5.0)가 정한다. 셰이크다운은 `max_speed`로 낮출 것.
- `mcl_launch.py` `publish_odom_base_tf`: 항상 true → **런치 인자, 기본 false**.

---

## 1. 지도 만들기 / 옮기기 (새 트랙일 때만)

### 매핑 (**랩탑**에서 실행. 젯슨은 f110 bringup만 떠 있으면 됨)
```bash
cd ~/slam_toolbox && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false \
  slam_params_file:=$HOME/F1tenth_control/tools/odom_diag/slam_toolbox_params.yaml
```

⚠️ **`slam_params_file`을 꼭 넘길 것.** 안 주면 slam_toolbox 패키지 기본값이 쓰이는데
`map_update_interval: 5.0`이라 `/map`이 5초에 한 번만 나온다 — 짧게 돌리면 맵이 거의
안 그려져서 "안 되는 것처럼" 보인다(2026-07-31 실제로 헷갈렸던 지점).
우리 파일은 트랙(46.9 m)에 맞춰 조정돼 있다:
`map_update_interval 1.0` / `minimum_travel_distance 0.3` / `minimum_travel_heading 0.2` /
`max_laser_range 10.0`(UST-10LX 실사용 범위 — 트랙 밖 잡동사니로 스캔매칭이 흔들리는 것 방지).

ℹ️ 로그의 `Message Filter dropping message ... queue is full`은 **정상**이다.
스캔은 40 Hz로 들어오는데 `minimum_time_interval: 0.5`라 slam은 2 Hz만 쓴다 — 나머지가
버려지면서 찍히는 INFO다. 진짜로 TF가 없으면 `Registering sensor`가 아예 안 뜬다.

매핑 요령:
- **천천히**(0.5~1 m/s) 밀 것. 2 m/s면 처리 스캔 간격이 1 m라 스캔매칭이 성겨진다
- 한 바퀴 이상 돌 것 (10초로는 아무것도 안 나온다)
- **중간에 몇 초씩 정지** — odom 헤딩이 자이로 순수 적분이고 바이어스는 정지 중에만
  학습된다(τ=10 s). 계속 굴러다니면 갱신이 안 돼 10분에 ~19°까지 쌓인다
- 진행 확인: `ros2 topic hz /map` / `ros2 run tf2_ros tf2_echo map odom`

### 저장 (다른 터미널 — 🔴 **slam_toolbox를 끄기 전에**)
```bash
cd ~/slam_toolbox && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 run nav2_map_server map_saver_cli -f ~/slam_toolbox/map --fmt png \
  --ros-args -p save_map_timeout:=30.0 -p map_subscribe_transient_local:=true
```

🔴 **순서가 전부다: 저장 → 확인 → 그다음에 slam_toolbox에 Ctrl+C.**
`map_saver_cli`는 `/map`을 **구독해서** 받아 적는 도구라 발행자(slam_toolbox)가 살아 있어야 한다.
먼저 끄면 30초(`save_map_timeout`)를 기다렸다가 이렇게 죽고 **맵은 그대로 날아간다**:
```
[ERROR] [map_saver]: Failed to spin map subscription
[ros2run]: Process exited with failure 1
```
(2026-07-31에 실제로 겪음 — slam Ctrl+C 4초 뒤 map_saver 실행, 정확히 30.0초 뒤 실패.
매핑 주행을 통째로 다시 했다.) 저장되면 파일 시각으로 확인할 것:
```bash
ls -l --time-style=+%H:%M:%S ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml
```

⚠️ **`-f`는 디렉터리가 아니라 "파일 이름 접두사"다.** 예전 문서는 `-f ~/slam_toolbox`라
적어놨는데, 그러면 폴더 안이 아니라 홈에 **`~/slam_toolbox.pgm` / `.yaml` 파일**이 생긴다
(같은 이름의 폴더가 있어서 헷갈린다 — 2026-07-31에 실제로 헤맸다).
위처럼 `-f ~/slam_toolbox/map`으로 줘야 아래 전송 단계가 기대하는
`~/slam_toolbox/map.png` + `map.yaml`이 나온다.

ℹ️ `--fmt png`를 주면 pgm→png 수동 변환이 필요 없다. yaml의 `image:`도 자동으로
`map.png`가 되므로 아래 "yaml 확인" 단계도 통과한다.

### 글로벌 패스 생성 (**랩탑**에서. 젯슨은 CPU가 좁아 여기서 만드는 게 맞다)

**① 새 지도를 로컬 워크스페이스에 넣는다**
```bash
cp ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml \
   ~/2026_IFAC/src/monte_carlo_localization/maps/
```

**② GUI로 라인 확인하며 생성** (새 트랙이면 이쪽. 라인을 눈으로 보고 조정)
```bash
cd ~/2026_IFAC
python3 offline_trajectory_generator/trajectory_gui.py \
  --map-yaml src/monte_carlo_localization/maps/map.yaml
```
- 파라미터를 만지면 `offline_trajectory_generator/gui_params.yaml`에 **자동 저장**된다
- `Save`를 누르면 `output/map/`에 `global_waypoints.json` / `global_waypoints.csv` /
  `centerline.csv` / `metadata.json` (+`debug_overlay.png`)가 생성된다

**③ 헤드리스로 생성** (같은 결과를 재현하거나 파라미터를 스윕할 때)
```bash
cd ~/2026_IFAC
python3 offline_trajectory_generator/generate_global_trajectory.py \
  --map-yaml src/monte_carlo_localization/maps/map.yaml \
  --output-dir offline_trajectory_generator/output/map \
  --optimizer mincurv --width-mode hybrid \
  --waypoint-step 0.25 --optimizer-step 0.46 \
  --safety-width 0.5 --boundary-margin 0.5 --max-width-distance 2.0 \
  --max-speed 6.5 --min-speed 4.0 \
  --max-lateral-accel 6.0 --max-accel 3.5 --max-decel 2.5 \
  --max-curvature 1.18 \
  --smooth-sigma 3.7 --raceline-smooth-sigma 1.0
```
전체 인자는 `python3 offline_trajectory_generator/generate_global_trajectory.py --help`.
`--output-dir`을 생략하면 `output/<지도이름>`이 기본이다.

#### ⚠️ 생성기 인자를 **차량 실측 한계**에 맞출 것 (2026-07-31 측정)

생성기가 차보다 낙관적인 값을 쓰면 **컨트롤러가 못 따라가는 프로파일**이 나오고,
곡률 사전감속이 매 코너에서 그걸 깎느라 싸운다.

| 생성기 인자 | GUI 저장값 | 차량 실측 한계 | 권장 |
|---|---|---|---|
| `--max-curvature` | 1.2 | **1.286** (조향 23°, R 0.777 m) | **1.18** (R 0.85 m, 여유 15%) |
| `--max-lateral-accel` | 10.0 | 컨트롤러 캡 **6.0** | **6.0** |
| `--max-accel` | 3.7 | VESC 램프 `s_pid_ramp_erpms_s` 15600 = **3.69** | **3.5** |
| `--max-decel` | 2.0 | 브레이크 하드웨어 4.8 / 현 튜닝 **2.5** | **2.5** |
| `--max-speed` | 6.5 | 실제 도달 7.4 (직선 13.3 m라 8.0엔 못 닿음) | 6.5~7.0 |

🔴 **`--max-curvature`가 실제로 지켜졌는지 반드시 확인할 것.** `ifac_track_v2`는 이 값이
1.2인데도 결과 κ가 **1.485**까지 나와 있어서, 187점 중 2점이 차량 최소 선회반경보다 급하다
(= 어떤 제어 튜닝으로도 못 도는 코너. 07-25 시케인·07-26 헤어핀 크래시의 근본 원인).
```bash
cd ~/2026_IFAC && python3 -c "
import csv
r=[x for x in list(csv.reader(open('offline_trajectory_generator/output/map/global_waypoints.csv')))[1:] if x]
k=[abs(float(x[5])) for x in r]
print('최대 kappa %.3f -> R_min %.3f m  (차량 한계 R 0.777 m = kappa 1.286)' % (max(k), 1/max(k)))
print('차량 한계 초과 점: %d / %d' % (sum(1 for v in k if v > 1.286), len(k)))
"
```
초과 점이 0이 아니면 `--max-curvature`를 더 낮추거나 `--safety-width`를 키워 다시 뽑는다.

### 본체 → 젯슨 전송
```bash
# 지도 (MCL용) — 재빌드 필요
scp ~/slam_toolbox/map.png ~/slam_toolbox/map.yaml \
    miru@10.1.1.3:~/2026_IFAC/src/monte_carlo_localization/maps/

# 글로벌 패스 — 재빌드 불필요
ssh miru@10.1.1.3 'mkdir -p ~/2026_IFAC/offline_trajectory_generator/output/map'
scp -r ~/2026_IFAC/offline_trajectory_generator/output/map \
    miru@10.1.1.3:~/2026_IFAC/offline_trajectory_generator/output/
```

ℹ️ **둘의 재빌드 여부가 다르다.**
- **지도**는 MCL이 `install/.../share`의 **실제 복사본**에서 읽으므로 §2 재빌드가 필요하다
- **글로벌 패스**는 `output_base_dir: "offline_trajectory_generator/output"`이 **런치 cwd 기준
  상대경로**라 소스 트리를 직접 읽는다 → 재빌드 없이 즉시 반영(단 `~/2026_IFAC`에서 launch)

🔴 **`F1_MAP`과 지도 이름이 어긋나면 조용히 실패한다.** 지도 이름은 **`map`이 기본**인데
젯슨 `~/.zshrc`는 `export F1_MAP=ifac_track`으로 되어 있다(2026-07-31 확인). 이러면
`global_planning`이 없는 `output/ifac_track`을 본다 — 젯슨 `output/`에는 `map`뿐이다. 둘 중 하나:
```bash
# (a) 젯슨 F1_MAP을 map으로 고친다  ← 권장
ssh miru@10.1.1.3 "sed -i 's/^export F1_MAP=.*/export F1_MAP=map/' ~/.zshrc"

# (b) 매번 인자로 넘긴다
ros2 launch global_planning global_planning.launch.py map_name:=map
```
MCL은 어차피 `F1_MAP`을 안 읽으므로(§0-1) `map_name:=map`을 계속 명시한다.

### yaml 확인 — `image:`를 `.png`로
```yaml
image: map.png            # ← .pgm 으로 써져 있으면 반드시 고친다
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
출력에 `map.yaml`(쓰는 지도 이름)이 보여야 한다.

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
젯슨 T5  state_machine    → /state
젯슨 T6  wpnt_publisher   → /local_waypoints
젯슨 T7  control_real     → 전부 통과한 뒤에만
```

### T1 (젯슨) — 하드웨어 bringup
```bash
f110
```
(= `cd ~/f1tenth_ws && source install/setup.zsh && ros2 launch f1tenth_stack bringup_launch.py`.
다른 launch를 쓰면 VESC 파라미터가 달라진다)

떠야 하는 노드: `/urg_node` `/vesc_driver_node` `/vesc_to_odom_node` `/ackermann_to_vesc_node`
`/ackermann_mux` `/drive_mode_manager` `/joy` `/static_baselink_to_laser`

검증:
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
ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map use_rviz:=false
```
`use_rviz:=false` — RViz는 본체에서 띄운다(젯슨 렌더 부하 0).

검증:
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
export ROS_DOMAIN_ID=67
ros2 daemon stop && ros2 daemon start
rviz2 -d "$(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/rviz/particle_filter.rviz"
```
Fixed Frame `map` / Map `/map` / LaserScan `/scan` / Odometry `/pf/pose/odom` / PoseArray `/pf/viz/particles` / TF.

**초기 위치**: 차량 정지 → `2D Pose Estimate` → 실제 위치 클릭 → 드래그로 방향 → 스캔이 벽과 겹치는지 확인.
- 스캔이 벽과 **안 맞음** → 지도 / initialpose / laser TF / MCL
- 스캔은 맞는데 **경로만 어긋남** → `global_waypoints.json` 또는 지도 `origin`/`resolution`



### T3 (젯슨) — global planning
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```
⚠️ **반드시 `~/2026_IFAC`에서** — `output_base_dir`이 상대경로다.

검증:
```bash
ros2 param get /global_trajectory_publisher_node map_name
ros2 topic info /global_waypoints --verbose        # publisher 1개
ros2 topic info /car_state/frenet/odom --verbose   # /frenet_odom_node
```
RViz MarkerArray 추가: `/global_waypoints/markers`, `/trackbounds/markers`, `/centerline_waypoints/markers` → 지도와 겹쳐야 한다.



### T4 (젯슨) — local planning
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch local_planning local_planning.launch.py simulator:=false
```
`reference_map`은 `F1_MAP`에서 자동으로 풀린다(인자 불필요).
```bash
ros2 node list | grep -i map      # map_server 실제 노드 이름 확인
```



### T5 (젯슨) — state machine
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```
```bash
ros2 topic echo /state
```



### T6 (젯슨) — wpnt_publisher
```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 run wpnt_publisher wpnt_publisher
```
```bash
ros2 topic hz /local_waypoints          # publisher = /wpnt_publisher 하나
```
**안 나올 때** — 세 입력이 다 있어야 GLOBAL 상태에서 발행된다:
```bash
ros2 topic echo /global_waypoints --once
ros2 topic echo /car_state/frenet/odom --once
ros2 topic echo /state --once
ros2 topic echo /car_state/frenet/odom --field child_frame_id   # '0','125' 같은 정수 문자열이어야 함
```
`child_frame_id`가 `''`/`'base_link'`면 `Invalid child_frame_id for index` 경고와 함께 발행이 멈춘다.



### T7 (젯슨) — control (마지막)
**실행 전**: 바퀴 들거나 스탠드 / E-stop 준비 / `/scan`·`/odom`·`/pf/pose/odom` 정상 /
경로·지도 정합 / TF 정상(§6) / `/state` 정상.

```bash
source ~/f1tenth_ws/install/setup.zsh
source ~/2026_IFAC/install/setup.zsh
cd ~/2026_IFAC
ros2 launch f1tenth_control control_real.launch.py
```
셰이크다운(캡 낮춰서):
```bash
source ~/f1tenth_ws/install/setup.zsh
source ~/2026_IFAC/install/setup.zsh
cd ~/2026_IFAC
ros2 launch f1tenth_control control_real.launch.py max_speed:=2.5 min_speed:=0.5
```
```bash
ros2 topic echo /drive
```
조이스틱: **A**=자율, **B**=E-stop, **X**=수동.
수동/자율/E-stop Mux는 `f1tenth_stack`의 `drive_mode_manager`+`ackermann_mux` 담당,
우리 `drive_source_selector`는 `control_map_node`의 `/drive_autonomous`를 `/drive`로
포워딩만 한다(2026-08-01 MPPI 제거로 RB 토글 없음).

---

## 4. rosbag 녹화 — **랩탑에서 `f1rec`** (T7 이후에 시작해도 된다)

```bash
f1rec [태그]     # → ~/rosbag_log/MMDD/run_MMDD_HHMMSS[_태그]/
                # 주행 끝나면 Ctrl+C → 토픽 달성률 자동 검사
```

🔴 **`ROS_DISCOVERY_SERVER`를 export 하지 말 것.** 젯슨은 디스커버리 서버를 안 쓴다
(2026-07-31 재확인: 11811 리스닝 없음, fastdds 프로세스 없음 — 평범한 도메인 67 멀티캐스트로 동작).
이 변수가 있으면 없는 서버한테만 물어보게 돼서 **토픽이 하나도 안 잡힌다.** `.zshrc`에도
안 들어 있으니 새 터미널은 그냥 쓰면 되고, 실수로 export 했으면 `unset ROS_DISCOVERY_SERVER`.
(§7의 디스커버리 서버 안내는 젯슨에서 `fastdds discovery -i 0 -l 10.1.1.3 -p 11811`을
**먼저 띄웠을 때만** 유효하다.)

### 시작 시점 — "control 전"이 아니라 "**A(자율) 누르기 전**"이다

예전 문서는 control 실행 전에 녹화를 걸라고 했는데, 실제 제약은 그게 아니다.
`/global_waypoints`·`/tf_static`는 전부 **transient_local(latched)**이고
rosbag2가 발행자 QoS에 맞춰 구독하므로, **늦게 붙어도 과거 발행분을 받아온다.**
2026-07-31 실측(풀스택 가동 중에 새로 녹화 시작): `/tf_static` 1개 ✓ / `/global_waypoints` 5개 ✓.

따라서 **control(T7)을 다 띄운 뒤에 `f1rec`을 시작해도 된다.** 진짜로 놓치면 안 되는 것은
자율 진입 순간의 과도구간(engage 게이트·런치 킥)이므로 **조이스틱 A를 누르기 전에만**
녹화가 돌고 있으면 된다.

### 젯슨에서 직접 녹화해야 할 때
랩탑 녹화의 달성률이 나쁘거나(무선 유실) `/sensors/core`가 꼭 필요하면:
```bash
f1rec --remote [태그]     # 젯슨에서 녹화 후 랩탑으로 자동 회수
f1rate                    # 최신 bag 달성률 재검사
```
⚠️ `-s sqlite3` 고정(`tools/bag_analyzer`는 `.db3`만 파싱) — `f1rec`이 알아서 준다.

---

## 5. 종료

```bash
ros2 node list | sort
ps -ef | grep '[r]os2 launch'
kill -SIGINT <정확한_PID>
```
⚠️ `pkill ros2` / `killall` / 광범위한 `pkill -f` 금지 — 필요한 노드까지 죽인다.

**하드웨어 bringup 내릴 때 순서**: 차량 고정 → E-stop(B) → 자율 명령 중단 →
`ros2 topic echo /commands/motor/speed --field data`로 0 확인 → bringup에 Ctrl+C.

---

## 6. TF 책임 구조

```
particle_filter     → map → odom
vesc_to_odom_node   → odom → base_link
static TF           → base_link → laser
```
`odom→base_link`를 MCL과 `vesc_to_odom`이 동시에 내던 것을 07-29에 해소
(`publish_odom_base_tf` 런치 인자, 기본 false). odom을 믿는 근거는 07-28 검증
(34 m 폐합 15.6 cm, 2바퀴 헤딩 +717.6°/720°, 자이로 스케일 오차 +0.06%).

⚠️ `odom→base_link`를 내는 쪽이 **반드시 살아 있어야 한다** — 실차는 f110 bringup(`vesc.yaml`
`publish_tf: true`), 시뮬은 gym_bridge. 둘 다 없는 bag 재생에서만 켠다:
```bash
ros2 launch particle_filter_cpp mcl_launch.py mod:=bag publish_odom_base_tf:=true
```

주행 전 확인:
```bash
ros2 topic info /tf --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```
⚠️ MCL은 팀 공용 패키지다. 이 변경은 젯슨 `~/2026_IFAC`(`jazzy_port_backup`)에만 있고 팀 repo 미반영.

---

## 7. 통신이 안 될 때

```bash
# 양쪽에서
printenv ROS_DOMAIN_ID          # 67
printenv RMW_IMPLEMENTATION     # rmw_fastrtps_cpp
date                            # 시각 차이 크면 TF가 안 보인다
```
테스트 — 젯슨:
```bash
ros2 topic pub /network_test std_msgs/msg/String "{data: 'hello from jetson'}" -r 1
```
본체:
```bash
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=67
ros2 daemon stop && ros2 daemon start
ros2 topic echo /network_test
```
⚠️ 젯슨은 멀티홈(`wlP1p1s0` 10.1.1.3 통신 / `enP8p1s0` 192.168.0.15 **라이다 전용** / `l4tbr0` USB).
wifi AP가 멀티캐스트를 막으면 Discovery Server를 쓴다. ⚠️ **먼저 젯슨에서 서버를 띄워야 한다** —
현재 젯슨에는 안 떠 있고(2026-07-31 확인), 서버 없이 아래 변수만 export 하면 **토픽이 하나도
안 잡힌다**(있지도 않은 서버한테만 물어보게 된다). 실제로 07-20·07-31 두 번 이걸로 헤맸다.
```bash
# 젯슨에서 먼저
fastdds discovery -i 0 -l 10.1.1.3 -p 11811
# 그 다음에야 본체에서
export ROS_DISCOVERY_SERVER="10.1.1.3:11811"
export ROS_SUPER_CLIENT=true     # ros2 topic list 열거까지 하려면
```
유선(피트)에서는 불필요. ⚠️ `ROS_LOCALHOST_ONLY`는 Jazzy에서 폐기 예정 — 실차에서는
`ROS_LOCALHOST_ONLY`/`ROS_AUTOMATIC_DISCOVERY_RANGE` **둘 다 건드리지 않는 것**이 기본값(SUBNET)이라 안전.

---

## 8. 증상별 첫 확인 지점

| 증상 | 먼저 볼 것 |
|---|---|
| 본체에서 젯슨 토픽이 안 보임 | `ROS_DOMAIN_ID` 양쪽 67 / 같은 서브넷 / `ros2 daemon stop && start` / AP client isolation / VPN·방화벽 |
| 스캔이 벽과 안 맞음 | 지도, `2D Pose Estimate`, `base_link→laser` static TF, MCL |
| 스캔은 맞는데 경로만 어긋남 | `global_waypoints.json`의 `map_info_str`, 지도 `origin`/`resolution` |
| MCL만 다른 지도를 봄 | `map_name:=`을 안 넘겼다 (§0-1) |
| 지도를 넣었는데 없다고 함 | 재빌드 안 함 (§2) |
| `/local_waypoints` 무발행 | `child_frame_id`가 정수 문자열인지 (T6) |
| `/joy` 무발행 | F710 절전 — 로지텍 버튼. `input` 그룹은 새 로그인 세션에만 적용 |
| 자율 진입 시 급발진 | `engage_gate_enable`(기본 true), `/drive_mode`가 실제로 발행되는지 |
| odom 헤딩 이상 | 기동 로그 `yaw rate from MEASURED gyro` 유무, `IMU stale` 반복 여부 |
