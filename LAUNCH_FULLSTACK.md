# 실차 풀스택 실행 가이드 (ROS 2 Jazzy)

2026-07-29 젯슨 재설치 이후 **실제 젯슨 상태를 하나씩 확인해서** 작성했다.
팀원의 Humble 기준 가이드를 옮긴 것이지만, 단순 배포판 치환이 아니라 **틀린 항목 5건을
바로잡았다**(아래 "옛 가이드에서 바뀐 것" 참고). 새 지도 이름은 `new_track`으로 가정한다.

| | 값 | 확인 방법 |
|---|---|---|
| 젯슨 | `miru@10.1.1.3` (wifi `wlP1p1s0`), Ubuntu 24.04.4 / JetPack R39.2 / **ROS 2 Jazzy** | `ssh jetson` |
| 본체 PC | `10.1.1.24` (wifi `wlo1`), **ROS 2 Jazzy** | 같은 서브넷, ping 2.8 ms |
| 워크스페이스 | 젯슨 `~/f1tenth_ws`(하드웨어) + `~/2026_IFAC`(제어·플래닝·MCL) | |

---

## 0. 옛 가이드에서 바뀐 것 (먼저 읽을 것)

| # | 옛 가이드 | 실제 (2026-07-29 확인) |
|---|---|---|
| 1 | `source /opt/ros/humble/setup.zsh` | **Jazzy**. 양쪽 다 `/opt/ros/jazzy`만 설치돼 있다 |
| 2 | `export ROS_DOMAIN_ID=67` 등을 매번 입력 | 젯슨 `~/.zshrc`·`~/.bashrc`에 **이미 들어있다**(`ROS_DOMAIN_ID=67`, `F1_MAP=ifac_track`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`). 새 셸이면 그냥 된다 |
| 3 | `global_planning.yaml` 15번째 줄 `map_name`을 고쳐라 | **고칠 필요 없다.** `global_planning.launch.py`가 `map_name` 인자(기본값 `$F1_MAP`)로 yaml을 덮어쓴다 |
| 4 | `local_planning.launch.py`에 `fuck_f1.yaml`이 박혀 있으니 `reference_map:=`을 넘겨라 | **이미 `F1_MAP`을 쓴다**(32번째 줄 `os.environ.get('F1_MAP', 'map')`). 인자 안 넘겨도 된다 |
| 5 | `jq -r '.map_info_str.data' ...` | 젯슨에 **jq가 없다.** 아래 python3 대체 명령을 쓸 것 |

그리고 **옛 가이드에 없던 함정 3개**를 이 문서에서 다룬다 — MCL만 `F1_MAP`을 안 읽는 것(§2),
지도 추가 후 재빌드가 필요한 것(§3), `odom→base_link` TF 이중 발행(§9, **2026-07-29 해소**).

### 2026-07-29에 젯슨에서 바꾼 것

| 항목 | 이전 | 현재 | 근거 |
|---|---|---|---|
| `vesc.yaml` `speed_min/max` | ±33856 (= 8.0 m/s) | **±40000** (= 9.45 m/s) | WORKLOG 07-26 후속과제 5(게이트 1-C). 33856은 근거 없는 중간값이었다 |
| `mcl_launch.py` `publish_odom_base_tf` | real/bag에서 무조건 true | **런치 인자, 기본 false** | TF 이중 발행 해소 (§9) |

⚠️ `speed_max`는 **허용 상한을 푸는 것이지 목표 속도가 아니다.** 실제 속도는 컨트롤러
`max_speed`(실차 기본 5.0)가 정한다. 23250(=5.49 m/s)일 때는 VESC가 조용히 잘라서
`max_speed`를 올려도 소용이 없었던 것이 문제였다. 셰이크다운은 `max_speed`로 낮출 것.

---

## 1. 네트워크 확인

젯슨 zshrc에 이미 있으므로 **젯슨에서는 export가 필요 없다.** 본체에서만 맞춰주면 된다.

```bash
# 양쪽에서
printenv ROS_DOMAIN_ID          # 67 이어야 함
printenv RMW_IMPLEMENTATION     # rmw_fastrtps_cpp
date                            # 양쪽 시각 차이가 크면 TF가 안 보인다
```

⚠️ **`ROS_LOCALHOST_ONLY`는 Jazzy에서 폐기 예정(deprecated)이다.** 아직 동작은 하지만
대체 변수는 `ROS_AUTOMATIC_DISCOVERY_RANGE`다. 실차에서는 **둘 다 건드리지 않는 것**이
기본값(SUBNET)이라 가장 안전하다. 시뮬을 랩탑에 가둘 때만 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

### 가장 단순한 통신 테스트

젯슨:
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

⚠️ **젯슨은 멀티홈이다** — `wlP1p1s0`(10.1.1.3, 통신용), `enP8p1s0`(192.168.0.15, **라이다 전용**),
`l4tbr0`(192.168.55.1, USB). wifi AP가 멀티캐스트를 막으면 디스커버리가 안 붙는다. 그때는
팀원이 세팅한 Discovery Server를 쓴다 — 본체에서 `export ROS_DISCOVERY_SERVER="10.1.1.3:11811"`
(토픽 열거까지 하려면 `ROS_SUPER_CLIENT=true`도). 유선(피트)에서는 필요 없다.

---

## 2. 새 지도 파일 넣기

### 파일 위치

```
~/2026_IFAC/src/monte_carlo_localization/maps/new_track.png
~/2026_IFAC/src/monte_carlo_localization/maps/new_track.yaml
```

본체에서 전송:
```bash
scp new_track.png new_track.yaml \
    jetson:~/2026_IFAC/src/monte_carlo_localization/maps/
```

### YAML 내용 — `pgm`을 `png`로 고칠 것

```yaml
image: new_track.png      # ← 생성기가 .pgm 으로 써두면 반드시 고친다
mode: trinary
resolution: 0.025
origin: [-20.3043, -1.4273, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

`origin`과 `resolution`은 **global path 좌표와 직결**된다. 이 둘이 틀리면 스캔은 벽에 맞는데
경로만 어긋난다.

### 🔴 MCL만 `F1_MAP`을 안 읽는다

세 스택의 지도 지정 방식이 다르다. 확인한 결과:

| 스택 | `F1_MAP` 사용 | 지정 방법 |
|---|---|---|
| global_planning | ✅ | `map_name` 인자 기본값이 `$F1_MAP` |
| local_planning | ✅ | `reference_map` 기본값이 `particle_filter_cpp/maps/$F1_MAP.yaml` |
| **MCL** | ❌ | `map_name` 인자 기본값이 **`map` 고정** |

→ **MCL은 `map_name:=new_track`을 반드시 명시**해야 한다. 안 하면 MCL만 `map.yaml`을 보고
플래닝은 `new_track`을 봐서 "경로 ≠ 위치추정"이 된다(2026-07-25에 실제로 겪은 증상).

---

## 3. 🔴 지도를 넣었으면 반드시 재빌드

MCL은 `src/`가 아니라 **설치된 share 디렉터리**에서 지도를 읽는다:

```
mcl_launch.py:161  map_file_path = <pkg_share>/maps/<map_name>.yaml
```

`--symlink-install`로 빌드해도 **`install(DIRECTORY)`로 복사된 실제 파일**이라 심볼릭 링크가
아니다(확인함). `src/`에만 넣고 실행하면 **없는 지도**라며 실패한다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
MAKEFLAGS="-j4" colcon build --symlink-install --executor sequential \
  --packages-select particle_filter_cpp global_planning local_planning
source install/setup.zsh
```

⚠️ Orin Nano는 메모리가 좁다. `MAKEFLAGS="-j4" --executor sequential`을 빼면 OOM으로 죽는다
(젯슨 `cb` alias가 이미 이 설정이다).

설치 확인:
```bash
ls -lh "$(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/maps/new_track."{yaml,png}
```

---

## 4. 새 global path 준비

지도 PNG/YAML만 넣어도 `/global_waypoints`는 바뀌지 않는다. 별도로 생성한 raceline이 필요하다.

```
~/2026_IFAC/offline_trajectory_generator/output/new_track/global_waypoints.json
```

🔴 **재설치 직후 젯슨의 `offline_trajectory_generator/output/`은 비어 있다.** 본체에서 옮겨야 한다:

```bash
ssh jetson 'mkdir -p ~/2026_IFAC/offline_trajectory_generator/output/new_track'
scp global_waypoints.json global_waypoints.csv centerline.csv metadata.json \
    jetson:~/2026_IFAC/offline_trajectory_generator/output/new_track/
```

### JSON이 어느 지도로 만들어졌는지 확인 (jq 없이)

젯슨에 `jq`가 없으므로:
```bash
cd ~/2026_IFAC
python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['map_info_str']['data'])" \
  offline_trajectory_generator/output/new_track/global_waypoints.json
```
출력에 `new_track.yaml`이 보여야 한다. 다른 지도 이름이 나오면 **경로가 그 지도 좌표계라**
새 지도와 안 맞는다 — 다시 생성할 것.

---

## 5. 지도 이름 지정 (한 곳만)

`F1_MAP` 하나로 global·local이 따라오고, MCL만 인자로 준다.

```bash
# 젯슨 ~/.zshrc 에 이미 export F1_MAP=ifac_track 이 있다. 새 지도를 쓰려면:
export F1_MAP=new_track        # 이 터미널에만 적용
# 계속 쓸 거면 ~/.zshrc 의 값을 바꾼다
```

확인:
```bash
grep -nE 'output_base_dir|map_name' ~/2026_IFAC/src/global_planning/config/global_planning.yaml
# map_name: "map" 이 보여도 정상이다 — 런치 인자가 F1_MAP 으로 덮어쓴다
```

---

## 6. 기존 노드 안전하게 종료

```bash
ros2 node list | sort
jobs -l                          # 현재 터미널의 백그라운드 작업
ps -ef | grep '[r]os2 launch'    # 다른 터미널에서 띄운 것
kill -SIGINT <정확한_PID>
```

⚠️ `pkill ros2` / `killall` / 광범위한 `pkill -f`는 쓰지 말 것 — 필요한 노드까지 죽인다.

**하드웨어 bringup을 내릴 때 순서:**
1. 차량을 들어 올리거나 고정
2. E-stop 활성화 (조이스틱 **B**)
3. 자율 명령 중단
4. 모터 명령 0 확인 — `ros2 topic echo /commands/motor/speed --field data`
5. bringup에 Ctrl+C

---

## 7. 실행 순서

각 단계를 **통과하기 전에는 다음 단계로 넘어가지 않는다.** 특히 제어기는 마지막이다.

### 터미널 1 (젯슨) — 하드웨어 bringup

```bash
f110
```
alias 내용은 `cd ~/f1tenth_ws && source install/setup.zsh && ros2 launch f1tenth_stack bringup_launch.py`.
⚠️ 임의의 다른 launch를 쓰면 VESC 파라미터가 달라진다.

떠야 하는 노드: `/urg_node` `/vesc_driver_node` `/vesc_to_odom_node` `/ackermann_to_vesc_node`
`/ackermann_mux` `/drive_mode_manager` `/joy` `/static_baselink_to_laser`

검증 (다른 터미널):
```bash
ros2 topic hz /scan     # ≈ 40 Hz, publisher = /urg_node 하나
ros2 topic hz /odom     # ≈ 50 Hz, publisher = /vesc_to_odom_node 하나
ros2 topic hz /joy      # ≈ 20 Hz
ros2 topic echo /drive_mode --field data     # estop → (X 누르면) manual
```

정지 상태에서:
```bash
ros2 topic echo /odom --field twist.twist.linear.x    # 0.0
ros2 topic echo /odom --field twist.twist.angular.z   # 0 근처 (실측 4e-5 rad/s)
```

ℹ️ **odom 헤딩은 2026-07-28부터 실측 자이로 기반이다.** 기동 로그에
`yaw rate from MEASURED gyro`가 보여야 정상이고, `IMU stale` 경고가 반복되면 IMU가 끊긴 것이다.

ℹ️ 조이스틱은 **F710을 X 모드**로 둘 것(뒷면 스위치). 무선이라 절전에 들어가면 `/joy`가
끊기니 안 나오면 로지텍 버튼을 눌러 깨운다.

### 터미널 2 (젯슨) — MCL

```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py \
    mod:=real \
    map_name:=new_track \
    use_rviz:=false
```
**`use_rviz:=false`가 중요하다** — RViz는 본체에서 띄운다(젯슨 렌더 부하 0).

검증:
```bash
ros2 param get /particle_filter_map_server yaml_filename   # .../maps/new_track.yaml
ros2 topic hz /pf/pose/odom
ros2 topic info /map --verbose                             # publisher = /particle_filter
```

`map_server`가 중복으로 뜨면(`/map` publisher 2개) 끈다:
```bash
ros2 service call /lifecycle_manager_particle_filter/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"
ros2 lifecycle get /particle_filter_map_server              # unconfigured [1]
```
ℹ️ `start_map_server` 인자 기본값이 `auto`라 이미 떠 있으면 알아서 건너뛴다. 보통은 불필요.

### 본체 PC — RViz2

```bash
source /opt/ros/jazzy/setup.zsh
source ~/2026_IFAC/install/setup.zsh
export ROS_DOMAIN_ID=67
rviz2 -d "$(ros2 pkg prefix particle_filter_cpp)/share/particle_filter_cpp/rviz/particle_filter.rviz"
```

Fixed Frame `map`, display: Map `/map` / LaserScan `/scan` / Odometry `/pf/pose/odom` /
PoseArray `/pf/viz/particles` / TF.

**초기 위치 지정**: 차량 정지 → `2D Pose Estimate` → 실제 위치 클릭 → 드래그로 방향 →
**스캔이 지도 벽과 겹치는지 확인**.

판정 기준:
- 스캔이 벽과 **안 맞음** → 지도 / initialpose / laser TF / MCL 문제
- 스캔은 맞는데 **global path만 어긋남** → `global_waypoints.json` 또는 지도 `origin`/`resolution` 문제

### 터미널 3 (젯슨) — global planning

```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch global_planning global_planning.launch.py
```
⚠️ **반드시 `~/2026_IFAC`에서 실행할 것** — `output_base_dir`이 상대경로라 실행 위치 기준으로 풀린다.

검증:
```bash
ros2 param get /global_trajectory_publisher_node map_name    # new_track
ros2 topic info /global_waypoints --verbose                  # publisher 1개
ros2 topic info /car_state/frenet/odom --verbose             # /frenet_odom_node
```
RViz에 MarkerArray 추가: `/global_waypoints/markers`, `/trackbounds/markers`,
`/centerline_waypoints/markers` → 지도와 정확히 겹쳐야 한다.

### 터미널 4 (젯슨) — local planning

```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch local_planning local_planning.launch.py simulator:=false
```
`reference_map`은 `F1_MAP`에서 자동으로 풀린다(인자 불필요).

```bash
ros2 param get /local_planning_map_server yaml_filename      # .../maps/new_track.yaml
ros2 topic info /local_planning/reference_map --verbose
```
ℹ️ `/local_planning_map_server`라는 노드 이름은 옛 가이드에서 그대로 가져온 것으로 **아직
라이브로 확인하지 못했다.** 안 나오면 `ros2 node list | grep -i map`으로 실제 이름을 찾을 것.
RViz에 Map display로 `/local_planning/reference_map`을 추가하면 `/map`과 겹쳐야 한다.

### 터미널 5 (젯슨) — state machine

```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
ros2 topic echo /state
```

### 터미널 6 (젯슨) — wpnt_publisher

```bash
cd ~/2026_IFAC && source install/setup.zsh
ros2 run wpnt_publisher wpnt_publisher
ros2 topic hz /local_waypoints          # publisher = /wpnt_publisher 하나
```

**`/local_waypoints`가 안 나올 때** — 세 입력이 다 있어야 GLOBAL 상태에서 발행된다:
```bash
ros2 topic echo /global_waypoints --once
ros2 topic echo /car_state/frenet/odom --once
ros2 topic echo /state --once
```
특히 `wpnt_publisher`는 `/car_state/frenet/odom`의 `child_frame_id`를 **웨이포인트 인덱스**로
쓴다. `'0'` `'125'` 같은 정수 문자열이어야 하고, `''`/`'base_link'`면
`Invalid child_frame_id for index` 경고와 함께 발행이 멈춘다.
```bash
ros2 topic echo /car_state/frenet/odom --field child_frame_id
```

### 터미널 7 (젯슨) — control (마지막)

**실행 전 체크리스트**: 바퀴 들거나 스탠드 / E-stop 준비 / `/scan`·`/odom` 정상 /
`/pf/pose/odom` 정상 / 경로와 지도 일치 / TF 정상(§9) / state machine 정상.

```bash
source ~/f1tenth_ws/install/setup.zsh      # ⚠️ 두 워크스페이스 다 필요 (순서 무관)
source ~/2026_IFAC/install/setup.zsh
cd ~/2026_IFAC
ros2 launch f1tenth_control control_real.launch.py
```

셰이크다운은 캡을 낮춰서:
```bash
ros2 launch f1tenth_control control_real.launch.py max_speed:=2.5 min_speed:=0.5
```

```bash
ros2 topic echo /drive
```

ℹ️ 실차의 수동/자율/E-stop Mux는 `f1tenth_stack`의 `drive_mode_manager`+`ackermann_mux`가
담당한다. 우리 `drive_source_selector`는 **MAP↔MPPI 선택(RB 버튼)만** 한다.
조이스틱: **A**=자율, **B**=E-stop, **X**=수동, **RB**=MAP↔MPPI.

---

## 8. 실행 순서 요약

```
젯슨 T1  f110 bringup        → /scan 40Hz, /odom 50Hz, /joy 20Hz
젯슨 T2  MCL (map_name:=)    → /map, /pf/pose/odom
본체     RViz2               → 2D Pose Estimate → 스캔·벽 정합 확인
젯슨 T3  global_planning     → 경로·지도 정합 확인
젯슨 T4  local_planning      → reference_map 확인
젯슨 T5  state_machine       → /state
젯슨 T6  wpnt_publisher      → /local_waypoints
젯슨 T7  control_real        → 전부 통과한 뒤에만
```

---

## 9. TF 책임 구조 (2026-07-29 이중 발행 해소됨)

```
particle_filter     → map → odom
vesc_to_odom_node   → odom → base_link
static TF           → base_link → laser
```

**예전엔 `odom→base_link`를 둘이 동시에 냈다.** `mcl_launch.py`가 real/bag 모드에서
`publish_odom_base_tf: true`였고 `vesc.yaml`도 `publish_tf: true`였다. TF는 한 링크에
publisher가 하나여야 하고, 둘이면 수신자가 도착 순서에 따라 다른 값을 봐서 위치가 조용히 흔들린다.

→ `publish_odom_base_tf`를 **런치 인자(기본 `false`)로 분리**했다. 오도메트리 소스가
`odom→base_link`를 내는 ROS 관례를 따른 것이고, odom을 믿어도 되는 근거는 07-28 검증이다
(34 m 경로 폐합 15.6 cm, 2바퀴 누적 헤딩 +717.6°/참 720°, 자이로 스케일 오차 +0.06%).

⚠️ **`odom→base_link`를 내는 쪽이 반드시 살아 있어야 한다** — 실차는 f110 bringup
(`vesc.yaml`의 `publish_tf: true`), 시뮬은 gym_bridge. 둘 다 없는 상황(그 TF가 없는 bag 재생)
에서만 켠다:
```bash
ros2 launch particle_filter_cpp mcl_launch.py mod:=bag publish_odom_base_tf:=true
```

주행 전 확인:
```bash
ros2 topic info /tf --verbose            # odom→base_link publisher 가 하나인지
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

⚠️ MCL은 팀 공용 패키지다. 이 변경은 젯슨 `~/2026_IFAC`(`jazzy_port_backup`)에만 있고
**팀 repo에는 아직 반영 안 됐다** — 공유 시 합의 필요.

---

## 10. rosbag 녹화

⚠️ **`-s sqlite3`를 반드시 붙일 것.** Jazzy는 기본 storage가 mcap일 수 있는데 우리
`tools/bag_analyzer`는 `.db3`만 파싱한다.
⚠️ **녹화는 젯슨에서** 한다(무선으로 빼면 `/scan`이 드롭된다).

control 실행 **전에**:
```bash
source ~/f1tenth_ws/install/setup.zsh
source ~/2026_IFAC/install/setup.zsh
mkdir -p ~/rosbags && cd ~
ros2 bag record -s sqlite3 -o ~/rosbags/run_$(date +%m%d_%H%M%S) \
  /drive_autonomous /drive_mppi /drive /joy /drive_mode /mppi_active /estop_lock \
  /pf/pose/odom /odom /tf /tf_static /scan /sensors/imu/raw /imu/data \
  /global_waypoints /local_waypoints \
  /commands/motor/speed /commands/motor/brake /commands/servo/position /sensors/core
```

확인:
```bash
LATEST=$(ls -td ~/rosbags/run_* | head -1)
ros2 bag info "$LATEST"
```

---

## 11. 문제별 첫 확인 지점

| 증상 | 먼저 볼 것 |
|---|---|
| 본체에서 젯슨 토픽이 안 보임 | `ROS_DOMAIN_ID` 양쪽 67 / 같은 서브넷 / `ros2 daemon stop && start` / AP client isolation / VPN·방화벽 |
| 스캔이 벽과 안 맞음 | 지도, `2D Pose Estimate`, `base_link→laser` static TF, MCL |
| 스캔은 맞는데 경로만 어긋남 | `global_waypoints.json`의 `map_info_str`, 지도 `origin`/`resolution` |
| MCL만 다른 지도를 봄 | `map_name:=` 을 안 넘겼다 (MCL은 `F1_MAP`을 안 읽는다, §2) |
| 지도를 넣었는데 없다고 함 | 재빌드 안 함 (§3) |
| `/local_waypoints` 무발행 | `child_frame_id`가 정수 문자열인지 (§7 T6) |
| `/joy` 무발행 | F710 절전 — 로지텍 버튼으로 깨우기. `input` 그룹은 새 로그인 세션에만 적용됨 |
| 자율 진입 시 급발진 | `engage_gate_enable`(기본 true) 동작 확인, `/drive_mode`가 실제로 발행되는지 |
| odom 헤딩이 이상 | 기동 로그에 `yaw rate from MEASURED gyro` 있는지, `IMU stale` 반복 여부 |

---

## 부록 — 확인에 쓴 근거 (2026-07-29)

- 양쪽 `/opt/ros`에 **jazzy만** 설치, 시각 동기 확인(차이 3초)
- 젯슨 wifi `10.1.1.3` / 본체 `10.1.1.24`, ping 2.8 ms
- `librcl.so`에 `ROS_LOCALHOST_ONLY`·`ROS_AUTOMATIC_DISCOVERY_RANGE` **둘 다 존재**
  → 전자는 아직 동작하지만 폐기 예정
- `mcl_launch.py:115` `map_name` 기본값 `'map'`, 파일 내 `F1_MAP` 참조 **0건**
- `global_planning.launch.py:25` `default_value=os.environ.get("F1_MAP", "map")`,
  `parameters=[params, {"map_name": LaunchConfiguration("map_name")}]` → yaml을 덮어씀
- `local_planning.launch.py:32` `os.environ.get('F1_MAP', 'map') + '.yaml'`
- `mcl_launch.py:161` 지도는 `<pkg_share>/maps/`에서 로드, share의 지도는 심볼릭 링크가
  아닌 실제 복사본
- `mcl_launch.py:188-192` real 모드에서 `publish_map_odom_tf`·`publish_odom_base_tf` **둘 다 true**
- 젯슨에 `jq` 미설치
- 2026_IFAC 12개 패키지 Jazzy 빌드 성공(11분 22초, 실패 0)
