# 작업 로그 (WORKLOG)

2026 IFAC F1TENTH 제어 파트 작업 진행상황 기록.

---

## 2026-07-12 (3) — control_real.launch.py에 ackermann_to_vesc_node 배선(최종 /drive → VESC 명령 어댑터)

### 배경
`/drive`(Mux 최종 출력)가 실제 VESC 모터/서보에 도달하려면 AckermannDriveStamped를 VESC 명령
(commands/motor/speed=ERPM, commands/servo/position)으로 변환하는 `ackermann_to_vesc_node`가
떠 있어야 하는데, 조사 결과 이 노드가 워크스페이스 어디에서도 launch되지 않고 있었음(주석상
"하드웨어 브링업" 전제로만 언급). 노드 자체는 `vesc/vesc_ackermann`에 이미 컴파일돼 있어 새로
짤 필요는 없고 launch 배선만 필요.

### 변경 (`control_real.launch.py`에만 추가, sim엔 불필요)
`vesc_ackermann` 패키지의 `ackermann_to_vesc_node`를 `Node`로 추가:
- **⚠️ remapping `ackermann_cmd`→`/drive` 필수**: 노드는 `ackermann_cmd`를 구독하는데 우리
  최종 토픽은 `/drive`라, remap 안 하면 아무도 발행 안 하는 토픽을 구독해 조용히 무동작.
- 파라미터는 `vesc_ackermann/launch/ackermann_to_vesc_node.launch.xml` 레퍼런스 값:
  `speed_to_erpm_gain=4614.0`(vesc_to_odom과 반드시 동일 — 안 그러면 odom 속도 스케일 깨짐),
  `speed_to_erpm_offset=0.0`, `steering_angle_to_servo_gain=-1.2135`,
  `steering_angle_to_servo_offset=0.5304`(서보 링키지 기준값, 조향 방향/중립 어긋나면 튜닝).
- 기존 `ackermann_to_vesc_node.launch.xml`을 IncludeLaunchDescription으로 안 쓴 이유: 그 xml엔
  remapping이 없어 그대로 include하면 `ackermann_cmd` 구독으로 무동작. aeb_node처럼 직접 `Node`로
  선언해 remap을 붙이는 게 깔끔(파일 내 기존 패턴과 일치).

### ⚠️ 남은 것 (다음 실차 세션)
- **이 노드만으론 모터 안 돔** — `vesc_driver_node`(commands/motor/speed를 시리얼로 VESC에 전달)가
  별도로 떠야 함. 현재 워크스페이스에 vesc_driver를 자동 기동하는 통합 브링업 launch가 없음
  (`vesc/vesc_driver/launch/vesc_driver_node.launch.py`를 수동으로 띄워야 함). 필요 시 통합
  브링업 launch 신설 검토.
- servo gain/offset(-1.2135/0.5304)은 표준 F1TENTH 레퍼런스값 — 실제 조향 방향/중립 실측 검증 필요.

### 검증
- `colcon build` 클린, `ros2 pkg executables vesc_ackermann`로 실행파일 확인,
  `control_real.launch.py --show-args` 정상 파싱(exit 0). 실제 하드웨어 구동 검증은 실차 세션에서.

---

## 2026-07-12 (2) — 대시보드에 트리거→속도 표시 추가, base_max_accel/base_max_decel 터미널 인자로 승격

### 배경
"조이스틱 트리거 입력에 따른 속도도 대시보드에 보이게 해달라"는 요청과, 이어서 "수동 풀스로틀 시
젯슨이 받는 최대속도가 어느 코드 기준이냐"는 질문에 답하는 과정에서 `base_max_accel`/
`base_max_decel`(곡률 사전감속 제동거리 계산에 직접 쓰이는 값)을 짚었고, 사용자가
`_control_common.py:139-140`을 보며 "여기서 `base_max_accel`은 못 바꾸냐"고 확인 — 그 줄은
함수 인자를 그대로 통과시키는 자리라 값이 각 진입점 launch 파일에 흩어져 있고, 터미널
`:=` 오버라이드도 안 되는 상태였음. 직전 턴에서 `base_max_decel=8.0`이 `max_lateral_accel`
마찰피크(~6.7) 대비 낙관적일 수 있다는 점도 이미 지적된 상태라, 사용자가 "터미널 인자로 승격"을
선택.

### 변경
1. **대시보드**: `joy_teleop_monitor.cpp`의 `display_dashboard()` `[Joystick Input]` 섹션에
   `Commanded Speed : X.XXX m/s (limit Y.YYY m/s)` 줄 추가. `target_speed_`는 모드와 무관하게
   `joy_callback`에서 항상 계산되므로 AUTONOMOUS 중에도 트리거를 그대로 쓰면 몇 m/s가 나갈지
   미리 확인 가능. `launch/dashboard.launch.py` 자체는 `teleop_dashboard_node`만 띄우는
   런치파일이라 내용에 관여하지 않아 변경 불필요.
2. **`base_max_decel`**: sim/real 값이 동일(8.0)해서 `_control_common.py`의
   `declare_common_args()`에 공용 `DeclareLaunchArgument`로 추가, `build_control_map_node()`
   내부 리터럴 `8.0`을 `LaunchConfiguration('base_max_decel')`로 교체.
3. **`base_max_accel`**: sim(4.0)/real(6.5) 값이 달라 공용 선언 대신 각 진입점 파일에
   `DeclareLaunchArgument`를 개별 선언(`control_real.launch.py` 기본 6.5, `control_sim.launch.py`
   기본 4.0 — 후자는 `DeclareLaunchArgument`/`LaunchConfiguration` import가 없어 새로 추가)하고
   `build_control_map_node(base_max_accel=...)` 호출부를 `LaunchConfiguration('base_max_accel')`로
   교체. 값 자체(4.0/6.5/8.0)는 변경하지 않음 — 구조만 터미널에서 즉시 조정 가능하게 승격.
4. `CLAUDE.md` "실차 튜닝 파라미터" ①번 표에 두 파라미터 추가(+`base_max_decel`이 실측 전
   추정값이라는 경고 문구 포함), ②번 그룹에서 `base_max_decel` 제거 + `base_max_accel` 예외
   문단 삭제(더 이상 예외가 아님).
5. `~/2026_IFAC/f1tenth_control/`(빌드 대상) 4개 사본 모두 diff 확인 후 동일 반영(격차 없음 확인).

### 검증
- `colcon build --packages-select f1tenth_control` 클린 빌드.
- `ros2 launch f1tenth_control control_real.launch.py --show-args` / `control_sim.launch.py
  --show-args`로 `base_max_accel`/`base_max_decel` 인자 노출 및 기본값(6.5/4.0, 8.0) 확인.

---

## 2026-07-12 — joy_teleop_monitor 기본 시작 모드 = MANUAL을 "의도된 동작"으로 확정 + 문서/로그 정합화

### 배경
"조이스틱만 연결하고 `control_real.launch.py`를 켜면 자율주행은 안 돼도 수동조작은 되냐"는
질문을 계기로 `joy_teleop_monitor.cpp`를 추적. `is_simulation` 파라미터 자체 기본값은
`false`(원 설계: 실차=AUTONOMOUS로 시작 + 수동 명령 포워딩 차단, CLAUDE.md도 그렇게 문서화돼
있었음)인데, 07-11 `_control_common.py` 리팩터([[control-launch-common-refactor]]) 이후
`build_joy_teleop_monitor()`가 `'is_simulation': True`를 파이썬 리터럴로 하드코딩해 sim/real
런치 양쪽 모두 이 값을 받고 있었음 — 실차도 MANUAL로 시작하고 수동 명령이 실제로 `/drive`에
포워딩되는 상태.

### 결정
처음엔 "안전 규칙이 리팩터 중 실수로 깨진 것"으로 보고 사용자에게 확인 질문을 던졌으나,
**사용자가 "노드 켜면 기본이 조이스틱 수동 조작이어야 한다, 그렇게 유지되게 만들어라"고 명시적으로
확정** — 버그가 아니라 원하는 동작. 로직은 이미 그렇게 동작 중이라 변경 불필요, 다만 코드/문서가
"우연히 하드코딩된 것"처럼 보여 나중에 실수로 되돌리거나(is_simulation=false로 "수정"), 실차
구동 중 터미널에 "SIMULATION (기본 수동 제어)"라는 혼란스러운 로그가 찍히는 문제가 남아있었음.

### 변경 (로직 변경 없음 — 문구/주석/문서만 실제 동작에 맞게 정정)
- `control_code/joy_teleop_monitor.cpp` 시작 로그: `is_simulation_` 대신 실제로 확정된
  `current_mode_`를 기준으로 "MANUAL (조이스틱 수동 대기)" / "AUTONOMOUS (자율주행)"를 찍도록
  변경 — 실차에서도 "SIMULATION"이 찍히던 오해 소지 제거.
- `launch/_control_common.py`의 `build_joy_teleop_monitor()`: `'is_simulation': True`가
  실차 포함 의도적 고정값이며 임의로 되돌리지 말라는 주석 추가.
- `CLAUDE.md`의 `joy_teleop_monitor` "안전 규칙" 문단을 실제/의도 동작(기본 MANUAL 시작,
  LB로 AUTONOMOUS 전환)으로 정정.
- `~/2026_IFAC/f1tenth_control/`(빌드 대상) 사본 3곳 모두 diff 확인 후 동일 반영(격차 없음 확인).

### 검증
- `colcon build --packages-select f1tenth_control` 클린 빌드 확인(문자열/주석 변경만이라 로직
  회귀 없음).

---

## 2026-07-11 (7) — 젯슨 배포 대상을 `simul_practice` → `main`으로 전환, `f1tenth_control` main에 첫 등록

### 배경
젯슨(실차)이 pull할 팀 레포 브랜치를 `main`으로 옮기기로 함(팀 전체가 planning/perception도 곧
main에 통합 예정). `main`엔 이전까지 `f1tenth_control`이 아예 없었고(`git ls-tree -r origin/main`
으로 직접 확인), `vesc_driver`/`urg_node2`/`state_machine`도 없어 그 자체로는 실차 하드웨어를
못 돌리는 상태였음(팀이 곧 올릴 예정, 사용자 확인 완료).

### 문제 — 단순 브랜치명 변경이 위험한 이유
기존 `f1up` alias는 `~/2026_IFAC`(현재 `simul_practice` 체크아웃, `CLAUDE.md`가 문서화한 실제
빌드/시뮬레이션 워크스페이스)에서 직접 `git checkout <branch>`로 전환 후 커밋·푸시했음. 이 디렉토리를
그대로 `main`으로 전환하면 `simul_practice`에만 있는 `state_machine`/`vesc`*/`urg_node2` 등이
디스크에서 즉시 사라져 로컬 빌드/시뮬레이션이 깨짐.

### 해결 — 별도 git worktree(`~/2026_IFAC_main`)로 배포 전용 작업 트리 분리
`~/2026_IFAC`(simul_practice, 빌드용)는 절대 건드리지 않고, `git worktree add ~/2026_IFAC_main main`
으로 `main`만 체크아웃한 별도 디렉토리를 신설(빌드 안 함, 순수 git add/commit/push 경유지).
`f1up` alias 수정: 기존 `~/2026_IFAC/f1tenth_control/`로의 rsync(로컬 빌드 환경 최신화용)는
그대로 유지하고, 그 뒤에 `~/2026_IFAC_main/f1tenth_control/`로도 rsync + `git add` + commit +
`push origin main`을 추가(레포 **루트 바로 아래** 배치 — `planning/global_planner`처럼 완결된
실행 노드 패키지 취급, 사용자 확인). 더 이상 `simul_practice`로는 푸시하지 않음.

### 검증
- `git worktree list`로 `~/2026_IFAC_main`이 `main`에 정상 연결 확인.
- `f1up` 1회 실행 → `~/2026_IFAC_main/f1tenth_control/` 신규 생성, 커밋(`e26f005`) 후
  `origin/main`에 정상 푸시 확인. `~/2026_IFAC`는 여전히 `simul_practice` 체크아웃 유지,
  `state_machine`/`vesc`/`urg_node2` 디스크 유지 확인(로컬 빌드 환경 무영향).
- 실행 중 `control_code/MAP_controller_reference.py`가 이미 디스크에서 삭제(unstaged)된 상태를
  발견 — 사용자 확인 결과 의도된 삭제였음, `f1up`으로 그대로 커밋. `CLAUDE.md` "참고/비활성 자산"
  목록에서도 해당 항목 제거.

---

## 2026-07-11 (6) — control_map_node 미노출 파라미터 15개 전부 launch 인자로 승격

### 배경
CLAUDE.md 정리 중 발견한 "③ 어디에도 노출 안 됨" 그룹(15개) — `control_MAP.cpp`엔 이미
`declare_parameter`/`get_parameter`로 코드가 다 있는데 launch 쪽에서 값을 안 넘겨 코드
기본값만 쓰이던 파라미터들 — 을 `_control_common.py`에 추가해달라는 요청. **C++ 코드는
전혀 안 건드림**, 파라미터 플러밍은 이미 있었으므로 launch 계층에서 인자로 노출만 하면 끝.

### 대상 15개
조향 스케일러(`acceleration_scaler_for_steering`, `deceleration_scaler_for_steering`,
`start_scale_speed`, `end_scale_speed`, `downscale_factor`, `speed_lookahead`,
`speed_lookahead_for_steering`), 롤 ESC(`max_roll_limit`, `decel_attenuation`),
경로소스/장애물회피(`local_fresh_timeout`, `obstacle_avoid_enable`, `obstacle_cone_halfangle`,
`obstacle_trigger_dist`, `obstacle_margin`, `obstacle_avoid_hold_cycles`).

### 구현
`_control_common.py`의 기존 `yaw_rate_gain` 패턴 그대로 반복: `declare_common_args()`에
15개 `DeclareLaunchArgument`(코드 기본값과 동일한 default) 추가 + `build_control_map_node()`
파라미터 dict에 `LaunchConfiguration('...')` 참조 추가. **`obstacle_avoid_hold_cycles`만
int라서** `joy.launch.py`의 `device_id` 선례대로 `ParameterValue(..., value_type=int)`
명시 캐스팅 필요(bare 문자열→int 자동추론이 불안정할 수 있어서). repo·2026_IFAC 양쪽에
동일 적용(2026_IFAC의 `waypoint_topic`/`controller_type` 격차는 그대로 보존).

### 검증
- `colcon build` 클린 성공, `--show-args`로 15개 인자 전부 올바른 기본값으로 노출 확인.
- 스모크 테스트: `obstacle_avoid_hold_cycles:=20`, `max_roll_limit:=0.2`로 오버라이드해
  기동 후 `ros2 param get`으로 실제 반영값 확인(int 20/double 0.2 정상, 미변경
  `decel_attenuation`은 기본값 0.6 정상) — 타입 캐스팅·오버라이드 모두 정상 동작.

### CLAUDE.md 갱신
"실차 튜닝 파라미터" ①번(터미널 인자) 표에 15개 추가, ③번 그룹(빈 그룹이 됨) 삭제.
향후 MPPI 노드가 붙으면 그 파라미터도 동일 패턴(`_control_common.py`에 추가, `control_MPPI.cpp`
코드는 안 건드림)으로 노출할 것이라는 메모 추가.

---

## 2026-07-11 (5) — control_sim/control_real 런치파일 공용 로직 추출 (`_control_common.py`)

### 배경
곧 실차 위주로 주행/튜닝할 예정이라 두 런치파일 관리 부담을 줄여달라는 요청. 조사해보니
"자율/수동 시작 차이뿐"이라는 전제는 틀렸음 — `force_autonomous`/`is_simulation`은 이미
두 파일에서 동일했고, 실제 차이는 `odom_topic`/`max_speed`/`max_lateral_accel`/
`base_max_accel`/`lookup_table_file` 값 차이 + **AEB·조이스틱드라이버·sim_imu_bridge_node
포함 여부**라는 구조적 차이였음. 나머지(`wheelbase, l1_gain, l1_distance, t_clip_min/max,
lateral_error_coeff, min_speed, curvature_lookahead_count, wall_safety_margin, use_imu,
yaw_rate_gain, curvature_ff_blend, heading_damping_gain`, `joy_teleop_monitor` 전체 설정)는
100% 중복 — 이게 진짜 드리프트 위험 지점(오늘 `yaw_rate_gain`도 두 파일에 손으로 미러링해야 했음).

### 채택안 — 진입점 파일 2개 유지 + 공용 헬퍼 추출 (완전 병합은 기각)
완전 병합(단일 파일 + `is_real` 같은 불리언 인자 분기)도 검토했으나, 인자를 깜빡하면 실차가
AEB·조이스틱 없이·시뮬 속도 캡으로 기동될 위험이 생겨 기각. 대신:
- **신규 `launch/_control_common.py`**: `declare_common_args()`(공통 인자 선언),
  `build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
  lookup_table_file='')`(공통 파라미터 고정 정의, 환경별 값만 인자로), `build_joy_teleop_monitor()`
  (완전 동일 설정) 제공. 런치파일이 아니라 순수 헬퍼 모듈.
- `control_sim.launch.py`/`control_real.launch.py`는 각자 환경별 값·AEB/조이스틱/
  sim_imu_bridge_node 포함 여부만 결정하고 공용 함수 호출로 축약.
- **모듈 임포트**: `ros2 launch`가 launch 파일을 스크립트로 실행해 패키지 상대임포트가 불안정할
  수 있어, 각 진입점 파일 상단에 `sys.path.insert(0, os.path.dirname(__file__))` 후
  `import _control_common as common`(plain top-level)으로 방어적으로 처리.
- 2026_IFAC 사본은 `waypoint_topic`(양쪽 공통) + `controller_type`(**sim 전용, real엔 원래
  없음 — 이 비대칭도 그대로 보존**, `build_control_map_node`에 `controller_type=None` 옵션
  인자로 처리) 격차가 있어 repo와 다른 내용의 `_control_common.py`를 별도 작성(diff 후 반영,
  [[steering-control-node-sync-gap]] 패턴 유지).
- CMakeLists.txt 변경 불필요(`install(DIRECTORY launch ...)`가 신규 파일도 자동 포함).

### 검증
- `colcon build --packages-select f1tenth_control` 클린 성공.
- `--show-args`로 리팩터 전후 인자 목록 동일 확인(sim 4개/real 8개, 회귀 없음).
- 스모크 테스트: `control_sim.launch.py force_autonomous:=true`(gym_bridge와 함께) →
  `sim_imu_bridge_node`/`control_map_node`(LUT 로드)/`joy_teleop_monitor` 전부 정상 기동.
  `control_real.launch.py` 단독 실행 → `joy_node`/`control_map_node`/`joy_teleop_monitor`/
  `aeb_node`(설정 로그 확인) 전부 정상 기동, 임포트 에러 없음.

### 앞으로
공통 파라미터를 추가/변경할 땐 `_control_common.py` 한 곳만 고치면 sim/real 양쪽에 반영됨.
AEB·조이스틱·sim_imu_bridge_node 같은 안전/환경 구조 차이는 계속 각 진입점 파일에 직접 유지.

---

## 2026-07-11 (4) — VESC 내장 IMU 축/부호·단위 조사 (실차 없이 코드로 확인, 조사 전용)

### 배경
VESC Tool에서 IMU 영점·세팅 후 롤/피치/요 부호를 확인했는데(요·롤은 우측+, 피치는 nose-up+),
"ROS에서는 방향이 반대로 인식된다"는 다른 팀 얘기를 듣고 사실 확인 요청. 실차가 없는 상태라
`~/2026_IFAC/vesc/vesc_driver` 소스를 직접 읽어 코드로만 조사(하드웨어 검증 없이 가능한 부분).

### 1. 축 반전/스왑 — vesc_driver는 순수 passthrough, ROS가 뒤집지 않음
`vesc_driver.cpp`가 `VescPacketImu`의 `acc_x/y/z`, `gyr_x/y/z`, `q0..q3`를 `sensor_msgs/Imu`의
`linear_acceleration`, `angular_velocity`, `orientation`에 **1:1 그대로** 대입(부호 반전·축
스왑 전혀 없음). `vesc_packet.cpp`의 게터도 전부 raw 값 그대로 리턴. firmware의 장착보정용
`imu_conf.rot_roll/pitch/yaw`(appconf.xml에 전부 0)도 드라이버 코드가 아예 안 읽음 — 순수
firmware 개념. **결론: "ROS가 축을 뒤집는다"는 이 드라이버 기준으론 사실 아님.** 만약 실제로
부호가 안 맞다면 그건 ROS 변환이 아니라 VESC 자체(펌웨어)의 roll/pitch/yaw 부호 관례와
ROS REP-103(오른손좌표계: yaw+=반시계, pitch+=nose-down, roll+=우측다운) 관례가 원래부터
다른 것 — VESC 펌웨어 소스가 이 저장소엔 없어 정확한 관례는 코드로 100% 확정 불가, 실측 필요.

### 2. ⚠️ 확정 버그 발견 — 자이로 deg/s→rad/s 변환 누락
`vesc_packet.cpp`: `roll()/pitch()/yaw()`는 raw(라디안)를 `*180/M_PI`로 deg 변환(디버그용)
하는데, `gyr_x/y/z()`는 `return gyr_x_; // deg/s` 주석과 함께 **변환 없이 그대로 리턴**.
`getFloat32Auto`는 순수 비트단위 float 디코더라 단위 변환 없음 확인. 즉 firmware가 각도는
라디안, 각속도는 deg/s로 보내는데 드라이버가 그대로 흘려보내 **`/imu/data.angular_velocity`가
REP-103(rad/s) 위반, 약 57.3배 스케일 버그**로 추정(경로/코드 추적으로 확정, 실측으로 최종
확인 필요).

**직결 위험**: 오늘 배선한 `calculate_yaw_rate_correction`(요레이트 카운터스티어)이
`filtered_yaw_rate_`(=angular_velocity.z, 이 버그 영향권)를 `v·tanδ/L`(진짜 rad/s)과 직접
비교 → 이 버그 상태로 실차에서 `use_imu=true` 켜면 "실측" 요레이트가 ~57배 부풀려져 보정항이
과도하게 튈 위험(안전 문제 소지). **롤 ESC(`calculate_roll_ratio`)는 자이로가 아니라 쿼터니언
기반 각도(`filtered_roll_`)를 쓰므로 이 버그와 무관 — 정상 동작 예상.**

### 3. "90도 장착 회전" 보정 — 가속도만 됐고 각속도는 미보정 (재확인)
저장소 전체 grep 결과 장착 회전 보정은 `-linear_acceleration.y`(LUT 종가속도용) **한 곳뿐**,
`angular_velocity.x/z`(롤레이트/요레이트)는 무보정 — 07-05 로드맵의 "⚠️ 축 검증" 항목이
여전히 미해결. 요축 90도 마운트 가설이 맞다면 현재 "롤레이트"로 쓰는 `gyr_x`가 실제론 차체
피치레이트일 가능성(추정, 실측 전 단정 불가).

### 결정 — 이번엔 코드 수정 안 함
`vesc_driver`가 팀 공용 vendored 패키지라, deg/s→rad/s 수정은 **실측 확인 후로 보류**(사용자
결정). 이번 세션은 조사·문서화만.

### ⚠️ 다음 실차 세션 체크리스트 (07-05 로드맵 "축 검증" 항목을 아래로 구체화·대체)
1. VESC 유닛을 손으로 각 축 회전시키며 VESC Tool 표시값과 `ros2 topic echo /imu/data`를
   동시에 비교 — 부호 일치 여부(REP-103과 다를 것으로 예상되는 pitch·yaw 우선 확인).
2. 알려진 각속도(대략 90도/초)로 손으로 굴려 `angular_velocity.z`가 ~1.57(정상, rad/s) vs
   ~90(버그 확정, deg/s) 중 어느 쪽인지 확인 → 버그 확정 시 `vesc_driver`의 `gyr_x/y/z()`
   게터(또는 `vesc_driver.cpp` 대입부)에 `* M_PI / 180.0` 추가, 팀 공지 후 반영.
3. 요축 90도 마운트 가설 검증: 차체를 롤/피치 각각 흔들었을 때 `angular_velocity.x`(현재
   "롤레이트"로 쓰는 축)가 실제 롤에 반응하는지, 혹은 피치에 반응하는지 확인.
4. 이 유닛 단품(차 전체 아니어도 OK)만 있으면 위 1·2번은 트랙 없이도 검증 가능.

---

## 2026-07-11 (3) — IMU 보정(use_imu) 시뮬/실차 런치 디폴트 활성화 + sim_imu_bridge_node 신설

### 배경
직전 세션에서 요레이트 카운터스티어를 배선·검증했으나, `control_sim.launch.py`는
`use_imu: False`(시뮬엔 IMU 없음)로 고정돼 있어 기본 실행에선 요레이트 보정은 물론
롤 인지 ESC까지 전부 비활성 상태였음. "시뮬/실차 런치파일에서 IMU 보정을 디폴트로"
요청에 따라 두 런치파일 모두에서 `use_imu` 경로가 기본으로 켜지도록 정리.

### ⚠️ 짚어야 했던 함정: `use_imu:=true`를 시뮬에 그냥 켜면 안 됨
시뮬(gym_bridge)은 `/imu/data`를 발행하지 않음 → `use_imu:=true`만 켜면 `StabilityController`가
IMU 콜백을 한 번도 못 받아 `filtered_yaw_rate_`가 항상 0으로 고정됨. 이 상태에서
`calculate_yaw_rate_correction`은 "실측 요레이트=0"으로 오인해 **명령 조향각이 의도하는 기대
요레이트 전체를 매번 오차로 착각해 조향에 계속 더함** — 단순 비활성이 아니라 **적극적으로
잘못된 조향 바이어스**가 걸리는 위험한 상태. 그래서 무작정 플래그만 켜지 않고, odom 요레이트를
실제로 `/imu/data`에 흘려주는 브릿지 노드를 새로 만들어 정직하게 동작하도록 함.

### 신규: `sim_imu_bridge_node` (control_code/sim_imu_bridge_node.cpp, 6번째 실행파일)
- odom(`twist.twist.angular.z`, 요레이트)을 `sensor_msgs/Imu`로 중계. orientation은 identity
  고정(롤=0 — 2D 시뮬 한계, 롤 인지 ESC는 시뮬에서 자연히 비활성으로 유지되고 이건 정상/의도됨).
  종가속도(linear_acceleration)도 0 고정(조향 스케일러의 acc_mean 경로는 시뮬에서 중립).
- CMakeLists.txt에 `add_executable`+`install(TARGETS)` 등록. C++ 컨벤션 유지(프로젝트가
  "Python은 참조용"이므로 앞서 만든 scratchpad 파이썬 shim 대신 정식 C++ 노드로 승격).
- `control_sim.launch.py`에만 포함(실차엔 절대 넣지 말 것 — 실제 VESC IMU 토픽과 충돌).

### 런치파일 변경
- **`control_sim.launch.py`**: `use_imu: False` → **`True`**(sim_imu_bridge_node가 실제 데이터를
  공급하므로 안전하게 켤 수 있게 됨). `yaw_rate_gain` 런치 인자 신설(기본 **0.08** — 직전 세션
  스윕 결과 0.15부터 채터링 뚜렷했던 것 반영한 보수값). `sim_imu_bridge` 노드 액션 추가.
- **`control_real.launch.py`**: `use_imu`는 이미 True였음(변경 없음). `yaw_rate_gain` 런치 인자
  신설(기본 0.08, 이전엔 노드 코드 기본값 0.1이 암묵 적용되던 걸 명시적으로 노출). 실차는 실제
  슬립이 있어 시뮬보다 채터링이 더 심할 수 있으니 첫 셰이크다운은 이 보수값에서 시작 권장.
- ⚠️ 두 런치파일 모두 `~/2026_IFAC` 사본이 `waypoint_topic`/`controller_type`(sim) 등으로
  repo보다 앞서있어([[steering-control-node-sync-gap]]) diff 확인 후 양쪽에 동일 diff를
  수동 반영(rsync 안 씀). CMakeLists.txt는 격차 없어 그대로 반영.

### 검증
- `colcon build --packages-select f1tenth_control` 클린 성공(신규 노드 포함).
- 두 launch 파일 `--show-args`로 신규 `yaw_rate_gain` 인자 노출 확인.
- 스모크 테스트(gym_bridge + control_sim.launch.py force_autonomous:=true): `sim_imu_bridge_node`
  기동, `/imu/data` 500Hz 발행 확인, `control_map_node` LUT 로드 성공 + use_imu 경로 에러 없이
  정상 기동 확인.

---

## 2026-07-11 (2) — 요레이트 피드백 카운터스티어 배선 + 시뮬 폐루프 검증

### 배경
`control_MAP.cpp`에 `StabilityController::calculate_yaw_rate_correction`이 정의만 되고
control_loop에서 호출되지 않는 죽은 코드로 남아있었음(07-05 계획에 명시). 오늘 이걸 실제
조향 명령에 배선하고, 시뮬(f1tenth_gym)로 폐루프 검증까지 진행.

### 배선
- 위치: `control_MAP.cpp` 헤딩 댐핑 직후 / rate limit·물리 클리핑 이전(약 627~636줄).
  방금 확정한 명령 조향각으로 기대 요레이트(`v·tanδ/L`)를 구해 실측 요레이트와의 오차에
  `yaw_rate_gain`을 곱해 더함 — 언더스티어(실측<기대) 시 더 꺾어 슬립 상쇄. `use_imu_` 게이트,
  보정분까지 rate limit/±0.41 안전한계에 함께 수렴하도록 클리핑 이전에 삽입.
- ⚠️ **`~/2026_IFAC/f1tenth_control/control_MAP.cpp`가 repo보다 `waypoint_topic`/
  `controller_type`/`/local_waypoints` 구독 기능만큼 앞서있는 상태**(구 `steering_control_node.cpp`
  때와 동일 패턴, 파일명만 바뀜)라 **rsync/동기화 스크립트로 반영하지 않고 2026_IFAC 사본에
  동일 diff를 수동으로 직접 적용**함(양쪽 모두 반영, 앞선 기능은 보존). repo↔2026_IFAC 재동기화는
  여전히 숙제로 남음.

### 시뮬 폐루프 검증
- 시뮬(gym_bridge)엔 `/imu/data`가 없어 `odom.twist.twist.angular.z`(요레이트)를 `/imu/data`로
  중계하는 `odom_to_imu_shim.py` 작성(orientation=identity로 롤 ESC는 격리, 요레이트 보정만 검증).
- `yaw_rate_gain` 0.0(기존)/0.08/0.15 3점 스윕(각 25초, fuck_f1 맵):

  | gain | 횡오차 평균/최대(m) | 평균속도 | 2랩째 랩타임 | 조향 std | 부호전환/s |
  |---|---|---|---|---|---|
  | 0.0 | 0.140/0.250 | 3.523 | 9.92s | 0.077 | 0.0 |
  | 0.08 | 0.146/0.262 | 3.522 | 9.90s | 0.084 | 0.4 |
  | 0.15 | 0.150/0.278 | 3.523 | 9.89s | 0.187 | **3.32** |

  트랙 이탈 0회, 안전한계 위반 없음. **결론**: 랩타임/속도 불변(요레이트 보정은 조향에만 개입),
  게인이 커질수록 횡오차는 미세 악화 + 조향 채터링은 뚜렷이 증가. f1tenth_gym은 선형 타이어라
  슬립/마찰 포화가 없어 **보정이 상쇄할 실제 오차 자체가 없음** — 시뮬은 "배선이 안전하게 도는지"만
  검증 가능하고, 실효(언더스티어 보정)는 실차에서만 확인 가능(예상된 한계, CLAUDE.md 시뮬 타이어
  모델 특성과 일치). **실차 첫 시도는 `yaw_rate_gain` 0.05~0.08 정도 낮게 시작해 채터링 보며
  올릴 것 권장** — 0.15는 시뮬에서도 이미 채터링 뚜렷.

### 겪은 함정 (재사용 가능한 툴링: scratchpad — 이번 세션 한정 경로라 재사용 시 재작성 필요)
- `set -u`(nounset)로 bash 스크립트를 짜면 `/opt/ros/humble/setup.bash`가 미정의 변수
  참조로 즉시 죽음 → ROS setup 스크립트를 소싱하는 스크립트에는 `-u` 쓰지 말 것.
- `global_trajectory_publisher_node` 실행파일의 실제 노드명은 `global_republisher_node`(소스
  내 하드코딩)라, launch 파일 밖에서 `ros2 run`으로 띄우며 yaml params-file(top key
  `global_trajectory_publisher_node`)을 먹이려면 `-r __node:=global_trajectory_publisher_node`
  리매핑이 반드시 필요(안 하면 "did not find any map_path param"로 조용히 실패).
- **`ros2 run` 백그라운드(`&`) 후 캡처한 `$!`는 래퍼 프로세스 PID이지 실제 노드 바이너리
  PID가 아님** — `kill $!`만 하면 실제 노드가 안 죽고 계속 `/drive_autonomous`에 발행,
  다음 게인의 노드와 동시에 떠서 데이터가 오염됨(실측 확인: steer_samples_n이 게인마다
  1x/2x/3x로 누적 증가). 실제 바이너리 경로(`lib/f1tenth_control/control_map_node` 등)를
  `pkill -9 -f`로 직접 잡고, 완전히 사라질 때까지 폴링 후 다음 단계로 넘어가야 함. 스윕류
  스크립트에서 게인/파라미터 전환 시 이 패턴 반드시 재사용.

---

## 2026-07-11 — 조이스틱 버튼 재배치 + MPPI 솔버 파일 리네임/빌드 연결

### 배경
다음 세션에 MPPI 컨트롤러 설계를 시작하기 전, 실차에서 조이스틱 버튼 하나로
MAP ↔ MPPI 제어 알고리즘을 전환할 수 있는 기반을 미리 마련해두기로 함.

### 조이스틱 버튼 재배치 (`joy_teleop_monitor.cpp`)
- **부스트 버튼: RB(5) → A(0)** 이동.
- **신규: RB(5) = MAP/MPPI 알고리즘 전환 버튼** (`algorithm_button` 파라미터, 기본 5).
  LB 토글과 동일한 상승 엣지 감지 패턴으로 구현, `current_algorithm_`
  (`ControlAlgorithm::MAP`/`MPPI`) 상태를 전환하고 `/teleop_dashboard`에
  "Active Algorithm" 줄로 표시.
- ⚠️ **현재는 상태 전환 + 대시보드 표시만** 한다. MPPI 쪽 `/drive_autonomous`
  소스가 될 노드가 아직 없어서, 실제 소스 라우팅(`auto_drive_callback` 확장)은
  MPPI 노드가 생기는 다음 세션으로 미룸.
- `launch/control_real.launch.py`, `launch/control_sim.launch.py`의
  `boost_button`/`algorithm_button` 파라미터도 함께 갱신.

### MPPI 솔버 파일 리네임 + 빌드 연결
- `control_code/mpc_controller.cpp` → `control_code/control_MPPI.cpp`,
  `include/f1tenth_control/mpc_controller.hpp` → `include/f1tenth_control/control_MPPI.hpp`
  로 `git mv` (include guard/include 경로만 갱신, `MPCController` 등 심볼명은 유지 —
  다음 세션에 이 파일 내부를 MPPI 알고리즘으로 재작성할 예정).
- 이 파일은 이전까지 **`CMakeLists.txt`에 전혀 등록되지 않은 죽은 코드**였음(확인
  결과 OSQP C API 기반 전역좌표 LTV-MPC 조향 솔버 클래스이며 ROS 노드/`main()`
  아님). 이번에 처음으로 빌드에 연결:
  - `find_package(osqp REQUIRED)` 추가 — `ros-humble-osqp-vendor`가 설치한
    업스트림 OSQP CMake 패키지(`/opt/ros/humble/lib/cmake/osqp/`)가 `osqp::osqp`
    임포트 타겟을 제공, 기존 코드의 `#include <osqp.h>`와 그대로 호환.
  - `add_library(control_mppi_solver STATIC control_code/control_MPPI.cpp)` +
    `target_link_libraries(... osqp::osqp)` 추가. 아직 어떤 노드도 링크하지
    않는 **컴파일 검증 전용** 정적 라이브러리(`install(TARGETS ...)`에도 미포함).
  - `package.xml`에 `<depend>osqp_vendor</depend>` 추가.

### MPPI 알고리즘 본체 설계 (control_MPPI.cpp 재작성)
지난 리네임에 이어 이 파일 내용을 비우고 **샘플링 기반 MPPI 컨트롤러**로 재설계.
- **정식화**: 정보이론 MPPI(Williams 2018, IEEE T-RO). K개 잡음 제어열 롤아웃 →
  동역학 롤아웃 비용 → 가중 `w_k=exp(-(J_k-β)/λ)/Σ` → `u_t+=Σ w_k ε_{t,k}` →
  첫 제어 출력 + warm-start 시프트. QP/OSQP 불필요(순수 샘플링).
- **제어/모델**: 제어 `u=[조향 δ, 종가속 a]`(조향+종방향 동시). 롤아웃 동역학은
  **동역학 자전거(single-track)+Pacejka 마법공식 횡력**. 저속(vx→0)에서 슬립각
  발산 → `v_switch`(기본 2m/s) 미만은 **기구학 자전거로 블렌드**(실차 정지출발 대응).
- **비용**: 위치·헤딩·속도 2차 추종 + 트랙 경계 소프트 페널티(반폭−마진 초과 시).
  제어비용은 λ 항으로 암묵 반영. 종단 가중 배수(`w_terminal`). 출력 저역통과 평활화
  (LP-MPPI 근사, 채터링 저감).
- **구조**: control_MAP.cpp처럼 **별도 헤더 없이 단일 파일에 인라인**(`MPPIController`
  클래스). 리네임으로 생겼던 `control_MPPI.hpp` 삭제. 파일 하단 `#ifdef MPPI_SMOKE_TEST`
  로 ROS 없이 폐루프 검증하는 스탠드얼론 하네스 내장.
- **빌드**: MPPI는 외부 솔버가 없으므로 `CMakeLists.txt`의 `find_package(osqp)` 및
  `target_link_libraries(... osqp::osqp)`, `package.xml`의 `<depend>osqp_vendor</depend>`
  제거. `control_mppi_solver` static lib 타겟은 컴파일 검증용으로 유지.
- **참조**: [MPPI overview](https://acdslab.github.io/mppi-generic-website/docs/mppi.html),
  [ForzaETH race_stack(참조: 상태/기준 인터페이스 규약)](https://github.com/ForzaETH/race_stack),
  [F1TENTH MPPI 사례](https://www.tdetlefsen.com/f1tenth-mppi.html),
  [LP-MPPI 2503.11717](https://arxiv.org/abs/2503.11717).
- **⚠️ LUT 정직성**: 기존 NUC6 Pacejka LUT는 (횡가속도,속도)→조향각의 *역맵*이라 전방
  롤아웃엔 직접 못 씀. 전방 Pacejka 파라미터는 f1tenth_gym 기본값 사용(추후 실차 보정).
- **검증**: 스모크 테스트 PASS — 원호 추종에서 횡오차 0.40→0.02m 수렴, 속도 3.0m/s
  추종, 최악 solve 12.3ms(<20ms@50Hz, -O2 기준. colcon -O3/-flto면 더 빠름),
  제어 한계·유한성 OK, vx=0 출발 특이점 없음.

### 다음 세션 To-do
- `control_MPPI.cpp`에 `ControlMppiNode`+`main()`을 **같은 단일 파일**에 추가
  (`control_MAP.cpp`와 유사: odom/imu/global·local_waypoints 구독, 웨이포인트→MppiRef
  변환, `/drive_autonomous` 발행). CMake `add_library`→`add_executable(control_mppi_node)`
  전환 + `install(TARGETS)` 등록.
- `joy_teleop_monitor`의 `auto_drive_callback`(또는 소스 선택 로직)을 확장해
  `current_algorithm_`(RB 토글) 상태에 따라 MAP/MPPI 소스를 실제 라우팅.
- Pacejka 타이어 파라미터(Bf/Cf/Df…) 실차/시뮬 보정 → 아래 "Pacejka 파라미터 보정" 참조.

### Pacejka 파라미터 보정 — lut_calibrator 재활용 가능성 (내일 착수)
**질문: MPPI 전방 Pacejka 파라미터를 기존 lut 갱신 메커니즘으로 적용 가능한가?**
결론: **직접 plug-in은 불가, "파라미터 식별(fitting)용 데이터"로는 적합.**

- 지난번 "LUT는 역맵이라 못 씀" 설명은 부정확했음(정정): `lut_calibrator_node.cpp`가
  셀 `(|δ|, v)`에 저장하는 값은 실측 **정상상태 횡가속도 `a_lat = v·ψ̇`** —
  즉 LUT **테이블 내용물 자체는 정상상태 순방향 맵 `a_lat=f(δ,v)`**이고, MAP이 그걸
  역방향 조회(`lookup_steer_angle`)해 쓸 뿐. 데이터는 순방향이 맞다.
- MPPI 전방 동역학도 정상상태 코너링(ω̇=0, v̇y=0, v 일정)에 이르면 `a_lat = vx·ω`를
  뱉는다(캘리브레이터와 동일 공식) → 정합 비교 가능.
  - 저-δ 기울기 → 코너링강성(α≈0의 B·C·D), 고-δ 포화값 → 마찰 피크 `D=μ·Fz` → μ.
- **한계**: ① 정상상태 정보만 → **요관성 Iz·과도(슬립 build-up) 특성은 LUT에서 안 나옴**
  (공칭값 0.047 사용 또는 별도 식별). ② single-track 정상상태만으론 **전/후축 분리가
  under-determined** → lf/lr로 Fzf/Fzr 고정하고 앞뒤 B,C,E(또는 μ) 공유 가정, 혹은
  등가 단일축 곡선으로 피팅. ③ 캘리브레이터가 `/drive` 명령 조향을 실제 서보각 근사로
  씀(조향 오프셋/지연 편향이 피팅에도 실림 — LUT가 이미 안고 가는 가정).

**권장 경로 2가지:**
- **A안 (1순위·재활용, 손 적음):** 캘리브레이션 CSV(`~/f1tenth_lut_calibration/
  NUC6_glc_pacejka_lookup_table_calibrated.csv`)를 읽어, 각 그리드점 `(δ,v)`에서 MPPI
  `step_dynamics`를 정상상태까지 굴려 나온 `a_lat`이 셀 값과 일치하도록 Pacejka
  파라미터(Bf/Cf/Df/Br/Cr/Dr/E)를 **오프라인 최소자승 피팅**하는 스크립트 1개 작성 →
  `MppiParams` 기본값 갱신. Iz·과도특성은 공칭값. 기존 캘리브 메커니즘 그대로 살림.
- **B안 (더 정합적·나중):** MPPI 노드가 선 뒤, lut_calibrator처럼 `(δ, v, ψ̇, ψ̈, vy)`
  로그를 모아 **전방 모델 파라미터를 직접 회귀**(B,C,D,E,Iz 동시)하는 전용 캘리브레이터
  신설. 손은 더 가지만 과도특성까지 잡음.
- 착수 순서 제안: 내일은 A안(재활용) 먼저 시도해 정상상태 정합만 확보 → MPPI 노드/실주행
  검증 후 필요하면 B안으로 과도특성 보강.

---

## 2026-07-08 — 모터 저속 코깅/탈조 진단 (VESC FOC 센서리스)

### 증상
실차 브링업 중 VESC Tool로 duty 0.2 / rpm 1000 테스트 시 바퀴가 미세하게 움찔거리기만
하고 매끄럽게 회전하지 않음(Fault 없음). VESC Tool RT Data로 라이브 파라미터 튜닝하며
원인 분리.

### 근본 원인 (확정)
**FOC 센서리스 오픈루프(강제 커뮤테이션) ↔ 관측기(observer) 핸드오프 구간의 구조적
불안정.** `foc_openloop_rpm≈800` ~ `foc_sl_erpm_start=2250` ERPM 사이("데드존")에서
**속도를 고정 유지(정적 홀드)하려고 하면** 오버슈트 후 완전히 0으로 추락 → 재시동을
무한 반복. 이 구간을 **그냥 통과(가속/감속 스윕)하는 것은 상대적으로 안전** —
목표가 데드존 밖(예: 0 또는 2500 이상)이면 통과 중 한 번 흔들리고 정상적으로
도달/정지함.

- Duty(오픈루프 단독) 제어는 이 문제와 무관 — 항상 매끄러움 (`ERPM 1200 / Ramp 0.10s /
  Boost 3.00A / Max 7.00A / Lock 0.00s` 조합에서 0→11746 ERPM 클린 램프 확인).
- Speed(rpm) 제어 모드에서만 재현 — `Speed PID Ramp`(10000→2000), `Openloop Current
  Max`(7→12~15A), `Heavy Inertial Load` 프리셋 등 여러 시도를 했지만 크래시 발생 지점
  (~2100~2250 ERPM 부근)이 전혀 안 바뀜 → PID 게인/전류 캡 문제가 아니라 데드존
  구조 자체의 문제로 결론.
- 실차 감속(목표 rpm→0) 재현 테스트: 데드존을 하강 통과할 때 짧게(~0.2~0.3s) 덜컹인
  뒤 정상적으로 0에 정지 — 무한루프는 아니고 "거친 정지" 수준.

### 실주행 영향 분석
`ackermann_to_vesc_node`(`~/2026_IFAC/vesc/vesc_ackermann`)가 `speed_to_erpm_gain=4614.0`
(오프셋 0)로 m/s→ERPM 변환 후 VESC Speed(ERPM) 모드로 명령 — 즉 이번에 재현한 문제는
**실제 주행 파이프라인과 동일한 제어 모드**임. 환산: `speed(m/s) = ERPM / 4614`.

| ERPM | 실속도 |
|---|---|
| 800 (데드존 시작) | 0.17 m/s |
| 2250 (데드존 끝) | 0.49 m/s |
| `min_speed` 파라미터 (2.0 m/s) | 9228 ERPM |

`steering_control_node`의 `min_speed_`는 일반 하한이 아니라 곡률 캡/롤 위험 시에만
적용되는 하한이라(`control_code/steering_control_node.cpp` 507, 679줄), 출발/정지 시
`final_speed`가 0에서/0으로 연속 램프되며 데드존을 매번 통과함. `min_speed=2.0m/s`
자체는 데드존보다 훨씬 위라 **정상 순항/코너링 중 이 속도로 목표를 잡는 일은 없음**.
결론: 출발 가속은 스윕 통과라 안전할 가능성 높고, 정지 시엔 짧은 덜컹임이 있을 수
있으나 무한 진동은 아님. 완전 매끄러운 정지가 필요하면 추후 `Openloop Hysteresis`
등으로 추가 튜닝 필요(보류, 다음 세션).

### 부수 발견
- **워크스페이스 동기화 어긋남**: `~/F1tenth_control/control_code/steering_control_node.cpp`
  (이 리포)와 `~/2026_IFAC/f1tenth_control/control_code/steering_control_node.cpp`(빌드
  대상)가 다름 — 배포본에 `controller_type`/`waypoint_topic`/MPC 파라미터가 추가돼
  있고 리포에는 없음. 향후 코드 수정 시 어느 쪽이 최신인지 확인 필요.
- `vesc_mcconf.xml`/`vesc_appconf.xml`(이 리포)도 실제 VESC 라이브 설정과 어긋난
  상태(라이브에서 여러 차례 파라미터를 바꿔가며 테스트함, 프리셋 적용 이력도 있어
  정확한 최종값 불확실). **다음 실차 세션에서 VESC Tool
  `ConfBackup > Save Motor/App Configuration XML`로 라이브 설정을 내보내 이 리포의
  XML을 갱신할 것.**

### 보류
- 감속 시 데드존 통과 덜컹임 추가 완화(`Openloop Hysteresis` 등) — 다음 세션.

---

## 2026-07-05 (예정) — 첫 실차 주행 & IMU/LUT 준비

세션(07-04)에서 도출한 내일 현장 작업 순서. 의존관계상 순서 지킬 것.

### 0. 사전: 코드 반영 · 빌드 · 깃 정리
- F1tenth_control → 2026_IFAC rsync 동기화 후 `colcon build`(신규 `control_real.launch.py` 포함).
- ⚠️ 깃: `simul_practice` 브랜치 non-fast-forward(팀원 선푸시) 상태.
  `cd ~/2026_IFAC && git pull --autostash origin simul_practice && git push origin simul_practice`
  로 병합 후 푸시. **`f1up` 재실행 금지**(빈 커밋에서 체인 끊김).

### 1. 첫 실차 주행 — control_real.launch.py (신규)
- 시뮬 런치와 차이: `odom_topic=/pf/pose/odom`, `max_speed=6.0`(직선캡), `use_imu=False`,
  `is_simulation=True`(수동 조작 허용), AEB 포함(odom remap), `joy_node` 포함.
- 하드웨어 브링업(vesc/lidar/particle_filter/planning) 먼저 떠서 `/scan`,`/pf/pose/odom`,
  `/global_waypoints`,`/joy` 발행 확인.
- 조이스틱(구형 Xbox + BT동글): `ls /dev/input/js0` 인식 확인(360 무선은 일반 동글 페어링 불가).
  `ros2 topic echo /joy`로 버튼 매핑 실측 — 조향 axes[0], RT axes[5], LB buttons[4],
  B buttons[1], X buttons[2], RB buttons[5]. 밀리면 코드/파라미터 조정.
- 시작은 MANUAL 대기 → **바퀴 들고** 스틱/RT 반응 + B 비상정지·X 해제 검증 → LB로 자율 전환.
- 자율 시 `ros2 topic echo /drive_autonomous`로 speed ≤ 6.0 캡 확인.

### 2. VESC 내장 IMU 조사
- VESC 모델 확인 → 내장 IMU 유무(VESC Tool Realtime Data의 IMU 탭 값 확인).
- ROS: `vesc_driver` IMU publish 옵션 켜서 `sensor_msgs/Imu` 토픽 확인 → `/imu/data`로 연결(remap).
- ⚠️ 축 검증: 손으로 롤/요 흔들며 `angular_velocity.x`(롤레이트)·`.z`(요레이트)·orientation
  부호가 코드 가정과 맞는지(VESC 90° 장착 회전 감안).
- VESC가 차 뒤쪽 장착(전후 중앙 아님): StabilityController는 각속도·자세만 써 **offset 영향 0**
  → 축만 맞추면 됨. 단 LUT용 선형가속도엔 레버암 보정 필요하니 **전후거리 d를 자로 실측해 둘 것**.

### 3. IMU 활성화 (조건 충족 시)
- `use_imu=True` → 롤 인지 ESC(`calculate_roll_ratio`)는 이미 배선돼 있어 바로 동작.
- 미배선 죽은 코드: `calculate_steering_attenuation`(과도 감쇄), `calculate_yaw_rate_correction`
  (요레이트 카운터스티어) — A안(롤 ESC) 축검증 후 필요하면 control_loop에 배선(B안).

### 4. LUT 진단 (IMU 완성 후에만)
- 정상상태 원선회로 IMU 실측 횡가속도 vs LUT(NUC6_glc) 예측 비교(과도구간 피하면 뒤쪽 offset 무관).
- 불일치 시: A안 스칼라 보정(CSV 셀×계수) / B안 정식 재피팅. ⚠️ 재피팅 툴체인 불완전
  (상류 steering_lookup의 helpers·모델설정·테이블생성기 없음 → 복구 필요).
- 교체는 `lookup_table_file` 파라미터로(코드 수정 불필요).

### 오늘(07-04) 완료
- 신규 `launch/control_real.launch.py` 작성·빌드 성공(실차 보수 프리셋 + joy_node + AEB).

---

## 2026-06-17 — 코너 이탈 문제 해결 + 코드 최적화

### 1. CLAUDE.md 생성
- 패키지 전체 구조·노드(steering_control_node / joy_teleop_monitor / aeb_node)·토픽 흐름·파라미터·빌드 위치(⚠️ `~/2026_IFAC`에서만 colcon 빌드, 이 폴더는 `COLCON_IGNORE`)를 문서화.

### 2. 코너에서 벽 충돌 후 트랙 이탈 문제 — 원인 분석 & 해결

**증상:** global_planner의 `/global_waypoints`를 추종해 f1tenth_gym 시뮬을 돌리면 코너에서
끝까지 못 돌고 외벽에 박은 뒤 트랙을 완전히 이탈.

**분석 방법:**
- 헤드리스 시뮬 스택(gym_bridge + global_trajectory_publisher + steering_control_node + joy_teleop_monitor)을 백그라운드로 재현, 텔레메트리 1000+샘플 수집.
- 서브에이전트 2팀(코드 정밀추적 / 텔레메트리 정량분석) 병렬 가동.
- 맵 점유격자(pgm)로 레이싱라인의 실제 벽 클리어런스 측정.

**근본 원인 (확정):**
- 오버스피드·언더스티어·제어 게인 문제 **아님** (실제속도가 플래너보다 평균 2.5m/s 낮고, 조향 포화 0.6%뿐, 충돌 직전까지 lat_err<0.3m로 추종 정상).
- **플래너 최적라인(iqp)이 벽에 과도하게 붙어 있음**(클리어런스 ~0.4m). 차체(0.58×0.31m)가 코너에서 각도가 틀어질 때 벽을 스쳐 충돌 → 이후 역주행·벽 관통으로 완전 이탈.
- 가장 타이트한 지점: idx9·idx89 등(우측 구간). 감속해도 같은 지점에서 충돌(속도 문제 아님 재확인).
- 참고: 센터라인은 클리어런스 0.76m로 안전하나 현 플래너가 `/centerline_waypoints` 실제 발행 안 함.

**채택한 수정 (사용자 선택: 제어측 안전라인):**
- `steering_control_node.cpp` `global_path_callback`에서 메시지의 `d_left`/`d_right`/`psi`로
  벽에 붙은 웨이포인트를 트랙 중심 쪽으로 밀어 최소 벽 여유 확보.
- 신규 파라미터 **`wall_safety_margin`**(기본 0.6, 런타임 조정 가능, 0이면 원본 라인).
- 부수: `heading_damping_gain`(기본 0, 튜닝 훅) 추가 — 단, 실험 결과 효과 없어 비활성 유지.

**검증 결과 (시뮬, 동일 트랙):**

| 지표 | Baseline | 수정 후(C=0.6) |
|---|---|---|
| 충돌 이벤트 | 15회 | **0회** |
| 트랙 이탈 | 다수 | **0** |
| 랩 완주 | ❌ | **~5랩 연속 (idx 107/107)** |
| 평균속도 | 2.71 m/s | **3.97 m/s** |

**커밋:** `4d39d1e` fix(control): 안전라인 시프트로 좁은 코너 벽 충돌·트랙 이탈 해결

### 3. 코드 최적화 (대회 미사용 죽은 코드 제거)
- 삭제: `velocity_profiler.{cpp,hpp}`(노드 미사용, 속도는 플래너 수신), `geometry_utils.{cpp,hpp}`(VelocityProfiler 전용).
- 제거: 미사용 파라미터/멤버 `sim`, `enable_obstacle_avoidance`, `last_lookahead_idx_`(write-only) + 관련 include·CMakeLists 정리.
- **IMU 관련은 전부 보존** (IMU 미장착, 추후 도입 예정): `use_imu`/`yaw_rate_gain`/roll ESC/acc 버퍼 및 `imu_stability_controller` 모듈 전체.
- 검증: 클린 빌드 성공, 시뮬 회귀 충돌 0·107/107 완주(평균 4.03 m/s). 순감 −180줄.
- **커밋:** `88da8dc` refactor(control): 대회 미사용 죽은 코드 제거 (IMU 관련은 보존)

### 보류/후속 과제
- **LUT 하드코딩 절대경로**(`/home/tenmeneat/...` 폴백 3개): 대회 노트북에선 깨질 수 있어 정리 권장(현재 이 PC에선 동작해 보류).
- **플래너 안전마진 재생성**(근본해결·속도 유지): 플래닝 담당에게 차폭+안전마진 반영 요청 시, 제어측 안전라인 시프트 없이 더 빠른 라인으로 완주 가능.
- **`MAP_controller_reference.py`**: 선배 코드 원본(참고용, 빌드 대상 아님). VS Code 노란 줄은 ROS2 패키지 unresolved import + 미사용 import 경고(에러 아님). 참고 끝나면 삭제 가능.
- `curvature_ff_blend`·`heading_damping_gain` 토글은 기본 0(무효), 튜닝 훅으로 보존.

---

## 작년 대회 코드 대비 개선점 (new_map_con / MAP_controller.py)

작년 대회 컨트롤러는 `2026_IFAC/new_map_con`의 Frenet 기반 Python MAP 컨트롤러
(참조 복사본: `control_code/MAP_controller_reference.py`). 현재 `steering_control_node.cpp`는
그 **L1 Guidance + Steering LUT 핵심 알고리즘을 그대로 포팅**하고 주변을 강화한 것.

| 항목 | 작년 (MAP_controller, Python) | 현재 (steering_control_node, C++) |
|---|---|---|
| 언어/주기 | Python(rclpy), 40 Hz | **C++17, 50 Hz** (-O3 -march=native -flto) |
| 좌표계 | **Frenet 의존**(FrenetConverter, /car_state/frenet/odom) | **Cartesian만**(odom+waypoints 직접) |
| 코어 조향(L1→sinη→a_lat→LUT) | 보유 | 동일(포팅) |
| 코너 감속 | 플래너 속도 그대로 | **곡률 룩어헤드 사전감속**(v=√(a_lat/κ)) 추가 |
| 벽 여유 | 없음 | **안전라인 시프트**(d_left/d_right) 추가 |
| 종방향 | 속도 명령 그대로 출력 | **가감속 rate-limit 램프** 추가 |
| IMU | 종가속도(acc_y)로 조향 스케일만 | **+롤 인지형 ESC**(전복/스핀 방지, 현재 IMU 미장착이라 대기) |
| 경로 끊김 시 | **정지**(speed=0) | **LiDAR Gap-follower 폴백**(계속 주행) |
| 구조 | 단일 631줄 클래스 | 기능별 모듈 분리(GapFollower/StabilityController/LUT) |

**핵심 요약:** 조향 계산의 "두뇌"(L1+LUT)는 작년 것 그대로. 차별점은 ① 더 빠른 실행(C++/50Hz),
② 의존성 축소(Frenet 제거), ③ 안전·강건성 레이어 추가(곡률 사전감속, 롤 ESC, 갭 폴백,
안전라인 시프트, 가감속 램프). 즉 "알고리즘 혁신"보다 **"검증된 알고리즘을 더 빠르고·안전하고·
독립적으로 돌게 만든 엔지니어링 개선"**이 핵심.

미세 최적화: sin(asin(x))=x 항등식으로 삼각함수 호출 제거, 경로 평균곡률 1회만 계산 등.
