# 작업 로그 (WORKLOG)

2026 IFAC F1TENTH 제어 파트 작업 진행상황 기록.

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
