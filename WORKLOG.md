# 작업 로그 (WORKLOG)

2026 IFAC F1TENTH 제어 파트 작업 진행상황 기록.

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
- **`MAP_controller_prev.py`**: 선배 코드 원본(참고용, 빌드 대상 아님). VS Code 노란 줄은 ROS2 패키지 unresolved import + 미사용 import 경고(에러 아님). 참고 끝나면 삭제 가능.
- `curvature_ff_blend`·`heading_damping_gain` 토글은 기본 0(무효), 튜닝 훅으로 보존.

---

## 작년 대회 코드 대비 개선점 (new_map_con / MAP_controller.py)

작년 대회 컨트롤러는 `2026_IFAC/new_map_con`의 Frenet 기반 Python MAP 컨트롤러
(참조 복사본: `control_code/MAP_controller_prev.py`). 현재 `steering_control_node.cpp`는
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
