# state_machine_node

## 1. 노드 목적

`state_machine_node`는 주행 상태 결정과 최종 waypoint 선택을 한 프로세스에서 수행합니다.
글로벌 주행(`GLOBAL`), 정적 장애물 회피(`AVOID`), 상대차 추종(`CRUISE`) 상태를
결정해 `/state`로 발행하고, 결정된 상태에 맞는 경로를 `/local_waypoints`와
`/local_waypoints/path`로 발행합니다.

별도 `wpnt_publisher` 노드는 사용하지 않습니다.

## 2. 동작 원리

1. 글로벌·회피 waypoint, Frenet odometry, `/opp_obs`를 각각 최신 캐시에 저장합니다.
2. `publish_rate_hz` 타이머에서 committed-state FSM을 한 단계 평가하고 `/state` heartbeat를
   발행합니다.
3. Frenet odometry가 도착할 때마다 현재 committed state에 맞는 경로를 선택합니다.
4. 선택한 `WpntArray`와 RViz용 `Path`를 같은 timestamp로 발행합니다.

`/state`는 기본 10 Hz 타이머 구동이고, local waypoint는 Frenet odometry 이벤트 구동입니다.
Frenet 입력이 멈추면 마지막 index로 경로를 재발행하지 않습니다.

### 2.1 상태 전환

- `GLOBAL`: 확인된 `/avoid_waypoints`가 있으면 `AVOID`, 그렇지 않고 `/opp_obs`의
  `is_interfering=true`이면 `CRUISE`로 진입합니다.
- `CRUISE`: 항상 글로벌 경로를 선택합니다. 간섭값이 false이거나 stale이면 `GLOBAL`로
  복귀하고, 확인된 회피 경로가 들어오면 간섭값과 관계없이 `AVOID`로 전이합니다.
- `AVOID` → `GLOBAL` 복귀의 전제는 **플래너의 핸드오프 표식**입니다 (2026-08-16 계약).
  장애물 제거 판정의 단일 소유자는 플래너이고, 플래너는 남은 blocking cluster가 없음을
  스스로 확인한 뒤에만 `ot_line=raceline_global_handoff`(파라미터 `handoff_ot_line`)를
  붙입니다. 이 표식이 있는 동안에만 기존 tail 도달·횡오차·지속시간 검사
  (`enter_to_global`)를 평가하며, 표식 없는 non-empty 경로가 우연히 기하 조건을 만족해도
  복귀하지 않습니다. FSM이 `/static_obs`와 ego 횡위치로 같은 것을 중복 판정하던
  `has_front_static_obstacle` / `stopped_path_clear` 계열은 제거했습니다 — ego가 회피
  오프셋에 있으면 라인 위 장애물과 겹치지 않아 오판하고, 플래너와 다른 장애물 토픽을
  봤습니다.
- 유일한 비표식 탈출은 **liveness**입니다: non-empty 회피 발행이
  `avoid_path_liveness_timeout_sec` 동안 끊기면(빈 경로만 오는 경우 포함) 장애물 여부와
  무관하게 `GLOBAL`로 복귀합니다. 플래너가 정지 경로라도 계속 발행하는 한 발동하지
  않습니다 — 장애물 앞 정지는 올바른 상태입니다.
- 핸드오프 표식이 붙은 경로는 **AVOID 진입 M-of-N에 세지 않습니다**. 핸드오프는 "회피
  종료" 선언이지 새 회피 요청이 아니기 때문입니다.
- `AVOID`는 모든 상태에서 가장 높은 우선순위를 가집니다.

### 2.2 경로 유효성 및 선택

- GLOBAL: waypoint가 2개 이상이고 `s_m`이 엄격히 증가하며 `s_m/x_m/y_m`이 유한한 경로만
  저장합니다. 정상 경로는 정적 데이터로 계속 사용합니다.
- AVOID: 최신 경로가 non-empty면 그것을, 비었으면 **마지막 non-empty 경로를 유지**합니다.
  빈 메시지에서 글로벌 기하로 조용히 넘어가면 상태 전이 게이트를 기하로 우회해 장애물
  관통 글로벌 라인이 컨트롤러에 전달됩니다(2026-08-13 실차: 홀드 중 /drive 펄스).
- CRUISE: GLOBAL과 동일한 글로벌 전방 구간을 발행합니다. 경로는 바꾸지 않고
  `f1tenth_control/cruise_controller_node`가 종방향 속도 상한만 조절합니다.
- `global_fallback` 정책은 글로벌 기하를 정당하게 쓰는 상태(GLOBAL/CRUISE)에만
  적용됩니다. AVOID에서 non-empty를 한 번도 못 받은 예외 상황에서만 글로벌로 냅니다.

GLOBAL 출력은 Frenet odometry의 `child_frame_id`를 최근접 글로벌 segment index로 해석해
그 다음 waypoint부터 `waypoint_num`개를 원형으로 추출합니다.

## 3. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | QoS/역할 |
|---|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | Reliable + Transient Local, 글로벌 경로 |
| 구독 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | Reliable + Volatile, 정적 회피 경로 |
| 구독 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | Reliable + Volatile, 동적 상대차 간섭 여부 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Reliable + Volatile, 위치·발행 트리거 |
| 발행 | `/state` | `f110_msgs/msg/StateMachine` | Reliable + Transient Local, FSM 상태 |
| 발행 | `/local_waypoints` | `f110_msgs/msg/WpntArray` | Reliable + Volatile, 제어 입력 경로 |
| 발행 | `/local_waypoints/path` | `nav_msgs/msg/Path` | Reliable + Volatile, RViz 시각화 |

상태 값은 `GLOBAL=0`, `AVOID=1`, `CRUISE=2`입니다.

## 4. 주요 파라미터

운영 파라미터 파일은 `config/state_machine.yaml`입니다.

| 파라미터 | YAML 값 | 설명 |
|---|---:|---|
| `publish_rate_hz` | `10.0` | FSM 평가와 `/state` heartbeat 주기 |
| `waypoint_num` | `50` | GLOBAL에서 추출할 전방 waypoint 수 |
| `allow_avoid_transition` | `true` | 모든 상태에서 AVOID 진입 허용 |
| `allow_cruise_transition` | `true` | `/opp_obs` 기반 CRUISE 진입 허용 |
| `local_path_confirmation_window_size` | `5` | 진입 확인 메시지 창 크기 N |
| `local_path_confirmation_min_hits` | `3` | 필요한 non-empty 수 M |
| `opponent_stale_timeout_sec` | `0.3` | `/opp_obs`가 이 시간 이상 끊기면 간섭 해제 |
| `global_publisher_warn_timeout_sec` | `5.0` | 정적 GLOBAL 발행자 침묵 경고 시간 |
| `frenet_stale_timeout_sec` | `0.5` | 모든 local 출력의 Frenet freshness 제한 |
| `invalid_local_path_policy` | `global_fallback` | local 전용 경로 무효 시 정책 |
| `handoff_ot_line` | `raceline_global_handoff` | GLOBAL 복귀의 전제가 되는 플래너 표식 |
| `enter_global_sec` | `0.5` | 합류 조건 연속 유지 시간 |
| `enter_global_threshold` | `0.2` | 횡오차 게이트 [m] |
| `enter_global_tail_distance_m` | `6.0` | 경로 끝에서 거꾸로 잰 tail 창 호 길이 [m]. local_planning의 `state_handoff_tail_distance_m`와 동일해야 함 |
| `enter_global_s_gap_tol_m` | `0.5` | tail 도달 허용 s 거리 [m] |
| `avoid_path_liveness_timeout_sec` | `2.0` | non-empty 회피 발행 단절 시 AVOID 해제 (liveness 전용) |

현재 `invalid_local_path_policy`는 `global_fallback`만 지원합니다. 파라미터는 기동 시 한 번
읽으므로 값을 바꾼 뒤 노드를 재시작해야 합니다.

## 5. 빌드와 실행

Ubuntu 24.04와 ROS 2 Jazzy 환경에서 다음 순서로 실행합니다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select state_machine
source install/setup.zsh
ros2 launch state_machine state_machine.launch.py
```

다른 파라미터 파일을 사용하려면 다음과 같이 지정합니다.

```bash
ros2 launch state_machine state_machine.launch.py \
  params_file:=/absolute/path/to/state_machine.yaml
```

## 6. 단계별 확인

1. `/global_waypoints`, `/car_state/frenet/odom`, `/opp_obs`가 수신되는지 확인합니다.
2. `/state`가 `publish_rate_hz`와 같은 주기로 발행되는지 확인합니다.
3. `/local_waypoints`가 Frenet odometry와 같은 주기로 발행되는지 확인합니다.
4. `/local_waypoints` publisher가 `state_machine_node` 하나인지 확인합니다.
5. 회피 경로가 중간에 사라지는 시험에서는 ego가 마지막 경로 tail에 도달한 뒤 `/state`가
   `GLOBAL=0` 또는 간섭 중이면 `CRUISE=2`로 복귀하는지 확인합니다.

```bash
ros2 topic echo /state --once
ros2 topic echo /local_waypoints --once
ros2 topic hz /state
ros2 topic hz /local_waypoints
ros2 topic info -v /local_waypoints
```

GLOBAL 모드에서 `child_frame_id`가 빈 문자열, 음수, 숫자가 아닌 값 또는 글로벌 경로 범위
밖의 index이면 local waypoint를 발행하지 않고 경고를 출력합니다.
