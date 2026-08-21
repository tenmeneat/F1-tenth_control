# cruise 실차 검증 및 튜닝 가이드

`cruise_controller_node`를 실차에서 검증하고 파라미터를 조정하는 사람을 위한 문서입니다.
노드의 인터페이스 명세는 [`cruise_controller_node.md`](docs/cruise_controller_node.md)에 있고,
이 문서는 **왜 그 수식인지 / 무엇을 돌리면 무엇이 바뀌는지 / 어떤 순서로 재야 하는지**를 다룹니다.

> 🔴 **먼저 읽을 것**: §7의 지뢰 목록. 특히 §7-1(평형에서는 제동 캡이 비활성이다)과
> §7-2(state_machine과 값이 묶여 있다)는 모르고 만지면 차가 벽으로 갑니다.

---

## 1. cruise가 하는 일과 하지 않는 일

cruise는 **종방향 속도 상한 하나만** 발행합니다. 조향도, 경로 생성도, 추월 경로도 만들지
않습니다 — 그건 `local_planning`과 `state_machine`의 몫입니다.

```
                        ┌─ /state (STATE_CRUISE 일 때만 개입)
                        ├─ /opp_obs            (obstacle_detector: 전방 상대차 s, vs, 공분산)
cruise_controller_node ─┤─ /car_state/frenet/odom (global_planning: 에고 s, vs)
                        └─ /global_waypoints   (트랙 길이 = s 랩 처리용)
                        │
                        └──> /cruise_speed_limit (Float64, 50 Hz)
                        └──> /cruise/gap_data    (GapData, 진단 전용)
                                    │
                                    ▼
control_map_node:  target_speed = min(경로 프로파일 vx_mps, cruise_cap)
                   → 속도 램프(base_max_accel / base_max_decel) → /drive_autonomous
```

**`min()`이라는 점이 중요합니다.** cruise는 상한만 내리므로 플래너가 요구한 속도를 절대 못
높입니다. 즉 회피 중 권한 침범이 원리적으로 불가능하고, cruise가 죽거나 토픽이 끊겨도
"느려지는 방향"으로만 실패합니다(`cruise_speed_limit_timeout` 0.15 s 초과 시
`cruise_stale_speed` 1.5 m/s).

**GLOBAL·AVOID 상태에서는 `maximum_speed`(12.0)를 발행**하므로 기존 주행에 전혀 개입하지
않습니다. 즉 CRUISE로 전이되지 않으면 이 노드의 파라미터는 무엇을 돌려도 효과가 0입니다.

---

## 2. 수식 — 발행값이 나오기까지 7단계

`/cruise/gap_data`의 필드 이름이 아래 단계와 1:1로 대응합니다. 튜닝 중에는 이 토픽을 보면서
"지금 몇 단계가 발행값을 결정했나"를 판정하십시오.

### 단계 1 — 원시 간격 `raw_gap`

Frenet `s` 축에서 랩을 넘어가는 경우까지 처리한 전방 거리입니다.

```
center_ahead = forwardDelta(opp.s_center, ego_s, track_length)   # 항상 [0, L) 전방
span         = forwardDelta(opp.s_end,    opp.s_start, track_length)
raw_gap      = max(0, center_ahead − span/2 − ego_front_offset)
```

즉 **에고 앞범퍼에서 상대차 뒷면까지의 거리**입니다. 중심 간 거리가 아닙니다.

### 단계 2 — 목표 간격 `desired_gap`

```
거리 모드 (trailing_mode_distance: true):
    desired_gap = max(minimum_gap, trailing_gap)

시간 모드 (trailing_mode_distance: false):
    desired_gap = minimum_gap + trailing_gap × v_ego
                  └─ s0 ─┘   └───── T ─────┘

공통:  max_desired_gap > 0 이면  desired_gap = min(desired_gap, max_desired_gap)
```

🔑 시간 모드는 표준 ACC/IDM 형태 **`s0 + T·v`** 입니다. 예전 `max(s0, T·v)` 형태는
`v > s0/T`부터 `s0`가 사라져 **속도가 오를수록 시간 여유가 줄어드는** 비일관적 정책이었습니다.
`s0`는 정차 시 물리 여유, `T`는 속도와 무관하게 유지되는 반응시간 예산입니다.

### 단계 3 — 불확실도 반영 `effective_gap`

검출기가 준 공분산으로 간격을 보수적으로 깎습니다.

```
v_opp_lb = max(0, v_opp − opp_speed_confidence_z × √vs_var)          # 상대속도 하한
τ_s      = clamp((v_ego − v_opp_lb) / relative_deceleration, 0, gap_uncertainty_horizon_max)
σ_g      = √( max(0, s_var + 2·τ_s·s_vs_cov + τ_s²·vs_var) )
effective_gap = max(0, raw_gap − uncertainty_sigma × σ_g)
gap_error     = effective_gap − desired_gap
```

`τ_s`는 "상대속도를 지우는 데 걸리는 시간"이고, 그 시간만큼 상대차 위치 추정의 불확실도가
등속 모델로 커집니다. 접근이 빠를수록 τ_s가 커지고 → σ_g가 커지고 → 간격을 더 깎습니다.

### 단계 4 — 비상정지 (여기서 끝날 수 있음)

```
emergency_gap = max(0, raw_gap − uncertainty_sigma × √s_var)      # ← τ=0, 시간 전파 안 씀
if emergency_gap ≤ emergency_stop_distance:
        speed_limit = 0;  active_constraint = emergency;  종료
```

🔑 **비상정지만 τ=0을 씁니다.** 미래 예측 불확실도가 "지금 당장 멈춰야 하는가"를 늦추면 안
되기 때문입니다. 안전 판정은 예측이 아니라 **현재 점유**로 합니다.

### 단계 5 — PID `feedback_speed`

```
feedback_speed = v_opp
               + trailing_p_gain × gap_error
               + trailing_i_gain × ∫gap_error dt      (적분은 ±integral_limit로 clamp)
               + trailing_d_gain × (v_opp − v_ego)
```

**기준선이 `v_opp`라는 점이 핵심입니다.** 간격이 딱 맞으면(`gap_error=0`) 상대차와 같은
속도를 명령합니다. 그래서 **P만으로 정상상태 간격 오차가 0**이고, I는 구조적으로 필요
없습니다(§6-3 참고). D항은 상대속도에 직접 작용해 접근 중 미리 감속시킵니다.

### 단계 6 — 제동거리 캡 `braking_speed`

상대차가 지금 제동하기 시작해도 비상 경계 전에 멈출 수 있는 속도입니다.

```
usable_gap = max(0, effective_gap − emergency_stop_distance)

무충돌 조건:   v·τ  +  v²/(2·b_ego)   ≤   usable_gap  +  v_opp_lb²/(2·b_opp)
              └지연┘  └ego 제동거리┘                    └상대차가 벌어주는 거리┘

이 2차식을 v에 대해 풀면:
braking_speed = −b_ego·τ + √( (b_ego·τ)² + 2·b_ego·(usable_gap + v_opp_lb²/(2·b_opp)) )
```

| 기호 | 파라미터 | 뜻 |
|---|---|---|
| `τ` | `actuation_latency` | 상대차 감속을 **관측한 순간**부터 ego 제동이 **실제로 듣기까지** |
| `b_ego` | `ego_deceleration` | ego가 실제로 낼 수 있는 감속도 |
| `b_opp` | `opponent_deceleration` | 상대차가 낼 수 있다고 **가정하는** 감속도 |

🔑 **착지 기본값은 셋 다 0.0이고, 이는 구 식과 비트 단위로 동일합니다.**
`ego_deceleration`/`opponent_deceleration`이 0 이하면 `relative_deceleration`으로 폴백하고,
`τ=0`·`b_ego=b_opp`이면 정확히 예전의 `√(v_opp_lb² + 2a·usable_gap)`이 됩니다
(`test_cruise_controller.cpp`의 `DecomposedBrakingDefaultsAreBitIdenticalToLumpedFormula`가
이를 강제합니다). **실측 없이 켜지 마십시오.**

### 단계 7 — 최종 발행

```
speed_limit = clamp( min(feedback_speed, braking_speed), 0, maximum_speed )
if !allow_accel_trailing:  speed_limit = min(speed_limit, v_ego)
```

`active_constraint`가 이 중 무엇이 이겼는지를 알려줍니다:
`pid` / `braking` / `emergency` / `max_speed` / `no_accel`.

---

## 3. `/cruise/gap_data` 읽는 법

```bash
ros2 topic echo /cruise/gap_data
```

| 필드 | 의미 | 이걸로 판정하는 것 |
|---|---|---|
| `active_constraint` | 0=pid 1=braking 2=emergency 3=max_speed 4=no_accel | **가장 먼저 볼 것.** 어느 항을 튜닝해야 하는지가 여기서 정해진다 |
| `raw_gap` → `effective_gap` | 불확실도로 깎이기 전/후 | 둘 차이가 크면 문제는 게인이 아니라 **검출기 공분산**이다 |
| `desired_gap` | 단계 2 결과 | 시간 모드에서 속도 따라 움직이는지 확인 |
| `sigma_gap` / `horizon_tau` | σ_g와 τ_s | σ_g가 크면 `uncertainty_sigma`가 아니라 검출기를 먼저 볼 것 |
| `feedback_speed` / `braking_speed` | 단계 5 / 단계 6 값 | 둘 중 작은 쪽이 `speed_limit`. 어느 쪽이 이기는지가 곧 튜닝 대상 |
| `gap_diff` `vs_diff` `gap_int` | 구 필드(간격오차·상대속도·적분값) | `gap_int`가 ±`integral_limit`에 붙어 있으면 windup(§7-3) |

**튜닝 세션에서는 이 토픽을 반드시 rosbag에 넣으십시오.** 없으면 사후에 "그때 왜 느렸나"를
가릴 방법이 없습니다.

---

## 4. 실차 검증 절차

### 4-0. 사전 조건

1. **2대 구성이어야 합니다.** 시뮬은 `~/f1sim_C/f1tenth_gym_ros/config/sim.yaml`의
   `num_agent: 2`, 실차는 상대 차량이 실제로 있어야 합니다. 1대면 `/opp_obs`가 비어
   CRUISE 전이가 아예 안 일어나고 이 노드는 계속 `maximum_speed`만 발행합니다.
2. **CRUISE 전이 확인이 먼저입니다.**
   ```bash
   ros2 topic echo /state          # state: 2 (STATE_CRUISE) 가 떠야 함
   ros2 topic echo /opp_obs --no-arr
   ```
   전이가 안 되면 cruise가 아니라 `state_machine`/`obstacle_detector` 문제입니다.
3. **E-stop에 손을 올려두십시오.** 아래 §5는 차가 상대차에 더 가까이 붙게 만드는 절차입니다.

### 4-1. 회귀 확인 (파라미터 변경 전, 필수)

지금 착지된 코드는 **구 거동과 동일해야** 합니다. 먼저 그것부터 확인하십시오.

```bash
ros2 launch f1tenth_control control_real.launch.py cruise_enable:=true
ros2 topic echo /cruise/gap_data
```

`config/cruise_controller.yaml`의 튜닝 값은 `_control_common.py`에 같은 이름의 launch
인자로 노출돼 있습니다. 실험값은 YAML을 바꾸지 않고
`trailing_p_gain:=1.0`과 같이 인자로 덮어쓰면 실험 간 기준값이 섞이지
않습니다. 최고속도는 기존 `max_speed`를 크루즈와 공유합니다.

확인 항목: `desired_gap`이 항상 **5.00** 고정 / `horizon_tau`와 `sigma_gap`이 예전과 같은
범위 / 등속 추종 중 `active_constraint`가 `pid` / 추격 중 `braking`.

### 4-2. 🔴 지연시간 τ 측정 — 다른 모든 것보다 먼저

**이 숫자 하나가 목표 간격을 얼마까지 좁힐 수 있는지를 결정합니다.** 추정하지 말고 재십시오.

상대차가 감속하는 구간을 포함해 bag을 녹화한 뒤, 아래 네 시각의 간격을 봅니다.

| # | 신호 | 토픽 |
|---|---|---|
| ① | 상대차가 실제로 감속 시작 | `/opp_obs`의 `vs` 하강 시작 |
| ② | cruise가 상한을 내림 | `/cruise_speed_limit` 하강 시작 |
| ③ | 제어기가 명령을 내림 | `/drive_autonomous`의 `drive.speed` 하강 시작 |
| ④ | 차가 실제로 감속 | `/odom`의 `twist.linear.x` 하강 시작, 또는 IMU `a_x` |

`τ = ④ − ①` 이 `actuation_latency`에 넣을 값입니다. ①→② 가 검출·판정 지연,
③→④ 가 VESC 제동 응답입니다.

> ⚠️ **①의 시작점을 `/opp_obs`의 `vs`로 잡는 것이 중요합니다.** 상대차가 물리적으로 감속한
> 시각이 아니라 **우리가 그것을 알게 된 시각**이 기준입니다 — 검출기 지연은 이미 그 안에
> 포함되어 있고, 이중으로 세면 τ가 과대해집니다.

### 4-3. `b_ego` 실측 — 코너에서 재십시오

`prebrake_decel`(3.5)과 **같은 값을 그대로 쓰면 안 됩니다.** 그건 직선 실측 상한입니다.

- 하드웨어 상한: `brake_max_current` 8.0 A ≈ **4.8 m/s²**
- 직선 실측: 지속 −2.69 ~ −3.17, 피크 −4.23 m/s² (`run_0819_115512`)
- 🔴 **코너에서는 그립을 횡방향에 쓰므로 이보다 훨씬 낮습니다.** cruise가 개입하는 상황은
  대개 코너 진입이므로, **코너에서도 반복 가능했던 감속도의 보수적 하한**을 쓰십시오.

현재 `relative_deceleration: 1.8`은 그 취지의 보수값입니다. `ego_deceleration`을 켤 때도
같은 기준으로 잡으십시오 — 2.0~2.5가 출발점이고, 실측 없이 3.0 이상을 넣지 마십시오.

### 4-4. `b_opp` 는 어떻게 잡나

상대차가 낼 수 있다고 **가정**하는 감속도입니다. 보수적인 방향은 **크게** 잡는 것입니다
(상대차가 더 급하게 설 수 있다고 보면 요구 간격이 늘어남).

| 상황 | 권장 |
|---|---|
| 상대차가 같은 F1TENTH(같은 하드웨어) | `b_opp = b_ego` (= 구 거동, 안전한 출발점) |
| 상대차가 벽·정지 장애물이 될 수 있음 | `b_opp`를 크게 (예: 8.0) — 사실상 "즉시 정지" 가정 |
| 상대차가 확실히 더 느리게 선다고 아는 경우 | `b_opp < b_ego` (캡이 올라감, 공격적) |

⚠️ `b_opp`를 크게 잡으면 요구 간격이 급증합니다. 예: `usable_gap`이 같아도
`v_opp=6`에서 `b_opp`를 2.5 → 8.0으로 올리면 상대차가 벌어주는 거리가 7.2 m → 2.25 m로
줄어 캡이 크게 내려갑니다.

---

## 5. 파라미터 → 효과 표

### 5-1. 간격 정책

| 파라미터 | 현재 | 올리면 | 내리면 |
|---|---:|---|---|
| `trailing_gap` (거리 모드) | 5.0 | 더 멀리 떨어져 추종 | 더 붙음. **§7-2의 상한 확인 필수** |
| `trailing_gap` (시간 모드 = T) | — | 속도 비례로 간격 증가 | 고속에서 급격히 붙음 |
| `minimum_gap` (= s0) | 0.8 | 저속·정차 시 여유 증가 | 정차 시 더 붙음 |
| `max_desired_gap` | 0.0(끔) | — | 목표 간격 상한. `interference_distance_m` 이하로 |
| `trailing_mode_distance` | true | — | `false`면 시간 모드(`s0 + T·v`) |

### 5-2. PID

| 파라미터 | 현재 | 올리면 | 내리면 | 증상별 처방 |
|---|---:|---|---|---|
| `trailing_p_gain` | 1.0 | 목표 간격 복원이 빠름/공격적 | 부드럽지만 늦게 따라붙음 | 간격 양쪽으로 가감속 반복 → 내림 |
| `trailing_d_gain` | 0.5 | 접근 시 더 일찍 감속 | 상대차 가속에 빨리 반응 | 접근 오버슈트 → 올림 / 상대속도 노이즈로 상한 떨림 → 내림 |
| `trailing_i_gain` | **0.0** | — | — | **0을 유지하십시오.** §6-3 |
| `integral_limit` | 2.0 | — | — | I=0이면 무효 |

권장 스윕: `P`를 0.5 / 1.0 / 1.5, 각 `P`에서 `D`를 0.2 / 0.35 / 0.5.
평가는 랩타임이 아니라 **최소 간격 · 속도 진동 · 제동 jerk**로 하십시오.

### 5-3. 안전 마진

| 파라미터 | 현재 | 효과 |
|---|---:|---|
| `emergency_stop_distance` | 0.45 | 이 이내면 무조건 정지. **지연시간 여유를 사실상 여기서 흡수하고 있음** |
| `relative_deceleration` | 1.8 | 제동 캡의 감속도(폴백) + τ_s 계산. 낮을수록 보수적 |
| `ego_deceleration` | 0.0(폴백) | ego 제동력. **낮출수록** 캡이 내려가 보수적 |
| `opponent_deceleration` | 0.0(폴백) | 상대차 제동력 가정. **높일수록** 캡이 내려가 보수적 |
| `actuation_latency` | 0.0 | 지연. **높일수록** 캡이 내려가 보수적 |
| `uncertainty_sigma` | 2.0 | σ_g 배수. 2.0 ↔ ε≈2.3% |
| `opp_speed_confidence_z` | 1.0 | 상대속도 하한 신뢰 배수. 너무 일찍 느려지면 1.0→0.5 |
| `gap_uncertainty_horizon_max` | 1.0 | τ_s 상한 [s]. 0이면 시간 전파 끔 |

🔑 **불확실도 항이 너무 세게 깎는다고 느끼면 `uncertainty_sigma`부터 내리지 마십시오.**
`gap_data`의 `raw_gap`과 `effective_gap` 차이를 먼저 보고, 그게 크면 원인은 검출기의
`s_var`/`vs_var`가 실제 오차보다 크게 나오는 것입니다. **검출기 공분산 교정이 먼저입니다.**

### 5-4. 페일세이프

| 파라미터 | 현재 | 효과 |
|---|---:|---|
| `blind_trailing_speed` | 1.5 | 입력 끊김 시 상한 |
| `opponent_timeout` / `ego_timeout` / `state_timeout` | 0.15 / 0.20 / 0.30 | 각 입력 신선도 |
| `clear_confirm_sec` | 1.00 | 이만큼 `/opp_obs`가 비어야 타깃 해제. **짧으면 코너 차폐 중 가속** |

---

## 6. 2 m 추종으로 좁히는 절차

### 6-1. 먼저: 왜 5 m가 과한가

현재 고정 5 m는 4 m/s에서 1.25 s, 6 m/s에서 0.83 s 헤드웨이입니다. 레이싱 기준으로 과합니다.
반면 목표를 상수 2 m로 두면 저속에서는 적절하지만 고속에서 시간 여유가 부족해집니다 —
**둘 다 모양이 물리와 안 맞습니다.** 필요 간격은 대략 `τ·v`에 비례하므로 `s0 + T·v`가 맞는
형태입니다.

### 6-2. 🔑 무엇이 2 m를 가능하게 하는가 — 제동 캡이 아닙니다

등속 추종 중(`gap_error≈0`, 상대속도≈0) 계산해 보면:

| 목표 간격 | v=4일 때 `braking_speed` | v=6일 때 | 평형에서 캡이 구속하나 |
|---|---:|---:|---|
| 5.0 m | 5.69 | 7.24 | **아니오** (feedback 4.0 / 6.0이 이김) |
| 2.0 m | 4.65 | 6.45 | **아니오** |

🔴 **두 경우 모두 평형에서는 제동 캡이 한 번도 걸리지 않습니다.** 캡이 규정하는 것은
"얼마나 가까이 붙느냐"가 아니라 **"얼마나 빨리 접근하느냐"** 입니다. 그러므로:

- 2 m를 막고 있는 것은 제동 캡이 아니며, PID를 잘 튜닝해서 되는 문제도 아닙니다.
- **평형에서 추돌을 막는 것은 `emergency_stop_distance`(0.45 m)와 지연시간뿐입니다.**
- 상대차도 같은 F1TENTH라 제동력이 비슷하므로, 둘 다 최대로 밟으면
  **간격 손실 ≈ v × τ** 입니다. τ=0.15 s, v=6 m/s면 0.9 m 손실 → 2.0 m에서 시작하면
  1.1 m가 남아 0.45 m보다 큽니다. **성립합니다.**
- τ=0.3 s면 1.8 m가 날아가 2 m는 성립하지 않습니다. **그래서 §4-2가 먼저입니다.**

### 6-3. I 게인은 계속 0으로 두십시오

`feedback_speed`의 기준선이 `v_opp`이므로, 평형(`v_ego=v_opp`)이 성립하려면
`P·gap_error = 0` → `gap_error = 0`입니다. **구조적으로 정상상태 오차가 없어 I가 메울 것이
없습니다.**

그런데 windup은 실재합니다: 상대차가 멀거나 코너에서 곡률 캡이 더 낮아 **cruise의 상한이
채택되지 않는 동안에도** `gap_error > 0`이 계속 적분되어 `integral_limit`에 포화합니다.
cruise는 자기 상한이 실제로 쓰였는지 피드백을 받지 못하므로 이를 구조적으로 알 수 없습니다.
`gap_data`의 `gap_int`가 ±2.0에 붙어 있는 것으로 관측만 가능합니다.
→ **I를 켜려면 anti-windup을 먼저 넣으십시오.** 지금은 0이라 무해합니다.

### 6-4. 단계별 적용 (한 번에 하나씩)

```bash
# 0단계 — 회귀 확인 (파라미터 변경 없음). §4-1
ros2 launch f1tenth_control control_real.launch.py cruise_enable:=true

# 1단계 — 지연시간·감속도 실측. §4-2, §4-3. 파라미터 변경 없음.

# 2단계 — 제동거리 분해를 실측값으로 켠다 (간격은 아직 5 m 그대로)
#          캡이 내려가는 방향이라 안전 측이다.
ros2 launch f1tenth_control control_real.launch.py \
  ego_deceleration:=2.2 opponent_deceleration:=2.2 actuation_latency:=0.15

# 3단계 — 간격을 시간 모드로 전환. 여기서 처음으로 차가 더 붙는다.
ros2 launch f1tenth_control control_real.launch.py \
  ego_deceleration:=2.2 opponent_deceleration:=2.2 actuation_latency:=0.15 \
  trailing_mode_distance:=false minimum_gap:=0.8 trailing_gap:=0.2 \
  max_desired_gap:=5.0

# 4단계 — PID 미세조정. §5-2 스윕.
```

각 단계마다 `/cruise/gap_data`를 녹화하고 **최소 간격**과 `active_constraint` 분포를
확인한 뒤 다음으로 넘어가십시오.

시간 모드 `s0=0.8, T=0.2`의 목표 간격:

| v [m/s] | 2 | 4 | 6 | 8 |
|---|---:|---:|---:|---:|
| `desired_gap` [m] | 1.2 | 1.6 | **2.0** | 2.4 |
| 헤드웨이 [s] | 0.60 | 0.40 | 0.33 | 0.30 |

---

## 7. 🔴 지뢰 목록

### 7-1. 평형에서는 `relative_deceleration`이 아무것도 보호하지 않는다

§6-2에서 계산한 대로, 등속 추종 중에는 제동 캡이 비활성입니다. `relative_deceleration`을
1.8로 보수적으로 잡아둔 것이 **평형 추종 중의 안전을 보장한다고 오해하지 마십시오.**
그 값은 **접근 중**에만 작동합니다. 평형에서의 방어선은 `emergency_stop_distance`와
지연시간뿐이고, 그래서 §4-2의 τ 측정이 이 문서에서 가장 중요한 절차입니다.

### 7-2. `desired_gap`은 `state_machine`의 `interference_distance_m` 이하여야 한다

`src/state_machine/config/state_machine.yaml`의 `interference_distance_m`(현재 5.0)은
CRUISE 진입 판정 거리입니다(`P(gap < 이 값) > 0.7`이면 진입).

- `desired_gap > interference_distance_m` 이면 → cruise가 유지하려는 간격에서 상태머신이
  "간섭 없음"으로 판정해 CRUISE를 빠져나감 → 상한 해제 → 가속 → 재진입 →
  **급정지/재가속 리밋사이클**
- `desired_gap < interference_distance_m` 이면 안전합니다 (5 m에서 진입해 2 m까지 붙는
  정상 동작).

⚠️ 시간 모드는 목표 간격이 속도에 따라 커지므로, `s0 + T·v_max`가 이 값을 넘을 수 있습니다.
`max_desired_gap`을 반드시 함께 설정하십시오.

### 7-3. `gap_int`가 포화해 있는 것은 정상이다 (I=0인 동안은)

§6-3 참고. I 게인을 켜기 전에는 무시하고, 켜려면 anti-windup이 먼저입니다.

### 7-4. `f110_msgs`는 저장소 루트 한 벌뿐이다

```
~/2026_IFAC/f110_msgs/    ← 유일한 발생지. `GapData.msg`는 여기서만 고칩니다
```

예전에는 `src/f1tenth_control/f110_msgs/`에 완전한 중복 사본이 같이 있었습니다.
`colcon`은 `src/f1tenth_control`을 패키지로 인식한 순간 그 하위로 내려가지 않아
그 사본은 한 번도 빌드된 적이 없으면서, 정작 `.msg`를 고칠 때마다 손으로 미러링해야
하는 드리프트 위험만 남겨 **2026-08-21에 삭제했습니다.**

🔴 **nested 사본을 다시 만들지 마십시오.** 다른 브랜치를 머지할 때 따라 들어오면
그 자리에서 지우십시오 — 빌드에 영향이 없어서 살아 있어도 아무도 모릅니다.

### 7-5. 시뮬 검증은 안전 마진을 증명하지 못한다

시뮬로 확인할 수 있는 것과 없는 것을 구분하십시오.

| 시뮬로 확인 **가능** | 시뮬로 확인 **불가능**(낙관 방향으로 틀림) |
|---|---|
| 리밋사이클 부재 (§7-2) | **지연시간 τ** — 실차의 LiDAR 이더넷·VESC 체인이 없어 훨씬 짧다 |
| 정상상태 간격 오차 수렴 | **검출기 공분산 현실성** — `s_var`/`vs_var`가 낙관적이면 `effective_gap`이 후해진다 |
| `active_constraint` 전이가 의도대로인지 | **코너 제동 능력** — 횡·종 그립 배분이 담기지 않는다 |
| 기본값 회귀 0 | |

🔴 그리고 현재 `opponent_simulator`는 `speed_scale: 0.8` 고정 pure-pursuit라
**상대차가 감속하지 않습니다.** 즉 §6-2의 제동 분해가 보호하는 유일한 상황을 지금 시뮬
구성으로는 발동시킬 수 없습니다. 시뮬로 검증하려면 상대차에 감속 시나리오를 먼저
넣어야 합니다.

**결론: "시뮬에서 됐다"를 근거로 실차 목표 간격을 좁히지 마십시오.** 시뮬은 구조 회귀
검증용이고, 2 m의 가부는 §4-2의 실차 측정에서만 나옵니다.

---

## 8. 참고

- 노드 인터페이스 명세: [`cruise_controller_node.md`](docs/cruise_controller_node.md)
- 파라미터 파일: `config/cruise_controller.yaml`
- 단위 테스트: `test/test_cruise_controller.cpp`
  (`colcon test --packages-select f1tenth_control --ctest-args -R test_cruise_controller`)
- 상태머신 결합: `src/state_machine/config/state_machine.yaml`의 `interference_distance_m`
- 하류 소비: `control_code/control_map_node.cpp`의 `cruise_speed_limit_callback` 및
  `target_speed = min(target_speed, cruise_cap)`
