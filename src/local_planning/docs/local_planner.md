# local_planner_node

## 1. 노드 목적

`local_planner_node`는 정적 장애물이 글로벌 Race Line을 막을 때만 로컬 회피 세그먼트를 만듭니다.
동적 상대 차량 정보는 `obstacle_detector`의 `/opp_obs`로 분리되며, 이 노드는 정적 장애물용
`/confirmed_static_obs`만 구독하고 `/avoid_waypoints`를 발행합니다.

검증된 analytic P3와 M1 active-set closure도 동일 패키지와 동일 planning callback 안에서 실행할
수 있습니다. 외부 evaluator는 frozen parity 회귀에만 사용하며 runtime 경로를 공급하지 않습니다.
운영 기본값은 `p3_mode=OFF`이므로 기존 P0 동작이 그대로 유지됩니다.

### P3/M1 runtime mode

- `OFF`: P3 M1 maneuver lifecycle을 우회하고 `plan()` 파이프라인 결과만 발행합니다.
  (2026-08-15부터 `plan()`의 후보 생성기 자체가 P3이므로, OFF는 "P0 후보"가 아니라
  "lifecycle 없는 P3 후보 + 기존 커밋/안전정지 관리"를 뜻합니다.)
- `SHADOW`: 같은 ego/장애물/reference snapshot으로 P3/M1 및 exact validator를 실행하지만
  `/local_planning/p3_shadow` 진단만 발행합니다. `/avoid_waypoints` ownership은 `plan()`
  파이프라인입니다.
- `TEST_ACTIVE`: hard-valid P3/M1 또는 현재 geometry에 대해 재검증된 committed suffix가
  `/avoid_waypoints`를 소유할 수 있습니다. 사용할 수 없으면 `plan()` 파이프라인이
  `P0_BACKUP_ONLY`(이름은 역사적 유래) 소유자로 즉시 처리하고 safe-stop 의미도 그대로
  유지합니다.

### P0/P3 계층 구분 — "P0를 끈다"가 무엇을 뜻하는가 (2026-08-14)

P0(`RacelineSplinePlanner`)는 한 덩어리가 아니라 세 층이고, P3는 그중 둘 위에 서 있습니다.

| 층 | 내용 | P3와의 관계 |
|---|---|---|
| ① 안전 계층 | `expandVisibleObstacles`(추종오차 LUT + `localization_reserve_m`), `validateCandidate`(회전 footprint + `wall_safety_margin_m`), `applyAvoidanceVelocityLimit`(gap 기반 속도 제한), `measureCandidate` | **P3가 직접 호출**. 없으면 P3가 동작하지 않음 |
| ② 안전정지 | `buildSafeStop`(+정지점 탈출 검증), safe-stop 래치/lifecycle, margin slow pass, last-path brake, emergency hold | **P3에 대응물 없음**. 어느 모드에서나 P0 몫 |
| ③ 회피 후보 생성 | `generateP3Candidates` — `plan()` 내부에서 P3 analytic ladder를 호출 | **P3 자체** (2026-08-15부터 유일한 생성기) |

2026-08-15부터 ③은 P3 하나뿐입니다. 과거의 P0 quintic 격자(target_d 5 × entry 4 × exit 3 ×
2측 = 120후보 전수 대입)와 `p0_avoidance_candidates_enable` 토글은 삭제됐습니다 — 실차/시뮬
비교에서 P0가 통과하는 모든 곳을 P3가 통과했고, 생성기가 둘이면 "탈출 가능" 판정과 실제
재계획이 어긋나는 교착이 생기기 때문입니다(아래 2026-08-15 절 참고). ①②의
추종오차 LUT·`localization_reserve_m`·`wall_safety_margin_m`·`safe_stop_buffer_m`·탈출 검증
튜닝은 전부 그대로 유효합니다.

**탐색 방식**: P3는 스테이션별 통과 가능 d 구간을 교집합해 장애물 구간 전체에 걸쳐
연결된 통로를 먼저 구하고(`connectedConstantRanges`) 그 안으로 해를 닫힌 형태로 풉니다 —
통로가 존재하면 찾아냅니다. 통로 교집합이 비면 후보가 0이고 곧바로 폴백 사다리(margin slow
pass → safe stop)로 갑니다. 순위 기준(`minimum_normalized_safety_slack` 사전식)은 P0 시절과
동일합니다.

P3/M1은 authoritative nonempty `/confirmed_static_obs` snapshot이 들어오면 즉시 candidate generation을
수행합니다. P0가 이미 사용하던 `initial_observation_count`,
`initial_observation_min_duration_sec`, `initial_observation_max_wait_sec` 동안의 envelope union은
계속 형성하지만, `selection_guard_ready` 자체는 ownership veto가 아닙니다. 준비 중에도 선택
경로가 현재까지 누적한 conservative geometry와 같은 callback의 raw obstacle geometry 양쪽에서
기존 exact validator로 hard-valid일 때만 즉시 ownership을 허용합니다. 어느 한쪽이라도
hard-invalid면 발행하지 않으며, 별도의 P3 전용 waiting/TTL/hysteresis parameter는 없습니다.
`SHADOW`의 P3 평가 상태는 P0 commitment나 발행 결과를 변경하지 않습니다.

P3 lifecycle은 최초 선택 path와 identity를 immutable하게 보관합니다. 이후 callback에서는 실제
Frenet progress로 이미 지난 prefix만 제거하고, 남은 geometry를 현재 obstacle/reference에 exact
재검증합니다. fresh empty obstacle snapshot은 detector dropout으로 취급하여 immutable path의
current exact validation을 계속합니다. 반면 source epoch/reference generation, non-empty conflicting
identity, stale/hard-invalid/safe-stop 변화는 즉시 무효화합니다. 확대된 장애물 종방향 영역을 완전히
지나면 짧은 tail을 validator에 넣기 전에 `COMPLETE`로 종료합니다.

P3 commit 시에는 선택에 사용한 exact-ID obstacle Guard를 immutable path identity와 함께
저장합니다. 이후 같은 ID의 live envelope가 저장된 Guard 안에 포함되면 원래 Guard와 suffix를
그대로 유지합니다. Guard 밖으로 확장되면 현재 Guarded geometry로 suffix를 exact 재검증하고,
장애물 충돌만 원인인 경우 raw obstacle geometry로 한 번 더 exact 재검증합니다. raw geometry도
충돌하면 즉시 `INVALIDATED`로 전환합니다. raw geometry가 안전하면 P0 commitment가 이미 쓰는
`commitment_soft_violation_confirm_cycles` 계약을 재사용해 일시적인 conservative-Guard 위반을
처리합니다. P3 전용 TTL, waiting frame, hysteresis parameter는 추가하지 않습니다.
commit 뒤의 live envelope를 frozen Guard에 영구 union하지 않습니다. 실제 확장은 위의
guarded/raw exact 재검증으로 판별하므로 일회성 AABB 확대가 maneuver 전체 Guard를 오염시키지
않고, raw 충돌은 여전히 즉시 invalidation됩니다.

가장 중요한 설계 조건은 다음과 같습니다.

> 출력 경로는 항상 `/global_waypoints`의 진행 순서와 `s_m`을 유지하며, 각 점의 로컬 `d_m`만
> 바꾼다.

따라서 Cartesian 공간에서 가까운 점을 다시 찾거나 자유공간을 가로질러 새 경로를 연결하지
않습니다. 스네이크 구간처럼 서로 다른 트랙 조각이 가까이 있어도 차량은 현재 Race Line의 다음
점들만 따라갑니다.

## 2. 참고 구현과 적용 범위

알고리즘 개념은 `vaithak/f1tenth-icra-race`의 `scripts/spliner.py`를 참고했습니다.

- 참고 저장소: `git@github.com:vaithak/f1tenth-icra-race.git`
- 분석 기준 커밋: `ac9a4c98948cc74077f0435d40017468d68d4d6c`
- 가져온 핵심 개념: Frenet `s`를 독립변수로 하는 pre/apex/post 전환 구조,
  장애물 좌우 여유에 따른 회피 방향 선택, 트랙 폭 검사

현재 프로젝트에는 다음 차이를 반영해 C++17로 새로 구현했습니다.

1. CSV 대신 `/global_waypoints`를 사용합니다.
2. `/confirmed_static_obs`의 `s_start/s_end/d_right/d_left`를 장애물의 authoritative Frenet 경계로
   사용합니다. `obstacle_detector`가 map-frame Cartesian AABB 전체를 CLCS로 투영하고, 가까운
   Race Line 선분과 AABB 면 사이의 최단거리까지 반영합니다. local planner는 이 좌표변환을
   반복하지 않습니다. Cartesian AABB와 enclosing-circle `radius`는 회피 형상에 사용하지
   않습니다.
3. 한 점 apex가 아니라 장애물 군집의 앞·뒤에서 목표 `d`를 유지해 긴 정적 장애물도 처리합니다.
4. 글로벌 waypoint 자체를 출력 표본으로 사용해 Race Line의 위상 순서를 강제합니다.
5. 좌우 모두 불가능하면 장애물 앞 감속 경로를 발행합니다.
6. 이동 후 heading·curvature를 다시 계산하고 velocity-limit 표로 회피속도를 제한한 뒤
   acceleration을 다시 계산합니다.

## 3. 동작 원리

### 3.1 입력 준비

1. `/global_waypoints`의 모든 값이 유한하고 `s_m`이 엄격히 증가하는지 검사합니다.
2. 마지막 `s_m`과 waypoint 중앙 간격으로 폐루프 트랙 길이를 구합니다.
3. Frenet odometry의 `position.x`를 ego `s`, `position.y`를 ego `d`로 읽습니다.
4. `/confirmed_static_obs`(파라미터 `obstacles_topic`)를 detector Layer 2 계약에 따라 그대로
   입력받습니다. 이 토픽에는 `CONFIRMED` + map-frame 위치 지속성으로 `Static` 판정까지 끝난
   객체만 실립니다. `/static_obs`에 함께 실리는 `CONFIRMED+UNKNOWN` provisional 객체는
   포함되지 않으므로, 정지 증거를 못 모으는 벽 조각·산란 클러스터가 계획 입력이 되지
   않습니다 (2026-08-16 전환). 두 토픽은 같은 track/physical ID를 쓰므로 관측 횟수·guard·
   완료 ID 같은 ID 기반 계약은 그대로입니다.
5. detector가 채운 Frenet 값이 유한하고 `d_right <= d_left`이며 종·횡방향 중 하나 이상의
   폭이 양수인지 검사합니다. 조건을 만족하지 않으면 해당 장애물을 제외합니다. Cartesian
   AABB는 planner geometry로 사용하지 않습니다.

### 3.2 가장 가까운 정적 장애물 군집

1. ego 앞 `detection_lookahead_m` 안의 장애물 상자를 폐루프 `s`로 펼칩니다.
2. 최초 commitment용 Frenet 경계 합집합은 종방향에만
   `uncertainty_sigma_scale * sqrt(s_var) + uncertainty_min_longitudinal_inflation_m`을 더합니다.
   횡방향 `d_right/d_left`는 실측 합집합을 그대로 유지합니다.
3. 이 Guard를 종방향 `obstacle_longitudinal_padding_m`만큼 넓혀 전환 시작·종료 구간을
   확보합니다.
4. 장애물 횡방향 계획 clearance는 `vehicle_half_width_m + safety_margin_m + e_track`입니다.
   `e_track`은 velocity-limit 표로 제한한 속도와 절대곡률 LUT를 bilinear interpolation하며,
   최초 목표 계산에는 장애물 reference 구간의 최댓값을 사용합니다. 현재 균일 LUT에서는
   `0.1435 + 0.0147893 + 0.140 = 0.2982893 m`입니다.
5. Guard의 가장 가까운 면이 글로벌 `d=0`에서 이 clearance 안에 들어올 때만 blocking
   장애물로 봅니다.
6. `obstacle_cluster_gap_m`보다 가까운 후속 장애물은 같은 기동으로 처리합니다.

횡방향 처리 순서는 다음과 같습니다.

```text
detector raw AABB
  -> 같은 ID의 실측 d_right/d_left 합집합(추가 횡팽창 없음)
  -> vehicle_half_width_m + safety_margin_m + LUT(|v|, |kappa|)를
     한 번 적용한 차량 중심 허용 범위
```

`vehicle_length_m=0.56`, `vehicle_half_width_m=0.1435`는 evaluator와 같은
base_link 중심 직사각형 실차 footprint이고 마진이 아닙니다.
`safety_margin_m`는 물리 안전 여유입니다. 추종오차 LUT는 장애물 위치가 아니라 실제 후보
waypoint의 제한된 `|vx_mps|`와 `|kappa_radpm|`에 따라 달라집니다. 회피속도는 먼저
`avoidance_velocity_limit_*` 표에서 `v²|κ| <= a_lat,max(v)`를 만족하도록 제한합니다.

tracking-error LUT는 더 이상 균일 상수가 아니라 실측값입니다.
`tools/cmaes_tuning/tracking_error_lut_from_traces.py`가 장애물 없는 주행 구간의 `lat_err`를
(속도 × 곡률) 셀별로 모아 안전계수를 곱해 생성합니다. 장애물이 없으면 발행 경로 = 글로벌
라인이므로 `lat_err`가 곧 추종오차입니다. 2026-08-12에
`tools/cmaes_tuning/clean_lap_lut_traces.sh`로 클린 맵에서 속도 상한 1.6/2.0/2.5/2.9 +
race 프로파일 총 6랩(5,360 표본)을 돌려 **전 셀을 실측**했습니다: 저속(1.5–3.0 m/s) 코너
셀 max 0.19–0.22(이전에는 표본이 없어 0.295로 비관 충전), 저속 직선 max 0.136(이전 0.095는
과소 = 안전 구멍). `tracking_error_reserve_m`은 세 LUT 배열을 모두 비운 경우에만 쓰는
fallback입니다.

⚠️ **`maximum_curvature_radpm = 1.3163`은 임의 튜닝값이 아니라 차량 풀락 조향의 물리
한계입니다** (tan(0.41 rad)/0.33 m = 1.317, 시뮬·실차 실측 도달각 동일). 레이스라인 최대
곡률과 같은 값이므로, **최대 곡률 코너 안/직후에 놓인 장애물은 어느 쪽으로도 회피가
불가능**합니다 — 회피 경로는 코너 구간에서 곡률 증폭(κ/(1−dκ))을 일으켜 반드시 이 한계를
넘습니다. v1 frozen FINALS/Q2 1번 장애물(s=9.114, κ=1.06 코너 출구 2.5 m 뒤)이 그 사례로,
당시 CMA의 0/26 완주는 파라미터 문제가 아니라 이 물리적 불가능이 원인이었습니다. 이런
배치에서 safe-stop은 버그가 아니라 옳은 동작입니다.

#### 실차 LUT 갱신 절차

현재 표는 시뮬레이션 실측입니다. 실차 제어 오차는 다르므로 현장에서 아래 순서로 갱신합니다.

1. **고속 셀 (v ≥ 3.0, 6칸)** — Q1/Q3 연습·예선 랩은 규정상 장애물이 없으므로 그냥 주행하며
   `lap_referee`를 함께 띄워 trace CSV를 받습니다. 별도 주행이 필요 없습니다.
2. **저속 행 (v < 3.0)** — 레이싱 주행은 이 셀을 방문하지 않으므로 의도적 저속 랩이 필요합니다.
   제어기 `max_speed`를 2.0으로 걸고 2랩, 2.5로 걸고 2랩 주행합니다. `avoidance_minimum_speed_mps`
   = 2.0이 실제로 이 행의 예약값을 사용하므로 **이 저속 측정이 가장 중요합니다.**
3. 두 세션 모두 장애물이 없어야 하고, trace마다 차가 정지·복구한 구간이 있으면 그 파일은
   제외합니다.
4. 생성:

   ```bash
   python3 tools/cmaes_tuning/tracking_error_lut_from_traces.py \
     <trace1.csv> <trace2.csv> ... \
     --waypoints offline_trajectory_generator/output/ifac_track/global_waypoints.csv \
     --safety-factor 1.25
   ```

   출력의 YAML 블록을 `config/local_planning.yaml`의 세 `tracking_error_lut_*` 배열에 그대로
   붙여넣습니다. 셀별 표본 수(n)가 함께 출력되므로 n이 0인 셀(단조 확장으로 채워진 셀)이
   실주행 영역에 남아 있으면 해당 속도 대역을 추가 주행합니다.
5. 갱신 후 규정 최소 간격 통과 조건을 재확인합니다: 직선 셀 예약이 `2.0 m/s에서 ≤ 0.10882 m`
   이어야 하한 2.0으로 규정 간격(자유 폭 0.5 m)을 지날 수 있습니다. 실측이 이보다 크면
   하한을 낮추거나(저속 행 재측정 필요) 회피 불가로 받아들여야 합니다.

🔴 **2026-08-13 실차 실측 완료 — 위 5번 조건 불통과가 실측 결론입니다.** 실차 6랩
(slow20×2·slow25×2·race×2, 3,926 샘플, 라이브 /global_waypoints 기준선)으로 표를 전면
교체했습니다. 실차 추종오차는 시뮬의 1.5~2배: 2.0 m/s 직선 예약 0.265 m,
v→0 바닥도 0.200 m — p95 방식으로 계산해도 결론 동일하게 **최소간격
(자유폭 0.5 m) 통로는 어떤 속도로도 통과하지 않습니다**(margin-pass/safe-stop 사다리가
처리). 최소간격 통과가 필요해지면 제어 추종 정확도 개선이 선행돼야 합니다.
⚠️ **예산 분할 명확화(제어팀 협의, 2026-08-13)**: 0.10882는 `trackingErrorReserve()`
총예산이고, 코드가 `LUT + localization_reserve_m(0.06)`을 합산해 이 예산과 비교하므로
**분할은 위치추정 0.06 + 제어(LUT) 0.049**다. 현 제어 실측(2.0 m/s 직선, MCL 기준
p95 0.178/max 0.199)은 제어 몫의 약 4배 — 통과하려면 제어 오차를 1/4로 줄이거나
실차 MCL 실측(선행 필요) 후 분할을 재협상해야 한다. 참고:
실트랙 최대 |κ|=0.694라 κ≥0.9 열은 존재하지 않는 영역(단조 확장값, n=0 정상)이고,
v[3,4.5) k[0.2,0.5)의 max 0.446에는 MCL 보정 스파이크 의심 표본이 포함돼 있습니다
(p95는 0.224 — 고속 회피가 과하게 느려지면 p95 재생성을 검토).

### P3 기본화와 continuation-first 재계획 정책 (2026-08-12)

- `p3_mode` 기본값이 **TEST_ACTIVE**(P3 주도 + P0 백업)로 바뀌었습니다 (launch·노드·YAML 모두).
- **continuation-first**: 이전에는 fresh hard-valid M1 결과가 항상 저장된 committed 경로보다
  우선이라 매 콜백(25 Hz) 경로가 재선택되어 RViz에서 회피 경로 모양이 계속 바뀌었습니다.
  이제 활성 maneuver가 있으면 `continueCurrent`(고정 경로 재검증)를 먼저 시도합니다:
  frozen guard 안에 현재 장애물 envelope가 **포함되는 동안 경로를 고정**하고, envelope가
  guard를 벗어나면 raw 기하 재검증 + 기존 soft-violation confirm cycles로만 유지하다
  확정되면 무효화합니다. 무효화된 그 콜백에서 fresh 재계획이 즉시 실행되므로 P0로
  떨어지는 공백 사이클이 없습니다. 안전 검증 로직 자체는 변경 없음 — 우선순위만 바뀝니다.

### P3 completion handback = 글로벌 handoff 루프 (합류 지연의 실제 해법)

후보 순위는 slack 사전식이라 항상 가장 완만한(긴) exit를 선택하고, exit는 최대 ~18 m까지
커집니다. 이전에는 P3 완료 후 **불변 frozen tail**을 STATE_GLOBAL 확인까지 발행했는데, 이
tail은 ego 진행에 따라 잘리지 않아 몇 사이클 만에 차 뒤로 밀리고, FSM의 tail-도달 확인
창(`enter_global_*`)을 놓쳐 합류가 무기한 지연됐습니다 (2026-08-12 20:49 실주행:
COMPLETION_HANDOFF ~14초 지속, merge 확인 0회). 이제 P3 완료 시 **P0가 쓰는 것과 동일한
`activateGlobalHandoff` 폐루프**(현재 ego에서 시작하는 글로벌 라인 루프)로 전환합니다:
루프의 tail이 활성화 순간부터 ego 위치에 있으므로 FSM 확인이 수 초 안에 성립하고, 기존
P0 해제 로직(STATE_GLOBAL 확인 → 커밋 해제 → 빈 경로 발행)이 그대로 마무리합니다.

**복귀 램프 (2026-08-13 추가, `merge_ramp_min_length_m`/`merge_ramp_time_sec`)**: 위 폐루프가
d=0 라인을 그대로 발행하면 "라인까지 돌아가는 방법"이 계획에 없어서, 복귀 속도가 컨트롤러의
자연 수렴에 맡겨진다(2026-08-12 실측 0.055 m/m — 연속 장애물에서 다음 기동이 남은 오프셋 위에서
시작되며 누적). 이제 `buildGlobalHandoffPath`가 ego의 현재 d에서 0까지
smoothstep(여집합 (1-t)²(1+2t), 양 끝 기울기 0)으로 내려가는 램프를 핸드오프 앞머리
`max(min_length 3.0, |v|·1.5 s)` 구간에 접붙인다. FSM 합류 확정은 램프와 무관하게 물리적
`|ego_d| ≤ enter_global_threshold` 지속 조건이 계속 게이트하므로, 램프는 합류를 앞당길 뿐
조기 확정을 만들지 않는다. 램프의 추가 곡률 상한은 6|d0|/L² (d0=0.5, L=3에서 0.33 rad/m).

⚠️ **현재 기본 비활성(0/0)이다.** 시뮬 회귀(.regression_check10/11)에서 램프가 FINALS 벽
협착부 최소 여유를 0.117→0.082 m로 깎았고, 벽 클램프는 웨이포인트 d_left/d_right의 낙관
(제어팀 실측 0.16~0.23 m) 때문에 물리지 않았다. 합류 자체는 크게 좋아진다(FINALS 회피 완료
13.5→6.7 s). 제어팀 섹터별 라이다 벽 여유 테이블로 경계를 보정한 뒤 활성화하고, 활성화 시
반드시 락스텝 베이스라인을 다시 돌려 벽 여유를 확인할 것.

### 커밋 경로 retention 밴드 (`commitment_retention_reserve_fraction`) — 조건부 고정

"장애물이 (계획 당시의) 마진보다 안쪽으로 들어오지 않는 한 경로를 고정한다"는 요구
(2026-08-12 22:34, 종방향 연쇄 장애물에서 경로 흔들림)의 구현입니다. **이미 커밋된 경로**를
재검증할 때만 장애물 클리어런스의 추적오차 예약 부분을 이 비율(기본 0.5)로 줄여 검사합니다.
물리 클리어런스(0.158 m)는 절대 줄지 않고, 신규 계획·fresh 후보는 항상 전체 예약으로
검증합니다. 효과는 히스테리시스 밴드입니다:

- envelope가 점진 노출로 자라거나 흔들려도 **released 밴드(예약의 절반) 안이면 커밋 경로
  유지** — P3 continuation과 P0 commitment 모두.
- uncertainty guard 위반만으로는 더 이상 재계획하지 않습니다. 이전에는 guard 위반이
  N사이클(≈0.1~0.3 s) 지속되면 거의 동일한 기하를 다시 만드는 만료 규칙이 있었고, 이것이
  점진 노출 구간에서 보이는 경로 churn의 주범이었습니다 (만료 제거, 카운터는 진단용 유지).
- 원본 envelope + 물리 클리어런스 + 유지 예약이 **실제로 침범될 때만** 즉시 무효화→재계획.
- 1.0이면 밴드가 꺼지고 전체 예약 기준으로 복귀합니다.

**완료 장애물 id 등록(필수 유지)**: P3 완료 분기는 `clearCommitment()`가
`completed_obstacle_ids_`까지 지우기 때문에, 완료한 maneuver의 장애물 id를 wipe 전에 보관해
wipe 후 다시 등록합니다(P0 완료 흐름의 `resetForChainedManeuver`와 동일한 의미론). 검출기의
정적 차폐 hold(`static_lost_hold_sec`)로 통과 직후 장애물 track이 계속 발행되므로, 이 등록이
없으면 방금 통과한 장애물이 `buildNextManeuverInput`에 다시 들어와 ego가 그 padded span 안에
있는 상태로 "새 블로킹 클러스터"가 잡히고, stop prefix가 비어 즉시 safe-stop 래치 → 자기
위험구간 안 정지 → 영구 미해제로 스톨합니다 (2026-08-12 .regression_check2에서 두 시나리오
모두 이 경로로 실패, 등록 추가 후 해소).

### exit 길이 절대 상한 (`maximum_exit_length_m`) — 기본 비활성

exit를 절대 길이로 자르는 상한을 구현했으나 **기본 0(비활성)** 입니다. 회귀 실험에서 exit를
5 m로 자르자 경로가 일찍 d≈0으로 복귀했고, 3.6 m 뒤 두 번째 장애물을 detector가 드러내는
순간 잘린 exit가 그 inflated box를 관통해 커밋이 무효화됐으며(그 거리에선 fresh 재계획
불가) P0 safe-stop으로 정지했습니다. **긴 exit는 연쇄 장애물을 offset 유지 상태로 넘겨주는
역할을 하고 있으므로 자르면 안 됩니다.** 합류 지연은 위의 completion handback이 해결합니다.
단일 장애물이 보장되는 환경에서만 5.0 등으로 켜십시오.

### 간격 기반 회피 속도 상한 (`avoidance_minimum_speed_mps`)

`avoidance_velocity_limit_*`의 횡가속 한계는 곡률이 있을 때만 걸립니다. 그래서 직선에 놓인
장애물은 레이스라인 속도 그대로 계획되어 LUT에서 가장 큰 추종오차를 예약하고, 실제로는
지나갈 수 있는 간격이 막힙니다. 이를 곡률이 아니라 **간격** 기준으로 한 번 더 제한합니다.

- 장애물 s 구간을 지나는 waypoint마다 경로와 장애물 면 사이에 남은 횡여유를 구하고,
  거기서 `vehicle_half_width_m + safety_margin_m`를 뺀 값이 그 waypoint가 쓸 수 있는
  추종오차 예산입니다.
- LUT 예약이 그 예산 안에 들어갈 때까지만 감속하며, `avoidance_minimum_speed_mps` 아래로는
  내려가지 않습니다. 그 하한에서도 안 들어가면 후보를 infeasible로 두어 safe-stop이
  판단합니다. 지킬 수 없는 예약으로 기어서 통과하지 않습니다.
- 장애물 구간을 지나지 않는 waypoint는 감속하지 않으므로, 장애물 없는 랩과 넓은 간격의
  회피는 속도 손실이 없습니다.
- 역산은 validator 허용치를 더하지 않고 엄격하게 합니다. 예약이 허용치만큼 초과된 속도를
  반환하면 validator가 같은 허용치로 비교하면서 상쇄되어, 이 상한이 열어주려던 후보를
  오히려 탈락시킵니다.
- 기본값 2.0 m/s의 근거: 규정 최소 간격(자유 폭 0.5 m)에서 통과에 필요한 예약은 0.10882 m인데
  2.5 m/s의 직선 예약은 0.11833 m로 9.5 mm 부족했습니다. 2.0 m/s의 예약은 0.10667 m로
  안전계수 1.25를 유지한 채 들어갑니다.

타깃 게이트의 track 한계는 **차량 중심** 기준이므로 `wall_safety_margin_m`과 함께
`vehicle_half_width_m`도 뺍니다. 이것이 빠지면 footprint 검사가 절대 통과시킬 수 없는 타깃만
제안하게 됩니다. 레이스라인 속도 기준 게이트가 안 맞으면 `avoidance_minimum_speed_mps` 기준
게이트로 한 번 더 시도한 뒤에 그 side를 막습니다. 이는 *검토 대상*만 넓히며 *합격 기준*은
넓히지 않습니다 — 모든 후보는 최종적으로 자기 속도에서 hard validation을 통과해야 합니다. 글로벌 waypoint의 `d_left/d_right`는 reference에서 물리
track boundary까지의 거리입니다. 각 candidate waypoint의 실제 `x/y/yaw`에 0.56 x 0.287 m
직사각형 네 모서리를 배치하고, 동일 track branch의 local reference segment에 투영해 보간한
좌·우 boundary와 비교합니다. 벽 검사에서는 `wall_safety_margin_m=0.04 m`만 정확히 한 번
차감합니다. heading=reference heading이어도 물리 반폭을 포함하며, heading이 다르면 전후 corner
돌출이 추가됩니다. 추종오차·장애물 물리
clearance·simulator TTC sweep·scan-noise guard는 이 물리 track-bound 검사에 포함하지 않습니다.
종방향 uncertainty inflation은 접근 및 정지 시점을 보호하지만, 횡방향 팽창값은 최소·최대 모두
`0.0 m`라서 장애물 여유에 추가되지 않습니다.

### 3.3 최초 군집 안정화

처음 blocking 장애물이 들어오면 곧바로 좌우 spline을 확정하지 않습니다. 먼저 글로벌 `d=0` 위의
검증된 감속 prefix를 `ot_line=raceline_static_prepare`로 발행합니다. 가장 가까운 군집의 각
ID가 서로 다른 `/confirmed_static_obs` 메시지에서 `initial_observation_count`회 관측되고
`initial_observation_min_duration_sec`도 지날 때까지 wrap-aware Frenet 경계 합집합과 가장 큰
`s_var/d_var`를 누적합니다. planning timer가 같은 메시지를 여러 번 사용하더라도 관측 횟수는
한 번만 증가합니다. 기본 최소 0.15초를 함께 요구하므로 약 250Hz detector의 연속 3개 메시지만
약 12ms 동안 받은 상태에서 곧바로 commitment하지 않습니다.

기본 3회 관측과 최소 0.15초가 모두 끝나면 다음 순서로 고정 Guard를 만듭니다.

1. 같은 ID의 세 detector Frenet 경계를 폐루프 `s`를 고려해 합집합으로 만듭니다.
2. 좌표변환 없이 이 종·횡 경계를 최초 obstacle envelope로 사용합니다.
3. 종방향 양쪽에
   `uncertainty_min_longitudinal_inflation_m + uncertainty_sigma_scale * sqrt(s_var)`를
   더합니다.
4. 횡방향 최소·최대 팽창은 모두 `0.0 m`로 두어 `d_var`와 관계없이 2번의 실측 합집합을
   그대로 사용합니다.
5. 이 Guard 전체를 피하는 spline을 만들고 Guard와 경로를 함께 commitment에 저장합니다.

`s_var`는 종방향 Guard에만 사용합니다. 횡방향은 별도 covariance/fixed 팽창을 사용하지 않으며,
차량 중심 clearance 세 항만 실측 합집합에 적용합니다. 종방향 분산이 음수이거나 유한하지 않으면
sigma 항을 0으로 두고 종방향 최소 팽창값만 적용합니다. 관측 횟수와 최소 시간을 모두 만족하면
계획하며, 입력이 누락되어 관측 횟수를
채우지 못해도
`initial_observation_max_wait_sec`에 도달하면 그동안의 가장 보수적인 합집합과 분산으로 계획합니다.
장애물이 이미 `safe_stop_buffer_m` 안에 있어 감속 prefix조차 만들 수 없으면 3회를 기다리지 않고
즉시 zero-speed safe-stop을 latch합니다.

latch 해제 조건은 3개입니다: (A) 위험 구간 통과, (B) latch 장애물 대상 hard-valid 회피가
`safe_stop_release_cycles` 사이클 연속 유효, (C) 정지 상태에서 신선한 비어 있지 않은 detector
프레임이 전방 코리도 클리어를 명시적으로 입증. **B의 연속 카운터는 계획 유효성만 셉니다**
(2026-08-13 실차 교착 수리): 홀드 경로가 2점 퇴화 경로일 때 state machine이 AVOID↔GLOBAL로
틱마다 요동하는데, 구현이 GLOBAL 틱마다 카운터를 리셋하면 유효한 탈출 계획이 매 틱 존재해도
B가 영원히 미달해 수동 개입 없이는 빠져나오지 못합니다 (run_0813_220641/221339 재현).
FSM 선택 가능성은 카운터가 아니라 **해제 승인 순간**에 요구하므로, GLOBAL 틱에 해제되어
글로벌 라인이 장애물을 관통하는 일은 여전히 차단됩니다. 정지 상태 재계획 자체는 진입 길이가
자차→클러스터 실거리에 비례해 ~1.5 m 앞 정지에서도 후보가 성립합니다
(`StandstillCloseBehindObstacleStillPlansEscape` 회귀 테스트). 장애물 0.5 m 미만 초근접 정지는
여전히 후진 없이는 탈출 불가한 물리 영역으로, 후진 탈출 기동은 미구현 항목입니다.

### 3.4 좌우 목표 d 계산

먼저 장애물에 적용할 차량 중심 계획 clearance `C_obs`를 계산합니다.

```text
C_obs = vehicle_half_width_m + safety_margin_m
      + max_obstacle_span LUT(limited_reference_v, |reference_kappa|)
```

- 왼쪽 최소 후보: 군집 Guard의 가장 큰 `d_left + C_obs`
- 오른쪽 최소 후보: 군집 Guard의 가장 작은 `d_right - C_obs`

각 방향에서 이 최소 offset부터 obstacle span의 트랙 경계와
`maximum_target_offset_m`가 허용하는 최대 offset까지 `target_d_candidate_count=5`개를 균등
표본화합니다. 후보 생성 단계의 예비 centerline 범위는
`[-d_right + wall_safety_margin_m, d_left - wall_safety_margin_m]`입니다.
이 범위를 벗어나면 폐기합니다. 이때 장애물 군집의 확대된 `s_start~s_end` 구간에서 `target_d` 자체가 이 범위를
벗어나는 방향은 5차 전환 프로파일을 만들기 전에 조기 폐기합니다. 이 검사는 명백히 불가능한 방향의
전체 target/전환 후보 생성을 생략하기 위한 gate이며, 통과한 방향도 전환 구간의 좁은 벽이나 다른
장애물을 놓치지 않도록 실제 직사각형 footprint hard validation을 포함한 전체 waypoint
경계·충돌·곡률 검사를 그대로 수행합니다.

완성된 candidate에는 추가 hard validation을 적용합니다.

1. candidate의 map-frame `x/y/yaw`에 0.56 x 0.287 m 직사각형 네 모서리를 계산합니다.
2. 각 corner를 같은 track branch의 인접 reference segment에 투영하고 segment 비율로
   `d_left/d_right` 물리 boundary 거리를 보간합니다.
3. 네 모서리 중 하나라도 `wall_safety_margin_m`을 적용한 boundary 밖이면
   `footprint_track_bound`로 즉시 폐기합니다.
4. `PLAN_CANDIDATE`에는 centerline/footprint 최소 clearance, 위반 side/index/s/x/y/yaw,
   reference 대비 heading과 wallward corner protrusion을 기록합니다.

이 검사는 물리 footprint만 사용합니다. simulator의 약 20 mm TTC sweep과 7.5 mm scan-noise
guard는 evaluator consistency 진단에만 남으며 production vehicle 크기에는 더하지 않습니다.
기존 diagnostic에서 waypoint 및 10 ms interpolation sampling loss가 모두 0이었고 실제 글로벌
waypoint 간격도 충분히 조밀하므로 이번 구현은 path geometry를 바꾸는 별도 continuous solver를
추가하지 않고 모든 기존 waypoint pose를 결정적으로 검사합니다.

안정화가 끝난 최초 계획에서는 허용된 양쪽의 모든 `target_d × entry × exit` 조합을 먼저
생성합니다. hard-invalid 후보를 제거한 뒤 다음 normalized slack의 최솟값이 가장 큰 후보를
선택합니다.

```text
S = min(wall_headroom / maximum_target_offset,
        obstacle_headroom / maximum_target_offset,
        (kappa_limit - peak_abs_kappa) / kappa_limit,
        (kappa_rate_limit - peak_rate) / kappa_rate_limit)
```

`S`가 수치 허용오차 안에서 같으면 velocity-limit에 의한 평균 speed loss가 작은 후보, 그다음
평균 `|d|`가 작은 후보를 고릅니다. 임의 가중합을 사용하지 않으므로 벽 여유 0~7 mm인 후보는
장애물과 벽 사이에 더 균형 잡힌 후보가 있는 한 우선 선택되지 않습니다. 완전 동률은 고정된
생성 순서로만 해소해 lockstep 결과를 결정적으로 유지합니다.

#### 순위의 `wall_headroom`은 차체 기준이다 (2026-08-15)

`S`의 두 여유 항은 **같은 기준점**에서 재야 합니다.

```text
obstacle_headroom = 경로 → 장애물 면 거리 − (vehicle_half_width + safety_margin + 추종오차 튜브)
wall_headroom     = 경로 → 트랙 경계 거리 − (wall_safety_margin + vehicle_half_width + 추종오차 튜브)
```

이전에는 `wall_headroom`이 **경로 중심선 기준**이라 `wall_safety_margin_m` 하나만 뺐습니다.
장애물 항은 반폭 + 안전마진 + 튜브(최대 0.455 m)를 이미 쓰고 있었으므로 두 항의 영점이
어긋났고, 그 최솟값을 최대화하면 균형점이 항상

```text
편향 = (obstacleSafetyClearance − wall_safety_margin_m) / 2      ← 최대 0.257 m
```

만큼 **벽 쪽으로** 밀렸습니다. 이 편향은 틈의 폭과 무관한 상수라, 장애물과 벽 사이가 아무리
널널해도 차가 벽에 붙어 지나갔습니다. 실측(2026-08-15, `stuck_case_harness`, 라이브 기준선
`output/map`, 자차 s=30 v=2.5, 장애물 s=[35.0,35.5]):

| 장애물 면↔벽 틈 | 기존 차체→벽 / →장애물 | 수정 후 차체→벽 / →장애물 |
|---|---|---|
| 1.40 m | 0.404 / 0.709 | **0.556 / 0.556** |
| 1.20 m | 0.202 / 0.711 | **0.406 / 0.507** |
| 1.00 m | 0.152 / 0.560 | **0.256 / 0.457** |
| 0.90 m | 0.127 / 0.486 | **0.181 / 0.431** |

⚠️ **이것은 순위 항일 뿐이며 hard validation은 그대로입니다.** 트랙 경계 검증은 여전히
회전 사각형 코너에 `wall_safety_margin_m`만 정확히 한 번 적용하고, 추종오차 튜브를 더하지
않습니다. 따라서 **feasible 후보 집합이 바뀌지 않아 회피/안전정지 판정 자체는 변하지
않습니다** — 같은 후보들 중 어느 것을 고르느냐만 바뀝니다.

`wall_clearance_m`(진단·audit)은 이제 이 차체 기준 값이고, 기존 중심선 여유는
`centerline_wall_clearance_m`로 계속 발행됩니다.

장애물에는 `C_obs`를 한 번 적용하고, 트랙 경계 **검증**에는 `wall_safety_margin_m`만 한 번
적용합니다.
완성된 spline은 재계산된 waypoint 곡률로 회피속도를 제한한 다음, 그 속도와 곡률로 LUT를 다시
보간해 장애물 clearance를 점별 검사합니다. 별도 commitment reserve와 reduced-clearance
fallback은 없습니다. 한쪽이 불가능하면 반대쪽을 평가하고, 양쪽 모두 불가능하면 safe-stop으로
넘어갑니다.

회피속도 표는 `upstream/jazzy_main`의
`offline_trajectory_generator/config/velocity_limits.csv` 커밋 `3d5fb38`에서 speed와
`max_lateral_accel` 열을 옮겼습니다. 0~5 m/s는 7.0 m/s², 6~9 m/s는 6.5 m/s²이며 중간값은
선형 보간합니다. 이 제한은 정상 회피 spline에만 적용하고 safe-stop 감속과 global handoff 속도는
각자의 기존 규칙을 유지합니다.

commitment 뒤 기존 경로가
위험해졌더라도 ego가 `commitment_lock_lateral_threshold_m`만큼 횡이동하거나
`commitment_lock_longitudinal_m`만큼 전진하기 전이라면 반대쪽도 다시 평가할 수 있습니다.
단, 진입 전 한 번 반대편으로 전환한 뒤에는 다시 되돌리지 않습니다. 정중앙 장애물의 동점
재계획이 반복되어도 차량이 좌우로 흔들리지 않도록 하기 위함입니다.
둘 중 하나에 도달해 실제 회피에 진입한 뒤에는 진행 중 갑자기 반대편으로 꺾지 않도록 방향을
고정합니다. 측정 흔들림은 고정 uncertainty Guard가 담당하고, 물리 안전거리는 경로 생성과
재검증 모두 같은 `C`를 사용합니다.

### 3.5 로컬 d-offset spline

진입과 복귀를 서로 독립적인 5차 smoothstep 구간으로 만듭니다. 정규화된 진행률
`u=clamp((s-s_start)/length, 0, 1)`에 대해 다음 함수를 사용합니다.

```text
q(u) = 10u^3 - 15u^4 + 6u^5
entry d(s) = ego_d + (target_d - ego_d) * q(u)
exit  d(s) = target_d + (0 - target_d) * q(u)
```

`q`는 양 끝에서 1차와 2차 미분이 모두 0입니다. 따라서 `ego_d` 유지 구간에서 횡이동을 시작할
때, 장애물 앞에서 `target_d` 유지 구간에 들어갈 때, 장애물 뒤에서 글로벌 `d=0`으로 복귀할 때
`d`, `dd/ds`, `d2d/ds2`가 연속입니다. natural cubic처럼 중간 제어점 때문에 반대 방향으로
overshoot하지 않으며, 결과 `d`는 `ego_d`, `target_d`, `0`의 최소·최대 안에 머뭅니다.

`pre_apex_distances_m=[먼 점, 중간 점, 가까운 점]`에서 먼 점은 진입 nominal 길이입니다.
중간·가까운 점의 `d`는 위 식에서 계산되므로 모두 0으로 고정되지 않고 목표 쪽으로 점진적으로
이동합니다. `post_apex_distances_m=[가까운 점, 중간 점, 먼 점]`도 같은 방식으로 복귀 구간을
표시하며, 먼 점이 전체 복귀 길이입니다. 중간·가까운 값은 실제 프로파일을 RViz 제어점으로
표본화하는 위치이고 프로파일 자체를 꺾지 않습니다. 현재 운영값은 각각
`[9.6694674, 6.4463116, 3.2231558]`, `[1.7024449, 3.4048897, 5.1073346]` m입니다.
`entry_transition_fractions=[0.5, 0.75, 1.0]`의 각 값에 대해
ego에서 확대 군집 시작까지 실제 가용 거리 `available`을 사용해 다음처럼 실제 진입 길이를
계산합니다.

```text
effective_entry = available
                * (pre_apex_far / detection_lookahead)
                * entry_fraction
```

`pre_apex_far <= detection_lookahead`와 설정된 `entry_transition_fractions`에 대해
`0 < entry_fraction <= 1`을 요구하므로 기존
`min(requested, available)` clamp와 달리 nominal 값이 커지면 실제 시작 위치와 형상이 항상
선형으로 바뀝니다. 이 세 기존 후보는 값과 생성 순서를 그대로 유지합니다. 그 뒤 YAML이나 CMA
parameter를 추가하지 않고 `entry_scale=detection_lookahead/pre_apex_far`로 계산한 후보를 정확히
하나 덧붙입니다. 이 후보는 위 식의 effective fraction이 1이므로 ego에서 확대 군집 시작까지의
실제 가용 거리 100%를 진입에 사용합니다. 진입은 이 값들만 사용하고,
`transition_distance_scales`는 복귀에만 사용합니다.
`outside_line_transition_scale`도 바깥쪽 복귀에만 적용하므로 entry와 exit 의미가 다시 섞이지
않습니다.

이 운영값과 `safety_margin_m=0.0147893`, `obstacle_longitudinal_padding_m=0.4149925`,
`transition_distance_scales=[0.2740569, 0.6991538, 3.5816013]`는 변경된 동일 장애물 배치를 대상으로
1.5 m/s CMA-ES 탐색 후 동일 후보를 포함해 3회 연속 무충돌 완주한 조합입니다.

그다음 ego부터 merge 뒤 global tail까지의 글로벌 waypoint를 순서대로 복사합니다. tail 길이는
다음처럼 거리 하한과 계획 당시 속도 기준 시간 하한 중 큰 값입니다.

```text
tail_distance = max(post_merge_lookahead_m, ego_speed * post_merge_min_time_sec)
```

따라서 고속에서도 상태 전환이 끝나기 전에 열린 회피 경로의 끝점에 도달하지 않습니다.
각 점은 다음 식으로만 이동합니다.

```text
x_local = x_global - d(s) * sin(psi_global)
y_local = y_global + d(s) * cos(psi_global)
```

`s_m`과 글로벌 진행 순서는 바뀌지 않습니다. 가까운 다른 트랙 조각을 검색하는 단계가 없으므로
비볼록 트랙에서도 잘못된 branch로 이동하지 않습니다.

### 3.6 안전성과 속도

완성된 후보는 다음 조건을 모두 통과해야 합니다.

1. 모든 점이 해당 글로벌 waypoint의 좌우 트랙 폭 안에 있음
2. 팽창된 정적 장애물 Frenet 상자와 겹치지 않음
3. `|dd/ds|`, Cartesian 곡률, 거리당 곡률 변화율 제한 만족
4. 이동된 geometry에서 계산한 곡률로 횡가속도 속도 상한 만족
5. 전·후방 속도 패스로 종가속도와 종감속도 제한 만족

`entry_transition_fractions`의 기존 세 후보와 추가한 actual-available 100% 후보 각각에 대해
`transition_distance_scales`의 모든 조합을 각 target 표본마다 평가합니다. 기존 이름을 호환성
때문에 유지한 `transition_distance_scales`는 이제 exit-only이며,
첫 valid 조합을 즉시 반환하지 않습니다. hard validation 뒤 wall/obstacle/curvature/curvature-rate
slack 순위가 실제 선택을 결정합니다. 이것은 자유공간 탐색이 아니라 동일한 ordered 글로벌 점에
적용하는 target과 5차 프로파일 형상 비교입니다.

좌우가 모두 실패해도 곧바로 정지로 가지 않습니다. 두 단계의 완화가 먼저 적용됩니다
(2026-08-12 21:11 실주행 분석 반영).

**1) margin-only 감속 통과 (`margin_pass_speed_cap_mps`)**: "회피가 필요한가"와 "정지가
필요한가"는 다른 판정입니다. blocking 판정은 장애물 상자를 `기본 클리어런스(0.158 m) + 추종오차
LUT 예약(헤어핀 0.28 m)`만큼 부풀리므로, 라인 밖 장애물(예: 헤어핀 바깥쪽)도 "막는다"로
들어옵니다. 그러나 클러스터의 모든 멤버가 **원본 상자 + 물리 클리어런스(차폭반 0.1435 +
safety_margin 0.0148)** 기준으로 d=0에 닿지 않으면(margin-only) 라인 자체는 물리적으로 주행
가능합니다. 이때는 safe-stop 대신 라인을 그대로 따라가되 속도를 `margin_pass_speed_cap_mps`
(기본 2.0)로 제한한 **감속 통과 경로**(kind=kAvoidance, `margin_pass=true`)를 커밋합니다.

⚠️ **접근 실현성 램프(`approach_feasibility_decel_mps2`, 기본 2.0, 2026-08-14)**: 캡을 자차
위치부터 flat으로 명령하면 자차가 캡보다 빠른 순간 계단 감속이 됩니다 — 0814 실차에서
4.4 m/s 접근에 flat 2.0이 발행돼 서비스 브레이크 포화 → 마찰 한계 초과 슬립(조향 상실) →
벽 충돌로 이어졌습니다(run_0814_111210 t=188.7). 지금은 군집 시작 전 구간에 한해 **실측 자차
속도에서 이 감속도로 내려가는 프로파일**까지 허용하고, 군집 시작까지 캡 도달이 안 되면 닿는
만큼만 더 가파르게 잡습니다(여유가 전혀 없으면 기존 flat 캡으로 수렴). 같은 파라미터가 회피
spline 경로에도 적용됩니다: 간극/곡률 캡으로 느려진 장애물 스팬 앞에, 스팬 시작 속도에서
거꾸로 올라가는 **후방 제동 램프**를 씌워(낮추기만 함) 제동이 스팬 경계 계단 대신 훨씬 전에
완만하게 시작되게 합니다. 스팬 내부 속도(예약 기반)는 절대 건드리지 않습니다. `0 이하`면
비활성(구 계단 거동).
margin-pass 커밋은 의도적으로 마진 밴드 안을 지나므로 마진 기반 커밋 검증을 건너뛰고, 대신
매 사이클 물리 blocking 재검사로 지킵니다 — 원본 상자가 라인에 닿는 순간 일반 재계획으로
복귀합니다. 최초 안정화 대기(preparation) 단계에서도 margin-only면 정지 준비 대신 같은 감속
통과를 발행합니다. `0.0`이면 기능이 꺼지고 기존 escalation으로 복귀합니다.

**2) 물리적으로 막힌 경우의 정지**: 글로벌 `d=0` 위에서 longitudinal padding이 적용된 장애물
경계 앞 `safe_stop_buffer_m`까지의 충돌 없는 prefix를 만들고 마지막 속도를 0으로 둡니다.
현재 설정은 `safe_stop_buffer_m=2.60 m`입니다.

**2-a) 정지점 탈출 검증 (2026-08-14 신규)** — 정지 자체보다 **어디에 서느냐**가 교착을
만듭니다. 실차에서 버퍼가 1.20 m였을 때 탈출 임계(2.0~2.5 m)보다 작아, 정지하는 순간 이미
회피 후보가 0개 생성되는 구역이었습니다(`run_0101_090639`: 장애물 1.6 m 앞에 28.8초 정지,
그 지점 좌 1.17 m / 우 1.36 m로 **횡공간은 충분**했음).

`safe_stop_escape_check_enable`이 켜져 있으면 정지점을 확정하기 전에 그 지점에서 `v=0`으로
회피 경로가 생성되는지 확인합니다. 안 되면 **탈출 가능한 가장 늦은 지점**까지 물립니다.
탈출 가능성은 정지점을 뒤로 물릴수록 단조 증가하므로(진입 거리가 길어짐) 선형 후퇴가 아니라
이분탐색을 씁니다. 판정에는 `plan()`과 **같은 후보 생성 함수**(`generateP3Candidates`)를
쓰므로 "정지점에서 회피 가능"이라는 판정과 실제 재계획이 어긋날 수 없습니다.

어느 지점에서도 불가능하면 원래 정지점을 유지하고(후퇴가 아무것도 사지 못하므로),
`RacelineSplineResult::safe_stop_escape_verified=false`와 reason 경고를 남기며 노드가
2초 throttle ERROR를 찍습니다. 이전에는 이 상태가 아무 로그 없이 30초씩 매달렸습니다.

⚠️ **좌표계 주의**: `ExpandedObstacle`의 `center/start/end`는 **자차 상대거리**입니다
(`expandVisibleObstacles`가 `forwardDistance(ego.s, ...)`로 만듭니다). 따라서 가상의 정지점
기준으로 판정하려면 **절대 s를 담은 원본 장애물로 그 지점에서 다시 확장**해야 합니다.
`ego.s`만 옮기고 기존 `visible/cluster`를 재사용하면 장애물이 정지점에서도 같은 거리에 있는
것으로 보여 검증이 통째로 무의미해집니다(이분탐색도 항상 같은 답). 2026-08-14 리뷰에서
실제로 이 상태로 한 번 커밋됐다가 수리했습니다. 회귀 가드:
`EscapeCheckReexpandsObstaclesAtTheCandidateStopPoint`.

비용(랩톱 실측, `plan()` 1회): 회피가 성립하는 정상 경로는 7.40 → 7.55 ms로 **사실상 변화
없음**(검증은 안전정지 경로에서만 호출). 탈출이 아예 불가능한 최악의 안전정지 경로가
5.30 → 14.85 ms(선형 후퇴 초안은 23.80 ms였습니다). 젯슨에서 안전정지 중 루프 지연이
관측되면 `safe_stop_escape_check_enable: false`로 즉시 이전 동작으로 돌아갑니다.

**2-b) 정지 prefix 세분 보간 (2026-08-14 신규)** — 정지 prefix는 `minimum_path_points`보다
짧아도 거부하지 않지만, 그 상태로 발행하지도 않습니다. 이전에는 가드가 `size()<2`뿐이라
2점 경로 `[v, 0]`이 그대로 나갔는데, 제어기(`control_map_node`)는 **룩어헤드 지점의 속도**를
읽으므로 2점에서는 룩어헤드가 곧바로 끝점 `0`에 걸려 감속 프로파일을 통째로 건너뛰고 즉시
정지를 명령했습니다(실차 관측: `/local_waypoints [1.08, 0.00]` → `/drive_autonomous 0.00`).
이제 최장 구간을 반복 이등분해 `minimum_path_points`를 채우고, 보간으로 생긴 점에도 같은
`sqrt(2·a·거리)` 제동 프로파일을 다시 씌웁니다(선형 보간 속도는 항상 낙관적이므로).
보간은 점 수만 늘리고 정지 지점은 옮기지 않습니다.

**3) stop prefix가 없을 때 — 직전 유효 경로 제동**: 장애물이 이미 buffer 안에 있어 prefix를
만들 수 없으면, 그 자리 0속도 emergency hold 대신 **마지막으로 발행한 유효 안내 경로**
(`last_valid_guidance_path_`: 회피 spline·P3 maneuver·handoff 루프·margin-pass 모두 포함)를
따라 `safe_stop_deceleration_mps2`로 제동하는 경로를 발행합니다(우선 그 경로 위 충돌 전
prefix, 그것도 없으면 충돌 탐색 없는 순수 제동 `buildLastPathBrake`). 검증된 기하 위에서
조향 연속성을 유지한 채 같은 감속 권한으로 멈춥니다. 유효 경로가 아직 하나도 없을 때만
기존 emergency hold가 최후 fallback으로 남습니다. 따라서 정지 실패가 빈 `/avoid_waypoints`를
통해 global 경로 선택으로 이어지지 않습니다.

### 3.7 commitment와 합류

안전 경로가 선택되면 방향, 경로 geometry, ID별 uncertainty Guard를 고정합니다. 매
`/confirmed_static_obs`에서 같은 ID의 최신 Frenet 경계에도 동일한 uncertainty 확장을 적용합니다. 그 전체가
저장된 Guard 안에 있으면 live envelope 대신 고정 Guard로 기존 경로를 재검증하므로 중심과 크기가
조금 변해도 `target_d`와 출력 waypoint가 바뀌지 않습니다. Guard는 직전 관측을 따라 이동하지
않으므로 작은 변화가 누적된 실제 이동은 결국 Guard 밖으로 나옵니다.

최신 uncertainty envelope가 Guard를 벗어나면 기존 경로를 두 단계로 검사합니다.

1. detector 원본 Frenet 경계에 동일한 `C_obs`를 적용한 hard 영역과 겹치면 실제 차체 충돌
   가능성이므로 그 planning cycle에서 즉시 재계획합니다.
2. 원본 hard 영역은 피하지만 uncertainty Guard에 동일한 `C_obs`를 적용한 영역만 침범하면 soft
   충돌로 분류합니다. `commitment_soft_violation_confirm_cycles`회 연속일 때만 재계획하고,
   그 전에 해소되면 카운터를 지우고 고정 경로를 유지합니다.
3. 충돌 로그에는 장애물 ID, 충돌 waypoint의 `s/d`, 장애물 `s` 범위, 입력 및 검사 `d` 범위와
   적용된 장애물 clearance `C_obs`를 기록합니다.

soft와 hard의 차이는 마진 크기가 아니라 검사 입력입니다. soft는 불확실성이 포함된 Guard,
hard는 detector raw AABB를 검사합니다. 둘 다 차량 반폭, `safety_margin_m`, 해당 충돌
waypoint의 LUT 추종오차를 정확히 한 번만 사용합니다.

기본 planning 주기 25ms와 3회 확인은 약 75ms입니다. hard 충돌과 경로 끝 소진, 트랙 경계 및
기하 오류에는 이 지연을 적용하지 않습니다. 재계획 시 회피 진입 전에는 반대편 전환을 commitment당
한 번만 허용하고(정중앙 동점 재계획에 의한 좌우 진동 방지), 이후 또는 회피 진입 후에는 같은
방향만 평가합니다. 허용된 방향의 대체 경로도 불가능하면
`raceline_static_safe_stop`을 latch합니다. 최초 activation에서 장애물 ID 목록과 입력 sequence,
확장된 danger `s_start/s_end`, 정지 target `s`, 당시 ego `s`, activation ROS timestamp를 함께
고정합니다. 이후 detector가 empty array를 보내도 이 danger region이 ego 앞에 있는 동안 latch와
기존 정지 경로를 유지합니다. empty cycle 수 자체는 release 판정에 사용하지 않습니다.

release는 다음 세 조건으로만 허용합니다.

1. ego가 latched danger end를 `safe_stop_buffer_m`만큼 더 통과했습니다.
2. 동일 latched obstacle ID를 대상으로 새 hard-valid avoidance가 생성되고 state machine이
   `STATE_AVOID`라 실제 선택할 수 있는 상태가 `safe_stop_release_cycles`회 연속 확인됐습니다.
3. 차량 속도가 한 planning cycle의 설정 감속량 이하로 내려간 뒤, 비어 있지 않은 새 detector
   frame들이 detector health를 증명하면서 전방 lookahead corridor가 clear임을 같은 횟수 동안
   명시적으로 확인했습니다. empty array는 이 확인에 포함되지 않습니다.

기본 25ms 주기와 8회 설정은 조건 2/3의 0.2초 보조 debounce입니다. danger region이 여전히 앞이면
`raceline_global_handoff` 생성 자체를 거부합니다. 조건 2는 global handoff가 아니라 검증된
avoidance를 선택합니다. safe-stop이 활성화된 동안 state machine은 정지 경로의 `d=0` tail을 합류
완료로 해석하지 않고 `STATE_AVOID`를 유지합니다. 정지 waypoint의 속도와
`safe_stop_deceleration_mps2` 계산은 lifecycle 변경과 분리되어 기존 값을 그대로 사용합니다.

장애물을 지난 뒤 perception에서 물체가 사라져도 검증된 spline과 뒤쪽 global tail을 유지합니다.
ego가 실제 spline merge 지점에 도달하고
`|ego d| <= merge_lateral_tolerance_m`을 `merge_confirm_cycles` 동안 만족하면 기하학적 합류가
확인됩니다.

기하학적 합류만으로 commitment를 해제하지는 않습니다. 합류가 확인되면 전체 global waypoint를
원래 순서 그대로 한 번 포함하는 폐루프 handoff 경로로 교체합니다. 배열 시작점만 회전해 현재
ego가 경로 끝 `state_handoff_tail_distance_m`[m] tail 구간의 첫 점에 위치하도록 하고
`ot_line=raceline_global_handoff`를 설정합니다. state machine은 이 표식을 받으면 고정 tail을
다시 만날 때까지 기다리지 않고 실제 ego가 global line에 0.5초 동안 유지되는지만 확인합니다.
컨트롤러에는 충분한 전방 global 경로가 계속 제공됩니다.

현재 commitment에서 `/state`의 `STATE_AVOID`를 한 번 이상 확인한 뒤 `STATE_GLOBAL` 복귀가
발행될 때까지 이 non-empty 폐루프를 계속 발행합니다. `STATE_GLOBAL` 확인 후에만 빈
`/avoid_waypoints`를 발행합니다. 기본 `planning_period_ms=25`, `merge_confirm_cycles=15`의
기하 확인 시간은 0.375초입니다. handoff 중 waypoint 속도는
`state_handoff_speed_cap_mps` 이하로 제한합니다.

### 3.8 연속 장애물 maneuver 연결

현재 commitment에 포함되지 않은 blocking 장애물은 기존 spline의 `merge_s` 전후와 관계없이
현재 ego 위치를 기준으로 **다음 maneuver 후보**로 미리 관측하고 안정화합니다. 따라서 완만한
복귀 구간 안에 다음 장애물이 들어와도 old merge 뒤로 넘어갈 때까지 관측 시작을 미루지 않습니다.

기존 merge 전의 장애물은 다음 경로가 실제로 커밋될 때까지 현재 commitment의 충돌 검사에도
계속 포함합니다. 기존 경로가 그 장애물과 충돌한다면 안전 검사를 생략하지 않습니다. 반면 실제
merge 뒤 controller 시야 확보용 global tail만 겹치는 장애물은 현재 maneuver를 실패시키지
않습니다.

다음 maneuver 군집은 첫 회피를 수행하는 동안에도 기존 최초 관측 조건, 즉 각 ID의 실제
`/confirmed_static_obs` 3회 관측, 최소 `initial_observation_min_duration_sec=0.15초`와 최대
`initial_observation_max_wait_sec=0.35초`를 사용해 동시에 안정화합니다. 현재 장애물 Guard의
뒤쪽을 `chain_release_distance_m`만큼 완전히 지난 뒤 다음 군집이 안정화되어 있으면 다음 순서로
직접 연결합니다.

1. 완료한 군집 ID를 이번 연속 회피가 끝날 때까지 제외 목록에 넣습니다.
2. 이전 maneuver의 좌우 방향 잠금을 해제합니다.
3. 현재 측정된 `ego.s`와 `ego.d`를 새 spline의 시작점으로 고정해 좌우를 새로 평가합니다.
4. 안전한 이동 경로가 있으면 기존 commitment를 다음 경로로 원자적으로 교체하고
   `STATE_AVOID`를 유지합니다. 중간 빈 경로, global 경로, 불필요한 정지는 발행하지 않습니다.
5. 아직 안전한 다음 경로를 만들 수 없으면 현재 commitment를 merge까지 유지합니다. merge에
   도착하면 미리 누적한 관측을 그대로 승계해 다음 계획을 이어갑니다.

이미 `raceline_global_handoff`를 발행 중이어도 새 blocking 군집이 들어오면 handoff를
선점합니다. 모든 미완료 blocking 군집이 사라진 뒤에만 최종 global handoff를 완료합니다.
안전한 이동 경로가 전혀 없을 때에는 먼저 현재 committed geometry 위에서 충돌 전까지
감속합니다. 그 prefix조차 만들 수 없을 때만 현재 `ego.d`를 유지하는 zero-speed hold를
최악 상황의 마지막 수단으로 사용합니다.

### 3.8.0 기동 범위 충돌 지평은 하나뿐이다 (2026-08-16)

한 기동의 **장애물** 검사 범위는 `확장 클러스터 끝 + post_merge_lookahead_m`입니다. 트랙 경계와
기하(곡률·경사) 검사는 지평과 무관하게 경로 전체에 적용됩니다. 이 범위는 다음 네 곳에서
**같아야** 합니다.

| 지점 | 함수 |
|---|---|
| 후보 검증 | `P3ShadowEvaluator::buildCandidate` |
| fresh 선택 | `P3ManeuverLifecycle::selectFresh` |
| continuation 재검증 | `P3ManeuverLifecycle::continueCurrent` |
| P0 재검증 | `RacelineSplinePlanner::generateP3Candidates` |

두 지점이 서로 다른 범위로 같은 경로를 판정하면 그건 더 엄격한 안전 검사가 아니라 **무한
루프**입니다. 넓은 쪽이 좁은 쪽의 합격 경로를 매번 기각하므로 계획 주기마다 선택과 폐기가
반복되고, 그 시점이 `safe_stop_buffer_m` 안이면 그대로 정지로 굳습니다.

2026-08-16 백에서 P3 라이프사이클만 지평이 없었습니다. 결과는 s=31.7 장애물 3 m 이내에서
fresh 후보 245/245 전멸(`NO_HARD_VALID_M1_CANDIDATE`)인데 같은 순간 P0의 `plan()`은 6개 중
3개를 통과시켰고, 매 랩 s=28~30에서 완전 정지했습니다. 한 곳에만 지평을 넣지 마십시오.

### 3.8.0.1 다음 장애물에 닿는 exit은 순위에서만 뒤로 민다

`exit_reaches_next_obstacle`은 클러스터 끝 이후 **merge 지점까지의 exit 램프**에서, 이 기동의
클러스터가 아닌 장애물의 물리 엔벨로프에 경로가 닿는지를 봅니다. merge 뒤 글로벌 꼬리는
정의상 `d=0`이므로 검사에 넣으면 라인 위 장애물에 대해 모든 후보가 참이 되어 우선순위가
무력해집니다.

이 후보를 **거부하지는 않습니다.** 장애물 간격이 좁으면 오프셋을 그대로 넘겨주는 것이 설계된
동작이고, 거부하면 2026-08-12/08-15의 "후보 전멸 → 영구 크립" 회귀가 돌아옵니다. 닿지 않는
exit이 하나라도 있으면 그쪽을 쓰고, 전부 닿으면 기존 slack 기준이 그대로 결정합니다.
`betterFeasible`(P3)과 `better_candidate`(P0)가 이 항을 **동일하게** 적용해야 합니다.

### 3.8.1 STATE_AVOID에서는 빈 경로를 내보내지 않는다 (2026-08-16)

state machine의 GLOBAL 복귀 판정(`enter_to_global`)은 **최신 `/avoid_waypoints`가 비어 있지
않을 때만 실행**됩니다. 빈 메시지에는 타임아웃도 대체 경로도 없으므로, AVOID 상태에서 빈
경로를 발행하면 FSM은 AVOID에 영구히 갇힙니다(그 뒤로 컨트롤러의 섹터 속도 스케일링도 계속
꺼진 상태가 됩니다). 유령 장애물이 commitment 전에 사라지는 경우가 이 함정의 전형입니다.

두 지점에서 막습니다.

1. 커밋이 없는 상태에서 트랙이 비면, "준비 감속을 발행했는가"가 아니라 **"직전 발행이
   non-empty였는가"**를 기준으로 글로벌 핸드오프 루프로 돌려줍니다. 안정화 중 조기회피
   분기가 `initial_prepare_published_`를 지우기 때문에, 예전 기준으로는 이 경로가 그대로
   빈 경로로 빠졌습니다.
2. 마지막 `publishEmpty` 지점에서 `plan()`이 `kNoObstacle`을 돌려주고 `/state`가 여전히
   `STATE_AVOID`이면, 빈 경로 대신 닫힌 글로벌 핸드오프 루프를 발행합니다. 이 루프는 tail이
   ego에 놓이고 `d=0`이라 FSM의 tail 도달·횡오차 게이트를 그대로 만족시켜 **정상 판정 경로로**
   GLOBAL 복귀가 확정됩니다. GLOBAL이 확인되면 기존 handoff 릴리즈 분기가 commitment를 지우고
   다시 빈 경로로 돌아갑니다.

`kNoObstacle`로 한정하는 이유는 그것만이 "트랙이 실제로 비었다"를 증명하는 결과이기
때문입니다. `kNoSafePath`/`kSafeStop`은 종전대로 안전정지 경로를 탑니다.

### 3.9 장애물 센서 stale과 다음 랩 기억

유효한 `/confirmed_static_obs`를 한 번 이상 받은 뒤
`obstacle_stale_timeout_sec` 동안 새 메시지가 없으면 planner는 **degraded perception
mode**로 전환합니다. 이때 stale을 장애물이 사라졌다는 뜻으로 해석하지 않습니다.

1. 진행 중인 검증된 회피 spline과 방향 commitment를 그대로 유지합니다.
2. Frenet odometry로 merge 도달을 계속 확인하고, 합류 뒤에는 평소와 같은
   `raceline_global_handoff`를 발행합니다.
3. `/state`가 `STATE_AVOID`를 거쳐 `STATE_GLOBAL`로 복귀하면 회피 출력은 정상적으로
   종료합니다. 센서 stale만으로 차량을 정지시키지 않습니다.
4. 마지막 유효 장애물 스냅샷은 지우지 않습니다. 센서가 계속 끊긴 채 다음 랩에서 같은
   장애물이 lookahead에 들어오면, 새 관측을 기다리는 준비 감속 없이 저장된 uncertainty
   Guard로 즉시 회피 계획을 다시 만듭니다.
5. frame이 잘못된 장애물 배열은 무시하되 기존 기억은 보존합니다. 올바른 frame의 새 배열이
   도착하면 빈 배열도 유효한 최신 관측으로 보고 저장된 기억을 교체합니다.

정지는 센서 stale 자체가 아니라 저장된 장애물에 대해 양쪽 회피와 검증된 정지 prefix가 모두
불가능하거나, 충돌 위험이 발생한 경우에만 사용합니다. Frenet odometry가
`odometry_stale_timeout_sec`를 넘겨 차량 위치를 신뢰할 수 없는 경우는 최악 상황으로 분류해
마지막으로 알려진 `s/d`에서 모든 속도가 0인 emergency hold를 발행합니다. 이때도 기존
commitment는 지우지 않으므로 odometry가 회복되면 다시 검증한 뒤 이어갈 수 있습니다.

## 4. 토픽과 메시지

| 구분 | 기본 토픽 | 메시지 | 설명 |
|---|---|---|---|
| 구독 | `/global_waypoints` | `f110_msgs/msg/WpntArray` | 순서를 고정할 글로벌 Race Line |
| 구독 | `/confirmed_static_obs` | `f110_msgs/msg/ObstacleArray` | Layer 2 confirmed-only(STATIC 확정) authoritative Frenet 경계와 `s_var/d_var` 중심 위치 분산. 파라미터 `obstacles_topic`으로 `/static_obs`로 되돌릴 수 있습니다 |
| 구독 | `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | `x=s`, `y=d` ego 상태 |
| 구독 | `/state` | `f110_msgs/msg/StateMachine` | AVOID 진입 및 GLOBAL handoff 완료 확인 |
| 발행 | `/avoid_waypoints` | `f110_msgs/msg/OTWpntArray` | ego부터 글로벌 합류 뒤 lookahead까지의 회피 세그먼트 |
| 발행 | `/local_planning/path` | `nav_msgs/msg/Path` | RViz용 현재 안전 경로 |
| 발행(진단) | `/local_planning/p3_shadow` | `std_msgs/msg/String` | `SHADOW`/`TEST_ACTIVE` callback별 P3 후보·validator·lifecycle·actual path owner JSON |
| 발행(진단) | `/cma_timing/events` | `std_msgs/msg/String` | default-off, T0/T1 monotonic event |
| 발행(진단) | `/cma_replay/planner_events` | `std_msgs/msg/String` | default-off, stabilization/commitment replay event |

`/avoid_waypoints.ot_line`은 최초 군집 관측용 감속 경로일 때 `raceline_static_prepare`, 정상
회피일 때 `raceline_local_d_offset_spline`, 허용된 회피 방향이 모두 막힌 감속 경로일 때
`raceline_static_safe_stop`입니다.

## 5. 주요 파라미터

모든 운영값은 `config/local_planning.yaml`에 있습니다.

- 검출: `detection_lookahead_m`, `obstacle_cluster_gap_m`
- 종방향 계획 확장: `obstacle_longitudinal_padding_m`
- 물리 footprint: `vehicle_length_m=0.56`, `vehicle_half_width_m=0.1435`
- 추종오차 LUT: `tracking_error_lut_speed_bins_mps`,
  `tracking_error_lut_curvature_bins_radpm`, `tracking_error_lut_values_m`
- 회피속도 제한표: `avoidance_velocity_limit_speed_bins_mps`,
  `avoidance_velocity_limit_lateral_accel_mps2`
- LUT fallback: `tracking_error_reserve_m` (세 LUT 배열이 모두 비었을 때만 사용)
- 위치추정 예비량: `localization_reserve_m` (기본 0.06 m, 0=비활성) —
  `trackingErrorReserve()`에 상수항으로 합산되어 엔벨로프 확장·하드 검증·gap 속도
  역산에 동일 반영, retention 스케일도 함께 적용. 값 근거: 2026-08-13 시뮬 실측
  (`tools/mcl_gt_error_probe.py`, 181.6s) GT-vs-MCL 횡오차 속도구간별 P95 최악 6.3 cm.
  10 cm 초과는 전부 ≤0.07 s 단일사이클 MCL 보정 스파이크(최대 29.6 cm)로, 이는 정적
  마진이 아니라 retention 밴드가 흡수하는 몫이다(스파이크 최대치로 잡으면 통로 폐색).
  실차 이행 시 재측정 필요(측정 절차는 위 실차 LUT 절차와 동일한 프로브 사용).
- 장애물 clearance: `vehicle_half_width_m + safety_margin_m + LUT(limited_v, |kappa|)
  + localization_reserve_m`
- margin-only 감속 통과: `margin_pass_speed_cap_mps` (0=비활성; 물리 판정은
  `vehicle_half_width_m + safety_margin_m`만 사용)
- 커밋 경로 retention 밴드: `commitment_retention_reserve_fraction` (기본 0.5, 1.0=비활성;
  커밋 경로 재검증에서만 추적오차 예약을 이 비율로 축소)
- 트랙 경계 reserve: `wall_safety_margin_m`
- 트랙 폭 fallback: `fallback_track_half_width_m`
- spline 제어점: `pre_apex_distances_m`, `post_apex_distances_m`
- spline 길이: `entry_transition_fractions`(진입),
  `transition_distance_scales`와 `outside_line_transition_scale`(복귀 전용)
- 합류 후 시야: `post_merge_lookahead_m`, `post_merge_min_time_sec`
- 목표 제한/표본: `minimum_target_offset_m`, `maximum_target_offset_m`,
  `target_d_candidate_count`
- 최초 관측: `initial_observation_count`, `initial_observation_min_duration_sec`,
  `initial_observation_max_wait_sec`
- 불확실성 Guard: `uncertainty_sigma_scale`, `uncertainty_min_longitudinal_inflation_m`
- 비활성 횡방향 Guard: `uncertainty_min_lateral_inflation_m=0.0`,
  `uncertainty_max_lateral_inflation_m=0.0`
- commitment 충돌 확인: `commitment_soft_violation_confirm_cycles`
- 방향 잠금: `commitment_lock_lateral_threshold_m`, `commitment_lock_longitudinal_m`
- maneuver 연결: `chain_release_distance_m`
- 기하 제한: `maximum_lateral_slope`, `maximum_curvature_radpm`,
  `maximum_curvature_rate_radpm2`
- 실패 시 정지: `safe_stop_buffer_m`(기본 2.60 m), `safe_stop_deceleration_mps2`,
  `safe_stop_release_cycles`
- 정지점 탈출 검증: `safe_stop_escape_check_enable`(기본 true, false=이전 동작),
  `safe_stop_escape_retreat_step_m`(이분탐색 해상도, 기본 0.30 m),
  `safe_stop_escape_max_retreats`(최대 탐침 횟수, 기본 8 — 흔한 경우 1회로 끝남).
  자세한 동작·비용은 위 "2-a) 정지점 탈출 검증" 참고.

정상 회피 경로는 변경된 heading·curvature를 계산한 뒤 velocity-limit 표로 `vx_mps`를 제한하고
`ax_mps2`를 다시 계산합니다. safe-stop은 별도의 `safe_stop_deceleration_mps2`를 사용합니다.
- 입력 freshness: `obstacle_stale_timeout_sec`, `odometry_stale_timeout_sec`
  - obstacle stale: 마지막 유효 경로와 장애물 기억으로 주행/다음 랩 계획 지속
  - odometry stale: 마지막 위치에서 zero-speed hold
- 합류 확인: `merge_lateral_tolerance_m`, `merge_confirm_cycles`, `state_topic`,
  `state_handoff_tail_distance_m`, `state_handoff_speed_cap_mps`
- 토픽과 프레임: `*_topic`, `frame_id`
- P3 production integration: `p3_mode=OFF|SHADOW|TEST_ACTIVE`,
  `p3_diagnostics_topic=/local_planning/p3_shadow`
- CMA timing 진단: `timing_diagnostics_enable=false`,
  `timing_diagnostics_topic=/cma_timing/events`
- Record/replay 진단: `replay_diagnostics_enable=false`,
  `replay_diagnostics_topic=/cma_replay/planner_events`

### 5.1 CMA timing event

진단을 켜면 planner 입력과 출력의 실제 event 지점에 companion JSON을 발행합니다.

- T0: node가 frame·geometry 검증을 통과한 첫 non-empty `/confirmed_static_obs`를 수신한 순간
- T1: 첫 non-empty `/avoid_waypoints`를 실제 발행한 순간

T0/T1에는 steady clock, ROS timestamp, ego `s/d`, speed, obstacle ID와 T1의 plan kind 및
committed `target_d`를 가능한 범위에서 기록합니다. 이 토픽은 planner 입력이 아니며 scenario
manifest와 obstacle GT를 포함하지 않습니다. 일반 runtime에서는 기본값이 `false`이므로
publisher도 생성하지 않습니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  timing_diagnostics_enable:=true \
  timing_diagnostics_topic:=/cma_timing/events
```

### 5.2 Deterministic replay event

`replay_diagnostics_enable=true`이면 최초 장애물 군집의 stabilization 시작·충족과 실제
commitment 시점에 JSON companion event를 발행합니다. 각 event에는 원본 `/confirmed_static_obs` header
timestamp, message sequence, ego `s/d`, 장애물 ID, 선택 방향, `target_d`, 선택된 entry/exit
길이가 들어갑니다. safe-stop 중에는 매 planning cycle마다 `SAFE_STOP_LIFECYCLE` event를 추가로
기록합니다. 이 event에는 latched obstacle ID/sequence와 `s` 범위, stop target/activation 정보,
현재 ego `s`/속도, `/confirmed_static_obs` empty 여부, obstacle passed 여부, release A/B/C의 세부 평가와
count, 최종 release reason, `raceline_global_handoff` 허용/거부 이유가 포함됩니다. 또한 생성한
후보마다 `PLAN_CANDIDATE` event를 발행하며 side, target,
requested/effective entry, exit, wall/obstacle clearance, peak curvature/rate, velocity loss,
rejection reason, final rank를 포함합니다. 유한하지 않은 값은 JSON `null`로 기록합니다. 이 기능은
이미 계산된 결정을 관측만 하며 planning timer, candidate 선택 또는 출력 경로를 변경하지 않습니다.
rosbag이 companion topic을 선택하지 않은 실험에서도 사후 분석할 수 있도록 동일한 후보 JSON을
`local_planner_node` log에도 저장합니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  replay_diagnostics_enable:=true \
  detector_replay_diagnostics_topic:=/cma_replay/detector_events \
  planner_replay_diagnostics_topic:=/cma_replay/planner_events
```

## 6. 빌드와 테스트

저장소의 `~/.zshrc` 빌드 별칭은 symlink-install과 Ninja를 사용합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
cb --packages-select local_planning
source install/setup.zsh
colcon test --packages-select local_planning --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/local_planning
```

`test/test_raceline_spline.cpp`는 다음을 검사합니다.

1. 비단조 글로벌 `s_m` 거부
2. 글로벌 waypoint의 `s`와 순서를 보존한 d-offset
3. 한쪽 트랙 폭이 부족할 때 반대쪽 선택
4. 회피 진입 전 반대편 재평가와 진입 후 commitment 방향 고정
5. 단일 safety margin이 목표·트랙·Guard/raw 충돌 검사에 동일하게 적용되는지 확인
6. 준비 감속 경로와 전체 blocking cluster ID 전달
7. safe-stop buffer 안의 장애물에 준비 지연을 적용하지 않음
8. 글로벌 라인과 원본 clearance가 충분한 옆 장애물 무시
9. 양쪽이 막혔을 때 점진 정지와 짧은 정지 prefix
10. 회피 중 현재 `ego.d`를 유지하는 safe-stop
11. 기존 committed geometry 위에서 충돌 전에 감속하는 정지 prefix
12. merge 뒤 controller tail 충돌을 현재 commitment 충돌로 오판하지 않음
13. 현재 `ego.d`에서 다음 maneuver spline으로 연속 연결
14. 0속도 emergency hold와 랩 경계 장애물 처리
15. 가까운 반대편 스네이크 branch로 점프하지 않음
16. 물리 track boundary에 회전 직사각형과 `wall_safety_margin_m`을 한 번 적용하고 추종오차·
    장애물 물리 마진을 섞지 않음
17. 양쪽의 다중 target/entry/exit 조합을 모두 생성하고 최대 최소 safety slack으로 선택
18. 5차 진입·복귀 표본의 `d`가 직선에서 점진적으로 증가·감소
19. 진입 nominal 변경이 ego clamp로 소실되지 않고 실제 entry geometry를 단조롭게 변경
20. 벽에 접한 feasible target보다 벽/장애물 slack이 균형 잡힌 target을 우선 선택
21. 동일 입력에서 후보 rank와 출력 waypoint가 bit-deterministic
22. 곡선 Race Line에서도 5차 회피 경로가 경계·곡률 검증을 통과
23. centerline은 유효하지만 회전 직사각형 corner가 좌/우 wall을 넘는 경우 hard reject
24. 같은 centerline의 heading-aligned footprint가 물리 반폭까지 포함해 충분히 안쪽이면 허용
25. footprint 검사 뒤에도 obstacle·curvature·curvature-rate hard constraint와 전체 후보
    enumeration/ranking이 유지되고 반복 결과가 bit-deterministic

`test/test_obstacle_guard.cpp`는 종방향 표준편차 확장, 폐루프 `s` wrap, 고정 Guard 안의 작은 중심
이동 허용, 누적 이동의 Guard 이탈, 잘못된 분산의 fallback과 횡방향 팽창 비활성 시 큰 `d_var`에도
실측 `d_right/d_left`가 그대로 유지되는지 검사합니다. 비영 횡팽창 알고리즘의 단위 검사도
회귀 보호용으로 유지합니다.

`test/test_safe_stop_lifecycle.cpp`는 obstacle이 아직 앞일 때 transient empty와 8 cycle 초과 empty가
latch/handoff를 해제하지 않는지, danger region 통과 release, 동일 obstacle의 selectable hard-valid
avoidance release, 정지 후 명시적 persistent corridor-clear release, 동일 입력 sequence의 결정적
반복 결과를 검사합니다. `test/test_raceline_spline.cpp`의 정지 회귀는 동일 입력의 `vx_mps/ax_mps2`
프로파일이 bit-exact하게 반복되는지도 확인하므로 lifecycle 변경이 감속 형상을 바꾸지 못합니다.

`test/frenet_static_pipeline_test.py`는 준비 감속 뒤 같은 ID의 detector-style Frenet 경계를
±1cm 흔들고 `s_var/d_var`를 제공해도 10회 연속 동일 commitment가 발행되는지 확인합니다.
Cartesian AABB-to-Frenet 투영 단위 테스트는 좌표변환의 소유자인
`obstacle_detector/test/test_aabb_frenet_projector.cpp`에 있습니다.
`test/initial_cluster_stabilization_pipeline_test.py`는 첫 검출 0.1초 뒤 같은 군집에 ID를 하나
추가해 최소 0.15초 및 실제 토픽 3회 관측을 모두 거친 뒤, 넓어진 군집을 반영한 방향으로 최초
commitment가 만들어지는지 확인합니다.
`test/soft_violation_confirmation_pipeline_test.py`는 한두 cycle의 soft 충돌에서 고정 경로를
유지하고, 지속되는 soft 충돌만 3회 확인 뒤 같은 방향으로 재계획하는지 검사합니다.
`test/pre_engagement_side_switch_pipeline_test.py`는 ego가 회피 진입
기준 전일 때 기존
방향을 막아 반대편 경로로 직접 교체되는지 확인합니다. `test/safe_stop_latch_pipeline_test.py`는
`local_planner_node`, `state_machine_node`, `wpnt_publisher` 사이에서 safe-stop이
`STATE_AVOID`/`local_waypoints`에 유지되고 연속 안전 판정 뒤에만 회피로 복귀하는지 확인합니다.
`test/sequential_obstacle_handoff_pipeline_test.py`는 첫 장애물은 왼쪽, 두 번째 장애물은 오른쪽만
통과할 수 있게 만들어 두 maneuver 사이에 global handoff나 빈 경로가 없고, 두 번째 계획에서
첫 번째 방향 잠금이 해제되는지 확인합니다. 같은 스크립트에 `--during-handoff`를 주면 두 번째
장애물을 global handoff 발행 뒤에 투입해 handoff 선점도 확인합니다.
`test/post_merge_tail_chaining_pipeline_test.py`는 두 번째 장애물이 첫 경로의 merge 뒤
controller tail에 놓여도 첫 경로를 safe-stop으로 바꾸지 않고, 첫 장애물을 지난 뒤 현재
`ego.d`에서 두 번째 회피 경로로 직접 연결되는지 확인합니다. `--before-merge`를 주면 두 번째
장애물을 old merge 1m 앞에 놓아, merge와 관계없이 현재 ego 기준 안정화와 조기 연결이
동작하는지 검사합니다.
`test/stale_obstacle_memory_pipeline_test.py`는 첫 회피 commitment 뒤 `/confirmed_static_obs` 발행을
중단해 stale timeout을 넘겨도 경로가 비지 않고 geometry가 유지되는지, merge 뒤 GLOBAL
handoff가 완료되는지, 센서가 계속 끊긴 다음 랩에도 마지막 장애물 스냅샷으로 다시 회피하는지
검사합니다.

## 7. 실행 방법

### 7.1 perception을 함께 실행

기본 launch는 `obstacle_detector`를 함께 실행합니다. local planning 전용 reference-map 서버를
`/local_planning/reference_map`에 올리고 detector의 지도 필터 입력을 그 토픽으로 remap합니다.
이 지도는 장애물이 미리 그려지지 않은 wall-only 지도여야 합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=ifac_track use_rviz:=false
```

다른 터미널에서 local planning을 실행합니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

기존 P0만 사용하는 운영 기본값은 명시적으로 다음과 같이 실행할 수 있습니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  p3_mode:=OFF
```

같은 production callback 입력으로 P3/M1을 진단만 하려면 다음과 같이 실행합니다. 이 모드에서
`/avoid_waypoints`는 계속 P0가 소유합니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  p3_mode:=SHADOW \
  p3_diagnostics_topic:=/local_planning/p3_shadow
```

`TEST_ACTIVE`는 사전 rule-check와 frozen parity를 통과한 bounded IFAC smoke에서만 사용합니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  p3_mode:=TEST_ACTIVE
```

### 7.2 외부 perception 사용

이미 `/confirmed_static_obs` 발행기가 실행 중이면 detector 포함을 끕니다.

```zsh
ros2 launch local_planning local_planning.launch.py \
  start_obstacle_detector:=false
```

시뮬레이션 clock을 쓰는 전체 파이프라인이면 `use_sim_time:=true`를 함께 지정합니다.

```zsh
ros2 launch local_planning local_planning.launch.py use_sim_time:=true
```

## 8. 실행 확인

글로벌 플래너, Frenet odometry, perception이 먼저 준비된 상태에서 확인합니다.

```zsh
ros2 topic echo /avoid_waypoints --once
ros2 topic echo /local_planning/path --once
```

RViz에서 `Path` display에 `/local_planning/path`를 지정하면 현재 검증된 회피 경로나 safe-stop
경로를 확인할 수 있습니다.

local planner가 실제 사용하는 Frenet 장애물 영역을 그대로 확인하려면 detector의
`/static_obs/markers`를 추가합니다. 이 토픽은 최종 `/static_obs`의
`s_start/s_end/d_right/d_left`에서 생성되며 predicted-only 객체도 옅은 테두리로 표시합니다.

Frenet 입력 계약 확인:

```bash
python3 src/local_planning/test/frenet_static_pipeline_test.py \
  --waypoints-csv /path/to/global_waypoints.csv
```

별도 터미널에서 `local_planner_node`가 실행 중이어야 합니다. 테스트는 detector-style Frenet
경계를 넣고 `/avoid_waypoints`의 모든 `x_m/y_m`이 유한하며 횡방향 회피가 실제로 생성됐는지
확인합니다.

실제 detector 연결을 포함한 전체 경로는 두 노드를 실행한 상태에서 다음으로 확인한다.

```bash
python3 src/local_planning/test/static_obs_pipeline_test.py
```

이 테스트는 원형 글로벌 경로, free map, ego odometry, TF와 정적 장애물이 있는 LaserScan을 발행하고,
`obstacle_detector`가 유효한 Cartesian AABB와 이에 대응하는 Frenet 경계를 `/static_obs`에
만든 뒤 `local_planning`이 그 Frenet 경계로 횡방향 `/avoid_waypoints`를 만드는지 확인한다.

## 9. 전체 파이프라인 영향

perception 메시지는 유지하고, local planner와 state machine 사이의 `ot_line` 계약에 준비
감속 표식을 추가했습니다.

- `state_machine`: `raceline_static_prepare`를 합류 완료로 해석하지 않고 AVOID를 유지합니다.
- `wpnt_publisher`: `STATE_AVOID`일 때 기존처럼 `/avoid_waypoints`를 `/local_waypoints`로 중계합니다.
- `obstacle_detector`: Layer 2 `/confirmed_static_obs`의 `f110_msgs/msg/ObstacleArray`에
  authoritative `s_start/s_end/d_right/d_left`와 `is_static=true`를 채워 발행합니다. 현재 visible
  객체는 같은 footprint의 `has_cartesian=true`, Cartesian 중심/AABB/radius도 함께 제공합니다.
  local planner는 Frenet 경계만 회피 형상으로 사용합니다.
- 회피 결과: `/avoid_waypoints` 각 점의 `x_m/y_m`은 map-frame Cartesian 좌표입니다.

`f110_msgs` 형식과 토픽은 변경하지 않았습니다.

## 10. CMA deterministic lockstep 모드

`lockstep_mode=false`가 production 기본값입니다. CMA runner가 이 값을 `true`로 지정하면 25 ms
wall timer를 만들지 않고, 동일 header timestamp를 가진 `/confirmed_static_obs`와
`/car_state/frenet/odom`이 모두 준비된 때 기존 `RacelineSplinePlanner`를 한 번만 호출합니다.
두 번째 step부터는 직전 step의 `/state`도 수신됐는지 확인합니다. 출력
`/avoid_waypoints`에는 입력과 같은 logical timestamp를 사용합니다.

이 모드는 장애물 GT나 scenario manifest를 구독하지 않습니다. 장애물 입력은 production과
같이 detector의 `/confirmed_static_obs`뿐이며 planner parameter와 핵심 경로 생성 알고리즘도 같습니다.
전체 실행과 hash 검증 방법은 `tools/cmaes_tuning/docs/deterministic_lockstep_mode.md`에 있습니다.
### plan()의 후보 생성기를 P3로 통일 — P0 전용 경로 사각의 종결 (2026-08-15)

과거에는 `plan()`(P0 quintic 격자)과 P3 evaluator가 **서로 다른 후보 생성기**였고,
`p0_avoidance_candidates_enable: false` 구성에서 `plan()`이 절대 회피를 반환하지 않아
`plan()`에게 회피 여부를 묻는 모든 코드가 통째로 죽는 사각이 있었다. 실차 맵+장애물 6개
시뮬에서 두 건이 실제로 드러났다:

- **(a) 안전정지 영구 교착**: 해제 조건 B("래치된 장애물에 대한 hard-valid 회피")의 입력이
  `plan()` 결과라 구조적으로 생성 불가 → 조건 A는 전진이 필요하고 전진은 래치가 막아
  한 번 걸리면 영원히 정지 (시뮬 실측 2분+).
- **(b) 연쇄 기동 실패**: `beginChainedManeuverIfNeeded()`/`tryEarlyChainedManeuver()`가
  `plan()`만 물어 다음 클러스터 연쇄 계획이 항상 실패 → 다음 장애물이
  `safe_stop_buffer_m` 안에 들어와서야 반응, 그 거리에선 회피가 물리적으로 불가능.

**해결(2026-08-15)**: P0 격자(`generateSideCandidates`/`buildCandidate`)와 토글을 삭제하고,
`plan()`의 후보 생성기를 P3(`generateP3Candidates`)로 교체했다. 이제 회피 후보 생성기는
**하나뿐**이고, `plan()`을 묻는 모든 경로 — 조건 B, 연쇄 기동, 안전정지 탈출 검증
(`anyFeasibleCandidateFrom`) — 가 같은 생성기를 공유하므로 위 사각 자체가 성립하지 않는다.
과거의 임시 배선(`probeP3SafeStopEscape()` 탐침)은 불필요해져 함께 제거됐다.

조건 B의 "래치 장애물이 현재 스냅샷에 없으면 대상 일치를 요구하지 않는다" 완화는 생성기
통일과 별개의 교착(장애물이 FOV를 벗어난 뒤 대상 일치가 영원히 불가능)을 막는 것이므로
그대로 유지한다.

P3 후보는 발행 전 P0 시절과 동일한 안전 계층으로 재측정(`measureCandidate`)·exact
검증(`validateCandidate`)된다. P3 trace의 자체 지표는 순위·감사에 쓰지 않는다.

**순위 의미 변화 주의**: P0 격자는 최소 clearance 지점 후보를 포함했지만, P3는 유효 창
안에서 safety-slack 최대 지점을 고른다. 넓은 트랙에서는 선택된 `target_d`가 최소 clearance
보다 훨씬 클 수 있다(성능·안전 트레이드오프는 순위 규칙이 동일하므로 변화 없음).

### P3 콜백 비용 정리 (2026-08-15)

세 가지가 함께 정리됐다. 셋 다 **안전 로직은 건드리지 않는다** — 검증 항목, 마진, 임계값,
파라미터 어느 것도 바뀌지 않았고 판정 결과도 동일하다.

#### 1. 동일 콜백 재계획 제거 (도달 불가 분기였음)

`TEST_ACTIVE`에서 committed P3 suffix가 `CURRENT_RAW_OBSTACLE_COLLISION`으로 폐기되면, 예전에는
같은 snapshot으로 P3/M1을 한 번 더 돌렸다. **이 재시도는 성공할 수 없었다.**

lifecycle이 그 사유를 내놓는 조건 자체가 `evaluation.would_recover == false`이고
(`advanceP3Lifecycle`의 continuation 분기), 같은 불변 snapshot을 순수 재평가하면 결과가 같으므로
`would_recover`는 여전히 false다. 즉 **진입 조건이 곧 실패 보장 조건**이었다. 반대로
`would_recover`가 true면 첫 호출이 이미 같은 함수 안에서 `selectFresh`로 넘어간다.

이제 폐기된 suffix는 발행하지 않고 기존 `P0_BACKUP_ONLY`/safe-stop 경로로 바로 내려간다.
진단 JSON의 `same_callback_replan_attempted`/`_succeeded` 필드도 함께 제거됐다.

#### 2. continuation-first를 계산 순서로 (`advanceP3Lifecycle` lazy 평가)

이전에는 `evaluateP3Snapshot()`을 **먼저 끝내고** 그 결과를 `advanceP3Lifecycle()`에 넘겼다.
continuation-first는 출력 권한 순서였을 뿐이라, 고정된 frozen suffix가 멀쩡히 유지되는
동안에도 매 콜백(25 ms) 후보 최대 24개 생성 + hard validation을 반복했다.

이제 `advanceP3Lifecycle`은 결과 대신 **lazy evaluator**
(`std::function<const P3ShadowResult &()>`)를 받고, continuation이 출력이나 완료를 내지 못했을
때만 그것을 호출한다. 유지 중인 기동은 후보 생성 비용이 0이 된다. 평가가 한 번도 호출되지
않은 콜백의 진단에는 `failure_classification`이
`EVALUATOR_NOT_INVOKED_CONTINUATION_HELD`로 찍힌다 (solver 실패와 구분하기 위함).

`SHADOW`는 관측 목적이므로 종전대로 매 콜백 평가한다.

#### 3. guarded 검증 중복 제거 (인증서 재사용)

`buildCandidate`가 후보를 만들 때 이미 `validateP3ShadowPath(ego, path, obstacles)`로 exact
검증을 한다. 그런데 `selectFresh`가 **완전히 같은** ego/guarded-obstacle/path로 한 번 더
검증하고 있었다.

이제 `P3ShadowResult`가 선택된 후보의 검증 결과를 `selected_validation`과
`selected_validation_available`로 실어 나르고, `selectFresh`는 그것을 재사용한다. 입력이 같은
snapshot이라는 보장은 바로 위의 `FRESH_RESULT_SNAPSHOT_LINEAGE_MISMATCH` 검사가 이미 해준다.

⚠️ **뒤이은 raw geometry 검증은 중복이 아니므로 그대로 남는다.** guarded 인증서는 raw
기하에 대해 아무것도 보증하지 않는다. 회귀 테스트
`CertifiedCandidateStillRejectedWhenRawGeometryCollides`가 인증서가 재사용된 상태에서도 raw
충돌이 후보를 기각하는지 확인한다.
