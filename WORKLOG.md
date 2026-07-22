# 작업 로그 (WORKLOG)

2026 IFAC F1TENTH 제어 파트 작업 진행상황 기록.

---

## 2026-07-22 — MPPI 첫 시뮬 폐루프 검증: 완주 실패 + 원인 4건 규명

### 배경
`control_mppi_node`는 07-11 작성 이후 **합성 원호 스모크 테스트만** 거쳤고, ROS 폐루프로
트랙을 돈 적이 한 번도 없었다. 목표를 "MAP과 동등한 안정 완주"로 잡고 시뮬 검증 착수.

### 기준선 (ifac_track, max_speed 12.0)
| | 랩수 | 베스트 | 평균 | 최고속 |
|---|---|---|---|---|
| MAP | 7 (클린 6) | 10.78s | 10.80s | 6.61 m/s |
| MPPI | **0** | — | — | 인계 1~2초 내 벽 |

### 증상
MPPI로 전환(RB)한 **직후부터** 조향이 40ms 간격으로 ±0.3 rad 부호 반전(요레이트 ±3 rad/s),
횡오차 0.25→−0.73 m로 발산해 벽. 육안으로는 "헤어핀 탈출 후 옆벽 반복 충돌 / 시케인 진입 실패".
`max_speed`를 4.0으로 낮춰도 동일 → **그립·속도 문제가 아니라 구조적 문제**.

### 원인 1: λ=1.0이 비용 스케일 대비 2자릿수 작음 (가중치 퇴화)
계측 결과 **ESS(유효 샘플수) 1~5 / K=512**. 즉 매 사이클 "가장 운 좋은 난수 시퀀스 1개"를
그대로 복사 = 랜덤서치. 이게 채터링의 직접 원인. 비용 spread(mean−min)가 100~350인데
λ=1이면 `exp(-300/1)=0`. λ=60에서 ESS 150~400으로 정상화됐다.
⚠️ λ는 **비용 스케일에 종속**이라 w_* 를 바꾸면 같이 재조정해야 한다(스케일 불변이 아님).

### 원인 2: 기준궤적이 물리적으로 도달 불가능 (수정 완료)
`build_reference`가 수평 간격을 **플래너 프로파일 속도**로 잡고 있었다. 차가 프로파일보다
느리면(가속한계·코너 탈출) 기준점이 차보다 훨씬 앞으로 달아나고, `w_pos` 점추종 비용이
"저 앞 점으로 최단거리로 가라" = **코너를 가로지르라**는 명령이 된다. 헤어핀 탈출 직후
안쪽/옆벽 충돌의 메커니즘. → **차량 현재 속도** 기준으로 수정(속도 목표는 `w_v`가 담당).

### 원인 3: 가속→속도명령 변환이 플랜트와 불일치 (파라미터로 분리)
`speed = vx + a·dt`(dt=0.05)라, a=9를 계획해도 명령은 실측속도 +0.45 m/s뿐이다. gym 종방향은
`accel = kp·(목표−현재)`, `kp = 10·a_max/v_max ≈ 4.75` → **실제 전달 가속이 요청의 1/4**.
차가 가속을 못 해 코너 전에 무너진다. `speed_cmd_horizon` 파라미터로 뺐다(하위 속도루프
게인의 역수 ≈ 0.21s가 정합).
⚠️ **실차 VESC는 속도루프 특성이 달라 값이 다르다** — 실차 적용 전 별도 확인 필요.

### 원인 4: 비용을 속도항이 지배
항목별 계측: `C[v]≈280` vs `C[pos]≈57` vs `C[yaw]≈2` vs `C[bnd]=0`. 경로 추종이 뒷전인
"속도 컨트롤러"였다. 경계 페널티(`w_boundary=500`)는 **한 번도 발동 안 함** — 튜브가
좁아서 생긴 문제라는 초기 가설은 기각.

### 추가한 것
- **솔버 진단 계측 `MppiDiag`** (control_mppi_solver_cpu.cpp) — ESS / n_finite(발산 롤아웃
  검출) / J(min·mean·max) / 항목별 비용 / nominal_max_dev. 노드가 `telemetry_decimation`
  주기로 출력. ⚠️ GPU 솔버에는 아직 없음(조건부 컴파일로 CPU 빌드에서만 출력).
- **런치 인자화**: `mppi_N` `mppi_K` `mppi_u_smooth` `mppi_w_pos` `mppi_w_yaw` `mppi_w_v`
  `mppi_w_boundary` `mppi_accel_max` `mppi_speed_cmd_horizon` (`_control_common.py`).
- **`control_sim.launch.py`의 `max_speed` 하드코딩(12.0) → 런치 인자**. 그동안 시뮬에서
  `max_speed:=X`가 **조용히 무시**되고 있었다(real에는 인자로 있었음). CLAUDE.md 표의
  "sim 12.0" 기술이 실제와 달랐던 셈.

### (이어서) MPPI 정식화 전면 개편 — 채터링 해소, GPU 솔버로 전환

위 진단을 근거로 **CPU/GPU 두 솔버와 노드를 함께** 고쳤다. 젯슨에서 GPU로 돌릴 것이므로
GPU 솔버가 실제 경주 코드다 — 모든 변경을 양쪽에 동일하게 넣고 정합성을 확인했다.

| # | 문제 | 수정 |
|---|---|---|
| 1 | 점추종이 코너 가로지르기를 유발 | **컨투어링 비용**: 오차를 경로 접선 기준으로 분해해 횡오차 `w_lat`(50)는 강하게, 진행방향 `w_lon`(1.0)은 약하게. `w_pos` 제거 |
| 2 | 백색잡음이라 고주파 제어열이 뽑히면 그대로 출력 | **AR(1) 시간상관 잡음** `noise_beta`(0.7). σ 튜닝과 독립이도록 √(1−β²) 정규화 |
| 3 | 채터링을 막는 항이 아예 없음 | **제어 변화율 비용** `w_dsteer`(100)/`w_daccel`(0.5). t=0 기준은 **직전에 실제 출력한 제어**라 사이클 간 불연속에도 벌점 |
| 4 | λ가 비용 스케일에 종속(knife-edge) | **적응 λ**: `λ_eff = lambda_rel·(J_mean − J_min)`, `lambda_rel`(0.02). w_*를 바꿔도 재조정 불필요 |
| 5 | 대칭 튜브가 넓은 쪽 여유까지 깎음 | **비대칭 경계** `d_left`/`d_right` 분리 |
| 6 | 기준궤적 수평 속도 | 현재 속도에서 **도달가능 가속으로 프로파일까지 램프**(아래 참고) |

#### ⚠️ 기준궤적 수평 속도 — 양 극단이 모두 실패한다 (이번에 실측으로 확인)
- **프로파일 속도 고정**(원래 코드): 차가 못 따라가면 기준점이 달아나 코너 가로지르기.
- **현재 속도 고정**(07-22 1차 수정): 이번엔 **가속에 벌점**이 걸린다. 곡선에서 기준점보다
  Δs 앞서면 그 lag가 **횡오차로 둔갑**하기 때문(원호에서 약 `Δs²/2R`). 원호 하네스에서
  속도가 3.0 목표 대비 **0.53 m/s까지 붕괴**했다.
- ⇒ 채택: 현재 속도에서 시작해 `accel_max`로 프로파일 속도까지 램프. "이 계획대로 가속하면
  실제로 지나갈 자리"를 기준으로 삼는다.

#### 스모크 하네스 강화 (튜닝 반복을 2분 → 1초로)
`control_mppi_solver_cpu.cpp`/`_gpu.cu`의 `#ifdef` 스모크 테스트를 **환경변수 오버라이드**
(`W_LAT=80 LAMBDA_REL=0.03 /tmp/mppi_smoke`) + **진단 출력**(ESS·항목별 비용) + **조향
부호전환 카운터**(채터링 정량 지표)로 확장. 두 하네스가 노드와 **동일한 기준궤적 규약**을
쓰므로 CPU↔GPU 결과를 직접 비교할 수 있다.

정합성 확인(같은 원호 시나리오): CPU ESS=50.5 / λ_eff=46.8 / 횡오차 0.070,
GPU ESS=47.6 / λ_eff=50.0 / 횡오차 0.055. float32↔double 차이 수준으로 일치.

#### 개발 PC를 GPU 빌드로 전환
`CMAKE_CUDA_COMPILER:NOTFOUND`가 캐시돼 있어 **그동안 CPU 솔버로 빌드되고 있었다**(젯슨은 GPU
→ 다른 코드를 튜닝하고 있던 셈). `rm -rf build/f1tenth_control` 후 CUDA PATH를 넣고 재빌드:
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH   # 실행 시에도 필요(libcudart)
cd ~/2026_IFAC && rm -rf build/f1tenth_control && colcon build --symlink-install --packages-select f1tenth_control
```
- 확인: `ldd build/f1tenth_control/control_mppi_node | grep cudart`, 노드 기동 로그의 `(GPU 솔버, ...)`
- **solve 시간 8ms(CPU) → 0.37~0.79ms(GPU, RTX 4060)** — N·K를 키울 예산이 크게 남는다
- CUDA 없는 팀원 PC용 CPU 경로도 컴파일 검증 완료(`compile_commands.json`에서 `-DUSE_MPPI_GPU`를
  빼고 `-fsyntax-only`)

### 결과 (ifac_track 시뮬)
- ✅ **채터링 해소** — 이전: 40ms 간격 ±0.3 rad 부호 반전. 이후: 조향 연속(0.07→0.05→0.03…),
  횡오차 ±0.24 m 유지, 경계비용 0, ESS 35~79/512.
- ❌ **완주는 아직** — 헤어핀에서 크래시. 진입 3.4 m/s에 요레이트 3.05 rad/s
  ⇒ `a_lat = v·ω = 10.4 m/s²`로 **마찰한계(μg=10.3) 정확히 포화** = 언더스티어.
  프로파일 `vx≈3.5`를 그대로 따라가는데, 라인에서 0.2 m만 벗어나도 유효 반경이 줄어 한계를 넘는다.
- ⚠️ **수평을 늘리면 악화된다**(N=25→40→60에서 maxlat 0.4→1.30→2.53). 램프가 프로파일 속도까지
  올라가면 N=60은 40 m 앞을 보게 되고, `w_terminal`(20)이 걸린 먼 종단점이 비용을 지배한다.
  N을 키우려면 종단 가중·수평 속도 규약을 같이 손봐야 한다.

### (이어서) 안정 완주 튜닝 — ✅ **300초 23랩 무정지 달성**

| 지표 | 개편 직후 | 튜닝 후 | MAP 기준선 |
|---|---|---|---|
| 연속 완주 | 0랩(헤어핀 크래시) | **23랩 / 300초 무정지** | 무제한 |
| 베스트 / 평균 | — | 12.58s / **12.87s** | 10.78s / 10.80s |
| 명목 계획 최대 횡오차 | 0.4~1.4 m | **0.05~0.12 m** | — |
| 조향 지터(평균\|Δδ\|) | — | 0.019 rad/샘플 | — |
| solve (GPU) | 0.37 ms | 0.47~0.69 ms (K=2048) | — |

#### 확정된 기본값 (코드·런치 기본값에 반영 완료 — 인자 없이 그냥 뜨면 이 설정)
`w_lat 150` / `w_v 0.5` / `ref_max_lateral_accel 8.0` / `K 0(자동: GPU 2048·CPU 512)`

#### 튜닝에서 얻은 규칙 3가지
1. **`w_v/w_lat` 비율이 안정성을 정한다.** 실측: 0.0033·0.004 안정 / 0.0067 이상은 헤어핀 크래시.
   속도항이 크면 감속이 비싸져서(프로파일 3.5→2.0으로 늦추는 비용이 횡오차 0.3의 비용과 맞먹음)
   브레이킹이 선택되지 않는다. **속도는 `w_v`가 아니라 기준속도(`r.v`)로 조절할 것.**
2. **곡률 기반 기준속도 클램프가 결정적**(`ref_max_lateral_accel`, 신규). 플래너 프로파일은
   이상적 라인 기준이라 0.2 m만 벗어나도 마찰한계를 넘는다. 8.0(모델 μg≈10.3 대비 여유 22%)에서
   5~7랩 → 11랩 이상으로 도약. ⚠️ 5.0으로 더 조이면 오히려 나빠졌다(과도한 감속이 다른 문제를 만듦).
3. **직선 미세 사행은 K로 잡는다.** 샘플링 추정 분산이 ~1/ESS라 K를 키우면 그대로 줄어든다
   (512→2048에서 지터 0.034→0.020, 4096에서 0.015). `sigma_steer`↓·`u_smooth`↑로 잡으면 지터는
   더 줄지만(0.012) **탐색이 줄어 코너 복구력을 잃고 크래시**한다 — K는 공짜, 평활화는 유료.

#### 🔴 GPU 솔버 버그 발견·수정: K > 1024에서 조용히 실패
`weighted_update_kernel`을 `blockDim = nextpow2(K)`로 띄웠는데 **CUDA 블록당 스레드 상한은
1024**다. K=2048이면 런치가 실패하고, 아무도 `cudaGetLastError()`를 안 봐서 **U_가 갱신되지 않은
채 쓰레기 제어가 그대로 발행**됐다(증상: 출발 직후 벽으로 직진).
- 수정: 블록 크기를 1024로 제한하고, 스레드당 여러 k를 그리드-스트라이드로 누산한 뒤 트리 리덕션.
- 수정: `solve()` 종료 시 `cudaGetLastError()`로 **런치 실패를 반드시 보고**하게 함.
- 교훈: CUDA 커널은 실패해도 예외를 안 던진다. 제어 루프에서는 "조용한 실패 = 폭주"다.

### 남은 문제 (다음 세션)
1. **랩타임** — MPPI 12.87s vs MAP 10.80s(19% 느림). 최고속도 4.5 vs MAP 6.6. 수평이 속도에
   비례해 길어져 먼 코너를 미리 끌어오는 구조라 직선에서 속도가 스스로 눌린다. N·w_terminal·
   수평 속도 규약을 함께 재설계해야 풀린다(`w_terminal`만 낮추는 건 효과 없었음: 20→5→1 실측)
2. `/local_waypoints` 미구독 — MPPI 전환 시 회피 경로 추종이 사라진다(기능 갭)
3. 실차 Pacejka 미보정(gym 기본값) + `speed_cmd_horizon`이 VESC 속도루프에서 얼마인지 미확인
4. GPU 진단 계측은 `diag_enable:=false`로 끌 수 있다 — 젯슨 실시간 예산이 빡빡하면 사용

---

## 2026-07-21 (예정) — SLAM 맵 왜곡 원인 규명 (vesc_to_odom 미기동 / 도메인ID 재발 / 조향 게인)

### 배경
07-20 slam_toolbox로 트랙 맵 작성 중, **맵이 코너 없이 거의 직선으로 펴지는** 증상. 요(yaw)가
제대로 안 잡히는 것으로 추정. 당일 저녁 SSH 원격으로 젯슨 설정·소스를 조사해 후보 3건을 확보했고,
아래는 내일 실차에서 확인할 순서. **작업 1→2→3 순으로 진행할 것**(앞 단계를 건너뛰면 뒤 단계
검증이 오염됨).

### 07-20 원격 조사로 확인된 사실
- **`/scan`은 정상** — 도메인 67에서 `urg_node`가 퍼블리셔 1개로 발행 중. 스캔 소스는 문제 없음.
- **`vesc_to_odom_node`가 안 떠 있음** — 도메인 67 `ros2 node list` 결과가
  `/ackermann_mux /joy /rviz /rviz /static_baselink_to_laser /urg_node /vesc_driver_node`뿐.
  `bringup_launch.py`에 정의된 `drive_mode_manager` / `ackermann_to_vesc` / `vesc_to_odom` /
  `throttle_interpolator` **4개가 통째로 누락**. `/drive` 토픽도 `Unknown topic` 상태.
- ~~**`~/.zshrc`의 `ROS_DOMAIN_ID`가 `67`로 되돌아가 있음** — 07-17에 바꿨던 수정이 사라짐~~
  🔴 **07-21 정정: 오판이었다.** 67이 확정값이라 "되돌아간" 게 아니라 원래 맞는 값이다.
  아래 작업 2 참고.
- **`/rviz` 노드가 이름 중복 2개** + `nodes share an exact name` 경고.
- ⚠️ **07-21 발견된 모순**: 노드 목록엔 `ackermann_to_vesc`가 없는데, 서보 클리핑 경고가 50Hz로
  찍히려면 누군가 `commands/servo/position`을 발행하고 있어야 한다(`vesc_driver`의 servoCallback).
  ⇒ 두 관측은 **서로 다른 시점**의 것이며, **매핑 당시 실제로 4개 노드가 없었는지는 미확인**.
  작업1에서 이것부터 가릴 것(매핑 재현 중에 `ros2 node list`를 찍어야 함).
- 도메인 **0**에는 팀원 PC의 시뮬(`bridge`, `controller_manager`, `mcl` 등)이 떠 있음.
  단 실차는 67이므로 **현재 실차와 섞이고 있다는 증거는 아님**(조사 초기에 bash로 접속해
  `.zshrc`를 안 읽고 도메인 0을 보는 바람에 한 번 오판했던 부분 — 기록해 둠).

### 작업 0 (매핑 전 선행): 서보/조향 링키지 재장착 → 센터 우측 6° 틀어짐
07-21 서보모터↔휠축 연결을 물리적으로 수정. 그 결과 **중립 명령에서 바퀴가 오른쪽으로 약 6°**
(0.105 rad) 향한다. 07-17 확정 트림값 `0.4633`에 붙어 있던 단서("서보/조향 링키지를 물리적으로
재장착하지 않는 한 유효")가 그대로 깨졌으므로 **트림 캘리브레이션은 무효**.

젯슨 라이브 확인(`~/f1tenth_ws/{src,install}/.../config/vesc.yaml`, src·install 둘 다 동일 파일
사본이라 **양쪽 다 고쳐야 함** — 심볼릭 링크 아님. 07-20 19:51에 누군가 수정한 흔적):
- `steering_angle_to_servo_offset: 0.4672` (07-17 확정값 0.4633 아님 — 위 작업3의 불일치 실재 확인)
- `steering_angle_to_servo_gain: -1.2135`
- `speed_to_erpm_gain: 4232.0` — **07-20 줄자 실측으로 확정한 값**(이론값 4614 대비 9% 차이).
  repo 코드 4곳(`joy_teleop_monitor`/`realcar_dashboard_node`/`_control_common.py`/`odom_calib_node`)은
  이미 `ebf331a sync: 07/20`으로 커밋 완료. `CLAUDE.md`만 4614로 남아 있던 것을 07-21에 정정.

**임시 보정값 계산** (07-17 2점 실측 민감도 `1° ≈ 0.00779` 서보값, 방향은 `offset↑ = 우측`:
0.51→+7°우 / 0.362→−12°좌):
```
Δoffset = −6 × 0.00779 = −0.0467
0.4672 − 0.0467 = 0.4205
```

**측정 기준**: VESC Tool에서 서보 **센터(0.5)** 출력한 상태에서 바퀴가 우측 6°. 암은 이미
"기계적 충돌 없게" 재클로킹해 꽂은 상태(스플라인 1톱니 ≈ 바퀴각 10°라 6°는 기계적으로 못 맞춤
→ 잔차는 소프트 트림이 맞다).

#### 비대칭의 진짜 원인 (07-20 기록 정정)
어제 적은 `+0.261 / −0.315 rad` 비대칭은 **서보 암 물림 때문이 아니다.** 순수 산수:
`servo_min/max = [0.15, 0.85]`의 중점은 **0.50**인데 offset이 0.4672라 창 안에서 중립이 치우친 것.
**암을 다시 꽂아도, offset만 바꿔도 안 없어진다**(새 offset이 0.50이 아닌 한).

반면 **스토퍼를 미는 문제는 실재**하며 원인이 다르다 — 실측 민감도 0.00779 서보/도로 환산하면
현재 창이 허용하는 명령 범위가 좌 40.7° / 우 49.1°로 **실측 스토퍼(좌 28° / 우 31°)를
각각 +12.7° / +18.1° 넘긴다.** `servo_min/max`가 링키지 대비 활짝 열려 있는 게 원인(암 위치 무관).

#### 실측 기계 가동범위 (07-21) — **차체 직진 방향 기준**
| | 값 |
|---|---|
| 우 스토퍼 | 31° |
| 좌 스토퍼 | 28° |
| 총 가동범위 | **59° = ±29.5° = ±0.515 rad** |
| 기계 중심 | 직진 대비 1.5° 우측 (사실상 대칭) |
| 중립(서보 0.5) 위치 | 직진 대비 **6° 우측** |

⚠️ **CLAUDE.md의 "최대 조향각 ±0.41 rad(±23.5°)"보다 기계 범위가 넓다.** 0.41은 하드웨어
한계가 아니라 **컨트롤러 소프트 한계**로 이해할 것 — 기계 스토퍼까지 좌 4.5° / 우 7.5° 여유가 있다.
⇒ 기계 중심이 대칭인데 중립만 6° 틀어진 것 = **랙이 아니라 서보 혼 클로킹이 원인**(확정).
소프트 트림으로 지우는 게 맞다.

- 배선 확인(07-21): `ackermann_to_vesc.cpp:75`는 **클램프를 전혀 안 한다** — `gain·δ+offset`을
  그대로 발행. 유일한 보호 클램프는 `vesc_driver.cpp:334`의 `servo_limit_`(= yaml `servo_min/max`).
  즉 **조향 권한도 서보 보호도 전부 이 두 값이 결정**한다.

#### 제안: offset·gain·servo_min/max **셋을 같이** 바꾼다
| 항목 | 현재 | 제안 |
|---|---|---|
| `steering_angle_to_servo_offset` | 0.4672 | **0.4533** (= 0.5 − 6×0.00779) |
| `steering_angle_to_servo_gain` | −1.2135 | **−0.4463** (= −0.00779×180/π, 실측 기반) |
| `servo_min` (좌 23.5°) | 0.15 | **0.2703** (= offset − 23.5°×0.00779) |
| `servo_max` (우 23.5°) | 0.85 | **0.6363** (= offset + 23.5°×0.00779) |
| 조향 권한 | 좌 +0.261 / 우 −0.315 rad | **±0.41 대칭** |
| 스토퍼 명령 초과분 | 좌 +12.7° / 우 +18.1° | **0** (여유 좌 4.5° / 우 7.5°) |

(서보값 ↑ = 우측. `servo_min` ↔ 좌, `servo_max` ↔ 우)

⚠️ **창은 스토퍼가 아니라 컨트롤러 한계(±0.41 rad)에 맞춰 잡는다.** "스토퍼에 막 닿게" 잡는
것보다 낫다 — 양쪽에 4.5°/7.5° 여유가 남아 서보가 스토퍼를 **아예 안 건드린다**(발열·전류).

⚠️ **창만 조이고 게인을 그대로 두면 안 된다** — 게인 −1.2135에서 δ=0.41을 명령하면 서보값이
−0.044까지 내려가 상시 클리핑, 조향이 ±0.151 rad로 반토막 난다. 게인을 −0.4463으로 고치면
`δ=0.41 → servo = 0.4533−0.183 = 0.2703` = 정확히 `servo_min` → **명령 범위가 기계 가동범위에 1:1**.

✅ **이게 아래 작업3의 "선형 게인 하나로는 이 링키지를 표현할 수 없음" 결론을 뒤집는다.**
그 반례(`(0.15−0.4672)/(−0.4463) = 0.711 rad ≫ 0.41`)는 **servo_min이 0.15로 열려 있어서** 나온 것.
창을 0.2703으로 조이면 정확히 0.41이 되어 포화 구간 자체가 사라지고, odom 요 1.6배 과소평가도
같이 해소된다.

#### ⚠️ 07-20 데이터가 드러낸 링키지 비선형성 (07-21 추가 분석)
위 07-20 편향 표 두 점에서 **중앙 부근 민감도**를 역산할 수 있다:
```
Δ조향각 0.767°  ←  Δoffset 0.0039     ⇒  0.00509 서보/도 (중앙 부근)
07-17 2점 실측(넓은 범위 평균)          ⇒  0.00779 서보/도
```
⇒ **중앙 부근이 넓은 범위 평균보다 약 1.5배 민감**하다. 07-20에 "중앙은 민감하고 끝은
포화되는 형태로 추정"이라 적어둔 가설이 데이터로 확인된 셈.

**적용한 게인 −0.4463은 넓은 범위 평균(0.00779) 기반**이므로:
- 스토퍼 근처는 정확(예측 0.695/0.235가 실측과 일치해 검증됨)
- **중앙 부근(=레이싱 라인 대부분)에서는 실제 조향이 명령보다 ~1.5배 더 나간다**
- 같은 이유로 `vesc_to_odom`은 소조향 구간에서 요를 ~35% 과소평가
- ⇒ 재셰이크다운에서 **작은 조향 입력에 과민/오버슈트**가 보이면 이 비선형성이 원인이다.
  게인을 더 낮추는 게 아니라 **작업3의 다점 측정 → 구간별(비선형) 맵**이 정답
- ⚠️ 단 위 역산은 등반경·토우 무영향·평탄 노면을 가정한 것이라 정밀치가 아님(경향만 신뢰)

- [ ] ⚠️ **위 값 전부가 `0.00779 서보/도` 민감도 하나에 매달려 있는데, 이건 재장착 전
      측정치라 미검증.** VESC Tool로 좌우 스토퍼에 닿는 서보값 2개를 읽어 검증할 것 —
      0.00779가 맞다면 **우 스토퍼(31°) ≈ 0.695 / 좌 스토퍼(28°) ≈ 0.235**가 나와야 한다.
      벗어나면 `실측 반스팬 ÷ 59°`로 민감도를 다시 뽑아 offset·gain·창을 전부 재계산
- [x] **07-21 적용 완료** — 젯슨 `vesc.yaml` src·install 양쪽에 아래 4개 값 반영,
      백업 `vesc.yaml.bak.20260721`. 스토퍼 서보값 실측이 예측(0.695/0.235)과 근사해 민감도
      0.00779 검증됨.
      ```yaml
      steering_angle_to_servo_gain:   -0.4463   # was -1.2135
      steering_angle_to_servo_offset:  0.4533   # was 0.4672
      servo_min: 0.2703                         # was 0.15
      servo_max: 0.6363                         # was 0.85
      ```
- [ ] `f110` 재기동(파라미터는 생성자 1회 읽기) 후 서보 클리핑 경고 소멸 확인
- [ ] ⚠️ **수동 조향 감각이 크게 바뀐다** — 이전엔 teleop 풀스틱(0.34 rad)이 실제로 53° 상당을
      요구해 스토퍼에 물려 있었다(발열·전류의 정체). 이제 0.34 rad → 19.5°로 정직하게 나가므로
      **"덜 꺾인다"가 정상**
- [ ] 팀 공용 `f1tenth_stack` 설정이므로 팀에 변경 공지
- [ ] ⚠️ 게인이 2.7배 바뀌므로 **같은 명령에 조향 응답이 완전히 달라진다** —
      `max_speed` 낮춰 재셰이크다운 필수
- [ ] ⚠️ **매핑(작업1) 전에 반드시 센터를 잡을 것.** 센터가 6° 틀어지면 직진하려고 반대로 6°를
      물게 되고, `vesc_to_odom`은 그 **명령각**으로 요를 적분한다:
      `ω_phantom = v·tan(0.105)/0.33 = 0.48 rad/s (≈27°/s) @1.5 m/s`.
      직진 중에 가짜 요가 실려 작업1·3의 요 검증이 통째로 오염됨

### 🔴 07-21 근본 원인 확정 — `vesc_to_odom` 속도 부호 반전 (SLAM 맵 왜곡의 진범)
`yaw_check.py`(젯슨 `~/yaw_check.py`, 이 repo 미포함)로 **odom 요 vs VESC 자이로**를 나란히
적분해 비교한 결과:

| | 값 | 판정 |
|---|---|---|
| 자이로 누적(deg/s 가정) | **−358.2°** | 물리적 1바퀴(시계방향)와 **오차 0.5%** → 기준으로 신뢰 가능, deg/s 확정 |
| odom 누적 | **+484.5°** | **부호 반대** + 크기 35% 과대 |
| 이동거리(odom) | 9.33 m | |

**원인**: `vesc_to_odom.cpp:102`
```cpp
double current_speed = (-state->state.speed - offset) / gain;   // ← 이 마이너스
```
이 차의 배선은 **전진 = 양수 ERPM**이고 명령 경로(`ackermann_to_vesc`)도 같은 규약인데, 이
노드만 반전시키고 있었다. `current_speed`는 **4곳 전부**에 쓰여 odom 전체가 거울상이 된다:
`ω = v·tanδ/L`(110행) / `x_dot`·`y_dot`(124-125행) / `odom.twist.linear.x`(156행).

- ⚠️ **SLAM만의 문제가 아니다** — `odom.twist.linear.x`가 전진에 음수라, `/pf/pose/odom`이
  이 twist를 패스스루하면 **`control_map_node`가 전진 중 속도를 음수로 읽는다.** 실차 자율주행을
  본격적으로 안 돌려봐서 안 드러났을 뿐, 그대로면 컨트롤러가 정상 동작할 수 없다.
- ⚠️ `speed_to_erpm_gain`을 음수로 만드는 우회는 **불가** — `/**:` 공유 파라미터라
  `ackermann_to_vesc`까지 뒤집혀 전진 명령에 차가 후진한다.
- ✅ **07-21 수정 완료**: 102행의 `-` 제거 + 한국어 사유 주석 3줄 추가,
  백업 `vesc_to_odom.cpp.bak.20260721`, `colcon build --packages-select vesc_ackermann` 성공.
  ⚠️ 팀 공용 `f1tenth_system` 저장소의 **upstream 클린 파일을 처음 수정**한 것 → 팀 공지 필요.
- [ ] 재기동 후 검증: `ros2 topic echo /odom --field twist.twist.linear.x`가 **전진에 양수**,
      시계방향 회전에 yaw **감소**

#### 남은 의문: 크기 35% 과대 (부호와 별개)
odom 484.5° vs 자이로 358.2°. 조향 캘리브레이션은 맞는 것으로 보인다 — 실측 원 직경 약 2 m가
teleop 풀스틱 0.34 rad(19.5°)의 기구학 예측 반경 0.93 m(직경 1.86 m)와 일치. 따라서 틀린 쪽은
**거리**일 가능성이 높다(odom 9.33 m vs 직경 2 m 원둘레 6.3 m).
- 유력 가설: **직경 2 m 급선회에서의 뒷바퀴 스크럽** → 모터 ERPM이 실제 이동거리보다 부풀려짐.
  어제 직선으로 잡은 `speed_to_erpm_gain 4232`가 틀린 게 아니라 급선회 한정 현상.
- [ ] 재검증은 **반경 3~4 m로 크게** 돌아서 할 것(스크럽 최소화)

#### ✅ 조향 트림 최종 확정: `offset 0.4672` (07-20 값 그대로) — 재장착은 중립을 안 옮겼다
07-21에 `0.4533`으로 바꿨다가 **좌측 편향이 생겨** 되돌린 경위. 기록해 두는 이유는 **같은
착각을 반복하기 쉽기 때문**이다.

**틀린 추론**: "서보 0.5에서 바퀴가 6° 우측" → `0.5 − 6×0.00779 = 0.4533`.
**오류**: 서보 0.5는 **서보의 기계적 중앙일 뿐 조향 중립이 아니다.** 조향 중립은 이미 07-20에
실측 확정된 `0.4672`였고, `(0.5−0.4672)/6° = 0.00547 서보/도`는 07-20 데이터에서 역산한 중앙부
민감도 `0.00509`와 거의 일치한다 ⇒ **"6° 우측"은 새로 생긴 편차가 아니라 서보 중앙과 조향 중립
사이의 원래 거리**였다. 서보 암 재장착이 중립을 거의 안 옮겼는데 그걸 새 오차로 보고 한 번 더 뺀 것.
- ⇒ "재장착으로 07-17/07-20 캘리브레이션이 전부 무효"라던 초기 판단도 **틀렸다**(중립은 유지됨).
  반면 게인(−0.4463)·`servo_min/max` 수정은 서보가 스토퍼를 상시 밀던 실제 결함을 고친 것이라 유효.

#### ⚠️ 직진 편향 측정법 정정 — 2차 공식을 그대로 쓰면 안 된다
같은 트림에서 잰 두 점: `3.32 m → 0.31 m` / `6.34 m → 0.66 m` (둘 다 좌측).
```
조향 편향(2차, y ∝ d²) 예측: 0.31 × 1.91² = 1.13 m
헤딩 오차(1차, y ∝ d)  예측: 0.31 × 1.91  = 0.59 m
실측                                        0.66 m   ← 1차에 훨씬 가깝다
```
⇒ 편향의 대부분이 **조향이 아니라 헤딩/얼라인먼트** 성분. 2성분 분해 결과
**잔여 조향 편향 0.13° / 헤딩 오차 4.7°**.
- ⚠️ WORKLOG 07-17·07-20의 편향→각도 환산은 전부 `y ≈ d²·tanδ/(2L)` 2차 가정으로 계산한 것이라
  **헤딩 성분이 섞여 과대평가돼 있다.** 여기서 역산한 민감도(0.00509 등)도 같은 오염을 안고 있음.
- ✅ **자이로 실측으로 확정**(`yaw_check.py`): 직선 4.09 m 주행에 **자이로 누적 +0.3°**(odom 0.0°).
  차가 사실상 안 돈다 ⇒ 조향 트림은 완료. 남은 좌측 밀림은 **조준 오차 또는 뒷축 얼라인먼트
  (도그트래킹)**이며 `vesc.yaml`로는 못 고친다.
- [ ] 3회 모두 좌측이었으므로 뒷축 얼라인먼트 점검(기구 작업). 벽·타일에 차체를 정렬해 재측정하면
      조준 오차와 구분됨
- **교훈**: 직진 편향은 조향/헤딩 두 성분이 섞이므로 **거리를 2가지 이상으로 재서 분해**하거나,
  아예 **자이로로 회전 유무를 직접 보는 게** 빠르고 정확하다.

#### `joy_teleop.yaml` `steering_scale` 0.34 → 0.41 (07-21 적용)
수동 조향이 풀스틱에서도 19.5°밖에 안 나가 기계 한계(28~31°)의 2/3만 쓰고 있었음. 0.41 rad은
서보 창(23.5°)과 자율 컨트롤러 한계에 정확히 일치 → 수동/자율 조작감 통일. 백업 `.bak.20260721`.

### ✅ 07-21 재기동 결과 — 어제 "4개 노드 누락"은 대부분 오진이었음
캘리브레이션 적용 후 `f110` 재기동해 실제로 확인한 상태:

| 노드 | 어제 판정 | 07-21 실제 |
|---|---|---|
| `ackermann_to_vesc_node` | 누락 | **정상 기동** |
| `vesc_to_odom_node` | 누락(맵 왜곡 1순위 용의자) | **정상 기동** |
| `drive_mode_manager` | 누락 | **정상 기동**(pid 확인). 기동 직후 `no joy messages - holding e-stop`을 찍은 뒤 조이스틱이 붙자 정상화 |
| `throttle_interpolator` | 누락 | **런치에서 의도적으로 주석 처리**(`bringup_launch.py:136`) — 버그 아님 |

⇒ **어제 본 `ros2 node list`는 조이스틱이 붙기 전/스택이 덜 뜬 시점의 스냅샷**이었을 가능성이
높다. 노드 목록은 반드시 **조이스틱 연결 + 모드 전환 후**에 찍을 것.

#### ⚠️ "정지 상태에서 바퀴가 안 똑바르다"는 캘리브레이션 문제가 아닐 수 있음 (07-21 실사례)
`steering_angle_to_servo_offset`은 **"조향 0을 명령했을 때 나갈 서보값"**이라, 아무도 `/drive`를
안 쏘면 서보에 명령 자체가 안 가고 서보는 **VESC Tool에서 맞춰둔 자기 센터(0.5)**에 앉아 있는다.
E-stop 상태나 딥맨 미입력 상태에서는 오프셋이 **적용될 기회가 없다.**
- 판별법: `ros2 topic echo /commands/servo/position` — 아무것도 안 흐르면 캘리브가 아니라
  **명령 체인이 끊긴 것**. 체인은 `joy → drive_mode_manager → teleop → ackermann_mux →
  /ackermann_cmd → ackermann_to_vesc → /commands/servo/position → vesc_driver`
- 07-21에 실제로 이걸로 "센터가 안 맞았다"고 오판했음. 버튼 눌러 명령이 나가자 즉시 정상 센터
  ⇒ **offset 0.4533 실차 검증 완료**

#### 참고: `joy_teleop.yaml`이 07-17 이후 재작성됨
`drive_mode_manager`가 `/joy`를 직접 읽는 구조로 바뀌어, 07-17에 고쳤던
`human_control.drive-steering_angle.axis: 2→0`은 **현재 설정에 존재하지 않는다**(superseded).
현재 매핑: `speed_axis: 1`(좌스틱 세로) / `steering_axis: 3`(우스틱 가로) /
`autonomous=0(A) / estop=1(B) / manual=2(X)` / `steering_scale 0.34` / `speed_scale 5.0`
— 우리 `joy_teleop_monitor` 관례와 일치.

### 작업 1 (최우선): `vesc_to_odom_node` 미기동 원인 규명 — 맵 왜곡 1순위 용의자
`vesc_to_odom`이 없으면 **`odom→base_link` TF가 아예 없고**, slam_toolbox는 이 TF를 모션
프라이어로 쓰므로 정상 동작이 불가능하다. 맵이 펴진 증상의 가장 직접적인 설명.

- [ ] `f110` 기동 후 `ros2 node list`로 4개 노드 기동 여부 재확인 (매핑 당시에도 없었는지)
- [ ] 없으면 `~/.ros/log/<최신>/launch.log`에서 해당 노드 사망 원인 확인.
      `vesc_to_odom`은 `speed_to_erpm_gain` 등을 **기본값 없이** `declare_parameter<double>()`로
      읽으므로 파라미터 미전달 시 예외로 죽는다 — `vesc_config` 인자 전달 여부부터 볼 것
- [ ] `ros2 run tf2_ros tf2_echo odom base_link`로 TF 존재 확인
- [ ] 4개 노드가 살아난 상태에서 SLAM 재시도 → 이것만으로 해결될 수 있음

### 작업 2: `ROS_DOMAIN_ID` — ✅ **67이 확정값** (07-21 정정, 아래 두 항목만 남음)

🔴 **이 절의 원래 전제가 틀렸다.** "67은 회피하고 88로 바꿔야 한다"고 적어뒀지만,
**67이 팀 합의 확정값이다**(07-21 확인). 따라서 아래 두 항목은 **폐기**한다:
- ~~젯슨 `~/.zshrc`의 `ROS_DOMAIN_ID=67`이 왜 되돌아왔는지 확인(설정 유실/덮어쓰기 경위)~~
  → 유실이 아니라 **정상값이다.** 07-20에 "수정이 사라졌다"고 본 것이 오판이었다.
- ~~팀과 도메인ID 고정 배정 조율 후 확정값 적용. 67은 충돌 이력이 있으므로 회피~~
  → 67로 확정. 88로 되돌리면 오히려 실차와 팀 스택이 갈라진다.

07-17의 충돌 사고는 **도메인 번호가 잘못돼서가 아니라 시뮬이 LAN으로 새서** 난 것이므로,
해법은 번호 변경이 아니라 **시뮬 쪽을 로컬로 가두는 것**이다(아래).

- [ ] `~/.bashrc`에도 `export ROS_DOMAIN_ID=67` 추가 — 현재 zshrc에만 있어 **bash로 붙으면
      도메인 0으로 떨어진다**(07-20 조사에서 실제로 이것 때문에 "bringup 노드 4개 누락"으로
      오판. SSH·스크립트·cron 등 비대화형 진입 경로 전부 영향)
- [ ] **시뮬 돌리는 모든 사람**(팀원 + 우리 자신)에게 `export ROS_LOCALHOST_ONLY=1` 공지.
      ⚠️ 우리 개발 PC `~/.zshrc`도 `ROS_DOMAIN_ID=67`인데 `ROS_LOCALHOST_ONLY`가 **없다**
      (07-21 확인). 즉 같은 wifi에서 실차가 켜져 있으면 **우리 시뮬 `/drive`가 실차
      `ackermann_mux`의 navigation 채널로 샌다** — 07-17 사고와 같은 경로, 방향만 반대.
      07-21 폐루프 시뮬 9회가 이 상태로 돌았다(젯슨이 꺼져 있어 사고는 없었음).
- [ ] `/rviz` 이름 중복 2개 정리

### 작업 3: 조향 게인 불일치 & 서보 클리핑 (요 과소평가의 구조적 원인)
작업 1·2로 해결돼도 **이건 별도로 남아 있는 실제 결함**이다. 07-20 스크린샷에서
`servo command value (0.129778) below low minimum limit (0.150000), clipping.`이 50Hz로 연속
발생 중이었음.

확인된 계산 근거:
- `vesc.yaml`: `steering_angle_to_servo_gain: -1.2135`(이론값), `offset: 0.4672`
  (✅ **해결**: 07-17의 `0.4633`이 아닌 이유는 **07-20 직진 편향 실측으로 재보정**했기 때문.
   누가 몰래 바꾼 게 아니라 확정 작업이었다 — 아래 표 참고)

| offset | 실측 직진 편향 | 역산 잔여 조향각 |
|---|---|---|
| 0.4633 (07-17) | 5.7 m에 **91.5 cm** | 약 1.07° |
| **0.4672 (07-20 확정)** | 3.9 m에 **12 cm** | 약 0.30° |

(역산: `y ≈ d²·tanδ/(2L)`, L=0.33)
- ⚠️ **07-21 링키지 재장착으로 이 절의 실측 기준선이 전부 무효화됨** — 위 작업0 참고.
  다점 재측정은 **재장착 이후 상태**에서 할 것(작업0의 트림 확정도 여기서 같이 처리)
- 07-17 실측: 실제 민감도 `1도 ≈ 0.00779` 서보값 vs 이론 `0.0212` → **약 2.7배 불일치**
- `vesc_driver.cpp:334-339` — `sensors/servo_position_command`로 **클리핑된 값**을 발행
- `vesc_to_odom.cpp:107-110` — 그 클리핑된 값으로 조향각 역산
- ⇒ **odom이 볼 수 있는 최대 조향각 = (0.15 − 0.4672) / (−1.2135) = 0.261 rad**.
  실제 바퀴는 기계 스토퍼인 0.41 rad에 물려 있음 → `tan(0.41)/tan(0.261) ≈ 1.6배 요 과소평가`
- `vesc_to_odom.cpp:103` — 속도 `< 0.05 m/s`면 0으로 죽임 → 저속 매핑 시 요가 통째로 0

- [ ] 서보값↔실제 조향각을 **여러 점**에서 재측정(2점 선형보간으로는 비선형을 못 잡음).
      중앙 부근은 민감하고 끝은 포화되는 형태로 추정됨
- [ ] `steering_angle_to_servo_gain` 재산출 + `servo_min/max` 재검토
- [ ] ⚠️ **이 게인은 `/**:` 공유 파라미터라 `ackermann_to_vesc`(명령 경로)도 같이 쓴다.**
      바꾸면 실제 주행 조향 응답이 바뀌므로 `max_speed`를 낮춰 **재셰이크다운 필수**
- [x] ~~단순히 `vesc_to_odom_node` 블록에만 보정 게인을 넣는 방식은 **불가**.
      포화 구간에서 역으로 과대평가된다(`(0.15−0.4672)/(−0.4463) = 0.711 rad` ≫ 실제 0.41).
      선형 게인 하나로는 이 링키지를 표현할 수 없음~~
      → **07-21 정정**: 이 반례는 `servo_min`이 0.15로 열려 있을 때만 성립. 작업0처럼
      `servo_min/max`를 기계 가동범위(±0.183)로 조이면 포화 자체가 사라져 단일 게인이 유효해진다.
      **작업0에서 게인·창을 함께 확정하면 이 항목은 소멸** — 아래 다점 재측정은 잔여 비선형성
      확인용으로만 남음
- [ ] 현재 조향 권한이 좌우 비대칭(+0.261 / −0.315 rad)인 점, 서보가 스토퍼를 계속 밀어
      발열·전류 부담이 있는 점도 같이 점검

### 작업 4 (작업 1~3으로 해결 안 될 때): 라이다 오도메트리로 우회
VESC 조향 오도메트리는 비선형 링키지 + 서보 클리핑 + 속도 데드밴드가 겹쳐 있어, 매핑용
요 소스로는 구조적으로 불리하다. 맵 작성은 일회성 작업이므로 우회가 합리적.

- [ ] `sudo apt install ros-humble-rf2o-laser-odometry`
- [ ] `vesc_to_odom`의 `publish_tf: false`로 두고 rf2o가 `odom→base_link` 발행
- [ ] 매핑 후 원복(주행 중 위치추정은 MCL 담당이므로 rf2o 불필요)

### 검증용 명령 모음
```bash
# 실차 도메인으로 붙기 (bash 진입 시 필수 — .zshrc를 안 읽음)
export ROS_DOMAIN_ID=<확정값>

ros2 node list                          # bringup 9개 노드 전부 떴는지
ros2 topic info /scan -v                # 퍼블리셔가 urg_node 하나인지
ros2 topic info /drive -v               # 외부(시뮬) 발행자 누수 없는지
ros2 run tf2_ros tf2_echo odom base_link  # odom TF 존재 + 코너에서 yaw 변화
ros2 launch f1tenth_control dashboard.launch.py mode:=calib   # odom 거리 스케일
```

### 요 측정 시 주의 (07-20 논의 정리)
- **차를 손으로 들고 돌리면 요는 0이다.** `ω = v·tan(δ)/L`이므로 `v=0`이면 어떤 회전도 안 잡힘.
  반드시 **조이스틱으로 조향을 고정한 채 원을 그리며 주행**해야 함
- 속도는 **1.0~1.5 m/s** — FOC 센서리스 데드존(0.17~0.49 m/s) + odom 데드밴드(0.05 m/s) 회피
- `tf2_echo`의 yaw는 ±π에서 wrap되므로 **90° 구간**으로 재는 게 읽기 쉬움
  (정상이면 1.571, 2.7배 축소면 0.58 부근)
- 요와 **주행 거리를 같이** 볼 것: 요만 축소면 조향 게인 문제, 요·거리가 같은 비율로 축소면
  `speed_to_erpm_gain` 문제(이 경우 맵 모양은 안 찌그러짐)

---

## 2026-07-17 — 실차 원격(SSH) 브링업 중 발견된 문제 3건 해결 (도메인ID 충돌 / joy_teleop 오배선 / 조향 서보 트림)

### 배경
젯슨에 모니터·키보드 없이 무선 SSH로만 접속해 `f110`(f1tenth_stack bringup) 첫 실차 조이스틱
테스트 진행. PC↔젯슨 SSH 키 인증 + VS Code Remote-SSH 세팅 완료 후, 실제 조작 중 문제 3가지를
순차 발견·해결.

### 문제 1: VESC USB 미연결로 `vesc_driver_node` 기동 직후 사망
- 증상: 조이스틱 입력이 차에 전혀 안 먹힘.
- 로그(`~/.ros/log/.../launch.log`): `vesc_driver_node FATAL: Failed to open the serial port
  /dev/sensors/vesc ... No such file or directory`.
- `lsusb`에 VESC 관련 장치 자체가 없었음(허브·블루투스·Xbox 컨트롤러만 보임) → USB 케이블
  실물 미연결/전원 문제로 확인, 재연결로 해결.

### 문제 2: `ROS_DOMAIN_ID` 충돌 — 팀원 시뮬레이션이 실차 서보를 직접 명령
- 증상: `f110` 켜자마자 바퀴가 오른쪽으로 확 꺾이고 안 풀림. `vesc_driver_node` 로그에
  `servo command value (0.970716) above maximum limit (0.850000), clipping.`이 초당 수십 회
  반복(수천 줄, 3분 넘게 고정값).
- 원인: 젯슨 `~/.zshrc`의 `ROS_DOMAIN_ID=67`이 같은 Wi-Fi(MIRU) 네트워크의 **다른 팀원 PC가
  돌리던 시뮬레이터(f1tenth_gym+planning stack)와 겹침**. `/drive` 토픽 발행자를 추적
  (`ros2 topic info /drive --verbose`)한 결과 로컬에 존재하지 않는 `map_controller` 노드였고,
  `/ego_racecar/odom` 등 시뮬 전용 토픽까지 같은 도메인에서 보였음 — 팀원의 시뮬 조향 명령이
  DDS 도메인 충돌로 그대로 실차 `ackermann_mux`→서보에 새어 들어간 것.
- 당시 조치: 젯슨 `.zshrc`의 `ROS_DOMAIN_ID`를 `67 → 88`로 변경. 이후 `ros2 node list`에서
  팀원 쪽 노드 전부 사라짐, `/drive` 누수 중단 확인.
- 🔴 **07-21 정정 — 이 조치는 되돌려졌고, 그게 맞다.** 현재 확정값은 **67**이다.
  도메인 번호를 옮기는 건 증상만 가리는 대증요법이었다. 근본 원인은 "번호가 겹쳤다"가 아니라
  **시뮬이 LAN으로 새어나간다**는 것이라, 번호를 어떻게 배정해도 같은 사고가 재발할 수 있다.
  올바른 해법은 **시뮬 쪽에 `export ROS_LOCALHOST_ONLY=1`** — 도메인이 겹쳐도 안전하다.
  자세한 내용은 07-21 작업 2 참고.

### 문제 3: f1tenth_stack `joy_teleop.yaml` 조향축 오배선(axis 2 = LT 트리거)
- 증상: LB(딥맨) 누르면 바퀴가 왼쪽으로 확 꺾임(조이스틱 스틱은 안 건드린 상태).
- 원인: `~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml`의
  `human_control.axis_mappings.drive-steering_angle.axis`가 `2`(LT 트리거, 실측 rest값
  `1.0`)로 잘못 매핑됨. `scale=0.34` 곱해져 LB 누르는 즉시 `steering_angle≈0.34 rad`
  (풀락에 근접) 고정 명령.
  - 실측 `/joy` axes: `axes[0]`(좌스틱 좌우, rest≈0) / `axes[2]`(LT, rest=1.0) / `axes[5]`
    (RT, rest=1.0) — `f1tenth_control` 자체 조향 축 관례(`axes[0]`, CLAUDE.md)와도 불일치.
  - ⚠️ f1tenth_stack의 `ackermann_mux`는 자체 `teleop`(우선순위100)을 `f1tenth_control`의
    `/drive`(우선순위10, `control_real.launch.py`의 `joy_teleop_monitor` 출력)보다 항상
    우선하므로, **`control_real.launch.py`를 켜도 이 버그는 안 고쳐짐** — f1tenth_stack
    자체 설정 파일을 직접 고쳐야 함.
- 해결: `human_control.drive-steering_angle.axis`를 `2 → 0`으로 수정.

### 조향 서보 중립(트림) 캘리브레이션
`vesc.yaml`의 `steering_angle_to_servo_offset`을 실측 2점 선형보간으로 확정.

| offset | 실측 오차 |
|---|---|
| 0.5304 (원본) | 살짝 오른쪽 (정량 미측정) |
| 0.51 | +7° (오른쪽) |
| 0.362 | −12° (왼쪽, 과보정) |
| 0.4555 (2점 보간) | 거의 정중앙 |
| **0.4633 (최종)** | **오른쪽 미세 트림 +1° 반영, 확정값** |

- 이론 게인(`steering_angle_to_servo_gain=-1.2135`, 1도≈0.0212 서보값)보다 **실측 민감도가
  약 2.7배 낮음**(1도≈0.00779 서보값) — 서보 범위 클리핑/조향 링키지 기구학 비선형 때문으로
  추정. 향후 이런 트림 작업은 이론 게인이 아니라 **실측 2점 보간**으로 하는 게 정확함.
- **최종값 `steering_angle_to_servo_offset: 0.4633`**, 위치:
  `~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml`. 서보/조향
  링키지를 물리적으로 재장착하지 않는 한 유효.

### 기타 — 젯슨 원격 접속 세팅
- PC↔젯슨: SSH 키 인증(`~/.ssh/config`의 `Host jetson`, 사용자 `miru`, `10.1.1.3` — MIRU
  네트워크 DHCP라 유동적일 수 있음) + VS Code Remote-SSH 확장 설치, `~/2026_IFAC` 원격
  편집 가능.
- 젯슨 `joy_node`가 `/joy`를 발행 안 하던 문제 → `miru` 계정이 `input` 그룹에 없어서였음
  (`/dev/input/js0`가 `group input` 소유). `sudo usermod -aG input miru` + 재로그인으로 해결.
- 젯슨 colcon 빌드 시 `f110_msgs`/`f1tenth_control` 둘 다 `CMake Error: source ... does not
  match ... used to generate cache` — 예전 워크스페이스 구조(최상위 패키지)에서 `src/`
  구조로 바뀌며 생긴 stale 빌드 캐시. `rm -rf build install && colcon build`(전체 재빌드)로
  해결.

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
