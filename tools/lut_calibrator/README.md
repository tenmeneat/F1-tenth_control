# rosbag → Steering LUT 실측 보정 (오프라인)

실차 rosbag을 넣으면 보정된 Steering LUT CSV를 만들어낸다. ROS 노드를 띄우고 bag을
실시간 재생할 필요가 없다. 두 가지로 쓸 수 있다:

- **CLI** `calibrate_lut_from_bag.py` (아래) — 여러 bag 일괄 처리, 스크립트에 엮기 좋음
- **웹앱** [`webapp/`](webapp/) — 브라우저에 폴더 드래그, ROS·터미널 불필요.
  베이스 LUT(표준/축 확장)와 prior weight를 UI에서 고를 수 있다(2026-08-05).
  배포 URL: https://claude.ai/code/artifact/0bd7693f-3c83-4f0b-bcad-630fbce98c49

둘은 **바이트 단위로 같은 CSV**를 만든다(SHA-256 대조 검증).

`lut_calibrator_node`(C++ 온라인 노드)와 **같은 수식·같은 파일 포맷**을 쓴다. 즉
`calibration_state.csv` 누적치가 양방향 호환이라, 실차에서 온라인으로 쌓다가 오프라인으로
이어 쌓아도 되고 그 반대도 된다. (양방향 로드 검증 완료)

## 사용법

```bash
source /opt/ros/<distro>/setup.bash        # 메시지 디시리얼라이즈에 필요 (humble/jazzy 등)
source ~/2026_IFAC/install/setup.bash

# bag 하나
python3 calibrate_lut_from_bag.py ~/rosbag_log/0725/rosbag_20260725_013231

# 여러 개 한 번에 (순서대로 누적)
python3 calibrate_lut_from_bag.py ~/rosbag_log/0725/rosbag_* ~/rosbag_log/0724/run_0724_*

# 누적 무시하고 처음부터
python3 calibrate_lut_from_bag.py <bag> --fresh
```

산출물(기본 `~/f1tenth_lut_calibration/`):
- `LUT_calibrated.csv` — control_map_node에 넘길 LUT. `control_code/LUT_calibrated.csv`와
  이름이 같아서, 그 자리에 그대로 덮어쓰면 디폴트 LUT가 바뀐다(재빌드/재시동만 필요)
- `calibration_state.csv` — 누적 상태(다음 실행에 자동으로 이어짐)

적용 A) 임시로 켜보기(재빌드 불필요):
```bash
ros2 launch f1tenth_control control_real.launch.py \
    lookup_table_file:=$HOME/f1tenth_lut_calibration/LUT_calibrated.csv
```

적용 B) 검증 끝나서 디폴트로 확정:
```bash
cp $HOME/f1tenth_lut_calibration/LUT_calibrated.csv <repo>/control_code/LUT_calibrated.csv
# → 2026_IFAC로 동기화 후 colcon build --symlink-install --packages-select f1tenth_control
```

## 온라인 노드 대비 이점

| | 온라인 노드 | 이 도구 |
|---|---|---|
| 95초 bag 처리 | 95초(실시간 재생) | 1초 미만 |
| 12개 bag | ~10분 | 수 초 |
| 메시지 처리 | DDS 큐(depth 10) 드롭 가능 | bag 타임스탬프 순 전량 결정론적 |
| IMU 토픽 | `/imu/data` 고정(리매핑 필요) | 자동 판별 |
| 단위 오보정 | 증상 없이 LUT 오염 | 사전 검출 후 중단 권고 |

## 자동 안전장치 2가지

**① IMU 각속도 단위 검증.** VESC 자이로는 `sensor_msgs/Imu` 규약을 어기고 deg/s로
발행한다. 보정(`×π/180`)이 빠지면 `a_lat = v·yaw_rate`가 57배가 되어 LUT가 통째로
오염되는데, 이 노드는 `/drive`를 발행하지 않는 관찰자라 **주행 중엔 아무 증상이 없다.**
그래서 실측 IMU 요레이트를 조향 기구학 기대치(`v·tanδ/L`, 정의상 rad/s)와 RMS 대조해
스케일이 어긋나면 올바른 값을 제시하고 결과 폐기를 권고한다.

정상 범위는 0.1~10배다. 1.0이 아니라 폭넓은 이유는 실제 슬립·언더스티어 때문 —
07-24 bag들에서 실측 0.72배가 나오는데, 이는 WORKLOG의 "조향 지시보다 덜 돎" 진단과
독립적으로 일치한다.

**② 시뮬 데이터 차단.** `/ego_racecar/odom`이 보이면 건너뛴다. LUT는 실차 sysid
자산이라 시뮬 데이터가 섞이면 안 된다. (`--allow-sim`으로 강제 해제 가능)

## 커버리지 리포트가 핵심

보정의 실제 한계는 도구가 아니라 **그 주행이 실제로 밟은 (조향, 속도) 영역**이다.
샘플이 0인 셀은 원본 LUT 값 그대로 남는다(베이지안 블렌딩의 prior).

07-24·07-25 실차 bag 12개 전부를 넣어도 **커버리지 9.1%**, 속도 범위 1.0~2.4 m/s에
그친다. 저속 셰이크다운만 있었기 때문이다. 조향은 0.40 rad(물리한계 0.41)까지 닿았으므로
**부족한 건 속도축** — 실그립 피크를 보정하려면 더 빠른 주행 bag이 필요하다.

터미널에 찍히는 ASCII 커버리지 맵으로 어느 영역이 비었는지 바로 보인다.

## 🔴 조향축 확장 (0.3897 → 0.4320) — 2026-08-05 신설

### 왜 필요한가
현 LUT(서드파티 NUC6 원본)는 **조향축이 0.4에서 끝나고 그 행은 전부 NaN**이라, 열별 피크가
항상 **0.3897 rad**에 걸린다. 기구학으로 κ_max 1.245(R 0.803 m)인데 트랙이 요구하는 κ는
1.343(R 0.745 m)이라 **헤어핀을 물리적으로 못 돈다**. 컨트롤러 명령한계(우 0.4320)는
LUT 경로에서 도달 불가능한 낙관치다.

코너 속도대역(0.5~2.63 m/s)에서 a_lat(δ) 곡선이 **축 끝에서 아직 상승 중**임을 확인했다 —
타이어 포화가 아니라 축이 잘린 것이므로, 늘리면 실제 이득이 있다(0.4320에서 +5~17% a_lat).

### 왜 그냥 주행 반복으로는 안 되나 — 닭-달걀
자율주행의 조향 명령을 만드는 게 LUT 자신이다. LUT가 0.3897에서 saturate하므로 **자율 bag에는
그 위 샘플이 원리적으로 0개**다. 몇 바퀴를 돌든 채워지지 않는다.

→ **수동 조종은 LUT를 통과하지 않는다**(`/teleop` → `ackermann_mux` → `/ackermann_cmd` →
`ackermann_to_vesc` → 서보). **수동 풀락 주행이 유일한 데이터 소스**다.

### 절차

1. **녹화** — `/ackermann_cmd`가 필수다(`jetson_rec.sh`에 2026-08-05 추가됨. `f1rec.sh`는
   `-a`라 이미 잡힌다). `/drive`에는 수동 조향이 안 실린다.
2. **수동 정상원(定常圓) 주행** — 좌/우 풀락 각각, 속도 1.0 / 1.5 / 2.0 / 2.5 m/s.
   각 조건 10초면 50 Hz × 10 s = 500샘플로 충분하다(새로 채울 셀이 22열 × 4행 = 88개뿐).
   ⚠️ **일정 속도 유지가 관건**이라 넓은 곳이 필요하다. 속도가 흔들리면 열이 번진다.
3. **확장 베이스로 보정 실행**:
   ```bash
   python3 calibrate_lut_from_bag.py <풀락 bag들> \
       --base-lut tools/lut_calibrator/LUT_base_extended.csv \
       --fresh --prior-weight 0.5
   ```
   `--prior-weight`를 기본 3.0보다 **낮게** 줄 것 — 새 행의 값은 측정이 아니라 시드라
   실측이 지배해야 한다.
4. 결과 CSV를 `control_code/LUT_calibrated.csv`로 교체 → `~/2026_IFAC` 동기화 → 빌드.

### `LUT_base_extended.csv`의 정체 ⚠️
- 조향축 **60행 → 63행**(상단 0.4000 / 0.4107 / 0.4213 / **0.4320** 추가, 간격은 상단 기존
  간격을 이어감). 기존 59행 값은 **바이트 단위로 무변경**(검증 완료).
- 새 셀 값은 **측정이 아니라 시드**다. 열별로 마지막 두 유효점의 국소 접선으로 외삽했고
  (음수 기울기는 평평하게 눌러 가짜 피크 방지), 0.3897이 NaN인 열(그립상 불가)은 확장하지
  않았다 → 22열 × 4행 = 88셀.
- 🔴 **이 파일을 그대로 배포하면 안 된다.** `blend()`가 베이스 NaN 셀은 샘플이 와도 NaN으로
  두기 때문에([:302](calibrate_lut_from_bag.py#L302)) 시드가 필요했을 뿐이고, 값 자체는
  NUC6 표면의 외삽이라 우리 차보다 낙관적이다. **반드시 위 2번 실측으로 덮어쓴 뒤** 쓸 것.
  그래서 이 파일은 `control_code/`가 아니라 여기 둔다(기본 로드 경로에서 제외).

### 덤: K_us 6배 불일치를 같이 갈라낼 수 있다
확장본을 만들며 LUT 전체를 정상상태 모델 `a_lat = δ/(L/v² + K_us)`로 적합해 보니
**K_us ≈ 0.0047**(잔차 중앙 0.0%, p90 11.6%)인데, 우리 차 실측
`understeer_gradient`는 **0.028**이다. NUC6가 다른 차이니 방향은 말이 되지만 6배는 크다.
풀락 실측 데이터가 들어오면 이게 진짜인지 바로 갈린다 — 사실이라면 **LUT 상단만이 아니라
표 전체가 우리 차에 낙관적**이라는 뜻이라 그쪽이 더 큰 발견이다.

## 주요 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--fresh` | off | 이전 누적 무시하고 새로 시작 |
| `--min-speed` | 1.0 | 샘플 기록 최소 속도 [m/s] (저속 노이즈 배제) |
| `--prior-weight` | 3.0 | 원본 LUT 가중. 클수록 보수적(샘플 적은 셀이 원본에 가깝게 남음) |
| `--alpha` | 0.3 | 요레이트 LPF 계수 |
| `--imu-angular-scale` | π/180 | IMU 각속도 단위 보정(실차 VESC = deg/s) |
| `--base-lut` | 자동탐색 | 저장소 `control_code/` → ament share 순 |
| `--imu/odom/drive-topic` | 자동판별 | 토픽명 수동 지정 |
| `--allow-sim` | off | 시뮬 bag 차단 해제 |

## 검증

같은 bag에 대해 C++ 온라인 노드와 대조:
- 샘플 수 3363 vs 3364, 커버리지 136/3900 셀 동일
- 블렌딩된 LUT는 3900셀 중 66셀만 미세하게 다름(최대 상대차 19%)

차이의 원인은 **콜백 도착 순서**다. 온라인 노드는 DDS 전달 순서에 따라 IMU 시점의
캐시된 speed/steering이 달라져 샘플이 인접 셀로 흩어진다. 이 도구는 bag 타임스탬프
순으로 처리하므로 결정론적이고 재현 가능하다.
