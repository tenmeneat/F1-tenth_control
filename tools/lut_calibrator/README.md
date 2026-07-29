# rosbag → Steering LUT 실측 보정 (오프라인)

실차 rosbag을 넣으면 보정된 Steering LUT CSV를 만들어낸다. ROS 노드를 띄우고 bag을
실시간 재생할 필요가 없다. 두 가지로 쓸 수 있다:

- **CLI** `calibrate_lut_from_bag.py` (아래) — 여러 bag 일괄 처리, 스크립트에 엮기 좋음
- **웹앱** [`webapp/`](webapp/) — 브라우저에 폴더 드래그, ROS·터미널 불필요.
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
- `NUC6_glc_pacejka_lookup_table_calibrated.csv` — control_map_node에 넘길 LUT
- `calibration_state.csv` — 누적 상태(다음 실행에 자동으로 이어짐)

적용:
```bash
ros2 launch f1tenth_control control_real.launch.py \
    lookup_table_file:=$HOME/f1tenth_lut_calibration/NUC6_glc_pacejka_lookup_table_calibrated.csv
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
