# LUT Calibrator 웹앱 — 무터미널 rosbag → 보정 LUT CSV

브라우저에 rosbag 폴더를 드래그하면 보정된 Steering LUT CSV를 내려받는 도구.
ROS·터미널·파이썬 전부 불필요하고, 파일은 서버로 올라가지 않고 브라우저 안에서만 처리된다.

**배포 URL**: https://claude.ai/code/artifact/0bd7693f-3c83-4f0b-bcad-630fbce98c49

CLI판(`../calibrate_lut_from_bag.py`)과 **바이트 단위로 같은 CSV**를 만든다(아래 검증 참고).
로컬/자체 호스팅은 `webapp.html`을 브라우저로 열면 된다(단일 파일, 외부 의존 0).

## 기능

- rosbag 폴더 **여러 개 동시** 드래그 → 폴더별로 묶어 순차 누적
- IMU 토픽 자동 판별(`/imu/data` ↔ `/sensors/imu/raw`), 분할 bag(.db3 여러 개) 병합
- **IMU 각속도 단위 자동 검증** — 어긋나면 올바른 스케일을 제시하고 결과 폐기 권고
- **시뮬 bag 차단**(`/ego_racecar/odom` 감지) — LUT는 실차 sysid 자산
- 커버리지 히트맵(조향×속도, 셀 hover 시 샘플 수·원본/보정 a_lat)
- 보정 LUT CSV + 누적 상태 CSV 내보내기. 다운로드가 막히는 환경을 위해 텍스트 복사 경로도 제공
- 누적은 브라우저에 저장되어 다음 방문에 이어짐(`누적 초기화`로 리셋).
  이전 `calibration_state.csv`를 같이 드래그하면 그 위에 이어 쌓는다

## 빌드

```bash
python3 build.py       # app.html + sql.js + 베이스 LUT → webapp.html / webapp.fragment.html
```

- `app.html` — 소스(플레이스홀더 3개: sql.js glue, wasm base64, 베이스 LUT CSV)
- `webapp.html` — 독립 실행형(로컬/자체 호스팅)
- `webapp.fragment.html` — 아티팩트 게시용 본문 조각
- `vendor/` — bag_analyzer 웹앱의 sql.js를 심볼릭 링크로 재사용

베이스 LUT(`control_code/LUT_calibrated.csv`)는 빌드 시 페이지에 인라인된다.
LUT 원본이 바뀌면 `build.py`를 다시 돌릴 것.

## 검증 (헤드리스 Chrome 엔드투엔드)

실차 bag `rosbag_20260725_013231`(5랩, 27MB)로 CLI판과 대조:

| | 웹앱 | CLI |
|---|---|---|
| 샘플 수 | 3363 | 3363 |
| 커버리지 | 136/3900 | 136/3900 |
| 단위 검증 비율 | 0.734 | 0.734 |
| 보정 LUT CSV | `8f2d9ac5…` | `8f2d9ac5…` |
| 누적 상태 CSV | `7ef028ae…` | `7ef028ae…` |

SHA-256이 양쪽 다 일치 = **바이트 단위 동일**.

추가로 확인한 경로: 여러 bag 동시 처리(폴더 그룹핑), 시뮬 bag 차단, 필수 토픽 없는 bag 스킵,
상태 CSV 로드 후 이어쌓기, 40MB bag 단독 처리, 라이트/다크 렌더.

⚠️ 축(첫 행·첫 열)은 원본 LUT 문자열을 그대로 보존한다. C++ `lut_calibrator_node`는 축까지
`%g`로 재포맷하므로 **축 표기만** 다를 수 있다(값은 동일, `std::stod` 파싱 결과도 동일).
데이터 셀은 세 구현 모두 같은 `%g` 포맷.

## 한계

- **sqlite3(.db3) bag만** 지원 — MCAP은 미지원(ROS 2 Jazzy 기본이 mcap이므로
  `ros2 bag record -s sqlite3` 필요). [[jetson-reinit-todo-2026-07-26]]
- 브라우저 메모리 한도상 아주 큰 bag을 **한 번에 여러 개** 넣으면 실패할 수 있다.
  앱은 bag을 하나씩 읽고 버리므로 순차 드래그하면 문제없다.
