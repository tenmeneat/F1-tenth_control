# bag_analyzer — 주행 rosbag 통합 분석 도구

F1TENTH 실차/시뮬 주행 rosbag 하나를 넣으면 **자체 실행형 `report.html`** 하나를 만든다.
셰이크다운마다 bag → HTML → 브라우저로 원인 분석 + 튜닝 방향을 바로 본다.

## 무엇이 나오나
1. **3D 주행 리플레이** (RViz 스타일, 순수 JS·의존성 0)
   — map 프레임 궤적(속도색) + LiDAR 스캔 + 차량 pose 애니메이션. 드래그 회전 / 휠 줌 / Shift+드래그 이동.
2. **주행상태 분석 그래프** (matplotlib) — 궤적 / 속도 / 요레이트 / **횡가속도(그립 한계선)** / **종가속도(감속 권한)** / IMU raw.
3. **자동 튜닝 가이드** — 아래 휴리스틱으로 원인을 분류하고 **권장 launch 명령**을 뽑는다.
   - `a_lat = vx·wz` 그립 초과·최댓값 → 스핀/조향 포화 감지
   - 사고 이전 구간의 `dvx/dt` → **감속 권한 없음** 자동 플래그(벽 충돌·E-stop 후 감속은 제외)
   - 선회반경이 최소반경(≈0.76m)에 닿는지 → 풀락 스핀 감지
   - 그립 초과 코너들이 요구한 안전속도 → `max_speed` 권장치

## 사용법
```bash
source /opt/ros/humble/setup.bash          # rclpy 디시리얼라이즈에 필요
python3 analyze_bag.py <bag폴더 | .db3> [옵션]
xdg-open <name>_report.html
```
옵션:
- `--out report.html` 출력 경로
- `--odom /odom` odom 토픽(실차 MCL이면 `/pf/pose/odom`)
- `--grip 5.0` 목표 그립 a_lat [m/s²] (BEXCO 저마찰 기준 5.0 권장)

## 전제 / 한계
- ROS 2(Humble) 소싱 + `matplotlib`, `numpy` 필요.
- bag에 **`/global_waypoints`·`/drive`가 있으면** 명령 vs 실측 비교(플래너 프로파일·명령 조향)까지,
  없으면 실측(odom·imu) 기반 분석만. 둘 다 자동 판별.
- map 프레임 pose는 `/tf`의 `map→odom→base_link` 합성으로 복원. TF가 없으면 odom pose로 폴백.
- 한글 그래프는 시스템에 Nanum/Noto CJK 폰트가 있으면 자동 사용(없으면 라벨만 깨짐, 분석엔 무관).

## 예 (2026-07-23 시케인 크래시)
`22_43_33` bag → `[CRITICAL] 감속 권한 없음` + `[CRITICAL] 그립 초과 → 스핀` →
권장 `max_speed:=2 max_lateral_accel:=5 min_speed:=1.5`.
