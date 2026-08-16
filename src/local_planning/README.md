# local_planning

글로벌 Race Line의 순서를 고정한 정적 장애물 회피 패키지입니다.
정적 장애물은 `obstacle_detector`가 Cartesian AABB 전체를 투영해 만든 `/confirmed_static_obs`
(Layer 2 confirmed-only, 파라미터 `obstacles_topic`)의 Frenet
경계(`s_start/s_end/d_right/d_left`)로 받습니다. local planner는 장애물 좌표를 다시 변환하지
않고 그 경계를 그대로 사용해 트랙 위상을 보존한 회피선을 만듭니다.
결과는 Cartesian `x_m/y_m`이 채워진 `/avoid_waypoints`로 발행합니다.
글로벌 waypoint의 `d_left/d_right`는 차량 중심 가용 한계로 사용하며,
`wall_safety_margin_m`만 좌우 경계에서 차감합니다. 추종오차·차량 반폭·장애물 물리 안전마진은
이 중심 한계에 다시 포함하지 않습니다. 장애물 목표는 속도별 횡가속 표로 제한한 회피속도와
곡률의 tracking-error LUT 최댓값으로 만들고, 완성된 경로도 waypoint별 제한 속도로 다시
검사합니다. 회피 waypoint 속도는 재계산 곡률에서 `v²|κ| <= a_lat,max(v)`를 만족하도록 낮춥니다.
같은 ID의 실측 Frenet AABB 합집합은 고정하지만 `d_var` 기반 횡방향 팽창은 적용하지 않습니다.

장애물이 나타나면 자유공간에서 새 경로를 검색하지 않습니다. 현재 글로벌 waypoint 구간을 그대로
선택하고 Frenet `d(s)`만 장애물 반대쪽으로 이동합니다. 진입과 복귀는 위치·기울기·2차 미분이
연속인 구간별 5차 프로파일로 만들며, 가장 긴 안전 후보부터 선택합니다. 따라서 여유가 있는
장애물은 멀리서 완만하게 피하고, 스네이크처럼 서로 다른 트랙 조각이 지도상 가까이 붙어 있어도
다른 조각으로 경로가 점프하지 않습니다.
선택된 경로는 최신 Frenet 장애물 경계에도 안전한 동안 geometry를 그대로 유지하며, 안전정지는 즉시 latch하고
연속 안전 판정 뒤에만 해제해 perception 흔들림이 경로 모드 진동으로 전달되지 않게 합니다.
첫 장애물 군집은 준비 감속 동안 같은 ID를 실제 `/confirmed_static_obs` 메시지에서 3회 모으고 최소
관측 시간도 기다립니다. 그 Frenet 경계 합집합은 종방향에만 `3*sqrt(s_var)`와 고정 크기
마진을 적용하고, 횡방향은 실측 `d_right/d_left` 합집합 그대로 commitment Guard로 고정합니다.
후속 같은-ID uncertainty envelope가 Guard 안에 있으면 출력 geometry를 그대로 유지합니다.
Guard 밖의 변화가 전체 여유만 침범하면 3 planning cycle을 확인한 뒤 재계획하고, detector 원본
경계에 차량 폭을 적용한 hard 영역과 겹치면 즉시 재계획하거나 safe-stop합니다.
실제 횡이동 전에는 Guard를 벗어난 새 장애물에 따라 방향을 다시 고를 수 있지만, 회피 진입 뒤에는
방향을 고정합니다.
첫 회피와 다른 blocking 군집은 기존 merge 전후와 관계없이 현재 ego 위치에서 미리 안정화합니다.
첫 장애물을 완전히 지난 뒤 현재 `ego.d`에서 다음 회피 spline을 직접 연결하며, 중간 global
복귀·빈 경로·불필요한 safe-stop을 만들지 않습니다. 이때 완료된 첫 maneuver의 방향 잠금은
다음 계획에 전달하지 않습니다.
정말 안전한 연결 경로가 없을 때는 기존 committed geometry 위에서 감속하고, 그마저 불가능할
때만 현재 `d`의 zero-speed hold를 사용합니다.
유효한 `/confirmed_static_obs`를 한 번 받은 뒤 센서 갱신이 끊기면 마지막 회피 commitment와 장애물
스냅샷을 유지합니다. 현재 회피가 끝나면 정상적으로 global handoff를 완료하고, 센서가 계속
끊겨 있더라도 다음 랩에서는 저장된 장애물로 다시 회피합니다. 새 유효 장애물 배열이 들어올 때만
이 기억을 교체하며, Frenet odometry까지 끊긴 최악 상황에서는 마지막 위치의 zero-speed hold를
발행합니다.

핵심 동작, 토픽, 파라미터, 실행 및 검증 절차는
[`docs/local_planner.md`](docs/local_planner.md)에 단계별로 정리되어 있습니다.

```zsh
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch local_planning local_planning.launch.py
```

계획 출력은 `/avoid_waypoints`, RViz 경로 출력은 `/local_planning/path`입니다. 최종
`/local_waypoints` 선택과 발행은 `/state`를 구독하는 `wpnt_publisher`가 단독으로 담당합니다.
