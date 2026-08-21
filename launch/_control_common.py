from typing import Any, Optional

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

IMU_LINEAR_SCALE_REAL = 9.80665      # g → m/s². VESC가 g로 발행(2026-07-19 소스 확인)
IMU_LINEAR_SCALE_SIM  = 1.0          # sim_imu_bridge_node는 0 고정
IMU_ANGULAR_SCALE_REAL = 0.0174533   # deg/s → rad/s (pi/180). VESC가 deg/s로 발행
IMU_ANGULAR_SCALE_SIM  = 1.0         # sim_imu_bridge_node는 이미 rad/s로 중계

# ⚠️ 조이스틱 드라이버·sim_imu_bridge_node 포함 여부 등 안전 관련 구조 차이는
# 일부러 여기로 옮기지 않고 각 진입점 파일에 그대로 둔다(환경을 잘못 골라 안전
# 기능이 빠진 채 기동되는 실수를 구조적으로 차단하기 위함).

def declare_common_args(
        sector_scale_enable_default='false', hfi_launch_guard_enable_default='false'):
    """두 런치파일에서 동일하게 쓰는 인자 선언 목록."""
    return [

        DeclareLaunchArgument(
            'cruise_enable', default_value='true',
            description='/opp_obs 기반 종방향 cruise speed cap 사용'
        ),
        # ── 크루즈 종방향 제어 튜닝 ──
        # config/cruise_controller.yaml은 단독 실행용 기준값으로 남기고,
        # sim/real 공통 launch에서는 아래 인자가 같은 이름의 ROS parameter를 덮어쓴다.
        DeclareLaunchArgument(
            'trailing_mode_distance', default_value='true',
            description='true=고정 거리 추종, false=minimum_gap + trailing_gap*v 시간 간격 추종'
        ),
        DeclareLaunchArgument(
            'trailing_gap', default_value='5.0',
            description='고정 거리 모드의 목표 간격 [m] 또는 시간 간격 모드의 headway [s]'
        ),
        DeclareLaunchArgument(
            'minimum_gap', default_value='0.8',
            description='정지 시에도 유지할 최소 목표 간격 [m]'
        ),
        DeclareLaunchArgument(
            'max_desired_gap', default_value='0.0',
            description='목표 간격 상한 [m], 0=비활성. state_machine interference_distance_m 이하 필수'
        ),
        DeclareLaunchArgument(
            'trailing_p_gain', default_value='1.0',
            description='크루즈 간격 오차 P 게인'
        ),
        DeclareLaunchArgument(
            'trailing_i_gain', default_value='0.0',
            description='크루즈 간격 오차 I 게인'
        ),
        DeclareLaunchArgument(
            'trailing_d_gain', default_value='0.5',
            description='크루즈 상대속도 D 게인'
        ),
        DeclareLaunchArgument(
            'integral_limit', default_value='2.0',
            description='크루즈 간격 오차 적분 절댓값 상한'
        ),
        DeclareLaunchArgument(
            'allow_accel_trailing', default_value='true',
            description='true이면 추종 상한이 현재 ego 속도보다 높아지는 것을 허용'
        ),
        DeclareLaunchArgument(
            'emergency_stop_distance', default_value='0.45',
            description='보수적 간격이 이하일 때 속도 상한을 0으로 만드는 거리 [m]'
        ),
        DeclareLaunchArgument(
            'relative_deceleration', default_value='1.8',
            description='분해 제동값이 0일 때 사용하는 보수적 기본 감속도 [m/s^2]'
        ),
        DeclareLaunchArgument(
            'ego_deceleration', default_value='0.0',
            description='실측 ego 감속도 [m/s^2], 0=relative_deceleration으로 폴백'
        ),
        DeclareLaunchArgument(
            'opponent_deceleration', default_value='0.0',
            description='가정할 상대차 감속도 [m/s^2], 0=relative_deceleration으로 폴백'
        ),
        DeclareLaunchArgument(
            'actuation_latency', default_value='0.0',
            description='상대차 감속 관측부터 ego 실제 제동 개시까지의 실측 지연 [s]'
        ),
        DeclareLaunchArgument(
            'ego_front_offset', default_value='0.25',
            description='ego Frenet 기준점에서 앞범퍼까지 거리 [m]'
        ),
        DeclareLaunchArgument(
            'uncertainty_sigma', default_value='2.0',
            description='간격 표준편차에 곱해 raw gap에서 빼는 보수 배수'
        ),
        DeclareLaunchArgument(
            'gap_uncertainty_horizon_max', default_value='1.0',
            description='상대차 위치·속도 공분산 시간 전파 지평 상한 [s]'
        ),
        DeclareLaunchArgument(
            'opp_speed_confidence_z', default_value='1.0',
            description='상대차 속도 하한 계산에 쓰는 표준편차 배수'
        ),
        DeclareLaunchArgument(
            'opponent_timeout', default_value='0.15',
            description='CRUISE 중 /opp_obs 신선도 timeout [s]'
        ),
        DeclareLaunchArgument(
            'ego_timeout', default_value='0.20',
            description='CRUISE 중 ego Frenet odom 신선도 timeout [s]'
        ),
        DeclareLaunchArgument(
            'state_timeout', default_value='0.30',
            description='CRUISE 중 /state heartbeat 신선도 timeout [s]'
        ),
        DeclareLaunchArgument(
            'clear_confirm_sec', default_value='1.00',
            description='/opp_obs 빈 메시지가 지속돼야 타깃을 해제하는 확인 시간 [s]'
        ),
        DeclareLaunchArgument(
            'blind_trailing_speed', default_value='1.5',
            description='CRUISE 중 상대차·ego·state 입력이 stale일 때 속도 상한 [m/s]'
        ),
        DeclareLaunchArgument(
            'cruise_speed_limit_timeout', default_value='0.15',
            description='control_map_node의 /cruise_speed_limit 신선도 timeout [s]'
        ),
        DeclareLaunchArgument(
            'cruise_stale_speed', default_value='1.5',
            description='/cruise_speed_limit이 stale일 때 control_map_node 속도 상한 [m/s]'
        ),

        # ── 조향 스케일러 (가감속/속도 구간별 조향 게인 완화) ──
        DeclareLaunchArgument(
            'acceleration_scaler_for_steering', default_value='1.0',
            description='가속 중(acc_mean>=1.0) 조향각에 곱하는 스케일러'
        ),
        DeclareLaunchArgument(
            # 🔴 2026-08-19: 0.85 → 1.0 (무력화). 제거가 아니라 기본값만 중립으로 돌린 것이라
            # `:=0.85`로 언제든 되돌릴 수 있다. 근거는 CLAUDE.md ②-w:
            #  ① **다른 노드가 이 항의 권한을 조용히 3배로 키웠다.** 게이트가 |a_x| ≥ ref(1.0)
            #     인데, 07-26 젯슨 서비스 브레이크 패치 전에는 감속이 coast −0.4 m/s²라 w≈0.4로
            #     거의 안 물렸다. 패치 후 실측 |a_x| p50 1.06~1.97 · 감속 사이클 28~52%라
            #     **코너 진입마다 0.85로 포화**한다 — 아무도 그렇게 결정한 적이 없다.
            #  ② 같은 저장소가 이미 같은 실수를 한 번 되돌렸다: 바로 아래 `start_scale_speed`
            #     주석의 "턴인 구간 상시 −15% 다운스케일 → 코너 탈출 바깥쪽 오차 +0.11~0.21 m".
            #     이 항은 그 −15%를 **턴인 구간에 무조건** 건다.
            #  ③ ②-p로 조향이 LUT → 자전거 역모델이 된 뒤로는 모델이 하중 의존성을
            #     K_us(a_lat)로 담는다. 최종 조향각에 곱하는 상수 배율은 기하항(L·κ/v²)까지
            #     같이 깎아서 물리적으로 틀린 자리에 걸린다.
            #  ④ 폐루프 시뮬: 0.85에서 max 0.658 / >0.2 m 47.7% (실측 0818 p95 0.608 ·
            #     52.5%와 일치) → 1.0에서 max 0.216 / 2.1%. **현재 추종오차의 최대 단일 원인.**
            #  ⚠️ 이 항을 켜 둔 채 steering_fb_gain을 내리면 **더 나빠진다**(0.658 → 0.805).
            #     배율이 FF까지 깎는데 FF에는 되돌릴 피드백이 없기 때문이다. 순서가 중요하다.
            'deceleration_scaler_for_steering', default_value='1.0',
            description='감속 중 조향각에 곱하는 스케일러. 1.0 = 무개입(2026-08-19 기본값). '
                        '0.85가 구 기본값이며 코너 진입에서 상시 −15%가 걸렸다 — ②-w'
        ),
        DeclareLaunchArgument(
            # ⚠️ 2026-08-16 08-15 실측 트랙 기준(최고 5.7~6.96 m/s)으로 3.2/4.6로 낮췄던 값을
            # 0814 확정값(7.0/8.0)으로 되돌린다. 3.2~4.6은 그립 캡이 아직 안 걸린 코너
            # 진입(턴인) 구간과 겹쳐 상시 -15% 다운스케일이 걸렸고, 같은 창에서 들어온 L1
            # 확장(avoidance_l1_damping)과 겹쳐 코너 탈출에서 바깥쪽(언더스티어 방향) 정상상태
            # 오차를 키웠다(추정 +0.11~0.21 m, 벽 여유 p5 0.145 m 섹터 마진의 대부분 소진).
            # 재도입하려면 턴인 구간(그립 캡이 걸리기 전 속도대)을 피해서 재측정할 것.
            'start_scale_speed', default_value='7.0',
            description='속도 비례 조향 다운스케일 시작 속도 [m/s]'
        ),
        DeclareLaunchArgument(
            'end_scale_speed', default_value='8.0',
            description='속도 비례 조향 다운스케일 종료 속도 [m/s] (이후 downscale_factor 최대 적용)'
        ),
        DeclareLaunchArgument(
            'downscale_factor', default_value='0.10',
            description='고속 구간 조향각 다운스케일 최대 비율'
        ),
        DeclareLaunchArgument(
            'speed_lookahead', default_value='0.15',
            description='종방향 목표속도용 예측 룩어헤드 시간 [s]'
        ),
        DeclareLaunchArgument(
            'speed_lookahead_for_steering', default_value='0.0',
            description='조향 계산용 속도 예측 룩어헤드 시간 [s]'
        ),

        # ── 경로소스 신선도 ──
        DeclareLaunchArgument(
            'odom_timeout', default_value='0.35',
            description=(
                'odom 워치독 [s], 0이면 비활성. 이 시간 넘게 <odom_topic> 미수신이면 '
                '조향을 직전 각으로 유지한 채 속도 0으로 안전 정지. '
                '0818 run_0818_134408: MCL 1.1s 정지 중 컨트롤러가 얼어붙은 명령을 '
                '50Hz로 계속 발행해 벽 충돌. '
                '2026-08-19: 0.5 -> 0.35. KICP max_dead_reckoning_sec를 2.0 -> 0.4로 '
                '함께 줄여 최악 블라인드 구간을 2.5s(약 14 m) -> 0.75s(약 5 m)로 압축. '
                '정상 주행 /pf/pose/odom 최대 공백이 48~161 ms라 0.35는 2배 이상 여유'
            )
        ),
        DeclareLaunchArgument(
            'local_fresh_timeout', default_value='0.3',
            description='이 시간(s) 넘게 /local_waypoints 미수신 시 글로벌 경로로 폴백'
        ),
        DeclareLaunchArgument(
            'closest_idx_max_heading_err', default_value='1.40',
            description='경로 접선과 차량 헤딩의 허용 오차 [rad]. 0이면 게이트 비활성(구 거동)'
        ),
        DeclareLaunchArgument(  
            'l1_offset', default_value='0.6',
            description='L1 룩어헤드 거리의 **절편** [m] (공식: l1_offset + v*l1_speed_gain). '
                        '구 이름 l1_gain'
        ),
        DeclareLaunchArgument(
            'l1_speed_gain', default_value='0.35',
            description='L1 룩어헤드 거리의 **속도 계수** [s] (공식: l1_offset + v*l1_speed_gain). '
                        '구 이름 l1_distance'
        ),
        DeclareLaunchArgument(
            't_clip_min', default_value='0.9',
            description='L1 룩어헤드 거리 하한 [m] (낮을수록 저속/시케인 구간에서 국소 지그재그를 '
                        '쫓아 고주파 조향 유발 가능)'
        ),
        DeclareLaunchArgument(
            't_clip_max', default_value='5.0',
            description='L1 룩어헤드 거리 상한 [m]'
        ),
        # ⚠️ 예전엔 t_clip_min을 횡가속 분모 하한으로 재사용했다 — t_clip_min은 룩어헤드
        #    튜닝 노브라, 낮추면 조용히 횡가속 명령 상한이 올라갔다(0.6이면 6 m/s에서
        #    최대 lat_acc 120 m/s²). 수치 발산 방지용 하한은 별도 파라미터로 분리했다.
        DeclareLaunchArgument(
            'l1_min_denom', default_value='0.9',
            description='L1 횡가속 분모 하한 [m] (목표점이 차량에 붙었을 때 발산 방지). '
                        't_clip_min과 무관하게 튜닝'
        ),
        DeclareLaunchArgument(
            'steering_speed_cap_measured', default_value='true',
            description='조향용 속도(횡가속 게인+자전거 역모델)를 실측 속도로 상한. '
                        'false면 구 거동(프로파일 속도만 사용) — 롤백용'
        ),
        DeclareLaunchArgument(
            'status_log_period_ms', default_value='2000',
            description='컨트롤러 상태 한 줄 로그 주기 [ms]. 0 = 끔 (디버깅 시 500 권장)'
        ),
        DeclareLaunchArgument(
            'l1_jump_warn_m', default_value='1.0',
            description='L1 목표점이 차량보다 이만큼[m] 더 튀면 경고+카운트. 0 = 검출 끔'
        ),

        DeclareLaunchArgument(
            'sector_scale_enable', default_value=sector_scale_enable_default,
            description='섹터별 max_lateral_accel 스케일 사용 (true/false). '
                        'false면 전 구간 전역 MLA — 구 거동과 완전히 동일'
        ),
        DeclareLaunchArgument(
            'sector_scale_file', default_value='',
            description='섹터 스케일 표(YAML) 경로. 비우면 자동 탐색하고, 못 찾으면 '
                        '/global_waypoints의 κ로 표를 자동 생성한다(scale 전부 1.0). '
                        '🟢 2026-08-19에 config/sectors.yaml을 삭제했으므로 기본은 자동 생성이다 — ②-x'
        ),
        DeclareLaunchArgument(
            'sector_scale_topic', default_value='/sector_scales',
            description='섹터 테이블 토픽 (std_msgs/Float32MultiArray, '
                        '[track_length, (s0,s1,scale)×N], transient_local)'
        ),
        DeclareLaunchArgument(
            'sector_scale_max', default_value='1.3',
            description='허용 최대 scale. 이보다 큰 값이 오면 테이블 전체를 버린다'
        ),
        DeclareLaunchArgument(
            'sector_scale_blend', default_value='0.5',
            description='섹터 경계 선형 블렌딩 폭 [m]. MCL Frenet s 지터 실측 σ 47mm/max 159mm — '
                        '단 근본 대책은 경계를 κ 최소점에 두는 것(bag_analyzer가 그렇게 뽑아준다)'
        ),

        DeclareLaunchArgument(
            'sector_scale_track_len_tol', default_value='0.02',
            description='테이블 선언 랩길이 vs 실제 글로벌 경로 길이 허용 오차 [m]. '
                        '넘으면 테이블 폐기 — 라인을 재생성하면 s가 다른 코너를 가리킨다'
        ),
        DeclareLaunchArgument(
            'sector_scale_global_only', default_value='true',
            description='회피/추월(/state != GLOBAL) 중에는 스케일을 끄고 전역 MLA로 복귀. '
                        'scale의 근거인 벽 여유는 차가 라인 위에 있을 때 잰 값이라 필수'
        ),
        DeclareLaunchArgument(
            'sector_scale_state_topic', default_value='/state',
            description='주행 상태 토픽 (f110_msgs/StateMachine). '
                        '미수신이면 회피 여부를 모르므로 스케일이 자동 비활성'
        ),
        DeclareLaunchArgument(
            'sector_scale_state_timeout', default_value='1.0',
            description='/state 신선도 [s]. 넘으면 스케일 비활성(모르면 끈다)'
        ),
        DeclareLaunchArgument(
            'sector_scale_timeout', default_value='3.0',
            description='섹터 테이블 데드맨 [s], 0이면 비활성. 발행자가 이만큼 조용하면 '
                        '전역 MLA로 복귀. 학습기가 값을 올린 채 죽는 경우를 막는다 '
                        '(발행자는 1 Hz로 같은 표를 재발행한다)'
        ),

        DeclareLaunchArgument(
            'sector_learn_mode', default_value='static',
            description='static=scale 고정(표를 그대로 발행, 게이트는 로그만 — 구 sector_pub 거동) / '
                        'guard=하향만(결선, 안전) / explore=상향+하향(연습 전용). '
                        '⚠️ explore는 차가 스스로 코너 속도를 올린다 — E-stop에 손을 올릴 것'
        ),
        # 🟢 2026-08-19: guard 모드에서 learned/의 최신 학습 결과를 자동으로 싣는다.
        # guard의 존재 이유가 "연습에서 수렴시킨 표를 싣고 들어가는 것"인데 매번 경로를
        # 손으로 줘야 하면 취지가 무너진다. 0랩 파일(학습 없음)은 건너뛰고, 라인이 바뀐
        # 표를 집어도 track_length 불일치로 폐기되어 scale 1.0에서 시작한다(안전 방향).
        # ⚠️ explore/static에는 적용되지 않는다 — 자동 로드는 항상 "빨라지는" 방향이라
        #    그 둘은 1.0에서 시작한다는 보장을 유지한다.
        DeclareLaunchArgument(
            'sector_learn_auto_latest', default_value='true',
            description='guard 모드에서 learned/ 최신 학습 결과를 자동 로드 (false면 끔). '
                        'sector_scale_file을 명시하면 그쪽이 우선 — ②-x'
        ),
        DeclareLaunchArgument(
            'sector_learn_watch', default_value='true',
            description='표 YAML 저장을 감시해 재로드(라이브 튜닝, --yaml을 준 경우). '
                        '⚠️ 재로드는 학습 상태를 초기화한다'
        ),
        DeclareLaunchArgument(
            'sector_learn_out', default_value='',
            description='학습 결과 저장 경로. 비우면 저장 안 함. '
                        '연습에서 뽑아 결선 sectors.yaml로 쓰는 것이 의도된 흐름'
        ),

        # ── 조향 생성: 자전거 역모델 + FF/FB 분리 (②-p) ───────────────────────
        # 구 LUT 역조회는 2026-08-17에 삭제됐다(롤백은 git 0d16173 — 메모리 참고).
        # 🔴 2026-08-20: 코드 기본값 1.35 → 1.0. 1.35에는 이 저장소의 실측 근거가 없었다
        #    (CLAUDE.md 파라미터 표에 항목 자체가 없고, nhw_ifac에서 통째로 들여온 값이다).
        #    08-16에 고친 것은 "언제 켜지나"(local_fresh → /state)였고 크기 1.35는 검토된 적이
        #    없다. 실효 L1 offset 0.6 → 0.81이 되는데, 08-14에 기각된 것은 0.6 → 0.4로 **줄이는**
        #    방향이었지 키우는 쪽이 검증된 것이 아니다.
        #    실측(2026-08-19 run_001453, 자율·장애물 없음·v>2.0, 곡률·속도 대역 정합):
        #      라인 추종오차 p50/p90  GLOBAL 0.120/0.296 m  vs  AVOID 0.169/0.464 m (+41%/+57%)
        #      L1 거리 실측          GLOBAL 2.25 m         vs  AVOID 2.73 m (+21%)
        #    같은 백에서 FSM이 자율 시간의 88%를 AVOID에 고착돼 있었고(복귀 조건 결함, 별건),
        #    그동안 이 배수가 상시 걸려 추종을 악화시키고 있었다. 1.0 = AVOID에서도 GLOBAL과
        #    같은 L1 → 고착의 피해가 사라진다.
        #    되돌리기: avoidance_l1_scale_max:=1.35 (진짜 회피 중 횡진동이 관측되면 1.15부터)
        DeclareLaunchArgument(
            'avoidance_l1_scale_max', default_value='1.0',
            description='/state != STATE_GLOBAL 일 때 L1 룩어헤드에 곱하는 배수. '
                        '1.0 = GLOBAL과 동일(기본). 키우면 감쇠가 늘어 회피 중 횡진동은 '
                        '줄지만 추종오차가 커진다. 2026-08-19 실측으로 1.35의 근거가 '
                        '없음이 확인되어 1.0으로 되돌렸다'
        ),
        DeclareLaunchArgument(
            'avoidance_l1_damping_enable', default_value='true',
            description='위 배수를 적용할지. false 면 배수와 무관하게 항상 GLOBAL과 같은 L1'
        ),
        DeclareLaunchArgument(
            'steering_fb_gain', default_value='0.8',
            description='FF/FB 분리 게인 (bicycle 모델 전용). L1 명령 중 경로 곡률로 '
                        '설명되지 않는 보정분에만 곱한다. 1.0 = 분리 전과 수학적으로 동일 '
                        '(안전한 출발점). 낮추면 경로 추종은 FF가, 오차 보정은 L1이 맡아 '
                        'l1_offset의 "정확도 vs 횡진동" 트레이드오프가 분리된다'
        ),
        DeclareLaunchArgument(
            'curvature_ff_preview', default_value='0.0',
            description='FF가 곡률을 읽을 전방 거리 [m]. 0 = 최근접점(정상상태 정의). '
                        '조향→요레이트 지연 실측 140 ms 보상이 필요하면 v*0.14 부근부터'
        ),
        DeclareLaunchArgument(
            'understeer_gradient_adapt_gain', default_value='0.0',
            description='K_us 온라인 적응 LPF 게인 1/τ [1/s]. **0 = 관측 전용**(추정·로그만 '
                        '하고 적용 안 함 — 기본). ⚠️ 켜면 조향 생성뿐 아니라 조향 권한 캡'
                        '(코너 진입속도)까지 함께 지배하므로, 관측 로그로 수렴을 먼저 볼 것'
        ),
        DeclareLaunchArgument(
            'understeer_adapt_min_lat_acc', default_value='3.0',
            description='K_us 학습 관측성 게이트 [m/s²]. 이보다 큰 횡가속에서만 배운다 '
                        '(코너 전용) — 직선에서 배우는 조향 트림 추정기와 영역을 갈라 둔 것'
        ),
        DeclareLaunchArgument(
            'steering_speed_floor', default_value='0.5',
            description='조향 계산에 쓰는 속도의 하한 [m/s]. local_planning이 safe stop으로 '
                        'vx_mps=0을 발행하면 조향까지 0이 되어 굴러가는 중에 바퀴가 곧게 '
                        '펴진다(코너 비상정지 = 바깥 벽 직진). 0 = 구 거동'
        ),
        DeclareLaunchArgument(
            'understeer_curve_enable', default_value='false',
            description='K_us를 하중별 곡선 K_us(a_lat)로 쓴다 (②-q). **false = 관측 전용**'
                        '(빈별 학습·로그만, 조향엔 스칼라 사용 — 기본). LUT가 담으려던 타이어 '
                        '비선형성의 1차원 대체다. 켜면 고하중에서 조향이 커지므로 저속부터'
        ),
        DeclareLaunchArgument(
            'understeer_curve_min_samples', default_value='300',
            description='K_us 곡선 빈을 실제로 쓰기까지 필요한 빈당 샘플 수. 미달 빈은 '
                        '스칼라값으로 폴백하므로 학습이 덜 된 하중대에선 정확히 구 거동이다'
        ),

        DeclareLaunchArgument(
            # ⚠️ 2026-08-16: 0.9(그리고 그 직전 1.0)는 근거 문서·주석 없이 08-15~16에
            # 들어왔다가 08-16에 0.9로 되돌아온 값이다. 08-07에 "실측치(≈1.0)로 올리자"는
            # 제안이 검토 후 기각되고 0.85로 확정된 결정을 근거 없이 뒤집은 것 — 0.85로 복귀.
            # 🔴 ②-p 주의: LUT가 전 속도에서 12~36% 덜 꺾고 있었고 1/0.85=+17.6%가 그
            # 범위 한복판이다 — 이 값이 링키지 손실이 아니라 **LUT 부족분**을 보정해 왔을
            # 가능성이 있다(07-31 각도기 실측은 링키지 정상 1.003). steering_model:=bicycle
            # 로 전환하면 그 부족분이 사라지므로 이 값도 재검토 대상이다. 단 δ_avail(조향
            # 권한 캡)까지 같이 움직이니 반드시 저속 A/B로 확인하고 단독으로 바꾸지 말 것.
            'steering_reach_ratio', default_value='1.0',
            description='명령 조향각 중 바퀴가 실제 도달하는 비율. 보상(1/ratio)과 조향권한 캡을 '
                        '동시 지배. 1.0 = 보상 없음(2026-07-31 실측: 링키지 정상, 단 08-07에 '
                        '"하중 걸린 주행 마진 아님"으로 상향 기각됨 — 실측만으로 다시 올리지 말 것)'
        ),
        # 50Hz에서 20 rad/s = 사이클당 0.4 rad = 풀락까지 2 사이클 = 구 하드코딩과 동일(무제한).
        # 서보 물리 속도(~7 rad/s 추정)로 낮추면 고주파 채터링을 막지만 실측 전이라 중립 유지.
        DeclareLaunchArgument(
            'max_steering_rate', default_value='20.0',
            description='조향 rate limit [rad/s] (dt 비례). 20.0 = 구 거동(사이클당 0.4rad)'
        ),
        DeclareLaunchArgument(
            'steering_trim_adapt_gain', default_value='0.25',
            description='조향 트림 자동 보상 LPF 게인 1/τ [1/s]. 0 = 비활성(기본). '
                        '0.25 = τ 4초 권장 — L1 대역(0.5~1Hz)보다 한 자릿수 아래라 안 싸운다'
        ),
        DeclareLaunchArgument(
            'steering_trim_limit', default_value='0.06',
            description='추정 트림 절대값 상한 [rad]. 0.06 ≈ 3.4° = 조향 권한의 8%'
        ),
        DeclareLaunchArgument(
            'steering_trim_max_steer', default_value='0.15',
            description='이 조향각[rad]을 넘으면 학습 정지 — 선형 자전거모델 유효 영역 밖'
        ),
        # ── 트림 웜업 단축 (2026-08-19) ──────────────────────────────────────────
        # 트림 게이트 듀티가 25~29%뿐이라 실효 시상수가 τ/듀티 ≈ 14 s → 수렴에 3~4랩이
        # 걸리고, 그 사이 직선 횡오차가 +0.11 m에서 시작한다(0819 실측).
        # 0819 bag 리플레이(±0.3° 정착 / 정상상태 리플):
        #     gain 0.25 단독            20.9 s / 0.056°
        #     + 웜업 상한 2.0            **9.8 s** / 0.055°   ← 리플이 늘지 않는다
        #     gain 0.5 로 올리기         65.6 s / 0.108°      ← 리플이 2배, 정착은 더 나쁨
        #   → **정상 게인을 올리는 것은 답이 아니다.** 웜업 구간만 빠르게 하는 게 맞다.
        DeclareLaunchArgument(
            'steering_trim_init', default_value='0.0',
            description='조향 트림 시작값 [rad] (재체결 리셋값도 이 값). '
                        '지난 주행 상태 로그의 `trim: xx°` 수렴값을 rad로 실으면 웜업이 0이 된다. '
                        '0819 실측 수렴값: 젯슨 offset 0.4672에서 -0.032 rad(-1.8°), '
                        'offset 0.48에서는 -0.014 rad(-0.8°) 부근이 예상값. '
                        '⚠️ 서보암/타이로드/젯슨 offset을 만졌으면 반드시 0으로 되돌리고 다시 배울 것'
        ),
        DeclareLaunchArgument(
            'steering_trim_warmup_gain', default_value='2.0',
            description='트림 웜업 상한 게인 [1/s], 0이면 비활성(구 거동). '
                        'g_eff = clamp(1/게이트누적시간, steering_trim_adapt_gain, 이 값). '
                        '초기엔 표본평균과 등가로 빠르게 붙고 t_g > 1/gain 이후 기존 LPF로 '
                        '정확히 복귀하므로 정상상태 리플이 늘지 않는다'
        ),
        # ③ 자동 저장/복원 — 사람이 값을 옮겨 적을 필요가 없다.
        #    지문(K_us 좌/우·공용, reach, lag, 조향 한계 좌/우)이 다르면 기동 시 폐기하고,
        #    나이가 max_age를 넘어도 폐기한다. 둘 다 통과하면 웜업 없이 시작한다.
        #    ⚠️ 지문으로 **젯슨 vesc.yaml의 steering_angle_to_servo_offset 변경은 못 잡는다**
        #       (다른 패키지라 안 보인다). 그래서 나이 제한 + 웜업 스케줄을 같이 둔다 —
        #       웜업이 켜져 있으면 틀린 값을 실어도 게이트 열린 뒤 ~10초에 실측으로 덮인다.
        #       즉 최악이 "0에서 시작한 것과 같음"이고 그보다 나빠지지 않는다.
        DeclareLaunchArgument(
            'steering_trim_persist_file', default_value='~/.f1tenth/steering_trim.yaml',
            description='조향 트림 자동 저장 경로. 빈 문자열이면 비활성(구 거동). '
                        '5초마다 원자적 교체로 쓰고, 기동 시 지문·나이 검사 후 싣는다'
        ),
        DeclareLaunchArgument(
            'steering_trim_persist_max_age', default_value='43200.0',
            description='트림 저장본 유효 나이 [s], 0이면 무제한. 기본 12시간 = 테스트 하루. '
                        '기계 중립은 정비/주행마다 움직인다(0810 실측 -2.2/-1.9/+1.6°)'
        ),
        DeclareLaunchArgument(
            'steering_trim_min_speed', default_value='2.0',
            description='이 속도[m/s] 미만이면 학습 정지 (저속은 요레이트 역산이 폭발)'
        ),
        DeclareLaunchArgument(
            'steering_trim_max_lat_acc', default_value='2.0',
            description='이 횡가속[m/s²]을 넘으면 학습 정지'
        ),
        DeclareLaunchArgument(
            'steering_trim_lag', default_value='0.14',
            description='조향→요레이트 지연 [s]. 0810 bag 상호상관 실측 140 ms (상관 0.96~0.98)'
        ),
        # 가감속 조향 스케일러가 완전히 적용되는 기준 |종가속| [m/s²]. 예전엔 ±1.0 하드 임계라
        # 넘는 순간 조향이 5% 계단 점프했고, 실측 coast 감속 −0.4에선 감속측이 급제동
        # 스파이크에서만 드물게 튀었다. 0~ref 선형 블렌딩으로 바꿨다(ref 이상은 구 거동).
        DeclareLaunchArgument(
            'steering_scaler_accel_ref', default_value='1.0',
            description='가감속 조향 스케일러 완전 적용 기준 |a_x| [m/s²] (0~이 값 선형 블렌딩)'
        ),

        # ── 종방향 감속: 두 개의 서로 다른 감속도 (튜닝 방향이 정반대라 분리했다) ──
        #   base_max_decel = 명령 속도를 초당 얼마나 빨리 떨어뜨릴 수 있나(램프 rate limit) → 높게
        #   prebrake_decel = 차가 **실제로** 낼 수 있는 감속도(제동거리 v²/2a) → 실측값에 맞춤

        DeclareLaunchArgument(
            'base_max_decel', default_value='8.0',
            description='명령 속도 하강 rate limit [m/s^2]. 낮추면 감속 명령이 늦게 도달하므로 높게 유지'
        ),
        DeclareLaunchArgument(
            'prebrake_decel', default_value='3.5',
            description='곡률 사전감속 제동거리 산출용 감속 권한 [m/s^2]. 낮을수록 코너를 일찍 봄'
        ),
        DeclareLaunchArgument(
            'ramp_lead_max', default_value='3.5',
            description='명령 속도 램프가 실측보다 앞설 수 있는 최대폭 [m/s]. 0이면 비활성(구 거동)'
        ),
        DeclareLaunchArgument(
            'hfi_launch_guard_enable', default_value=hfi_launch_guard_enable_default,
            description='HFI 실차 정지출발 중 저속 포착 구간 속도 상한 사용 여부'
        ),
        DeclareLaunchArgument(
            'hfi_launch_speed_cap', default_value='0.7',
            description='HFI 정지출발 포착 전 발행 속도 상한 [m/s]'
        ),
        DeclareLaunchArgument(
            'hfi_launch_exit_speed', default_value='0.5',
            description='실측 속도가 이 값을 넘으면 HFI 정지출발 상한 해제 [m/s]'
        ),
        DeclareLaunchArgument(
            'hfi_launch_standstill_speed', default_value='0.1',
            description='이 속도 미만의 완전 정지에서 HFI 정지출발 보호 재무장 [m/s]'
        ),
        DeclareLaunchArgument(
            'understeer_gradient', default_value='0.010',
            description='언더스티어 그래디언트 K_us [rad/(m/s^2)]. 0이면 조향 권한 캡 비활성. '
                        '좌/우 분리값(_left/_right)이 양수면 조향·권한캡·트림 추정은 그쪽을 쓰고 '
                        '이 값은 방향 미상(κ=0)일 때만 남는다'
        ),
        # ── 좌/우 분리 K_us (2026-08-18 실측, 08-19 이식) ────────────────────────
        # 되돌리기: understeer_gradient_left:=0.011 understeer_gradient_right:=0.024
        DeclareLaunchArgument(
            'understeer_gradient_left', default_value='0.014',
            description='좌회전 K_us [rad/(m/s^2)]. <=0 = 공용 understeer_gradient 사용'
        ),
        DeclareLaunchArgument(
            'understeer_gradient_right', default_value='0.019',
            description='우회전 K_us [rad/(m/s^2)]. <=0 = 공용 understeer_gradient 사용'
        ),
        # ── 조향 권한 마진 (2026-08-19 신설, CLAUDE.md ②-y) ──────────────────────
        # 조향 명령 클램프 = (섹터 스케일 적용된) max_lateral_accel × 이 값.
        # 🔑 `mla` 하나가 "코너를 얼마로 돌 계획인가"(속도 캡)와 "조향이 요구할 수 있는
        #    최대 a_lat"(클램프) 두 일을 겸하고 있었다. 라인이 전 코너에서 a_lat = mla를
        #    요구하면 둘이 같아져 **횡오차 보정 예산이 구조적으로 0**이 되고, 실제 K_us가
        #    가정보다 25%만 커도 오차를 되돌릴 권한이 없어 발산한다(max 1.29 m).
        # 🔑 이걸 분리하면 "라인은 보수적으로 뽑고 주행 중 섹터 학습으로 코너 속도를
        #    올린다"는 구조가 성립한다 — 속도 예산(= 학습 대상)과 보정 권한이 더 이상
        #    같은 숫자가 아니기 때문이다. 1.0 = 구 거동.
        DeclareLaunchArgument(
            'steering_accel_margin', default_value='1.15',
            description='조향 명령 클램프 = mla × 이 값 [배]. 1.0이면 구 거동(보정 예산 0). '
                        '속도 캡에는 적용되지 않는다 — ②-y'
        ),
        DeclareLaunchArgument(
            'steer_authority_ratio', default_value='0.95',
            description='조향 한계 중 곡률 추종에 배정할 비율. 나머지는 횡오차·요레이트 보정 여유 '
                        '(1.0이면 보정 여력이 0)'
        ),
        # ── U1 그립 권한 속도 클램프 (2026-08-21 재작업) ─────────────────────────
        # 요구 곡률 κ_L1 = 2|sinη|/L1 이 예산(마진 없는 MLA × margin)을 넘으면 목표
        # 속도를 v ≤ √(예산/κ_L1) 로 캡한다. target_speed 단계 적용(종방향 램프 통과),
        # 빠른 제한·느린 해제 필터, 하한 없음(안전 계산이 이김), 경로 전환 시 리셋.
        # 🔴 기본 false — 단독 셰이크다운(저속 2랩 → 정상 3랩, 포화 경고 감소·진동 없음
        #    확인) 후에만 켠다: grip_speed_clamp_enable:=true
        DeclareLaunchArgument(
            'grip_speed_clamp_enable', default_value='true',
            description='U1: L1 요구 횡가속이 예산을 넘으면 목표 속도를 캡 (기본 꺼짐)'
        ),
        DeclareLaunchArgument(
            'grip_speed_clamp_margin', default_value='1.0',
            description='속도 예산 = 마진 없는 MLA × 이 값. 실측 스윕: 0.9=과보수(랩+2.15s), '
                        '1.0=계약 기본(랩+1.93s 상한), 1.15=권한 일치(보정 여유 0, A/B용)'
        ),
        DeclareLaunchArgument(
            'grip_speed_clamp_release_alpha', default_value='0.05',
            description='요구 곡률 해제 필터 계수 (제한 진입은 즉시, 해제만 τ≈0.4 s)'
        ),
        # 전방 곡률 스캔 거리 = max(count*0.1, v²/(2·prebrake_decel)). count는 저속 하한.
        DeclareLaunchArgument(
            'curvature_lookahead_count', default_value='80',
            description='곡률 룩어헤드 스캔 거리 하한 (×0.1m). 80 = 8m'
        ),
        DeclareLaunchArgument(
            'min_speed', default_value='1.2',
            description='최저 순항 속도 [m/s] (곡률 감속 하한). 장애물 정지엔 미적용(0까지 허용)'
        ),

        DeclareLaunchArgument(
            'l1_use_actual_distance', default_value='true',
            description='L1 횡가속 분모로 목표점까지의 실제 직선거리 사용. false면 구 거동(명목 L1 거리)'
        ),

        DeclareLaunchArgument(
            'engage_gate_enable', default_value='true',
            description='자율 미체결(/drive_mode != autonomous) 중 속도 램프를 실측에 고정'
        ),
        DeclareLaunchArgument(
            'drive_mode_topic', default_value='/drive_mode',
            description='자율 체결 상태 토픽 (f1tenth_stack drive_mode_manager 발행)'
        ),
        DeclareLaunchArgument(
            'engaged_mode_value', default_value='autonomous',
            description='체결로 판정할 /drive_mode 문자열'
        ),
        DeclareLaunchArgument(
            'drive_mode_timeout', default_value='1.0',
            description='이 시간[s] 넘게 /drive_mode 미수신이면 게이트 자동 비활성(시뮬 호환)'
        ),

        DeclareLaunchArgument(
            'launch_boost_enable', default_value='false',
            description='런치 킥 on/off (자율 정지출발 데드존 관통 펀치)'
        ),
        DeclareLaunchArgument(
            'launch_boost_speed', default_value='2.0',
            description='데드존 관통용 펀치 속도 명령 [m/s]'
        ),
        DeclareLaunchArgument(
            'launch_boost_time', default_value='1.5',
            description='관통 실패 시 포기까지 최대 펀치 시간 [s]'
        ),
        DeclareLaunchArgument(
            'launch_exit_speed', default_value='0.9',
            description='실측이 이 속도[m/s] 넘으면 관통 성공 판정 → 킥 종료(데드존 상단 0.59보다 위)'
        ),
        DeclareLaunchArgument(
            'launch_standstill_speed', default_value='0.3',
            description='실측이 이 속도[m/s] 미만이면 정지 판정 → 킥 시작(exit보다 낮아 히스테리시스)'
        ),
        # 🟢 2026-08-20 신설. 킥이 launch_boost_time 안에 관통 못 하고 포기하면 예전엔
        #    **차가 실제로 launch_exit_speed를 넘을 때까지** 영구히 재시도하지 않았다 —
        #    못 나가고 있을 때 킥이 사라진다는 뜻이다. 회피 세이프스톱 재출발처럼 플래너
        #    목표가 킥 바닥(2.0)보다 낮은 상황에서 정확히 이게 손해다(0819 run_214041 실측:
        #    킥 만료 후 명령이 1.49로 떨어진 채 인계 시도 → 붕괴 → 총 1.82 s).
        #    이 값[s]만큼 **정지가 계속되고 여전히 갈 의도가 있으면** 다시 무장한다.
        # 🔑 성공하는 출발에는 비용이 정확히 0이다(래치가 안 서면 타이머가 돌지 않는다).
        # 🔴 기본 0 = 구 거동(영구 래치). 실차 A/B 전까지 켜지 않는다 —
        #    "인계 명령이 클수록 옵저버가 더 잘 깨진다"는 반대 방향 실측(0810)이 있어
        #    재시도가 이득인지 아직 데이터로 못 갈랐다. 켜기: launch_relatch_time:=2.0
        DeclareLaunchArgument(
            'launch_relatch_time', default_value='0.0',
            description='런치 킥 포기 후 재무장까지 필요한 정지 지속 시간 [s]. '
                        '0 = 재시도 안 함(구 거동). 회피 재출발에서 킥이 사라지는 것을 막는다'
        ),

        # IMU 보정 on/off. 끄면 조향 가감속 스케일러가 중립(acc_mean=0)으로 떨어져
        # 순수 L1(시뮬 검증 상태)이 된다.
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='IMU 종가속(조향 가감속 스케일러) 사용 여부. false면 스케일러 중립'
        ),

    ]

def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                           imu_linear_scale, imu_angular_scale,
                           max_steering_left, max_steering_right,
                           remappings: Optional[list] = None):
    """control_map_node — 환경별로 다른 값만 인자로 받고 나머지는 공용 정의.

    remappings: 실차에서만 필요한 토픽 리매핑(예: vesc_driver의 sensors/imu/raw →
    코드에 하드코딩된 /imu/data). 시뮬은 sim_imu_bridge_node가 /imu/data로 바로
    발행하므로 불필요.
    """
    return Node(
        package='f1tenth_control',
        executable='control_map_node',
        name='control_map_node',
        output='screen',
        remappings=remappings,
        parameters=[{
            'odom_topic': odom_topic,
            'wheelbase': 0.33,
            'l1_offset': LaunchConfiguration('l1_offset'),
            'l1_speed_gain': LaunchConfiguration('l1_speed_gain'),
            't_clip_min': LaunchConfiguration('t_clip_min'),
            't_clip_max': LaunchConfiguration('t_clip_max'),
            'l1_min_denom': LaunchConfiguration('l1_min_denom'),
            'steering_speed_cap_measured': LaunchConfiguration('steering_speed_cap_measured'),
            'status_log_period_ms': ParameterValue(
                LaunchConfiguration('status_log_period_ms'), value_type=int),
            'l1_jump_warn_m': LaunchConfiguration('l1_jump_warn_m'),
            'max_speed': max_speed,
            'min_speed': LaunchConfiguration('min_speed'),
            'max_lateral_accel': max_lateral_accel,
            'steering_accel_margin': LaunchConfiguration('steering_accel_margin'),
            'understeer_gradient': LaunchConfiguration('understeer_gradient'),
            'understeer_gradient_left': LaunchConfiguration('understeer_gradient_left'),
            'understeer_gradient_right': LaunchConfiguration('understeer_gradient_right'),
            'steer_authority_ratio': LaunchConfiguration('steer_authority_ratio'),
            'grip_speed_clamp_enable': ParameterValue(
                LaunchConfiguration('grip_speed_clamp_enable'), value_type=bool),
            'grip_speed_clamp_margin': LaunchConfiguration('grip_speed_clamp_margin'),
            'grip_speed_clamp_release_alpha':
                LaunchConfiguration('grip_speed_clamp_release_alpha'),
            'curvature_lookahead_count': ParameterValue(
                LaunchConfiguration('curvature_lookahead_count'), value_type=int),
            'base_max_accel': base_max_accel,
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'prebrake_decel': LaunchConfiguration('prebrake_decel'),
            'ramp_lead_max': LaunchConfiguration('ramp_lead_max'),
            'hfi_launch_guard_enable': ParameterValue(
                LaunchConfiguration('hfi_launch_guard_enable'), value_type=bool),
            'hfi_launch_speed_cap': LaunchConfiguration('hfi_launch_speed_cap'),
            'hfi_launch_exit_speed': LaunchConfiguration('hfi_launch_exit_speed'),
            'hfi_launch_standstill_speed': LaunchConfiguration('hfi_launch_standstill_speed'),
            'launch_boost_enable': ParameterValue(LaunchConfiguration('launch_boost_enable'), value_type=bool),
            'launch_boost_speed': LaunchConfiguration('launch_boost_speed'),
            'launch_boost_time': LaunchConfiguration('launch_boost_time'),
            'launch_exit_speed': LaunchConfiguration('launch_exit_speed'),
            'launch_standstill_speed': LaunchConfiguration('launch_standstill_speed'),
            'launch_relatch_time': LaunchConfiguration('launch_relatch_time'),
            'l1_use_actual_distance': ParameterValue(
                LaunchConfiguration('l1_use_actual_distance'), value_type=bool),
            # ⚠️ 좌우 조향 한계는 진입점 런치가 환경별로 넘긴다. 실차는 젯슨 vesc.yaml의
            #    servo_min/max와 **반드시 한 쌍** — 컨트롤러만 올리면 vesc_driver가 조용히
            #    자르고 컨트롤러는 꺾었다고 착각한다. 시뮬은 차량 모델이 대칭이라 ±0.41.
            'max_steering_left': max_steering_left,
            'max_steering_right': max_steering_right,
            'steering_reach_ratio': LaunchConfiguration('steering_reach_ratio'),
            'avoidance_l1_scale_max': ParameterValue(
                LaunchConfiguration('avoidance_l1_scale_max'), value_type=float),
            'avoidance_l1_damping_enable': ParameterValue(
                LaunchConfiguration('avoidance_l1_damping_enable'), value_type=bool),
            'steering_fb_gain': LaunchConfiguration('steering_fb_gain'),
            'curvature_ff_preview': LaunchConfiguration('curvature_ff_preview'),
            'understeer_gradient_adapt_gain':
                LaunchConfiguration('understeer_gradient_adapt_gain'),
            'understeer_adapt_min_lat_acc':
                LaunchConfiguration('understeer_adapt_min_lat_acc'),
            'steering_speed_floor':
                LaunchConfiguration('steering_speed_floor'),
            'understeer_curve_enable': ParameterValue(
                LaunchConfiguration('understeer_curve_enable'), value_type=bool),
            'understeer_curve_min_samples': ParameterValue(
                LaunchConfiguration('understeer_curve_min_samples'), value_type=int),
            'max_steering_rate': LaunchConfiguration('max_steering_rate'),
            'steering_trim_adapt_gain': LaunchConfiguration('steering_trim_adapt_gain'),
            'steering_trim_limit': LaunchConfiguration('steering_trim_limit'),
            'steering_trim_max_steer': LaunchConfiguration('steering_trim_max_steer'),
            'steering_trim_init': LaunchConfiguration('steering_trim_init'),
            'steering_trim_persist_file':
                LaunchConfiguration('steering_trim_persist_file'),
            'steering_trim_persist_max_age':
                LaunchConfiguration('steering_trim_persist_max_age'),
            'steering_trim_warmup_gain': LaunchConfiguration('steering_trim_warmup_gain'),
            'steering_trim_min_speed': LaunchConfiguration('steering_trim_min_speed'),
            'steering_trim_max_lat_acc': LaunchConfiguration('steering_trim_max_lat_acc'),
            'steering_trim_lag': LaunchConfiguration('steering_trim_lag'),
            'steering_scaler_accel_ref': LaunchConfiguration('steering_scaler_accel_ref'),
            'engage_gate_enable': ParameterValue(
                LaunchConfiguration('engage_gate_enable'), value_type=bool),
            'drive_mode_topic': LaunchConfiguration('drive_mode_topic'),
            'engaged_mode_value': LaunchConfiguration('engaged_mode_value'),
            'drive_mode_timeout': LaunchConfiguration('drive_mode_timeout'),
            'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
            'imu_linear_scale': imu_linear_scale,
            'imu_angular_scale': imu_angular_scale,
            # ⚠️ 2026-08-05 "sync: 08/05" 커밋(LUT calibrator 툴 변경이 메인이었던 커밋)에
            # 묻혀 0.0→0.2로 미문서화 변경됐던 것을 [오늘 날짜]에 원복. 코드 기본값
            # (control_map_node.cpp declare_parameter)도 0.0이고 CLAUDE.md도 "기본 비활성"
            # 이라 기록해온 값과 여기 하드코딩이 어긋나 있었다. 0816 크래시 재구성에서, 슬립이
            # 시작된 순간 이 항이 헤딩오차에 비례해 최대 −0.220 rad을 더해 조향을 풀락에
            # 박고 유지시키는 증폭 경로로 확인됨(단 08-14 7.7 m/s 무사고 run에도 0.2였으므로
            # 이것만으로 크래시 원인 전체를 설명하진 않음 — 슬립 자체의 원인은 별도).
            'heading_damping_gain': 0.0,
            'acceleration_scaler_for_steering': LaunchConfiguration('acceleration_scaler_for_steering'),
            'deceleration_scaler_for_steering': LaunchConfiguration('deceleration_scaler_for_steering'),
            'start_scale_speed': LaunchConfiguration('start_scale_speed'),
            'end_scale_speed': LaunchConfiguration('end_scale_speed'),
            'downscale_factor': LaunchConfiguration('downscale_factor'),
            'speed_lookahead': LaunchConfiguration('speed_lookahead'),
            'speed_lookahead_for_steering': LaunchConfiguration('speed_lookahead_for_steering'),
            'odom_timeout': LaunchConfiguration('odom_timeout'),
            'local_fresh_timeout': LaunchConfiguration('local_fresh_timeout'),
            'closest_idx_max_heading_err': LaunchConfiguration('closest_idx_max_heading_err'),
            'cruise_limit_enable': ParameterValue(
                LaunchConfiguration('cruise_enable'), value_type=bool),
            'cruise_speed_limit_topic': '/cruise_speed_limit',
            'cruise_speed_limit_timeout': ParameterValue(
                LaunchConfiguration('cruise_speed_limit_timeout'), value_type=float),
            'cruise_stale_speed': ParameterValue(
                LaunchConfiguration('cruise_stale_speed'), value_type=float),
            # 섹터별 횡가속 권한 스케일 (기본 꺼짐 — 켜기 전 bag_analyzer 판정 필수)
            'sector_scale_enable': LaunchConfiguration('sector_scale_enable'),
            'sector_scale_topic': LaunchConfiguration('sector_scale_topic'),
            'sector_scale_max': LaunchConfiguration('sector_scale_max'),
            'sector_scale_blend': LaunchConfiguration('sector_scale_blend'),
            'sector_scale_track_len_tol': LaunchConfiguration('sector_scale_track_len_tol'),
            'sector_scale_global_only': LaunchConfiguration('sector_scale_global_only'),
            'sector_scale_state_topic': LaunchConfiguration('sector_scale_state_topic'),
            'sector_scale_state_timeout': LaunchConfiguration('sector_scale_state_timeout'),
            'sector_scale_timeout': LaunchConfiguration('sector_scale_timeout'),
        }]
    )

def build_cruise_controller_node(*, max_speed):
    """전방 상대차 간격을 속도 상한으로 변환하는 종방향 보조 노드."""
    config_file = PathJoinSubstitution([
        FindPackageShare('f1tenth_control'), 'config', 'cruise_controller.yaml'
    ])
    return Node(
        package='f1tenth_control',
        executable='cruise_controller_node',
        name='cruise_controller_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('cruise_enable')),
        parameters=[config_file, {
            'maximum_speed': ParameterValue(max_speed, value_type=float),
            'trailing_mode_distance': ParameterValue(
                LaunchConfiguration('trailing_mode_distance'), value_type=bool),
            'trailing_gap': ParameterValue(
                LaunchConfiguration('trailing_gap'), value_type=float),
            'minimum_gap': ParameterValue(
                LaunchConfiguration('minimum_gap'), value_type=float),
            'max_desired_gap': ParameterValue(
                LaunchConfiguration('max_desired_gap'), value_type=float),
            'trailing_p_gain': ParameterValue(
                LaunchConfiguration('trailing_p_gain'), value_type=float),
            'trailing_i_gain': ParameterValue(
                LaunchConfiguration('trailing_i_gain'), value_type=float),
            'trailing_d_gain': ParameterValue(
                LaunchConfiguration('trailing_d_gain'), value_type=float),
            'integral_limit': ParameterValue(
                LaunchConfiguration('integral_limit'), value_type=float),
            'allow_accel_trailing': ParameterValue(
                LaunchConfiguration('allow_accel_trailing'), value_type=bool),
            'emergency_stop_distance': ParameterValue(
                LaunchConfiguration('emergency_stop_distance'), value_type=float),
            'relative_deceleration': ParameterValue(
                LaunchConfiguration('relative_deceleration'), value_type=float),
            'ego_deceleration': ParameterValue(
                LaunchConfiguration('ego_deceleration'), value_type=float),
            'opponent_deceleration': ParameterValue(
                LaunchConfiguration('opponent_deceleration'), value_type=float),
            'actuation_latency': ParameterValue(
                LaunchConfiguration('actuation_latency'), value_type=float),
            'ego_front_offset': ParameterValue(
                LaunchConfiguration('ego_front_offset'), value_type=float),
            'uncertainty_sigma': ParameterValue(
                LaunchConfiguration('uncertainty_sigma'), value_type=float),
            'gap_uncertainty_horizon_max': ParameterValue(
                LaunchConfiguration('gap_uncertainty_horizon_max'), value_type=float),
            'opp_speed_confidence_z': ParameterValue(
                LaunchConfiguration('opp_speed_confidence_z'), value_type=float),
            'opponent_timeout': ParameterValue(
                LaunchConfiguration('opponent_timeout'), value_type=float),
            'ego_timeout': ParameterValue(
                LaunchConfiguration('ego_timeout'), value_type=float),
            'state_timeout': ParameterValue(
                LaunchConfiguration('state_timeout'), value_type=float),
            'clear_confirm_sec': ParameterValue(
                LaunchConfiguration('clear_confirm_sec'), value_type=float),
            'blind_trailing_speed': ParameterValue(
                LaunchConfiguration('blind_trailing_speed'), value_type=float),
        }],
    )

def build_sector_learner_node():
    """섹터 scale 발행기 겸 온라인 학습기 (AIMD).

    /sector_scales의 **유일한** 발행자다 (2026-08-12에 sector_pub.py를 흡수해 은퇴시켰다).

    🔑 컨트롤러는 손대지 않는다 — 발행 토픽·레이아웃이 sector_pub과 동일하므로
       컨트롤러의 검증(scale ≥ 1.0 / track_length 대조 / /state 게이팅 / 데드맨)이
       출력에 그대로 걸린다. 기본 모드 static은 표를 안 바꾸므로 구 sector_pub과 동일하다.

    ⚠️ `--mode explore`는 **연습 전용**이다. 차가 랩마다 코너 속도를 스스로 올린다.
    """
    return Node(
        package='f1tenth_control',
        executable='sector_learner.py',
        name='sector_learner',
        output='screen',
        arguments=[
            LaunchConfiguration('sector_scale_file'),
            '--mode', LaunchConfiguration('sector_learn_mode'),
            '--watch', LaunchConfiguration('sector_learn_watch'),
            '--out', LaunchConfiguration('sector_learn_out'),
            '--auto-latest', LaunchConfiguration('sector_learn_auto_latest'),
            '--topic', LaunchConfiguration('sector_scale_topic'),
            '--odom', LaunchConfiguration('odom_topic'),
        ],
        condition=IfCondition(LaunchConfiguration('sector_scale_enable')),
    )
