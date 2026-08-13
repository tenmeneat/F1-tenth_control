from typing import Any, Optional

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

IMU_LINEAR_SCALE_REAL = 9.80665      # g → m/s². VESC가 g로 발행(2026-07-19 소스 확인)
IMU_LINEAR_SCALE_SIM  = 1.0          # sim_imu_bridge_node는 0 고정
IMU_ANGULAR_SCALE_REAL = 0.0174533   # deg/s → rad/s (pi/180). VESC가 deg/s로 발행
IMU_ANGULAR_SCALE_SIM  = 1.0         # sim_imu_bridge_node는 이미 rad/s로 중계

# ⚠️ 조이스틱 드라이버·sim_imu_bridge_node 포함 여부 등 안전 관련 구조 차이는
# 일부러 여기로 옮기지 않고 각 진입점 파일에 그대로 둔다(환경을 잘못 골라 안전
# 기능이 빠진 채 기동되는 실수를 구조적으로 차단하기 위함).

def declare_common_args(sector_scale_enable_default='false'):
    """두 런치파일에서 동일하게 쓰는 인자 선언 목록."""
    return [

        # ── 조향 스케일러 (가감속/속도 구간별 조향 게인 완화) ──
        DeclareLaunchArgument(
            'acceleration_scaler_for_steering', default_value='1.0',
            description='가속 중(acc_mean>=1.0) 조향각에 곱하는 스케일러'
        ),
        DeclareLaunchArgument(
            'deceleration_scaler_for_steering', default_value='0.95',
            description='감속 중(acc_mean<=-1.0) 조향각에 곱하는 스케일러'
        ),
        DeclareLaunchArgument(
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
            'local_fresh_timeout', default_value='0.3',
            description='이 시간(s) 넘게 /local_waypoints 미수신 시 글로벌 경로로 폴백'
        ),
        DeclareLaunchArgument(
            'closest_idx_max_heading_err', default_value='1.40',
            description='경로 접선과 차량 헤딩의 허용 오차 [rad]. 0이면 게이트 비활성(구 거동)'
        ),
        DeclareLaunchArgument(
            'l1_offset', default_value='0.4',
            description='L1 룩어헤드 거리의 **절편** [m] (공식: l1_offset + v*l1_speed_gain). '
                        '구 이름 l1_gain'
        ),
        DeclareLaunchArgument(
            'l1_speed_gain', default_value='0.3',
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
            description='조향용 속도(횡가속 게인+LUT 조회)를 실측 속도로 상한. '
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
            description='섹터 스케일 sectors.yaml 파일 경로 (비워두면 자동 탐색)'
        ),
        DeclareLaunchArgument(
            'sector_scale_topic', default_value='/sector_scales',
            description='섹터 테이블 토픽 (std_msgs/Float32MultiArray, '
                        '[track_length, (s0,s1,scale)×N], transient_local)'
        ),
        DeclareLaunchArgument(
            'sector_scale_max', default_value='1.5',
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
        DeclareLaunchArgument(
            'sector_learn_watch', default_value='true',
            description='sectors.yaml 저장을 감시해 재로드(라이브 튜닝). '
                        '⚠️ 재로드는 학습 상태를 초기화한다'
        ),
        DeclareLaunchArgument(
            'sector_learn_out', default_value='',
            description='학습 결과 저장 경로. 비우면 저장 안 함. '
                        '연습에서 뽑아 결선 sectors.yaml로 쓰는 것이 의도된 흐름'
        ),

        DeclareLaunchArgument(
            'steering_reach_ratio', default_value='0.85',
            description='명령 조향각 중 바퀴가 실제 도달하는 비율. 보상(1/ratio)과 조향권한 캡을 '
                        '동시 지배. 1.0 = 보상 없음(2026-07-31 실측: 링키지 정상)'
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
            'prebrake_decel', default_value='2.6',
            description='곡률 사전감속 제동거리 산출용 감속 권한 [m/s^2]. 낮을수록 코너를 일찍 봄'
        ),
        DeclareLaunchArgument(
            'ramp_lead_max', default_value='2.4',
            description='명령 속도 램프가 실측보다 앞설 수 있는 최대폭 [m/s]. 0이면 비활성(구 거동)'
        ),
        DeclareLaunchArgument(
            'understeer_gradient', default_value='0.014',
            description='언더스티어 그래디언트 K_us [rad/(m/s^2)]. 0이면 조향 권한 캡 비활성'
        ),
        DeclareLaunchArgument(
            'steer_authority_ratio', default_value='0.95',
            description='조향 한계 중 곡률 추종에 배정할 비율. 나머지는 횡오차·요레이트 보정 여유 '
                        '(1.0이면 보정 여력이 0)'
        ),
        # 전방 곡률 스캔 거리 = max(count*0.1, v²/(2·prebrake_decel)). count는 저속 하한.
        DeclareLaunchArgument(
            'curvature_lookahead_count', default_value='80',
            description='곡률 룩어헤드 스캔 거리 하한 (×0.1m). 80 = 8m'
        ),
        DeclareLaunchArgument(
            'min_speed', default_value='2.0',
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
            'launch_boost_enable', default_value='true',
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
            'launch_exit_speed', default_value='1.2',
            description='실측이 이 속도[m/s] 넘으면 관통 성공 판정 → 킥 종료(데드존 상단 0.59보다 위)'
        ),
        DeclareLaunchArgument(
            'launch_standstill_speed', default_value='0.3',
            description='실측이 이 속도[m/s] 미만이면 정지 판정 → 킥 시작(exit보다 낮아 히스테리시스)'
        ),

        # IMU 보정 on/off. 끄면 조향 가감속 스케일러가 중립(acc_mean=0)으로 떨어져
        # 순수 L1+LUT(시뮬 검증 상태)가 된다.
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='IMU 종가속(조향 가감속 스케일러) 사용 여부. false면 순수 L1+LUT 주행'
        ),

    ]

def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                           imu_linear_scale, imu_angular_scale,
                           max_steering_left, max_steering_right,
                           lookup_table_file: Any = '', remappings: Optional[list] = None):
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
            'understeer_gradient': LaunchConfiguration('understeer_gradient'),
            'steer_authority_ratio': LaunchConfiguration('steer_authority_ratio'),
            'curvature_lookahead_count': ParameterValue(
                LaunchConfiguration('curvature_lookahead_count'), value_type=int),
            'base_max_accel': base_max_accel,
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'prebrake_decel': LaunchConfiguration('prebrake_decel'),
            'ramp_lead_max': LaunchConfiguration('ramp_lead_max'),
            'launch_boost_enable': ParameterValue(LaunchConfiguration('launch_boost_enable'), value_type=bool),
            'launch_boost_speed': LaunchConfiguration('launch_boost_speed'),
            'launch_boost_time': LaunchConfiguration('launch_boost_time'),
            'launch_exit_speed': LaunchConfiguration('launch_exit_speed'),
            'launch_standstill_speed': LaunchConfiguration('launch_standstill_speed'),
            'l1_use_actual_distance': ParameterValue(
                LaunchConfiguration('l1_use_actual_distance'), value_type=bool),
            # ⚠️ 좌우 조향 한계는 진입점 런치가 환경별로 넘긴다. 실차는 젯슨 vesc.yaml의
            #    servo_min/max와 **반드시 한 쌍** — 컨트롤러만 올리면 vesc_driver가 조용히
            #    자르고 컨트롤러는 꺾었다고 착각한다. 시뮬은 차량 모델이 대칭이라 ±0.41.
            'max_steering_left': max_steering_left,
            'max_steering_right': max_steering_right,
            'steering_reach_ratio': LaunchConfiguration('steering_reach_ratio'),
            'max_steering_rate': LaunchConfiguration('max_steering_rate'),
            'steering_trim_adapt_gain': LaunchConfiguration('steering_trim_adapt_gain'),
            'steering_trim_limit': LaunchConfiguration('steering_trim_limit'),
            'steering_trim_max_steer': LaunchConfiguration('steering_trim_max_steer'),
            'steering_trim_min_speed': LaunchConfiguration('steering_trim_min_speed'),
            'steering_trim_max_lat_acc': LaunchConfiguration('steering_trim_max_lat_acc'),
            'steering_trim_lag': LaunchConfiguration('steering_trim_lag'),
            'steering_scaler_accel_ref': LaunchConfiguration('steering_scaler_accel_ref'),
            'engage_gate_enable': ParameterValue(
                LaunchConfiguration('engage_gate_enable'), value_type=bool),
            'drive_mode_topic': LaunchConfiguration('drive_mode_topic'),
            'engaged_mode_value': LaunchConfiguration('engaged_mode_value'),
            'drive_mode_timeout': LaunchConfiguration('drive_mode_timeout'),
            'lookup_table_file': lookup_table_file,
            'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
            'imu_linear_scale': imu_linear_scale,
            'imu_angular_scale': imu_angular_scale,
            'curvature_ff_blend': 0.0,
            'heading_damping_gain': 0.2,
            'acceleration_scaler_for_steering': LaunchConfiguration('acceleration_scaler_for_steering'),
            'deceleration_scaler_for_steering': LaunchConfiguration('deceleration_scaler_for_steering'),
            'start_scale_speed': LaunchConfiguration('start_scale_speed'),
            'end_scale_speed': LaunchConfiguration('end_scale_speed'),
            'downscale_factor': LaunchConfiguration('downscale_factor'),
            'speed_lookahead': LaunchConfiguration('speed_lookahead'),
            'speed_lookahead_for_steering': LaunchConfiguration('speed_lookahead_for_steering'),
            'local_fresh_timeout': LaunchConfiguration('local_fresh_timeout'),
            'closest_idx_max_heading_err': LaunchConfiguration('closest_idx_max_heading_err'),
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
            '--topic', LaunchConfiguration('sector_scale_topic'),
            '--odom', LaunchConfiguration('odom_topic'),
        ],
        condition=IfCondition(LaunchConfiguration('sector_scale_enable')),
    )
