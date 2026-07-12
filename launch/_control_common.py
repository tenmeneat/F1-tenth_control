from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# ============================================================================
# 시뮬/실차 런치파일 공용 헬퍼 (control_sim.launch.py / control_real.launch.py)
# ============================================================================
# 런치파일이 아니라 순수 헬퍼 모듈 — ros2 launch 진입점으로 직접 실행되지 않음.
# 두 환경에서 100% 동일했던 파라미터/노드 정의를 여기 한 곳에만 두어, 파라미터
# 추가/변경 시 두 파일에 수동으로 미러링해야 하는 드리프트 위험을 없앤다.
# ⚠️ 조이스틱 드라이버·sim_imu_bridge_node 포함 여부 등 안전 관련 구조 차이는
# 일부러 여기로 옮기지 않고 각 진입점 파일에 그대로 둔다(환경을 잘못 골라 안전
# 기능이 빠진 채 기동되는 실수를 구조적으로 차단하기 위함).


def declare_common_args():
    """두 런치파일에서 동일하게 쓰는 인자 선언 목록."""
    return [
        DeclareLaunchArgument(
            'force_autonomous',
            default_value='false',
            description='true 시 조이스틱 없이 자율주행 모드 즉시 기동'
        ),
        # 요레이트 피드백 카운터스티어 게인. 2026-07-11 시뮬 스윕(fuck_f1, 0.0/0.08/0.15) 결과
        # 랩타임/속도는 게인 무관, 0.15부터 조향 채터링이 뚜렷(부호전환 0→3.32/s) → 보수값 0.08.
        DeclareLaunchArgument(
            'yaw_rate_gain',
            default_value='0.08',
            description='요레이트 카운터스티어 게인 (낮게 시작해 채터링 보며 상향)'
        ),

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

        # ── 롤 인지형 ESC ──
        DeclareLaunchArgument(
            'max_roll_limit', default_value='0.15',
            description='롤 각도 전복 위험 임계치 [rad] (약 8.6도)'
        ),
        DeclareLaunchArgument(
            'decel_attenuation', default_value='0.6',
            description='롤 비율에 따른 가감속 한계 축소 비율'
        ),

        # ── 경로소스 신선도 / 장애물 회피 폴백(GapFollower) ──
        DeclareLaunchArgument(
            'local_fresh_timeout', default_value='0.3',
            description='이 시간(s) 넘게 /local_waypoints 미수신 시 글로벌 경로로 폴백'
        ),
        DeclareLaunchArgument(
            'obstacle_avoid_enable', default_value='false',
            description='글로벌 추종 중 장애물 차단 감지 시 GapFollower 회피 폴백 활성화. '
                        '기본 false — overtake 방해 방지(앞차를 장애물로 오인해 회피 전환하는 것 차단). '
                        'obstacle_avoid_enable:=true로 되살릴 수 있음.'
        ),
        DeclareLaunchArgument(
            'obstacle_cone_halfangle', default_value='0.14',
            description='장애물 차단 판정용 L1 방향 콘 반각 [rad] (~8도)'
        ),
        DeclareLaunchArgument(
            'obstacle_trigger_dist', default_value='1.5',
            description='이 거리[m] 이내 근접 장애물 감지 시 회피 폴백 트리거'
        ),
        DeclareLaunchArgument(
            'obstacle_margin', default_value='0.3',
            description='장애물 차단 판정 시 목표점 거리 대비 최소 여유 [m]'
        ),
        DeclareLaunchArgument(
            'obstacle_avoid_hold_cycles', default_value='15',
            description='회피 폴백 유지 사이클 수(50Hz 기준, 채터링 방지)'
        ),

        # ── L1 Guidance 룩어헤드 거리 (2026-07-13, initial_minimum/new_map_con 비교 기반 승격) ──
        # new_map_con(작년 완성형, fuck_f1.csv 0.18m 촘촘 웨이포인트에서 7.1s/랩) 대비 우리
        # control_map_node가 동일 웨이포인트에서 16.6s/랩 + 매랩 스핀. 공식 자체는 거의 동일
        # (L1 = clamp(l1_gain + v*l1_distance, max(t_clip_min, sqrt2*lat_err), t_clip_max)
        #  vs new_map_con의 clamp(lookahead_gain + v*lookahead_speed_gain, max(min_lookahead_distance,
        #  sqrt2*lat_err), max_lookahead_distance) — l1_gain/l1_distance 기본값 0.5/0.3이
        #  new_map_con의 lookahead_gain/lookahead_speed_gain과 완전히 동일), 유일한 실질 차이가
        #  하한 바닥값: 우리 t_clip_min=0.8 vs new_map_con min_lookahead_distance=1.5. 하한이
        #  낮으면 저속/시케인 구간에서 L1 목표점이 촘촘한 웨이포인트의 국소 지그재그를 그대로
        #  쫓아가며 조향이 고주파로 흔들릴 여지가 커짐 — 스핀의 유력 원인으로 추정, 시뮬 검증 중.
        DeclareLaunchArgument(
            'l1_gain', default_value='0.5',
            description='L1 룩어헤드 거리 베이스 오프셋 [m] (공식: l1_gain + v*l1_distance)'
        ),
        DeclareLaunchArgument(
            'l1_distance', default_value='0.3',
            description='L1 룩어헤드 거리 속도 게인 [s] (공식: l1_gain + v*l1_distance)'
        ),
        DeclareLaunchArgument(
            't_clip_min', default_value='0.8',
            description='L1 룩어헤드 거리 하한 [m] (new_map_con 대응값 min_lookahead_distance=1.5 — '
                        '촘촘/시케인 웨이포인트에서 낮을수록 국소 지그재그를 쫓아 고주파 조향 유발 가능)'
        ),
        DeclareLaunchArgument(
            't_clip_max', default_value='5.0',
            description='L1 룩어헤드 거리 상한 [m]'
        ),

        # ── 종방향 감속 한계 (곡률 사전감속 제동거리 계산에도 직접 쓰임) ──
        # sim/real 둘 다 동일값이라 base_max_accel과 달리 여기서 공용 선언. 8.0은 실측 검증
        # 전 추정값 — max_lateral_accel 마찰피크(~6.7) 대비 낙관적일 수 있어 실차에서 급제동
        # IMU 실측(acc_mean) 후 재조정 권장(2026-07-12 논의).
        DeclareLaunchArgument(
            'base_max_decel', default_value='8.0',
            description='종방향 최대 감속도 한계 [m/s^2] (곡률 사전감속 제동거리 계산에 사용, 실측 전 추정값)'
        ),

        # ── MPPI 컨트롤러 튜너블 (control_mppi_node 전용) ──
        # 나머지 MPPI 파라미터(N/K/차량/타이어/비용가중)는 노드 코드 기본값 사용.
        DeclareLaunchArgument(
            'mppi_lambda', default_value='1.0',
            description='MPPI 역온도 λ (작을수록 저비용 롤아웃에 집중)'
        ),
        DeclareLaunchArgument(
            'mppi_sigma_steer', default_value='0.15',
            description='MPPI 조향 탐색 노이즈 σ [rad]'
        ),
        DeclareLaunchArgument(
            'mppi_sigma_accel', default_value='1.5',
            description='MPPI 종가속 탐색 노이즈 σ [m/s^2]'
        ),
    ]


def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                            lookup_table_file='', remappings=None):
    """control_map_node — 환경별로 다른 값만 인자로 받고 나머지는 공용 정의.
    remappings: 실차에서만 필요한 토픽 리매핑(예: vesc_driver의 sensors/imu/raw →
    코드에 하드코딩된 /imu/data). 시뮬은 sim_imu_bridge_node가 /imu/data로 바로 발행하므로 불필요."""
    return Node(
        package='f1tenth_control',
        executable='control_map_node',
        name='control_map_node',
        output='screen',
        remappings=remappings,
        parameters=[{
            'odom_topic': odom_topic,
            'wheelbase': 0.33,
            'l1_gain': LaunchConfiguration('l1_gain'),
            'l1_distance': LaunchConfiguration('l1_distance'),
            't_clip_min': LaunchConfiguration('t_clip_min'),
            't_clip_max': LaunchConfiguration('t_clip_max'),
            'lateral_error_coeff': 1.0,
            'max_speed': max_speed,
            'min_speed': 2.0,
            'max_lateral_accel': max_lateral_accel,
            'curvature_lookahead_count': 20,
            'base_max_accel': base_max_accel,
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'wall_safety_margin': 0.6,
            'lookup_table_file': lookup_table_file,
            'use_imu': True,
            'yaw_rate_gain': LaunchConfiguration('yaw_rate_gain'),
            'curvature_ff_blend': 0.0,
            'heading_damping_gain': 0.0,
            'acceleration_scaler_for_steering': LaunchConfiguration('acceleration_scaler_for_steering'),
            'deceleration_scaler_for_steering': LaunchConfiguration('deceleration_scaler_for_steering'),
            'start_scale_speed': LaunchConfiguration('start_scale_speed'),
            'end_scale_speed': LaunchConfiguration('end_scale_speed'),
            'downscale_factor': LaunchConfiguration('downscale_factor'),
            'speed_lookahead': LaunchConfiguration('speed_lookahead'),
            'speed_lookahead_for_steering': LaunchConfiguration('speed_lookahead_for_steering'),
            'max_roll_limit': LaunchConfiguration('max_roll_limit'),
            'decel_attenuation': LaunchConfiguration('decel_attenuation'),
            'local_fresh_timeout': LaunchConfiguration('local_fresh_timeout'),
            'obstacle_avoid_enable': LaunchConfiguration('obstacle_avoid_enable'),
            'obstacle_cone_halfangle': LaunchConfiguration('obstacle_cone_halfangle'),
            'obstacle_trigger_dist': LaunchConfiguration('obstacle_trigger_dist'),
            'obstacle_margin': LaunchConfiguration('obstacle_margin'),
            'obstacle_avoid_hold_cycles': ParameterValue(
                LaunchConfiguration('obstacle_avoid_hold_cycles'), value_type=int),
        }]
    )


def build_control_mppi_node(*, odom_topic, max_speed, remappings=None):
    """control_mppi_node — control_map_node와 나란히 상시 구동되는 MPPI 컨트롤러.
    /drive_mppi로 발행하며, Mux가 RB 상태에 따라 /drive_autonomous(MAP)와 라우팅한다.
    솔버(CPU/GPU)는 빌드타임 자동선택 — 런치는 무관.
    max_speed는 노드의 v_max(직선 최고속도 캡)로 매핑. Pacejka/차량 파라미터는 노드
    기본값(gym) — 실차 보정 전까지 노출 최소화. remappings: 실차 /imu/data→sensors/imu/raw."""
    return Node(
        package='f1tenth_control',
        executable='control_mppi_node',
        name='control_mppi_node',
        output='screen',
        remappings=remappings,
        parameters=[{
            'odom_topic': odom_topic,
            'v_max': max_speed,
            'lambda': LaunchConfiguration('mppi_lambda'),
            'sigma_steer': LaunchConfiguration('mppi_sigma_steer'),
            'sigma_accel': LaunchConfiguration('mppi_sigma_accel'),
        }]
    )


def build_joy_teleop_monitor():
    """joy_teleop_monitor — sim/real 완전 동일 설정.
    'is_simulation': True는 sim/real 양쪽 다 의도적으로 고정한 값이다(2026-07-12 사용자 확정) —
    노드 파라미터 자체 기본값은 false(실차=AUTONOMOUS 시작+수동 차단)이지만, 이 프로젝트는 실차
    포함 항상 MANUAL(조이스틱 수동 대기)로 시작해 LB로 AUTONOMOUS 전환하는 쪽을 원한다. "실수로
    하드코딩된 sim 기본값"이 아니니 실차 안전 차단이 필요해졌다고 해서 임의로 false로 되돌리지
    말 것 — 그 결정은 joy_teleop_monitor 노드 설명(CLAUDE.md)에도 반영돼 있다."""
    return Node(
        package='f1tenth_control',
        executable='joy_teleop_monitor',
        name='joy_teleop_monitor',
        output='screen',
        parameters=[{
            'is_simulation': True,
            'force_autonomous': LaunchConfiguration('force_autonomous'),
            'max_speed': 6.0,
            'max_steering_angle': 0.41,
            'use_trigger_throttle': True,
            'emergency_button': 1,
            'boost_button': 0,
            'algorithm_button': 5,
        }]
    )
