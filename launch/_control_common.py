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
# ⚠️ AEB·조이스틱 드라이버·sim_imu_bridge_node 포함 여부 등 안전 관련 구조 차이는
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
            'obstacle_avoid_enable', default_value='true',
            description='글로벌 추종 중 장애물 차단 감지 시 GapFollower 회피 폴백 활성화'
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
    ]


def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                            lookup_table_file=''):
    """control_map_node — 환경별로 다른 값만 인자로 받고 나머지는 공용 정의."""
    return Node(
        package='f1tenth_control',
        executable='control_map_node',
        name='control_map_node',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'wheelbase': 0.33,
            'l1_gain': 0.5,
            'l1_distance': 0.3,
            't_clip_min': 0.8,
            't_clip_max': 5.0,
            'lateral_error_coeff': 1.0,
            'max_speed': max_speed,
            'min_speed': 2.0,
            'max_lateral_accel': max_lateral_accel,
            'curvature_lookahead_count': 20,
            'base_max_accel': base_max_accel,
            'base_max_decel': 8.0,
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


def build_joy_teleop_monitor():
    """joy_teleop_monitor — sim/real 완전 동일 설정."""
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
