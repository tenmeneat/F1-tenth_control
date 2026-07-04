from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 런치 인자: 조이스틱 없이 자율주행 즉시 기동 여부 (기본 false → MANUAL 대기)
    force_autonomous_arg = DeclareLaunchArgument(
        'force_autonomous',
        default_value='false',
        description='true 시 조이스틱 없이 자율주행 모드 즉시 기동'
    )

    # 런치 인자: 조향 컨트롤러 선택 ('l1' 기본 폴백 | 'mpc' 전역좌표 LTV-MPC)
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='l1',
        description="조향 컨트롤러: 'l1'(검증된 L1 Guidance) 또는 'mpc'(조향 MPC)"
    )

    # 런치 인자: 최대 속도 (MPC 랩타임 푸시 시 상향 비교용)
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='12.0',
        description='steering_control_node 최대 속도 [m/s]'
    )

    # 랩타임 단축 튜닝 인자들 (라이브로 올려가며 한계 탐색)
    # 기본값 = 물리적으로 정직한(실차 이식 가능) config. fuck_f1 폐루프 스윕으로 도출.
    # 핵심: max_lateral_accel을 LUT 마찰피크(~6.7) 이하로 캡하면 명령 횡가속도가 실 그립을 안 넘어
    # 실차로 이식 가능. f1tenth_gym은 선형 타이어(마찰 포화 없음)라 MLA를 더 올리면 시뮬에선 빨라지지만
    # (MLA=9.0 → 랩8.0s) 실차에선 슬라이드 → 순수 시뮬 알고리즘 A/B(MAP vs MPC) 비교 때만 상향할 것.
    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale', default_value='1.3',
        description='플래너 vx 곱계수. 이 맵은 코너 지배라 speed_scale는 1.0~1.6 내내 포화(직선이 max_speed에 걸림) — 평균속도 레버 아님'
    )
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='6.5',
        description='곡률 코너 속도 캡 a_lat [m/s^2] = 실 그립 자기제한선(LUT 마찰피크 ~6.7 이하). 이게 랩타임의 진짜 병목. '
                    '실차 이식 기준값. ⚠️시뮬 상대비교용으로만 상향 가능(MLA=9.0=랩8.0s지만 시뮬 무한그립 착시, 실차 이탈)'
    )
    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='6.5', description='종방향 가속 한계 [m/s^2] (스윕상 랩 영향 미미, 코너캡이 지배)'
    )
    base_max_decel_arg = DeclareLaunchArgument(
        'base_max_decel', default_value='8.0', description='종방향 감속 한계 [m/s^2] (곡률 룩어헤드 제동거리 v^2/2a와 결합. 완전 그립정직 원하면 6.5)'
    )

    steering_control = Node(
        package='f1tenth_control',
        executable='steering_control_node',
        name='steering_control_node',
        output='screen',
        parameters=[{
            'odom_topic': '/ego_racecar/odom',
            'wheelbase': 0.33,
            'l1_gain': 0.5,
            'l1_distance': 0.3,
            't_clip_min': 0.8,
            't_clip_max': 5.0,
            'lateral_error_coeff': 1.0,
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': 2.0,
            'max_lateral_accel': LaunchConfiguration('max_lateral_accel'),
            'curvature_lookahead_count': 20,
            'base_max_accel': LaunchConfiguration('base_max_accel'),
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'wall_safety_margin': 0.6,
            'speed_scale': LaunchConfiguration('speed_scale'),
            'use_imu': False,       # 시뮬레이터에는 IMU 없음
            'curvature_ff_blend': 0.0,
            'heading_damping_gain': 0.0,
            # ===== 조향 컨트롤러 선택 & MPC 파라미터 =====
            'controller_type': LaunchConfiguration('controller_type'),
            'mpc_N': 12,
            'mpc_Ts': 0.05,
            'mpc_q_x': 5.0,
            'mpc_q_y': 5.0,
            'mpc_q_yaw': 0.5,
            'mpc_r': 0.1,
            'mpc_r_delta': 5.0,
            'mpc_ddelta_max': 0.20,
        }]
    )

    joy_teleop_monitor = Node(
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
            'boost_button': 5,
        }]
    )

    return LaunchDescription([
        force_autonomous_arg,
        controller_type_arg,
        max_speed_arg,
        speed_scale_arg,
        max_lateral_accel_arg,
        base_max_accel_arg,
        base_max_decel_arg,
        steering_control,
        joy_teleop_monitor,
    ])
