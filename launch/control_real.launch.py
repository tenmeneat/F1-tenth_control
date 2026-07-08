import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ==========================================================================
    # 실차용 런치 (첫 실주행 셰이크다운 보수 프리셋)
    # ==========================================================================
    # 시뮬 런치(control_sim.launch.py)와의 차이 = 실차 안전/환경 정합:
    #   - odom_topic: /pf/pose/odom (파티클필터) — 시뮬의 /ego_racecar/odom 대체
    #   - is_simulation=False: 실차 모드(수동 조작 송출 차단, 초기 AUTONOMOUS)
    #   - max_speed 기본 6.0: 직선 상한 보수적 캡(하드웨어 최고 ~9 m/s 대비 여유)
    #   - use_imu=False: VESC 내장 IMU 코드 미적용 → 당분간 롤 인지 ESC 비활성
    #   - aeb_node 포함: /scan TTC 기반 독립 비상제동(odom은 PF로 remapping)
    # 전제: 하드웨어 브링업(vesc_driver, LiDAR, particle_filter, joy)과 planning이
    #       /scan, /pf/pose/odom, /global_waypoints, /joy 를 발행 중이어야 함.

    # 파티클필터 odom 토픽 (로컬라이제이션 스택에 맞춰 변경 가능)
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터/EKF)'
    )

    # 조이스틱 없이 자율주행 즉시 기동 여부 (기본 false → 초기 AUTONOMOUS이나 LB로 대기 가능)
    force_autonomous_arg = DeclareLaunchArgument(
        'force_autonomous',
        default_value='false',
        description='true 시 조이스틱 없이 자율주행 모드 즉시 기동'
    )

    # 조향 컨트롤러 선택 ('l1' 기본 폴백 | 'mpc' 전역좌표 LTV-MPC)
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='l1',
        description="조향 컨트롤러: 'l1'(검증된 L1 Guidance) 또는 'mpc'(조향 MPC)"
    )

    # 직선 최대 속도 [m/s] — 첫 실주행은 6.0 보수 캡. 셰이크다운 후 라이브 상향.
    # ⚠️ 하드웨어 ERPM(40000) 상한 = 바퀴 ~9 m/s. 6.0은 그 2/3 수준.
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='6.0',
        description='steering_control_node 직선 최대 속도 [m/s] (실차 보수 캡)'
    )

    # 랩타임 튜닝 인자 (실차 검증 전까지 그립 정직값 유지)
    #   max_lateral_accel = 코너 속도 캡 v=sqrt(a_lat/kappa). LUT 마찰피크(~6.7) 이하로 두어야
    #   명령 횡가속도가 실 그립을 안 넘어 실차 이탈 없음. 이게 코너 랩타임의 진짜 병목.
    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale', default_value='1.3',
        description='플래너 vx 곱계수 (직선은 max_speed에 포화)'
    )
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='6.5',
        description='곡률 코너 속도 캡 a_lat [m/s^2] = 실 그립 자기제한선(LUT 마찰피크 ~6.7 이하)'
    )

    steering_control = Node(
        package='f1tenth_control',
        executable='steering_control_node',
        name='steering_control_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
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
            'base_max_accel': 6.5,
            'base_max_decel': 8.0,
            'wall_safety_margin': 0.6,
            'speed_scale': LaunchConfiguration('speed_scale'),
            'use_imu': False,       # VESC 내장 IMU 코드 미적용 → 당분간 비활성(보류)
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
            # ⚠️ is_simulation=True = "조이스틱 수동조작이 실제 바퀴를 굴리게 허용".
            # 실차지만 첫 셰이크다운에서 스틱(조향)+RT(가속)로 직접 몰아 모터/조향을 검증하려는 목적.
            # 효과: 시작은 MANUAL 대기(자율 자동출발 안 함) → 스틱/RT 수동주행 → LB로 AUTONOMOUS 전환.
            # 비상정지(B)·AEB는 수동/자율 양 모드에서 최우선 제동으로 항상 동작.
            'is_simulation': True,
            'force_autonomous': LaunchConfiguration('force_autonomous'),
            'max_speed': 6.0,           # 수동 조이스틱 스로틀 스케일 [m/s] (RB 부스트 시 ×1.5)
            'max_steering_angle': 0.41,
            'use_trigger_throttle': True,  # RT(axes[5]) 가속 / LT(axes[2]) 감속
            'emergency_button': 1,      # B
            'boost_button': 5,          # RB
        }]
    )

    # 조이스틱 드라이버(joy_node)는 별도 런치로 분리됨 → `ros2 launch f1tenth_control joy.launch.py`.
    # joy_teleop_monitor는 그 노드가 발행하는 /joy(sensor_msgs/Joy)를 스틱+트리거 매핑으로 해석함.
    # (전제: `joy` 패키지 설치, 그리고 joy.launch.py를 함께 실행.)

    # AEB 노드: /scan TTC 기반 독립 비상제동. odom을 /ego_racecar/odom으로 하드코딩하므로
    # 실차 PF odom(/pf/pose/odom)으로 remapping 필요.
    aeb_config = os.path.join(
        get_package_share_directory('f1tenth_control'), 'config', 'aeb_params.yaml')
    aeb_node = Node(
        package='f1tenth_control',
        executable='aeb_node',
        name='aeb_node',
        output='screen',
        parameters=[aeb_config],
        remappings=[('/ego_racecar/odom', LaunchConfiguration('odom_topic'))]
    )

    return LaunchDescription([
        odom_topic_arg,
        force_autonomous_arg,
        controller_type_arg,
        max_speed_arg,
        speed_scale_arg,
        max_lateral_accel_arg,
        steering_control,
        joy_teleop_monitor,
        aeb_node,
    ])
