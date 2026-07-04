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
            'max_speed': 12.0,
            'min_speed': 2.0,
            'max_lateral_accel': 6.0,
            'curvature_lookahead_count': 20,
            'base_max_accel': 4.0,
            'base_max_decel': 8.0,
            'wall_safety_margin': 0.6,
            'use_imu': False,       # 시뮬레이터에는 IMU 없음
            'curvature_ff_blend': 0.0,
            'heading_damping_gain': 0.0,
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
        steering_control,
        joy_teleop_monitor,
    ])
