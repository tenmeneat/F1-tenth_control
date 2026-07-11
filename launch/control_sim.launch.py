import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
import _control_common as common


def generate_launch_description():
    steering_control = common.build_control_map_node(
        odom_topic='/ego_racecar/odom',
        max_speed=12.0,
        max_lateral_accel=6.0,
        base_max_accel=4.0,
    )

    # 시뮬 전용: odom 요레이트 → /imu/data 중계 (gym_bridge는 IMU를 발행하지 않음)
    sim_imu_bridge = Node(
        package='f1tenth_control',
        executable='sim_imu_bridge_node',
        name='sim_imu_bridge_node',
        output='screen',
        parameters=[{
            'odom_topic': '/ego_racecar/odom',
            'imu_topic': '/imu/data',
        }]
    )

    joy_teleop_monitor = common.build_joy_teleop_monitor()

    return LaunchDescription([
        *common.declare_common_args(),
        sim_imu_bridge,
        steering_control,
        joy_teleop_monitor,
    ])
