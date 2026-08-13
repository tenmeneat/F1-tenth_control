from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터)'
    )

    odom_calib = Node(
        package='f1tenth_control',
        executable='odom_calib_node',
        name='odom_calib_node',
        output='screen',
        parameters=[{'odom_topic': LaunchConfiguration('odom_topic')}],
    )

    return LaunchDescription([
        odom_topic_arg,
        odom_calib,
    ])
