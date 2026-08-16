from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("state_machine"),
            "config",
            "state_machine.yaml",
        ]),
        description="Path to state_machine_node parameter YAML",
    )

    state_machine_node = Node(
        package="state_machine",
        executable="state_machine_node",
        name="state_machine_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        state_machine_node,
    ])
