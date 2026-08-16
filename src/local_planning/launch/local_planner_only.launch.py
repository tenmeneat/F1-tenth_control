# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Start local_planner_node ALONE.

local_planner_node subscribes to /global_waypoints, /confirmed_static_obs (obstacles_topic),
/car_state/frenet/odom and /state. It does NOT subscribe to any occupancy map, so nothing here
starts a map server: the reference map exists only for the perception node that produces the
static-obstacle topics.

Use this when the detector (and its reference map) is already running, or when you want to drive
the planner from a recorded/synthetic /confirmed_static_obs source. For the whole
perception+planning stack use local_planning.launch.py, which includes this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('local_planning'), 'config', 'local_planning.yaml')

    arguments = [
        DeclareLaunchArgument(
            'params_file', default_value=default_config,
            description='Full path to the local_planning YAML parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use ROS simulation time (requires a /clock publisher)'),
        DeclareLaunchArgument(
            'timing_diagnostics_enable', default_value='false',
            description='Publish tuning-only monotonic timing events'),
        DeclareLaunchArgument(
            'timing_diagnostics_topic', default_value='/cma_timing/events',
            description='Companion timing-event topic'),
        DeclareLaunchArgument(
            'replay_diagnostics_enable', default_value='false',
            description='Publish tuning-only planner record/replay diagnostics'),
        DeclareLaunchArgument(
            'replay_diagnostics_topic', default_value='/cma_replay/planner_events',
            description='Planner record/replay companion-event topic'),
        DeclareLaunchArgument(
            'lockstep_mode', default_value='false',
            description='CMA-only deterministic event-driven execution'),
        DeclareLaunchArgument(
            'p3_mode', default_value='TEST_ACTIVE',
            description='Production P3/M1 mode: OFF, SHADOW, or bounded TEST_ACTIVE'),
        DeclareLaunchArgument(
            'p3_diagnostics_topic', default_value='/local_planning/p3_shadow',
            description='Non-commanding P3/M1 ownership and lifecycle diagnostic topic'),
    ]

    local_planner_node = Node(
        package='local_planning',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'timing_diagnostics_enable': ParameterValue(
                    LaunchConfiguration('timing_diagnostics_enable'), value_type=bool),
                'timing_diagnostics_topic': LaunchConfiguration('timing_diagnostics_topic'),
                'replay_diagnostics_enable': ParameterValue(
                    LaunchConfiguration('replay_diagnostics_enable'), value_type=bool),
                'replay_diagnostics_topic': LaunchConfiguration('replay_diagnostics_topic'),
                'lockstep_mode': ParameterValue(
                    LaunchConfiguration('lockstep_mode'), value_type=bool),
                'p3_mode': ParameterValue(
                    LaunchConfiguration('p3_mode'), value_type=str),
                'p3_diagnostics_topic': LaunchConfiguration('p3_diagnostics_topic'),
            },
        ]
    )

    return LaunchDescription(arguments + [local_planner_node])
