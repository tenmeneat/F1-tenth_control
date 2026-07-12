import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import _control_common as common


def generate_launch_description():
    # 종방향 최대 가속도 한계 [m/s^2] — real(6.5)과 값이 달라 _control_common.py 공용 선언 대신
    # 진입점 파일에 각자 둔다(의도된 구조, CLAUDE.md 참고).
    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='4.0',
        description='종방향 최대 가속도 한계 [m/s^2] (sim 기본값, real은 6.5)'
    )

    # 곡률 코너 속도 캡 a_lat [m/s^2] — 예전엔 이 파일에 6.0 리터럴로 하드코딩돼 재빌드 없이
    # 조정 불가했음(2026-07-13, 촘촘 웨이포인트 랩타임 조사 중 발견). real과 동일 패턴으로
    # 인자 승격. 기본값은 기존과 동일한 6.0 유지(회귀 없음) — LUT 마찰피크(~6.83) 이하로
    # 두어야 시뮬 결과가 실차로 이식 가능(그 이상은 시뮬 선형타이어 착시, real.launch.py 주석 참고).
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='6.0',
        description='곡률 코너 속도 캡 a_lat [m/s^2] (LUT 마찰피크 ~6.83 이하로 유지할 것)'
    )

    steering_control = common.build_control_map_node(
        odom_topic='/ego_racecar/odom',
        max_speed=12.0,
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=LaunchConfiguration('base_max_accel'),
    )

    # MPPI 컨트롤러 노드 — control_map_node와 나란히 상시 구동(/drive_mppi 발행).
    # 실차 전 시뮬 검증용. 조이스틱 RB로 MAP↔MPPI 즉시 전환.
    mppi_control = common.build_control_mppi_node(
        odom_topic='/ego_racecar/odom',
        max_speed=12.0,
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
        base_max_accel_arg,
        max_lateral_accel_arg,
        sim_imu_bridge,
        steering_control,
        mppi_control,
        joy_teleop_monitor,
    ])
