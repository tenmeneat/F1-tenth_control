import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import _control_common as common


def generate_launch_description():
    # 종방향 최대 가속도 한계 [m/s^2] — real과 값이 갈릴 수 있어 _control_common.py 공용 선언
    # 대신 진입점 파일에 각자 둔다(의도된 구조, CLAUDE.md 참고). 공격적 속도 프로파일(fuck_f1
    # 재생성분)을 직선에서 놓치지 않도록 상향된 값(폐루프 6.97s 달성). 히스토리/실차 경고는
    # CLAUDE.md "실차 튜닝 파라미터" 표 참고.
    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='9.0',
        description='종방향 최대 가속도 한계 [m/s^2] (공격적 프로파일 추종용, 상세는 CLAUDE.md)'
    )

    # 곡률 코너 속도 캡 a_lat [m/s^2] = backward-pass 사전감속의 그립 클램프
    # (min(프로파일 vx, √(a_lat/κ))). 프로파일 코너 a_lat보다 낮으면 컨트롤러가 코너에서
    # 프로파일보다 느려지므로 상향된 값. ⚠️ LUT 실그립 피크(~6.7)를 초과하는 시뮬 낙관치 —
    # 실차 검증 전 그대로 쓰면 코너에서 언더스티어/슬라이드 가능. 히스토리/경고는 CLAUDE.md 참고.
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='10.0',
        description='코너 그립 클램프 a_lat [m/s^2] (⚠️ sim 낙관치, 실차 검증 전 CLAUDE.md 참고)'
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
