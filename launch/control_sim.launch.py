import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import _control_common as common


def generate_launch_description():
    # 실차용 joy.launch.py(제거됨)와 달리, 시뮬은 편의상 joy_node를 여기 함께 번들한다
    # (device_id로 장치 선택). force_autonomous:=false로 켜서 조이스틱 수동 개입/오버라이드를
    # 시험하고 싶을 때 이 파일 하나로 끝나게 하기 위함(ifac_sim 등 일괄 실행 스크립트 연계).
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='조이스틱 디바이스 번호 (/dev/input/js<device_id>)'
    )

    # 종방향 최대 가속도 한계 [m/s^2] — real과 값이 갈릴 수 있어 진입점 파일에 각자 둔다.
    # 공격적 속도 프로파일(fuck_f1 재생성분)을 직선에서 놓치지 않도록 상향된 값(폐루프 6.97s
    # 달성). 히스토리/실차 경고는 CLAUDE.md "실차 튜닝 파라미터" 표 참고.
    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='9.0',
        description='종방향 최대 가속도 한계 [m/s^2] (공격적 프로파일 추종용, 상세는 CLAUDE.md)'
    )

    # 곡률 코너 속도 캡 a_lat [m/s^2] — backward-pass 사전감속의 그립 클램프
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
        # 시뮬 IMU는 sim_imu_bridge_node가 odom angular.z(이미 rad/s)를 중계 → 보정 불필요.
        # 실차 값(pi/180)을 여기 쓰면 요레이트가 1/57로 죽어 카운터스티어가 엉뚱해진다.
        imu_angular_scale=common.IMU_ANGULAR_SCALE_SIM,
        # 시뮬 브릿지는 linear_acceleration을 채우지 않아(0) 보정 불필요.
        imu_linear_scale=common.IMU_LINEAR_SCALE_SIM,
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

    # 시뮬 전용 joy_node 번들 — 실차는 f1tenth_stack이 별도로 띄우므로 여기 없음.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,   # 트리거를 계속 당기고 있어도 /joy·/drive 명령 지속되게 재발행
        }]
    )

    return LaunchDescription([
        *common.declare_common_args(),
        device_id_arg,
        base_max_accel_arg,
        max_lateral_accel_arg,
        sim_imu_bridge,
        steering_control,
        mppi_control,
        joy_teleop_monitor,
        joy_node,
    ])
