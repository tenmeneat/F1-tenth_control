import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import _control_common as common

def generate_launch_description():
    # ==========================================================================
    # 실차용 런치 (첫 실주행 셰이크다운 보수 프리셋)
    # ==========================================================================
 
    # 파티클필터 odom 토픽 (로컬라이제이션 스택에 맞춰 변경 가능)
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터/EKF)'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='8.0',
        description='control_map_node 직선 최대 속도 [m/s]'
    )

    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='7.0',
        description='코너 그립 클램프 a_lat [m/s^2] (LUT 그립 피크 ~6.7 이내)'
    )

    max_steering_left_arg = DeclareLaunchArgument(
        'max_steering_left', default_value='0.410',
        description='좌조향(δ>0) 명령 한계 [rad] = 실제 바퀴 각. 젯슨 vesc.yaml 의 '
                    'steering_angle_to_servo_gain_left(-0.5785) + offset(0.4672) 과 한 쌍'
    )
    max_steering_right_arg = DeclareLaunchArgument(
        'max_steering_right', default_value='0.410',
        description='우조향(δ<0) 명령 한계 [rad] = 실제 바퀴 각. 젯슨 vesc.yaml 의 '
                    'steering_angle_to_servo_gain_right(-0.4702) + offset(0.4672) 과 한 쌍'
    )

    lookup_table_file_arg = DeclareLaunchArgument(
        'lookup_table_file', default_value='',
        description='Steering LUT CSV 경로 (비워두면 기본 폴백 사용, 캘리브레이션 결과 적용 시 지정)'
    )

    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='3.5',
        description='종방향 최대 가속도 한계 [m/s^2]. VESC 천장은 s_pid_ramp_erpms_s(21160 ÷ 4336 '
                    '= 4.88) — 3.5는 그 72%라 여유가 있다. 4.88을 넘겨 주면 VESC가 깎고 '
                    '와인드업 위험만 커진다'
    )

    steering_control = common.build_control_map_node(
        odom_topic=LaunchConfiguration('odom_topic'),
        max_speed=LaunchConfiguration('max_speed'),
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=LaunchConfiguration('base_max_accel'),
        max_steering_left=LaunchConfiguration('max_steering_left'),
        max_steering_right=LaunchConfiguration('max_steering_right'),
        lookup_table_file=LaunchConfiguration('lookup_table_file'),
        imu_linear_scale=common.IMU_LINEAR_SCALE_REAL,
        imu_angular_scale=common.IMU_ANGULAR_SCALE_REAL,
        # vesc_driver_node는 IMU를 sensors/imu/raw로 발행하지만 control_map_node.cpp는
        # /imu/data를 구독하도록 하드코딩돼있어 리매핑 필요(안 하면 IMU 미수신 → 롤 ESC/
        # 요레이트 카운터스티어가 조용히 무효화됨).
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    drive_source_selector = Node(
        package='f1tenth_control',
        executable='drive_source_selector',
        name='drive_source_selector',
        output='screen',
    )

    sector_learner_node = common.build_sector_learner_node()

    return LaunchDescription([
        *common.declare_common_args(sector_scale_enable_default='true'),
        odom_topic_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        max_steering_left_arg,
        max_steering_right_arg,
        lookup_table_file_arg,
        base_max_accel_arg,
        steering_control,
        drive_source_selector,
        sector_learner_node,
    ])
