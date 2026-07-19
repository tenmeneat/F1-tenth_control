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
    # LUT 캘리브레이션 노드 단독 런치
    # ==========================================================================
    # control_real.launch.py와 "함께" 별도 터미널에서 띄운다. 이 노드는 /drive를
    # 절대 발행하지 않는 관찰 전용(observer)이라 주행 제어에 아무 영향이 없다.
    #
    # 여러 번 주행(여러 번 이 런치를 재실행)해도 ~/f1tenth_lut_calibration/
    # calibration_state.csv에 누적치가 저장되어 자동으로 이어서 평균이 쌓인다.
    # 결과 LUT는 같은 폴더의 NUC6_glc_pacejka_lookup_table_calibrated.csv로 출력.
    #
    # 실제로 이 결과를 다음 주행에 반영하려면, control_map_node의
    # lookup_table_file 파라미터를 그 출력 경로로 지정해서 재실행할 것.
    #   예) ros2 launch f1tenth_control control_real.launch.py \
    #         lookup_table_file:=$HOME/f1tenth_lut_calibration/NUC6_glc_pacejka_lookup_table_calibrated.csv
    #
    # 사용: ros2 launch f1tenth_control lut_calibration.launch.py
    #   중간에 강제 저장: ros2 topic pub -1 /lut_calibration/save std_msgs/msg/Empty {}
    #   진행 상황 확인: ros2 topic echo /lut_calibration/status

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (control_real.launch.py와 동일하게 맞출 것)'
    )

    # IMU 각속도 단위 보정 — 기본값을 _control_common.py의 하드웨어 상수에서 직접 가져오므로
    # control_real.launch.py와 **구조적으로 항상 같은 값**이다(수동 미러링 불필요).
    # 이 노드는 a_lat = v * yaw_rate(IMU)로 실측 횡가속도를 만들어 LUT를 보정하는데, 단위가
    # 틀리면 횡가속도가 57.3배가 되어 보정 LUT가 통째로 쓰레기가 된다. 게다가 /drive를 발행하지
    # 않는 관찰자라 주행 중엔 아무 증상이 없어 나중에야 알게 된다 — 그래서 인자로 매번 넘기는
    # 방식을 버리고 상수 공유로 바꿨다(2026-07-19). 값 변경은 _control_common.py에서.
    imu_scale_arg = DeclareLaunchArgument(
        'imu_angular_scale',
        default_value=str(common.IMU_ANGULAR_SCALE),
        description=f'IMU 각속도 단위 보정 계수 (하드웨어 상수, 기본 {common.IMU_ANGULAR_SCALE}). '
                    '_control_common.py의 IMU_ANGULAR_SCALE에서 공유'
    )

    lut_calibrator = Node(
        package='f1tenth_control',
        executable='lut_calibrator_node',
        name='lut_calibrator_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'imu_topic': '/imu/data',
            'drive_topic': '/drive',
            'min_speed_for_sample': 1.0,
            'prior_weight': 3.0,
            'yaw_rate_filter_alpha': 0.3,
            'save_interval_sec': 15.0,
            'imu_angular_scale': LaunchConfiguration('imu_angular_scale'),
        }]
    )

    return LaunchDescription([
        odom_topic_arg,
        imu_scale_arg,
        lut_calibrator,
    ])
