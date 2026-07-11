from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
        }]
    )

    return LaunchDescription([
        odom_topic_arg,
        lut_calibrator,
    ])
