import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import _control_common as common


def generate_launch_description():
    # ==========================================================================
    # 실차용 런치 (첫 실주행 셰이크다운 보수 프리셋)
    # ==========================================================================
    # 시뮬 런치(control_sim.launch.py)와의 차이 = 실차 안전/환경 정합:
    #   - odom_topic: /pf/pose/odom (파티클필터) — 시뮬의 /ego_racecar/odom 대체
    #   - is_simulation=False: 실차 모드(수동 조작 송출 차단, 초기 AUTONOMOUS)
    #   - max_speed 기본 6.0: 직선 상한 보수적 캡(하드웨어 최고 ~9 m/s 대비 여유)
    #   - use_imu=True: VESC 내장 IMU 영점/세팅 완료 → 롤 인지 ESC 활성
    #   - aeb_node 포함: /scan TTC 기반 독립 비상제동(odom은 PF로 remapping)
    #   - joy.launch.py include: 조이스틱 드라이버(joy_node)도 이 런치가 함께 기동
    # 전제: 하드웨어 브링업(vesc_driver, LiDAR, particle_filter)과 planning이
    #       /scan, /pf/pose/odom, /global_waypoints 를 발행 중이어야 함. (조이스틱은 이 런치가 기동)
    # 공통 파라미터(wheelbase, l1_gain 등)/joy_teleop_monitor 설정은 _control_common.py 참고 —
    # 시뮬과 겹치는 부분은 거기 한 곳만 고치면 됨. 아래는 실차 전용 인자/노드(AEB, 조이스틱)만.

    # 파티클필터 odom 토픽 (로컬라이제이션 스택에 맞춰 변경 가능)
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터/EKF)'
    )

    # 조이스틱 디바이스 번호 (/dev/input/js<device_id>) — include하는 joy.launch.py로 전달
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='조이스틱 디바이스 번호 (/dev/input/js<device_id>)'
    )

    # 직선 최대 속도 [m/s] — 첫 실주행은 6.0 보수 캡. 셰이크다운 후 라이브 상향.
    # ⚠️ 하드웨어 ERPM(40000) 상한 = 바퀴 ~9 m/s. 6.0은 그 2/3 수준.
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='6.0',
        description='control_map_node 직선 최대 속도 [m/s] (실차 보수 캡)'
    )

    # 랩타임 튜닝 인자 (실차 검증 전까지 그립 정직값 유지)
    #   max_lateral_accel = 코너 속도 캡 v=sqrt(a_lat/kappa). LUT 마찰피크(~6.7) 이하로 두어야
    #   명령 횡가속도가 실 그립을 안 넘어 실차 이탈 없음. 이게 코너 랩타임의 진짜 병목.
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='6.5',
        description='곡률 코너 속도 캡 a_lat [m/s^2] = 실 그립 자기제한선(LUT 마찰피크 ~6.7 이하)'
    )

    # 비워두면 기존 폴백 순서(steering_lookup share → f1tenth_control share)로 로드.
    # lut_calibrator_node가 만든 보정 LUT를 쓰려면 그 출력 경로를 지정할 것
    # (예: $HOME/f1tenth_lut_calibration/NUC6_glc_pacejka_lookup_table_calibrated.csv)
    lookup_table_file_arg = DeclareLaunchArgument(
        'lookup_table_file', default_value='',
        description='Steering LUT CSV 경로 (비워두면 기본 폴백 사용, 캘리브레이션 결과 적용 시 지정)'
    )

    steering_control = common.build_control_map_node(
        odom_topic=LaunchConfiguration('odom_topic'),
        max_speed=LaunchConfiguration('max_speed'),
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=6.5,
        lookup_table_file=LaunchConfiguration('lookup_table_file'),
        # vesc_driver_node는 IMU를 sensors/imu/raw로 발행하지만 control_map_node.cpp는
        # /imu/data를 구독하도록 하드코딩돼있어 리매핑 필요(안 하면 IMU 미수신 → 롤 ESC/
        # 요레이트 카운터스티어가 조용히 무효화됨).
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    # MPPI 컨트롤러 노드 — control_map_node와 나란히 상시 구동(/drive_mppi 발행).
    # 평소엔 Mux가 MAP을 라우팅(MPPI 출력 무시), 조이스틱 RB로 즉시 MPPI 전환.
    # 실차 IMU 리매핑은 control_map_node와 동일(/imu/data→sensors/imu/raw).
    mppi_control = common.build_control_mppi_node(
        odom_topic=LaunchConfiguration('odom_topic'),
        max_speed=LaunchConfiguration('max_speed'),
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    joy_teleop_monitor = common.build_joy_teleop_monitor()

    # 조이스틱 드라이버(joy_node)를 joy.launch.py include로 함께 기동한다.
    # joy 설정(deadzone/autorepeat_rate)은 joy.launch.py 한 곳에만 유지하고 device_id만 전달.
    # joy_teleop_monitor는 joy_node가 발행하는 /joy(sensor_msgs/Joy)를 스틱+트리거 매핑으로 해석함.
    # (전제: `joy` 패키지 설치. 조이스틱이 끊겨 재시작이 필요하면, 이 스택 전체 대신
    #  `ros2 launch f1tenth_control joy.launch.py`로 joy만 따로 다시 띄워도 된다.)
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('f1tenth_control'), 'launch', 'joy.launch.py')
        ),
        launch_arguments={'device_id': LaunchConfiguration('device_id')}.items()
    )

    # AEB 노드: /scan TTC 기반 독립 비상제동. odom을 /ego_racecar/odom으로 하드코딩하므로
    # 실차 PF odom(/pf/pose/odom)으로 remapping 필요.
    aeb_config = os.path.join(
        get_package_share_directory('f1tenth_control'), 'config', 'aeb_params.yaml')
    aeb_node = Node(
        package='f1tenth_control',
        executable='aeb_node',
        name='aeb_node',
        output='screen',
        parameters=[aeb_config],
        remappings=[('/ego_racecar/odom', LaunchConfiguration('odom_topic'))]
    )

    return LaunchDescription([
        *common.declare_common_args(),
        odom_topic_arg,
        device_id_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        lookup_table_file_arg,
        joy_launch,
        steering_control,
        mppi_control,
        joy_teleop_monitor,
        aeb_node,
    ])
