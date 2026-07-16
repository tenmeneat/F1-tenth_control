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
    # 시뮬 런치(control_sim.launch.py)와의 차이 = 실차 안전/환경 정합:
    #   - odom_topic: /pf/pose/odom (파티클필터) — 시뮬의 /ego_racecar/odom 대체
    #   - is_simulation=False: 실차 모드(수동 조작 송출 차단, 초기 AUTONOMOUS)
    #   - max_speed 기본 7.0: 직선 상한 보수적 캡(하드웨어 최고 ~9 m/s 대비 여유)
    #   - use_imu=True: VESC 내장 IMU 영점/세팅 완료 → 롤 인지 ESC 활성
    #   - 비상제동(AEB)은 제어 파트에서 제거됨 — 실제 비상정지는 planning 파트가 판단/발행
    #   - joy_node(조이스틱 드라이버)는 이 런치에 없음 — f1tenth_stack(f110 단축어)이
    #     라이다/조이스틱/vesc드라이버를 함께 기동하므로 중복 기동 방지 위해 여기서 제거함
    #     (2026-07-14). joy_teleop_monitor는 그쪽이 띄운 /joy를 그대로 구독.
    #   - ackermann_to_vesc_node 포함: 최종 /drive → VESC ERPM/서보 명령 어댑터
    #     (⚠️ 실제 모터 구동은 vesc_driver_node가 별도로 떠야 함 — 아래 전제 참고)
    # 전제: 하드웨어 브링업(f1tenth_stack의 vesc_driver, LiDAR, joy_node)과 particle_filter,
    #       planning이 /scan, /joy, /pf/pose/odom, /global_waypoints 를 발행 중이어야 함.
    #       (VESC 시리얼 드라이버 vesc_driver_node도 f1tenth_stack 쪽에서 기동.)
    # 공통 파라미터(wheelbase, l1_gain 등)/joy_teleop_monitor 설정은 _control_common.py 참고 —
    # 시뮬과 겹치는 부분은 거기 한 곳만 고치면 됨. 아래는 실차 전용 인자/노드(조이스틱,
    # ackermann_to_vesc)만.

    # 파티클필터 odom 토픽 (로컬라이제이션 스택에 맞춰 변경 가능)
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터/EKF)'
    )

    # 직선 최대 속도 [m/s] — 첫 실주행 셰이크다운 캡. 안정성 확인 후 단계적으로 상향.
    # ⚠️ 하드웨어 ERPM(40000) 상한 = 바퀴 ~9 m/s. 7.0은 그 약 78% 수준.
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='7.0',
        description='control_map_node 직선 최대 속도 [m/s] (실차 보수 캡)'
    )

    # max_lateral_accel = backward-pass 사전감속 그립 클램프 min(프로파일 vx, sqrt(a_lat/kappa)).
    # ⚠️⚠️ 실차 안전 경고: 10.0은 LUT 실 그립 마찰피크(~6.7)를 크게 초과하는 sim 낙관치.
    # 보수적 프로파일에선 backward-pass가 min(프로파일, sqrt(10/kappa))라 사실상 무효(프로파일이
    # 이김)이지만, 프로파일을 공격적으로 재생성해 코너속도가 sqrt(6.7/kappa)를 넘으면 실 타이어
    # 그립 초과로 언더스티어/슬라이드 위험. 실그립 매칭 프로파일+저속 셰이크다운 검증 필수,
    # 슬라이드 시 6.7로 되돌릴 것.
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='10.0',
        description='코너 그립 클램프 a_lat [m/s^2] (⚠️ sim 승리값, 실그립 ~6.7 초과 — 실차 검증 필수)'
    )

    # 비워두면 기존 폴백 순서(steering_lookup share → f1tenth_control share)로 로드.
    # lut_calibrator_node가 만든 보정 LUT를 쓰려면 그 출력 경로를 지정할 것
    # (예: $HOME/f1tenth_lut_calibration/NUC6_glc_pacejka_lookup_table_calibrated.csv)
    lookup_table_file_arg = DeclareLaunchArgument(
        'lookup_table_file', default_value='',
        description='Steering LUT CSV 경로 (비워두면 기본 폴백 사용, 캘리브레이션 결과 적용 시 지정)'
    )

    # 종방향 최대 가속도 한계 [m/s^2]. base_max_decel은 sim/real 동일이라 _control_common.py에 공용.
    # ⚠️ sim 승리값과 동일한 공격적 값 — 속도는 프로파일이 캡하므로 최고속은 안 변하나 램프가
    # 급해짐. 실차 모터/구동 여유와 저속 출발 안정성 실측 확인 후 유지할 것(급가속으로 휠스핀/
    # 앞들림 시 하향).
    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='9.0',
        description='종방향 최대 가속도 한계 [m/s^2] (⚠️ sim 승리값, 실차 급가속 검증 필요)'
    )

    steering_control = common.build_control_map_node(
        odom_topic=LaunchConfiguration('odom_topic'),
        max_speed=LaunchConfiguration('max_speed'),
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=LaunchConfiguration('base_max_accel'),
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

    # ackermann_to_vesc_node: Mux가 확정한 최종 /drive(AckermannDriveStamped)를 VESC 명령
    # (commands/motor/speed=ERPM, commands/servo/position)으로 변환하는 어댑터. 이게 있어야
    # 컨트롤 출력이 실제 모터/서보에 도달한다. 노드 자체는 vesc_ackermann 패키지 소속.
    #  - ⚠️ 노드는 'ackermann_cmd'를 구독하므로 우리 최종 토픽 /drive로 remapping 필수
    #    (안 하면 아무도 발행 안 하는 ackermann_cmd를 구독해 조용히 무동작).
    #  - 파라미터는 vesc_ackermann 레퍼런스 런치(ackermann_to_vesc_node.launch.xml)의
    #    하드웨어 보정값. speed_to_erpm_gain=4614.0은 vesc_to_odom과 반드시 동일해야 함.
    #    servo gain/offset은 실제 조향 링키지 기준값 — 조향 방향/중립이 어긋나면 여기서 튜닝.
    #  - ⚠️ 이 노드만으론 모터가 안 돈다. 실제 하드웨어 구동은 vesc_driver_node가 별도로
    #    떠서 commands/motor/speed를 시리얼로 VESC에 전달해야 함(하드웨어 브링업 소관).
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': LaunchConfiguration('speed_to_erpm_gain'),
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.5304,
        }],
        remappings=[('ackermann_cmd', '/drive')]
    )

    return LaunchDescription([
        *common.declare_common_args(),
        odom_topic_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        lookup_table_file_arg,
        base_max_accel_arg,
        steering_control,
        mppi_control,
        joy_teleop_monitor,
        ackermann_to_vesc_node,
    ])
