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
    #   - use_imu=True: VESC 내장 IMU 영점/세팅 완료 → 롤 인지 ESC 활성
    #   - 비상제동(AEB)은 제어 파트에서 제거됨 — 실제 비상정지는 planning 파트가 판단/발행
    #   - 수동/자율/E-stop Mux는 f1tenth_stack(drive_mode_manager + ackermann_mux)이 담당 →
    #     우리 joy_teleop_monitor는 이 런치에서 제외(2026-07-17). 대신 MAP/MPPI만 고르는
    #     drive_source_selector가 /joy(RB)를 구독해 /drive(navigation 채널)로 포워딩(아래 참고).
    #   - joy_node(조이스틱 드라이버)도 이 런치에 없음 — f1tenth_stack이 라이다/조이스틱/vesc
    #     드라이버를 함께 기동하므로 중복 방지 위해 제거(2026-07-14). 셀렉터는 그쪽 /joy를 구독.
    #   - ackermann_to_vesc_node도 없음(f1tenth_stack이 자체 기동, 아래 참고)
    # 전제: 하드웨어 브링업(f1tenth_stack의 drive_mode_manager, ackermann_mux, vesc_driver,
    #       ackermann_to_vesc, LiDAR, joy_node)과 particle_filter, planning이 /scan, /joy,
    #       /pf/pose/odom, /global_waypoints 를 발행 중이어야 함.
    # 공통 파라미터(wheelbase, l1_gain 등)는 _control_common.py 참고 — 시뮬과 겹치는 부분은
    # 거기 한 곳만 고치면 됨. 아래는 실차 전용 인자/노드만.

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

    # 실차 수동/자율/E-stop Mux는 팀 공용 f1tenth_stack이 담당한다(drive_mode_manager +
    # ackermann_mux). 따라서 우리 joy_teleop_monitor는 이 런치에서 제외한다(2026-07-17) —
    # 띄우면 /drive를 이중 발행해 f1tenth_stack의 navigation 입력과 충돌한다.
    #
    # 대신 MAP/MPPI 알고리즘 선택만 담당하는 슬림 셀렉터(drive_source_selector)를 띄운다.
    # f1tenth_stack 스택엔 MAP/MPPI 개념이 없고 자율 입력은 mux의 navigation 채널 'drive'
    # 하나뿐이라, 이 노드가 RB로 /drive_autonomous(MAP)↔/drive_mppi(MPPI)를 골라 /drive로
    # 포워딩한다. /drive는 f1tenth_stack ackermann_mux의 navigation 입력(우선순위10)과
    # 토픽명이 일치해 자동으로 흘러들어간다. RB(5)는 drive_mode_manager가 안 쓰는 버튼이라
    # 충돌 없음. E-stop은 drive_mode_manager가 estop_lock으로 mux 전체를 마스킹하므로 이
    # 노드가 계속 /drive를 내보내도 제동 중엔 차단된다(셀렉터는 E-stop을 몰라도 됨).
    drive_source_selector = Node(
        package='f1tenth_control',
        executable='drive_source_selector',
        name='drive_source_selector',
        output='screen',
        parameters=[{
            'algorithm_button': 5,   # RB
        }]
    )

    # ackermann_to_vesc_node도 이 launch에 없다(2026-07-17 제거). f110(f1tenth_stack)이 이미
    # 자체 ackermann_to_vesc_node를 띄우고, 그 입력('ackermann_drive')은 자체 ackermann_mux가
    # 'teleop'(drive_mode_manager 수동, 우선순위100)과 'drive'(navigation, 우선순위10 — 위
    # 셀렉터 출력)를 중재해 만든다. 우리가 또 띄우면 같은 VESC 명령 토픽에 중복 발행되어
    # 경합(조향 덜컹거림)한다.
    # ⚠️ 전제조건(f1tenth_stack 쪽, 이 repo 밖):
    #   1. drive_mode_manager가 AUTONOMOUS 모드에서 teleop을 침묵시켜 navigation('drive')이
    #      mux에서 이길 수 있어야 함(구 joy_teleop.yaml default 섹션 상시발행 이슈는 해소됨).
    #   2. vesc.yaml의 steering_angle_to_servo_offset이 실측 캘리브레이션값(0.4633,
    #      2026-07-17)과 동기화돼 있어야 함.

    return LaunchDescription([
        *common.declare_common_args(),
        odom_topic_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        lookup_table_file_arg,
        base_max_accel_arg,
        steering_control,
        mppi_control,
        drive_source_selector,
    ])
