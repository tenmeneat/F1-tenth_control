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
        default_value='5.0',
        description='control_map_node 직선 최대 속도 [m/s] (실차 보수 캡)'
    )

    # max_lateral_accel = backward-pass 사전감속 그립 클램프 min(프로파일 vx, sqrt(a_lat/kappa)).
    # 실차 기본값을 6.5로 보수화(2026-07-23) — LUT 실 그립 마찰피크(~6.7) 이내로 잡아 코너에서
    # 실 타이어 그립을 초과하지 않도록. 프로파일을 공격적으로 재생성해 코너속도가 sqrt(6.5/kappa)를
    # 넘으면 backward-pass가 여기서 클램프한다. 실그립 매칭 프로파일+저속 셰이크다운 후, 여유가
    # 확인되면 상향 가능(sim 런치는 여전히 낙관치 10.0 — 랩타임 튜닝 기준 유지).
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='5.1',
        description='코너 그립 클램프 a_lat [m/s^2] (실차 실측 마찰한계 ~3.1 m/s² 반영)'
    )

    # ── 좌우 조향 한계 (2026-07-28, 실차 전용) ──
    # 젯슨 vesc.yaml의 servo_min 0.2703 / servo_max 0.6363 은 servo **0.4533 중심 ±0.1830**
    # (= ±0.41 rad)으로 계산된 낡은 값이다. 이후 기계적 센터를 맞추며 트림 offset이
    # **0.4672로 의도적으로 이동**됐는데 servo_min/max는 갱신되지 않아, 실제 가동각이
    # 좌 +0.441 / 우 **-0.379(잘림)**로 갈려 있었다. 07-27 bag(run_0727_203040)에서
    # servo 0.6502가 발행돼 vesc_driver의 servo_limit에 잘린 것이 실제로 관측된다.
    #
    # 🔧 대칭화 시도(2026-07-28) → **롤백됨. 재적용 전 반드시 벤치 검증할 것.**
    #    vesc.yaml 범위를 현 트림 0.4672 기준으로 재계산해 ±0.42 대칭으로 바꿨었다
    #    (servo_min 0.2703→0.2798, servo_max 0.6363→**0.6546**). 그런데 되돌렸다:
    #
    #    🔴 이유 — odom이 조향 명령에서 요레이트를 **합성**한다.
    #       vesc.yaml의 `use_servo_cmd_to_calc_angular_velocity: true`이므로
    #         steering = (servo_cmd_clipped − offset) / gain ,  yaw_rate = v·tan(steering)/L
    #       즉 odom 회전은 센서가 아니라 **클립된 명령값**에서 나온다.
    #       구 범위에선 이게 자동으로 자기정합이었다 — 수동 풀스틱(steering_scale 0.41)이
    #       servo 0.6502를 명령해도 vesc_driver가 0.6363으로 자르고, **바퀴도 odom도 같이**
    #       -0.379가 되어 어긋날 수가 없었다.
    #       servo_max를 0.6546으로 넓히면 그 클립이 사라져 odom은 무조건 -0.410을 믿는데,
    #       링키지가 거기까지 못 가면 **하드 우회전마다 요레이트가 8% 과적분**된다.
    #       → SLAM 매핑 중 헤딩이 뭉개진다(2026-07-28 실제 증상, 이 변경 직후 발생).
    #
    #    ✅ 재적용 조건: 바퀴 띄우고 servo 0.6363 vs 0.6546을 직접 명령해 **바퀴가 실제로 더
    #       꺾이는지** 확인(스톨음·링키지 걸림 없이). 확인되면 vesc.yaml과 아래 값을 같이 0.42로.
    #       탐색 도구: tools/servo_range_probe.py. 보관본: vesc.yaml.sym042.20260728 (젯슨)
    #
    # ⚠️ 이 두 값과 젯슨 vesc.yaml은 **반드시 함께** 움직여야 한다. 컨트롤러만 올리면
    #    vesc_driver가 조용히 자르고 컨트롤러는 "꺾었다"고 착각한다(요레이트 카운터스티어와
    #    조향 권한 속도 캡이 전부 실제보다 낙관적이 됨). 반대로 vesc.yaml만 넓히면 **odom이
    #    깨진다**(위 참고) — 어느 쪽 한쪽만 바꾸는 게 제일 위험하다.
    max_steering_left_arg = DeclareLaunchArgument(
        'max_steering_left', default_value='0.41',
        description='좌조향(δ>0) 물리 한계 [rad]. vesc.yaml servo_min 0.2703(→+0.441)이지만 차량 스펙 0.41로 제한'
    )
    max_steering_right_arg = DeclareLaunchArgument(
        'max_steering_right', default_value='0.379',
        description='우조향(δ<0) 물리 한계 [rad]. vesc.yaml servo_max 0.6363이 여기서 자름 — 실제 가동각'
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
        'base_max_accel', default_value='2.5',
        description='종방향 최대 가속도 한계 [m/s^2] (⚠️ sim 승리값, 실차 급가속 검증 필요)'
    )

    steering_control = common.build_control_map_node(
        odom_topic=LaunchConfiguration('odom_topic'),
        max_speed=LaunchConfiguration('max_speed'),
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=LaunchConfiguration('base_max_accel'),
        max_steering_left=LaunchConfiguration('max_steering_left'),
        max_steering_right=LaunchConfiguration('max_steering_right'),
        lookup_table_file=LaunchConfiguration('lookup_table_file'),
        # VESC 자이로가 deg/s로 발행(2026-07-19 확인) → rad/s 환산. 근거는 _control_common.py 주석.
        imu_angular_scale=common.IMU_ANGULAR_SCALE_REAL,
        # VESC 가속도계가 g로 발행(2026-07-19 소스 확인) → m/s² 환산.
        imu_linear_scale=common.IMU_LINEAR_SCALE_REAL,
        # vesc_driver_node는 IMU를 sensors/imu/raw로 발행하지만 control_map_node.cpp는
        # /imu/data를 구독하도록 하드코딩돼있어 리매핑 필요(안 하면 IMU 미수신 → 롤 ESC/
        # 요레이트 카운터스티어가 조용히 무효화됨).
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    # ── MPPI 노드 배선 제거 (2026-07-27) ─────────────────────────────────────
    # 실차 bag 3건(07-26 20:41, 07-27 19:44/20:25) 모두에서 /drive_mppi가 50Hz 목표 대비
    # **10Hz**밖에 못 나왔다 — solve 하나에 ~100ms로 실시간 예산(20ms)을 5배 초과한다.
    # 그런데 실제로는 MAP만 쓰고 있어(RB 미사용) 출력은 셀렉터가 전부 버린다.
    # 즉 젯슨 CPU만 먹는 순수 낭비였고, 07-27 19:44 bag에선 /odom·/pf/pose/odom·/scan이
    # **동시에 ~140ms 멈추는** 시스템 전체 정지가 관측됐다(과부하 징후).
    #
    # 되살리려면: 아래 블록 주석 해제 + LaunchDescription에 `mppi_control,` 복원 +
    # drive_source_selector의 algorithm_button을 99 → 5(RB)로 되돌릴 것. 세 곳 다 필요하다.
    # (시뮬 control_sim.launch.py는 그대로 둔다 — 거긴 CPU 여유가 있고 MPPI 튜닝에 쓴다)
    #
    # mppi_control = common.build_control_mppi_node(
    #     odom_topic=LaunchConfiguration('odom_topic'),
    #     max_speed=LaunchConfiguration('max_speed'),
    #     remappings=[('/imu/data', 'sensors/imu/raw')],
    # )

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
            # ⚠️ 2026-07-27: MPPI 노드를 뺐으므로 RB 토글을 **의도적으로 무효화**했다.
            # 살려두면 RB를 누르는 순간 셀렉터가 MPPI를 활성 소스로 잡는데 /drive_mppi가
            # 아무도 발행하지 않아 /drive가 침묵 → mux navigation 입력이 끊겨 차가 선다.
            # drive_source_selector는 buttons.size() <= algorithm_button_ 이면 조기 반환하므로
            # 패드 버튼 수(F710 11개)보다 큰 값을 주면 토글이 안전하게 죽는다.
            # MPPI 복원 시 5(RB)로 되돌릴 것.
            'algorithm_button': 99,
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
    #   2. vesc.yaml의 조향 캘리브레이션이 2026-07-21 확정값과 동기화돼 있어야 함:
    #        steering_angle_to_servo_offset: 0.4672   # 조향 중립(07-20 직진편향 실측 확정)
    #        steering_angle_to_servo_gain:   -0.4463  # 실측 민감도 0.00779 서보/도 기반
    #        servo_min / servo_max: 0.2703 / 0.6363   # 명령범위를 ±0.41 rad에 1:1로 맞춤
    #      ⚠️ offset은 0.4633(07-17)도 0.4533(07-21 일시적용)도 아니다. 0.4533은 "서보 0.5가
    #      조향 중립"이라는 오해로 6°를 한 번 더 뺀 값이라 좌측 편향이 생겨 원복했다 —
    #      서보의 기계 중앙(0.5)과 조향 중립(0.4672)은 원래 다르다. 경위는 WORKLOG 07-21 참고.
    #      ⚠️ gain·servo_min/max는 서보가 스토퍼를 상시 밀던 결함을 고친 것이라 offset과 달리
    #      07-21 값이 유효하다. 이 게인은 `/**:` 공유 파라미터라 vesc_to_odom에도 함께 걸린다.

    return LaunchDescription([
        *common.declare_common_args(),
        odom_topic_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        max_steering_left_arg,
        max_steering_right_arg,
        lookup_table_file_arg,
        base_max_accel_arg,
        steering_control,
        # mppi_control,   # 2026-07-27 제거 — 위 블록 참고
        drive_source_selector,
    ])
