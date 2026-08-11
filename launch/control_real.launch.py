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

    # 직선 최대 속도 [m/s] — 2026-07-30 5.0→8.0 상향(사용자 결정, 고속 주행 세팅).

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='8.0',
        description='control_map_node 직선 최대 속도 [m/s]'
    )

     # 곡선 최대 횡가속도 [m/s^2] — 2026-07-30 5.0→7.0 상향(사용자 결정, 고속 주행 세팅).     
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='7.0',
        description='코너 그립 클램프 a_lat [m/s^2] (LUT 그립 피크 ~6.7 이내)'
    )

    # ── 좌우 조향 한계 (2026-07-28, 실차 전용) ──
    # 젯슨 vesc.yaml의 servo_min 0.2703 / servo_max 0.6363 은 servo **0.4533 중심 ±0.1830**
    # (= ±0.41 rad)으로 계산된 낡은 값이다. 이후 기계적 센터를 맞추며 트림 offset이
    # **0.4672로 의도적으로 이동**됐는데 servo_min/max는 갱신되지 않아, 실제 가동각이
    # 좌 +0.441 / 우 **-0.379(잘림)**로 갈려 있었다. 07-27 bag(run_0727_203040)에서
    # servo 0.6502가 발행돼 vesc_driver의 servo_limit에 잘린 것이 실제로 관측된다.
    #

    #
    # ⚠️ 명령각이 비대칭인 것이 **맞다.** 좌측 링키지가 우측보다 servo를 21.9% 더 써야 같은
    #    각이 나와서(좌 도달비 0.756 / 우 0.922), 명령을 비대칭으로 줘야 **실제 바퀴 각이
    #    좌우 23.5°로 대칭**이 된다. 조향 권한 캡은 min(좌,우)=0.4320(더 선형인 우측)을 쓰므로
    #    정확한 쪽이 기준이 된다 → δ_avail = 0.85 × 0.4320 × 1.0 = 0.367 rad.
    #    ℹ️ 더 근본적으로는, 서보 중립 트림(0.4672)이 가동범위 [0.23, 0.66]의 정중앙(0.445)이
    #       아니라서 중립 기준 가용 스트로크가 좌 0.2372 / 우 0.1928로 애초에 다르다.
    #       비대칭 명령 한계는 그 스트로크를 정확히 다 쓰기 위한 계산 결과다.
    #

    #
    # 🔴 이 값과 젯슨 vesc.yaml의 servo_min/servo_max는 **반드시 함께** 움직인다.
    #    컨트롤러만 넓히면 vesc_driver가 자르는데, 그때 **매 메시지마다 클리핑 로그가 찍히면서
    #    VESC 시리얼을 굶겨 /sensors/imu가 50Hz → 39Hz(최대 0.33s 공백)로 떨어지고,
    #    imu_timeout(0.2s)를 넘겨 odom이 "조향명령 기반" 구식 경로로 폴백한다**
    #    (2026-07-31 실측 재현). 그 경로는 선회 중 헤딩을 +36~39% 과적분한다.
    #    반대로 vesc.yaml만 넓히면 컨트롤러가 못 내는 각을 안 쓰게 되어 손해만 본다.
    #    → 08-03에 ackermann_to_vesc가 발행 전에 스스로 [servo_min, servo_max]로 클램프하도록
    #      고쳐서, 혹시 어긋나도 driver의 클리핑 로그 폭주 경로는 타지 않는다(안전망).

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

    # 비워두면 기존 폴백 순서(f1tenth_control share → steering_lookup share)로 로드.
    # tools/lut_calibrator가 만든 보정 LUT를 임시로 켜보려면 그 출력 경로를 지정할 것
    # (예: $HOME/f1tenth_lut_calibration/LUT_calibrated.csv). 검증 끝나면 그 파일을
    # control_code/LUT_calibrated.csv에 덮어써서 디폴트로 확정하는 편이 낫다(CLAUDE.md 참고).
    lookup_table_file_arg = DeclareLaunchArgument(
        'lookup_table_file', default_value='',
        description='Steering LUT CSV 경로 (비워두면 기본 폴백 사용, 캘리브레이션 결과 적용 시 지정)'
    )

    # 종방향 최대 가속도 한계 [m/s^2]. base_max_decel은 sim/real 동일이라 _control_common.py에 공용.
    #
    # 2026-07-31 2.5 → 3.5. 전방-후방 패스 시뮬(ifac_track_v2, a_lat 6.0 / decel 2.5 / v_max 8.0):
    #   랩타임 11.97 → 11.63 s (−0.34 s, −2.8%), 최고 도달속 6.93 → 7.44 m/s.
    #   ℹ️ `max_speed`는 8.0이지만 실제 도달은 7.4다 — 트랙이 46.9 m(최장 직선 13.3 m)라
    #      8.0에 닿기 전에 다음 코너 제동이 시작된다. **max_speed를 더 올려도 무의미하고**,
    #      최고속을 올리는 유일한 레버가 이 가속도다.
    #   ℹ️ 더 큰 레버는 제동 쪽에 있다 — 젯슨 brake_max_current 8.0A ≈ 4.8 m/s²가 하드웨어
    #      한계인데 brake_gain은 −2.5 목표로만 튜닝돼 있다. 제동을 4.8까지 쓰면 −0.62 s,
    #      가속·제동 둘 다 올리면 −1.32 s(−11%)다. 단 brake_gain과 prebrake_decel은 한 쌍.
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
        # VESC 가속도계가 g로 발행(2026-07-19 소스 확인) → m/s² 환산.
        imu_linear_scale=common.IMU_LINEAR_SCALE_REAL,
        imu_angular_scale=common.IMU_ANGULAR_SCALE_REAL,
        # vesc_driver_node는 IMU를 sensors/imu/raw로 발행하지만 control_map_node.cpp는
        # /imu/data를 구독하도록 하드코딩돼있어 리매핑 필요(안 하면 IMU 미수신 → 롤 ESC/
        # 요레이트 카운터스티어가 조용히 무효화됨).
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    # ── MPPI 배선 완전 제거 (2026-08-01) ─────────────────────────────────────
    # 실차 bag 3건(07-26 20:41, 07-27 19:44/20:25) 모두에서 /drive_mppi가 50Hz 목표 대비
    # 10Hz밖에 못 나왔고(solve ~100ms, 실시간 예산 20ms의 5배), 실제로는 MAP만 쓰고 있어
    # 출력은 버려지는 순수 낭비였다(07-27부터 이 런치에서 배선 제외). 대회가 한 달 앞으로
    # 다가와 MAP 하나에만 집중하기로 하고 2026-08-01에 MPPI 노드/솔버/셀렉터 로직 전체를
    # 코드베이스에서 제거했다 — 되살리려면 git 이력에서
    # control_code/control_mppi_{node,solver_cpu.cpp,solver_gpu.cu},
    # include/f1tenth_control/mppi_{gpu,types_gpu}.hpp,
    # _control_common.py의 build_control_mppi_node()를 함께 복원할 것.
    #

    drive_source_selector = Node(
        package='f1tenth_control',
        executable='drive_source_selector',
        name='drive_source_selector',
        output='screen',
    )

    # ackermann_to_vesc_node도 이 launch에 없다(2026-07-17 제거). f110(f1tenth_stack)이 이미
    # 자체 ackermann_to_vesc_node를 띄우고, 그 입력('ackermann_drive')은 자체 ackermann_mux가
    # 'teleop'(drive_mode_manager 수동, 우선순위100)과 'drive'(navigation, 우선순위10 — 위
    # 셀렉터 출력)를 중재해 만든다. 우리가 또 띄우면 같은 VESC 명령 토픽에 중복 발행되어
    # 경합(조향 덜컹거림)한다.


    sector_pub_node = common.build_sector_pub_node()

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
        sector_pub_node,
    ])
