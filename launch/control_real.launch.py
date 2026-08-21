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
        'max_lateral_accel', default_value='6.0',
        description='코너 그립 클램프 a_lat [m/s^2]. 조향 생성의 a_cmd 상한도 겸한다(②-p). '
                    '⚠️ 2026-08-19에 7.0으로 올라가 있던 것을 6.0으로 되돌렸다: 현재 라인은 '
                    'a_lat을 최대 6.03만 요구해서 그립 캡이 172점 중 0~1점만 구속한다 → '
                    '6.0 -> 7.0의 랩타임 이득이 정확히 0.000 s다. 반면 조향 a_cmd 클램프는 '
                    'mla * steering_accel_margin(1.15)이라 6.90 -> 8.05로 올라가는데, '
                    '마모 타이어 실측 그립 포락선이 p95 6.45 / max 7.40(0819 8랩 무사고)이라 '
                    '8.05는 타이어가 못 내는 값이다. 6.90이 라인 요구 6.03과 그립 7.40 사이에 '
                    '정확히 들어가는 유일한 구간이다'
    )

    max_steering_left_arg = DeclareLaunchArgument(
        'max_steering_left', default_value='0.410',
        description='좌조향(δ>0) 명령 한계 [rad] = 실제 바퀴 각. 젯슨 vesc.yaml 의 '
                    'steering_angle_to_servo_gain_left(-0.5785) + offset(0.4672) 과 한 쌍'
    )
    max_steering_right_arg = DeclareLaunchArgument(
        'max_steering_right', default_value='0.361',
        description='우조향(δ<0) 명령 한계 [rad] = 실제 바퀴 각. 젯슨 vesc.yaml 의 '
                    'steering_angle_to_servo_gain_right(-0.4702) + offset 과 한 쌍. '
                    '2026-08-19: 젯슨 offset이 0.4672 -> 0.49로 바뀌어 우조향이 '
                    '(0.66-0.49)/0.4702 = 0.3615 rad(20.72도)에서 servo_max에 걸린다. '
                    '컨트롤러가 0.410을 계획하면 낼 수 없는 각을 계획하는 것이라 0.361로 맞췄다. '
                    '이 트랙 랩타임 영향 0(조향 권한 캡이 0.410/0.361 어느 쪽에서도 172점 중 '
                    '0점만 구속, 실측 최대 명령각 0.287~0.291 = 여유 19.5%). '
                    '⚠️ **젯슨 offset을 바꾸면 이 값을 반드시 같이 고칠 것**: '
                    'max_steering_right = (servo_max 0.66 - offset) / gain_right 0.4702. '
                    'offset 0.4672 -> 0.410 / 0.48 -> 0.383 / 0.49 -> 0.361'
    )

    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='4.1',
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
        imu_linear_scale=common.IMU_LINEAR_SCALE_REAL,
        imu_angular_scale=common.IMU_ANGULAR_SCALE_REAL,
        # vesc_driver_node는 IMU를 sensors/imu/raw로 발행하지만 control_map_node.cpp는
        # /imu/data를 구독하도록 하드코딩돼있어 리매핑 필요(안 하면 IMU 미수신 → 롤 ESC/
        # 요레이트 카운터스티어가 조용히 무효화됨).
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    cruise_controller = common.build_cruise_controller_node(
        max_speed=LaunchConfiguration('max_speed')
    )

    drive_source_selector = Node(
        package='f1tenth_control',
        executable='drive_source_selector',
        name='drive_source_selector',
        output='screen',
    )

    sector_learner_node = common.build_sector_learner_node()

    return LaunchDescription([
        # 🔵 2026-08-19: 실차 기본값 true → **false**. 제거가 아니라 **결선 구성에서만 끈** 것이고
        #    `sector_scale_enable:=true` 한 인자로 되돌아온다. 근거는 CLAUDE.md ②-x:
        #     ① 지금 켜 봐야 **수학적으로 no-op**이다. `config/sectors.yaml`의 track_length
        #        35.592가 현재 라인 41.282와 안 맞아 컨트롤러가 표를 폐기하고, 학습기의
        #        autobuild가 scale 전부 1.0으로 다시 만들어 준다 →
        #        `w.mla = max_lateral_accel × 1.0` = 끈 것과 **비트 단위로 동일**.
        #     ② 그런데 켜 두면 `sector_learner` 노드·토픽·데드맨·`/state` 게이팅이 결선
        #        구성에 그대로 남아, 이득 0인 채로 조용히 실패할 수 있는 표면만 늘린다.
        #     ③ explore는 **연습 전용**이고 아직 실차 순이득이 확인된 적이 없다(0814 두 판
        #        순효과 0). 연습에서 쓸 때만 인자로 켤 것.
        *common.declare_common_args(
            sector_scale_enable_default='false',
            hfi_launch_guard_enable_default='true'),
        odom_topic_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        max_steering_left_arg,
        max_steering_right_arg,
        base_max_accel_arg,
        cruise_controller,
        steering_control,
        drive_source_selector,
        sector_learner_node,
    ])
