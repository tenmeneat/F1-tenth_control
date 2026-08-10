import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import _control_common as common


def generate_launch_description():
    # teleop(조이스틱 수동/자율/E-stop Mux)은 이 저장소 담당이 아니다(2026-07-29 제거) —
    # 실차는 f1tenth_stack(drive_mode_manager + ackermann_mux), 시뮬은 Mux 없이
    # drive_source_selector가 자율 명령(/drive_autonomous)을 /drive로 직결한다.
    # 기동 즉시 자율주행이며 별도 인자가 필요 없다(구 force_autonomous 폐지).
    # (MPPI 컨트롤러 노드는 2026-08-01 제거 — 대회 준비 기간 동안 MAP에만 집중)

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

    # 직선 최고속도 캡 [m/s]. 2026-07-22까지 이 파일에 12.0으로 하드코딩돼 있어
    # `max_speed:=X`가 시뮬에서 조용히 무시됐다(real에는 인자로 있었음). MPPI 스윕에서
    # 속도를 낮춰 원인분리를 하려다 발견 — 두 컨트롤러 모두에 같은 값이 가야 비교가 성립한다.
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='12.0',
        description='직선 최고속도 캡 [m/s] (control_map_node의 max_speed)'
    )

    steering_control = common.build_control_map_node(
        odom_topic='/ego_racecar/odom',
        max_speed=LaunchConfiguration('max_speed'),
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=LaunchConfiguration('base_max_accel'),
        # 시뮬 차량 모델은 좌우 대칭이라 ±0.41 그대로. 실차의 서보 트림 중심 어긋남
        # (좌 +0.441 / 우 -0.379, control_real.launch.py 주석 참고)은 하드웨어 특성이므로
        # 시뮬로 옮기지 않는다 — 옮기면 시뮬 랩타임 기준선이 하드웨어 결함을 따라 흔들린다.
        max_steering_left=0.41,
        max_steering_right=0.41,
        # 시뮬 브릿지는 linear_acceleration을 채우지 않아(0) 보정 불필요.
        imu_linear_scale=common.IMU_LINEAR_SCALE_SIM,
        imu_angular_scale=common.IMU_ANGULAR_SCALE_SIM,
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

    # 자율 명령(/drive_autonomous) → /drive 포워딩(gym_bridge가 구독).
    # (2026-08-01: MPPI 제거로 MAP/MPPI 셀렉터 기능도 걷어냄 — 순수 포워더)
    drive_source_selector = Node(
        package='f1tenth_control',
        executable='drive_source_selector',
        name='drive_source_selector',
        output='screen',
    )

    return LaunchDescription([
        *common.declare_common_args(),
        base_max_accel_arg,
        max_lateral_accel_arg,
        max_speed_arg,
        sim_imu_bridge,
        steering_control,
        drive_source_selector,
    ])
