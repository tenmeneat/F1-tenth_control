from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ==========================================================================
    # Odom 거리 스케일 실측 보정 노드 단독 런치 (별도 터미널, 우리 컴에서 실행)
    # ==========================================================================
    # (2026-08-01: 실차 원격 모니터링용 realcar_dashboard_node는 제거됨 — 라이브 모니터링은
    #  더 이상 이 저장소가 담당하지 않고, tools/bag_analyzer/webapp의 rosbag 오프라인
    #  분석으로 대체한다. 이 런치는 odom_calib_node 하나만 남는다. 구 파일명은
    #  dashboard.launch.py, mode:=calib 였다.)
    #
    # odom_calib_node — "명령 주고 자로 재기" 테스트를 자동화하는 관찰 전용 노드.
    # 젯슨의 원시 토픽을 직접 구독해 "우리 컴에서" 조립·계산한다(젯슨 렌더 연산 0).
    # /drive 미발행이라 주행 중 켜둬도 제어에 영향 없음.
    # 한 번의 직선 주행에서 독립적인 거리 3개를 동시 적분해 어느 게인이 틀렸는지 분리한다:
    #   ① 명령 ∫/drive.speed dt ② 휠 ∫odom vx dt(VESC erpm_to_speed 경로)
    #   ③ 맵 |끝−시작|(MCL 스캔매칭 — 휠과 독립)
    # 자세한 절차/주의는 CLAUDE.md "odom_calib_node" 항목 참고.
    #
    # ⚠️ 무선 원격 뷰의 네트워크 전제 (Fast DDS Discovery Server):
    #   무선 AP가 DDS 기본 멀티캐스트 디스커버리를 막고, 우리 컴·젯슨 둘 다 멀티홈이라
    #   유니캐스트 피어만으론 디스커버리가 안 붙는다. 젯슨을 Discovery Server로 세팅한
    #   뒤 우리 컴에서 `export ROS_DISCOVERY_SERVER="10.1.1.3:11811"` 후 실행하면 붙는다.
    #   유선(피트)에선 멀티캐스트가 되므로 env 없이도 붙는다.
    #
    # 사용(우리 컴, 별도 터미널):
    #   ros2 launch f1tenth_control odom_calib.launch.py
    #   ros2 launch f1tenth_control odom_calib.launch.py odom_topic:=/pf/pose/odom

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터)'
    )

    odom_calib = Node(
        package='f1tenth_control',
        executable='odom_calib_node',
        name='odom_calib_node',
        output='screen',
        parameters=[{'odom_topic': LaunchConfiguration('odom_topic')}],
    )

    return LaunchDescription([
        odom_topic_arg,
        odom_calib,
    ])
