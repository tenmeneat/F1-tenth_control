from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    # ==========================================================================
    # 대시보드 뷰어 단독 런치 (별도 터미널에서 실행) — sim / real 두 모드
    # ==========================================================================
    # mode:=sim (기본) — 시뮬용. joy_teleop_monitor가 발행하는 완성된
    #   /teleop_dashboard(std_msgs/String)를 그대로 그리는 뷰어(teleop_dashboard_node).
    #   전제: control_sim.launch.py(=joy_teleop_monitor 포함)가 실행 중이어야 함.
    #
    # mode:=real — 실차(젯슨) 원격 모니터링. 실차엔 joy_teleop_monitor가 없어
    #   /teleop_dashboard가 없으므로, realcar_dashboard_node가 젯슨의 원시 토픽
    #   (/drive_mode, /mppi_active, /drive, odom, /joy)을 직접 구독해 "우리 컴에서"
    #   조립·렌더링한다 → 젯슨 렌더 연산 0.
    #
    # ⚠️ 무선 원격 뷰의 네트워크 전제 (Fast DDS Discovery Server) — 2026-07-17 미완:
    #   무선 AP가 DDS 기본 멀티캐스트 디스커버리를 막고, 우리 컴·젯슨 둘 다 멀티홈이라
    #   (우리 컴 10.1.1.24+10.0.3.1 / 젯슨 10.1.1.3+192.168.0.15) 유니캐스트 피어만으론
    #   디스커버리가 안 붙는다. 견고한 해법은 Discovery Server(멀티캐스트 불필요):
    #     1) 서버 기동(젯슨 또는 상시 노드): fastdds discovery -i 0 -l 10.1.1.3 -p 11811
    #     2) 젯슨 bringup + 이 대시보드 양쪽에서:  export ROS_DISCOVERY_SERVER="10.1.1.3:11811"
    #   이 env만 걸려 있으면 이 런치는 그대로 붙는다(런치가 DDS를 따로 설정하지 않음).
    #   ※ 젯슨 bringup에 env를 넣는 건 팀 공용 스택 변경이라 차량 세팅 시 함께 진행.
    #   ※ 유선(피트)에선 멀티캐스트가 되므로 env 없이 mode:=real 만으로 붙는다.
    #
    # 사용(우리 컴, 별도 터미널):
    #   ros2 launch f1tenth_control dashboard.launch.py                 # 시뮬
    #   ros2 launch f1tenth_control dashboard.launch.py mode:=real       # 실차 원격
    #   ros2 launch f1tenth_control dashboard.launch.py mode:=real odom_topic:=/pf/pose/odom

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description="sim=완성문자열 뷰어(/teleop_dashboard), real=젯슨 원시토픽 원격 대시보드"
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/pf/pose/odom',
        description='real 모드 실측 속도 odom 토픽 (실차 파티클필터)'
    )

    # sim: 기존 문자열 뷰어
    sim_viewer = Node(
        package='f1tenth_control',
        executable='teleop_dashboard_node',
        name='teleop_dashboard_node',
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim'),
    )

    # real: 젯슨 원시토픽을 로컬에서 조립·렌더링 (DDS 연결은 ROS_DISCOVERY_SERVER env로, 위 참고)
    real_viewer = Node(
        package='f1tenth_control',
        executable='realcar_dashboard_node',
        name='realcar_dashboard_node',
        output='screen',
        parameters=[{'odom_topic': LaunchConfiguration('odom_topic')}],
        condition=LaunchConfigurationEquals('mode', 'real'),
    )

    return LaunchDescription([
        mode_arg,
        odom_topic_arg,
        sim_viewer,
        real_viewer,
    ])
