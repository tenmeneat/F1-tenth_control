from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ==========================================================================
    # 대시보드 뷰어 단독 런치 (별도 터미널에서 실행)
    # ==========================================================================
    # joy_teleop_monitor(Mux)가 발행하는 /teleop_dashboard(std_msgs/String)를 받아
    # 이 터미널에서 화면을 지우고 상태 대시보드를 렌더링한다.
    # 화면 클리어(\033[2J\033[H)를 Mux 밖(이 뷰어)으로 뺐기 때문에, control_real.launch.py
    # 터미널의 다른 노드 로그가 더 이상 덮이지 않는다.
    #
    # 사용: 조이스틱 상태를 보고 싶은 "별도 터미널"에서
    #   ros2 launch f1tenth_control dashboard.launch.py
    # 전제: control_real.launch.py(=joy_teleop_monitor 포함)가 실행 중이어야 함.

    teleop_dashboard = Node(
        package='f1tenth_control',
        executable='teleop_dashboard_node',
        name='teleop_dashboard_node',
        output='screen',
    )

    return LaunchDescription([
        teleop_dashboard,
    ])
