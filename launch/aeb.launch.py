import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 공유(share) 디렉토리 경로 획득
    package_dir = get_package_share_directory('f1tenth_control')
    
    # 파라미터 YAML 파일 경로 설정
    config_file = os.path.join(package_dir, 'config', 'aeb_params.yaml')

    # AEB 노드 선언
    aeb_node = Node(
        package='f1tenth_control',
        executable='aeb_node',
        name='aeb_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        aeb_node
    ])
