from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ==========================================================================
    # 조이스틱 드라이버 단독 런치 (control_real.launch.py에서 분리)
    # ==========================================================================
    # 하드웨어 조이스틱(/dev/input/js<device_id>)을 읽어 /joy(sensor_msgs/Joy)를 발행한다.
    # joy_teleop_monitor는 이 /joy를 스틱+트리거 매핑으로 해석한다.
    # (기존 D-pad teleop 노드는 함께 띄우지 말 것.)
    # 전제: `joy` 패키지 설치(없으면 sudo apt install ros-<distro>-joy).
    #
    # 사용: ros2 launch f1tenth_control joy.launch.py [device_id:=0]
    #   실차 주행은 joy.launch.py + control_real.launch.py 를 함께 실행.

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='조이스틱 디바이스 번호 (/dev/input/js<device_id>)'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,   # 20Hz 재발행 — 트리거를 계속 당기고 있어도 /joy·/drive 명령 지속
        }]
    )

    return LaunchDescription([
        device_id_arg,
        joy_node,
    ])
