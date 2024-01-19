from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # robot_state_publisher를 실행하는 노드를 설정합니다.
    tts_node = Node(
        package='bmkbot_io',
        executable='tts_server',
        name='tts_server',
        output='screen'
    )

    return LaunchDescription([
        tts_node,
    ])
