from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # robot_state_publisher를 실행하는 노드를 설정합니다.
    bmkbot_gui_node = Node(
        package='bmkbot_gui',
        executable='bmkbot_gui',
        name='bmkbot_gui_node',
        output='log'
    )
    bmkbot_client_gui_node = Node(
        package='bmkbot_gui',
        executable='bmkbot_client_gui',
        name='bmkbot_client_gui_node',
        output='log'
    )

    return LaunchDescription([
        bmkbot_gui_node,
        bmkbot_client_gui_node,
    ])
