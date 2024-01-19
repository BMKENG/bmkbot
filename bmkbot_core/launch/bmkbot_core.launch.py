from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # robot_state_publisher를 실행하는 노드를 설정합니다.
    navigator_node = Node(
        package='bmkbot_core',
        executable='navigator_node',
        name='navigator_node',
        output='screen'
    )
    joystick_node = Node(
        package='bmkbot_core',
        executable='joystick_node',
        name='joystick_node',
        output='screen'
    )
    

    return LaunchDescription([
        navigator_node,
        joystick_node
    ])
