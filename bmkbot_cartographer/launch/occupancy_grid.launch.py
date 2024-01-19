from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time = False
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # Cartographer occupancy grid 노드를 설정합니다.
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    return LaunchDescription([
        # 런치 파일에 사용할 인자들을 정의합니다.
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='발행되는 점유 격자의 그리드 셀 해상도'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid 발행 주기'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true이면 시뮬레이션 (Gazebo) 시계 사용'),

        cartographer_occupancy_grid_node,
    ])