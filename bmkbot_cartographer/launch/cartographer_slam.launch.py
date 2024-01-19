#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    use_sim = False

    # Cartographer 설정 파일이 있는 디렉토리 경로 설정
    cartographer_config_dir = PathJoinSubstitution(
        [
            FindPackageShare('bmkbot_cartographer'),
            'config',
        ]
    )
    
    # Cartographer 설정 파일의 기본 이름 설정
    configuration_basename = LaunchConfiguration('configuration_basename')

    # Occupancy Grid 맵의 해상도 설정
    resolution = LaunchConfiguration('resolution')

    # RViz 설정 파일 경로 설정
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('bmkbot_cartographer'),
            'rviz',
            'cartographer.rviz'
        ]
    )
    
    # Cartographer 노드 실행 설정
    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])
    
    # Cartographer Occupancy Grid 노드 실행 설정
    cartographer_ros_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            arguments=['-resolution', resolution])
    
    # RViz 노드 실행 설정
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz))
    
    return LaunchDescription([
        # 런치 파일에 사용할 인자들을 정의합니다.
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='rviz2를 실행할지 여부'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Gazebo 시뮬레이션에서 로봇을 실행할지 여부'),

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Cartographer 설정 파일이 있는 디렉토리 경로'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer_2d.lua',
            description='Cartographer 설정 파일의 기본 이름'),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy Grid 맵의 그리드 셀 해상도'),

        cartographer_node,
        cartographer_ros_node,
        rviz_node,
    ])
