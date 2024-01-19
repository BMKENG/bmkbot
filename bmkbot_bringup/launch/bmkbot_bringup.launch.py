#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    mcu_node = Node(
        package='bmkbot_bringup',
        executable='bmkbot_serial',
        output='screen',
    )
    #  LiDAR 노드
    # lidar_node = Node(
    base_to_laser_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "-0.045",
            "0",
            "0.3",
            "3.141592",
            "3.141592",
            "3.141592",
            "base_footprint",
            "laser_frame",
        ],
        output="screen",
    )
    g2lidar_prefix = get_package_share_directory("ydlidar_ros2_driver")
    start_g2lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g2lidar_prefix, "launch", "ydlidar_launch.py"))
    )
    start_robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bmkbot_description"), "launch", "bmkbot.launch.py"

            )
        )
    )
    return LaunchDescription([
        mcu_node,
        start_robot_description_cmd,
        start_g2lidar_cmd,
        base_to_laser_publisher,
    ])
