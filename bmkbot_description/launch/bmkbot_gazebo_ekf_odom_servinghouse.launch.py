import launch


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
 
    default_model_dir = PathJoinSubstitution(
        [
            FindPackageShare('bmkbot_description'),
            'urdf',
            'bmkbot_gazebo.urdf.xacro'
        ]
    )
    ekf_config_file_path = PathJoinSubstitution(
        [
            FindPackageShare('bmkbot_description'),
            'config',
            'ekf.yaml'
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('bmkbot_description'),
            'rviz',
            'display.rviz'
        ]
    )
    
    world_dir= PathJoinSubstitution(
        [
            FindPackageShare('bmkbot_description'),
            'world',
            'maze.world'
        ]
    )
    

    # robot_state_publisher를 실행하는 노드를 설정합니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]

    )

    # joint_state_publisher를 실행하는 노드를 설정합니다.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # rviz를 실행하는 노드를 설정합니다.
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)),
    
    # gazebo를 실행하여 월드를 불러옵니다.
    spawn_entity = Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'bmkbot', '-topic', 'robot_description', '-x', '0.5', '-y', '0.5', '-z', '0.01'],
        output='screen'
    )
    # yaml 파일을 불러와서 ekf 노드를 실행합니다.
    # ekf_template.yaml 파일은 bmkbot_description/config/ekf_template.yaml에 있습니다.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config_file_path, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # 런치 파일에 사용할 인자들을 정의합니다.
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation'),
            
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_dir,
            description='Absolute path to robot urdf file'),

        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_file,
            description='Absolute path to rviz config file'),
            
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'),
        

        # gazebo를 실행합니다.
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_dir],
            output='screen'),
        
        # 위에서 정의한 노드들을 실행합니다.
        # joint_state_publisher_node,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        spawn_entity,
        ekf_node,
        # rviz_node,
    ])
