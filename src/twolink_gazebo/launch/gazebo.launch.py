import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Declare launch argument for use_gazebo
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Flag to enable or disable Gazebo'
    )
    
    use_gazebo = LaunchConfiguration('use_gazebo')

    # Gazebo world configuration
    world_file_name = 'table_world.world'
    world_path = PathJoinSubstitution([
        FindPackageShare('twolink_gazebo'),
        'worlds',
        world_file_name
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                  'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_path}.items(),
        condition=IfCondition(use_gazebo)  # Execute only if use_gazebo is true
    )

    # Robot description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('twolink_gazebo'), 'urdf', 'twolinkbot.urdf.xacro'
        ])
    ])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    robot_spawn_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'twolinkbotO',
                    '-x', '0.4',
                    '-y', '0.05',
                    '-z', '1.015'],
        output='screen',
        condition=IfCondition(use_gazebo)  # Execute only if use_gazebo is true
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen',
        condition=IfCondition(use_gazebo)  # Execute only if use_gazebo is true
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Return the LaunchDescription
    return LaunchDescription([
        use_gazebo_arg,
        gazebo_launch,
        robot_state_publisher_node,
        robot_spawn_node,
        load_joint_trajectory_controller,
        rviz_node
    ])
