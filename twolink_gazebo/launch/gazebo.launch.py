import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription



def generate_launch_description():
    
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
        launch_arguments={'world': world_path}.items()  # 월드 파일을 인자로 전달
    )

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('twolink_gazebo'), 'urdf', 'twolinkbot.urdf.xacro'
        ])
    ])

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
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '1.015'
                    ],
        output='screen'
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )
    
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'twolink_gazebo/config/rviz.rviz'],
        output='screen'
    )
    
   
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        robot_spawn_node,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        controller_manager_node,
        rviz_node
    ])