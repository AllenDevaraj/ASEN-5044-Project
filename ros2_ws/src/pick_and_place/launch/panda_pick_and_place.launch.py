#!/usr/bin/env python3

"""
Launch file for Panda pick and place simulation in Gazebo with ROS2 Humble.

Author: Elena Oikonomou (adapted to ROS2)
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pick_and_place_dir = get_package_share_directory('pick_and_place')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file',
                                     default=os.path.join(pick_and_place_dir, 'worlds', 'pick_and_place.world'))
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    
    # Robot description (you'll need to adapt this based on your robot URDF/xacro)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('franka_description'),
            'robots',
            'panda_arm.urdf.xacro'
        ]),
        ' hand:=true',
        ' robot_ip:=dont-care',
        ' use_fake_hardware:=false',
        ' fake_sensor_commands:=false'
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'panda',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # Static TF publishers
    static_tf_world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0']
    )
    
    # Joint state publisher (if needed)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Object detector node
    object_detector_node = Node(
        package='pick_and_place',
        executable='object_detector',
        name='vision_object_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Controller node
    controller_node = Node(
        package='pick_and_place',
        executable='controller',
        name='panda_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # State machine node
    state_machine_node = Node(
        package='pick_and_place',
        executable='pick_and_place_state_machine',
        name='pick_and_place_state_machine',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        DeclareLaunchArgument(
            'world_file',
            default_value=world_file,
            description='Path to world file'),
        
        gazebo,
        robot_state_publisher_node,
        static_tf_world_to_base,
        spawn_entity,
        joint_state_publisher_node,
        
        # Delay object detector to allow Gazebo to start
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[object_detector_node]
            )
        ),
        
        # Add controller and state machine after object detector
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[controller_node]
            )
        ),
    ])

