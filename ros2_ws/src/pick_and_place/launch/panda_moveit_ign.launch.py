#!/usr/bin/env python3

"""Launch MoveIt2 for the Gazebo/Ignition Panda simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """Bring up MoveIt2 configured to connect to gz_ros2_control Panda controllers."""

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz pre-configured for MoveIt visualization.'
    )

    moveit_share = get_package_share_directory('moveit_resources_panda_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={'ros2_control_hardware_type': 'mock_components'}
        )
        .robot_description_semantic(file_path='config/panda.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )

    rviz_config = os.path.join(moveit_share, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        move_group_node,
        rviz_node,
    ])
