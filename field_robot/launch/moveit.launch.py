#!/usr/bin/env python3
"""
moveit.launch.py

Starts move_group and an RViz session configured for motion planning, on top of
the already running Gazebo simulation.

Important: the robot description here is OUR xacro (field_robot/urdf), not the
copy that the MoveIt Setup Assistant generated. Only the semantic description
(SRDF), kinematics and controller mappings come from the generated package.
Keeping a single source of truth for the URDF avoids the classic "RViz and
Gazebo disagree about the robot" problem.

Usage:
    # terminal 1
    ros2 launch field_robot simulation.launch.py
    # terminal 2
    ros2 launch field_robot moveit.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_robot = get_package_share_directory('field_robot')
    pkg_moveit = get_package_share_directory('field_robot_moveit_config')

    use_rviz = LaunchConfiguration('rviz')

    moveit_config = (
        MoveItConfigsBuilder('field_robot', package_name='field_robot_moveit_config')
        .robot_description(
            file_path=os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro'),
            mappings={'use_arm': 'true', 'ur_type': 'ur5e'},
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_moveit, 'config', 'field_robot.srdf'))
        .trajectory_execution(
            file_path=os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml'))
        .planning_scene_monitor(
            publish_robot_description=False,      # robot_state_publisher already does this
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True,
             # give Gazebo some slack: simulated execution is rarely exact
             'trajectory_execution.allowed_execution_duration_scaling': 2.0,
             'trajectory_execution.allowed_start_tolerance': 0.05},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        move_group,
        rviz,
    ])
