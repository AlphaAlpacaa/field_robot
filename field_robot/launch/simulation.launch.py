#!/usr/bin/env python3
"""
simulation.launch.py

Starts Gazebo with the field world, spawns the robot, sets up the ROS<->Gazebo
bridge and (when the arm is enabled) spawns the ros2_control controllers.

Examples:
    ros2 launch field_robot simulation.launch.py
    ros2 launch field_robot simulation.launch.py use_arm:=false
    ros2 launch field_robot simulation.launch.py world:=field x:=2.0 y:=1.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('field_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    use_arm = LaunchConfiguration('use_arm')
    ur_type = LaunchConfiguration('ur_type')
    x, y, z, yaw = (LaunchConfiguration(k) for k in ('x', 'y', 'z', 'yaw'))
    rviz = LaunchConfiguration('rviz')

    args = [
        DeclareLaunchArgument('world', default_value='field'),
        DeclareLaunchArgument('use_arm', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    world_path = PathJoinSubstitution([pkg, 'worlds', [world, '.sdf']])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 4 ', world_path]}.items(),
    )

    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_arm:=', use_arm,
                 ' ur_type:=', ur_type]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'field_robot',
                   '-x', x, '-y', y, '-z', z, '-Y', yaw],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )

    # ---- ros2_control controllers (arm only) ----
    # Spawned after the robot exists in Gazebo, otherwise controller_manager
    # is not up yet.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_arm),
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_arm),
    )

    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn,
                      on_exit=[joint_state_broadcaster]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster,
                      on_exit=[arm_controller]))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        after_spawn,
        after_jsb,
        rviz_node,
    ])
