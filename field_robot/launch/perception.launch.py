#!/usr/bin/env python3
"""
perception.launch.py

Runs the tomato detector twice, with different settings for different jobs.

  scanner  body camera, wide and forgiving. Finds fruit several metres away so
           the robot knows which plants are worth approaching. A tomato at 4 m
           covers only a handful of pixels, so the thresholds are low and the
           positions it reports are rough.

  picker   wrist camera, close and strict. Only looks within a metre and wants
           a big, round blob, which at that distance means the fruit is clearly
           in view. These are the positions the grasp is actually built from.

    ros2 launch field_robot perception.launch.py
    ros2 launch field_robot perception.launch.py scanner_range:=12.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    scanner_range = LaunchConfiguration('scanner_range')
    picker_range = LaunchConfiguration('picker_range')

    args = [
        DeclareLaunchArgument('scanner_range', default_value='8.0'),
        DeclareLaunchArgument('picker_range', default_value='1.0'),
    ]

    scanner = Node(
        package='field_robot',
        executable='tomato_detector.py',
        name='tomato_scanner',
        output='screen',
        parameters=[{
            'camera': 'front_camera',
            'target_frame': 'base_footprint',
            'min_area': 12,
            'min_circularity': 0.45,
            'max_range': scanner_range,
            'use_sim_time': True,
        }],
        remappings=[
            ('/tomatoes/poses', '/tomatoes/scan_poses'),
            ('/tomatoes/markers', '/tomatoes/scan_markers'),
            ('/tomatoes/debug_image', '/tomatoes/scan_debug'),
        ],
    )

    picker = Node(
        package='field_robot',
        executable='tomato_detector.py',
        name='tomato_picker_eye',
        output='screen',
        parameters=[{
            'camera': 'wrist_camera',
            'target_frame': 'base_footprint',
            'min_area': 250,
            'min_circularity': 0.6,
            'max_range': picker_range,
            'use_sim_time': True,
        }],
        remappings=[
            ('/tomatoes/poses', '/tomatoes/pick_poses'),
            ('/tomatoes/markers', '/tomatoes/pick_markers'),
            ('/tomatoes/debug_image', '/tomatoes/pick_debug'),
        ],
    )

    return LaunchDescription(args + [scanner, picker])
