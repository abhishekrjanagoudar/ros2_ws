#!/usr/bin/env python3
"""
world_obstacles.launch.py — convenience launcher for the pillars world.
Delegates to multi_robot.launch.py with world:=columns.

Usage:
  ros2 launch multi_tb3_system world_obstacles.launch.py
  ros2 launch multi_tb3_system world_obstacles.launch.py use_rviz:=true use_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multi_tb3_system')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description="Show RViz2 ('true'/'false')."),
        DeclareLaunchArgument('use_gui',  default_value='true',
                              description="Show Gazebo GUI ('true'/'false')."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'multi_robot.launch.py')
            ),
            launch_arguments={
                'world':    'columns',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_gui':  LaunchConfiguration('use_gui'),
            }.items(),
        ),
    ])
