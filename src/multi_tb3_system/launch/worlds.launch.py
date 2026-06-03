#!/usr/bin/env python3
"""
worlds.launch.py — resolves world name → absolute .world path, starts Gazebo.

Args:
  world: 'empty' (default) | 'pillars'
  gz   : 'true' | 'false' (default)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multi_tb3_system')

    world = LaunchConfiguration('world')
    gz    = LaunchConfiguration('gz')

    # e.g. 'pillars' → <pkg>/worlds/pillars.world
    world_file = PathJoinSubstitution([
        FindPackageShare('multi_tb3_system'),
        'worlds',
        PythonExpression(["'", world, "' + '.world'"]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description="World name: 'empty' or 'pillars'."),
        DeclareLaunchArgument('gz',    default_value='false',
                              description="Show Gazebo GUI ('true'/'false')."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world_file': world_file, 'gz': gz}.items(),
        ),
    ])
