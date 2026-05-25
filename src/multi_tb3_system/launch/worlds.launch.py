#!/usr/bin/env python3
"""
worlds.launch.py
================
Modular launch file — World file selection and Gazebo startup.

Resolves the world name (``empty`` / ``pillars``) to an absolute file path
inside this package's ``worlds/`` directory, then includes ``gazebo.launch.py``
to start the simulation.

Use this file directly when you want to bring up Gazebo independently of
the robot spawn chain (e.g. for world inspection / debugging).

Args:
  world : 'empty' | 'pillars'  (default: 'empty')
  gz    : 'true'  | 'false'    (default: 'false')

Standalone usage::

  ros2 launch multi_tb3_system worlds.launch.py world:=pillars gz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


# Allowed world names → maps to filename (without .world extension)
VALID_WORLDS = ('empty', 'pillars')


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multi_tb3_system')

    # ── Args ──────────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description=(
            "Gazebo world to load. "
            f"Allowed: {', '.join(VALID_WORLDS)}."
        ),
    )
    gz_arg = DeclareLaunchArgument(
        'gz',
        default_value='false',
        description="Show Gazebo GUI window ('true' / 'false').",
    )

    world = LaunchConfiguration('world')
    gz    = LaunchConfiguration('gz')

    # ── Resolve world file path at launch time ────────────────────────────────
    # PathJoinSubstitution + PythonExpression appends '.world' to the world name.
    world_file = PathJoinSubstitution([
        FindPackageShare('multi_tb3_system'),
        'worlds',
        PythonExpression(["'", world, "' + '.world'"]),
    ])

    # ── Include Gazebo launch ─────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'gz':         gz,
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        gz_arg,
        gazebo_launch,
    ])
