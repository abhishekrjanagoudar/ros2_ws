#!/usr/bin/env python3
"""
gazebo.launch.py
================
Modular launch file — Gazebo Sim lifecycle management.

Responsibilities:
  - Start the Gazebo Sim server  (headless, -s)
  - Optionally start the Gazebo GUI client (-g), controlled by ``gz`` arg
  - Bridge the global /clock topic to ROS

Args (all passed from robot.launch.py):
  world_file  : absolute path to the .world SDF file
  gz          : 'true' | 'false'  — show Gazebo GUI window
  use_sim_time: 'true' | 'false'  — consumed downstream, not used here

Standalone usage (debug only)::

  ros2 launch multi_tb3_system gazebo.launch.py \\
    world_file:=/path/to/empty.world gz:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # ── Args ──────────────────────────────────────────────────────────────────
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        description='Absolute path to the Gazebo world SDF file.',
    )
    gz_arg = DeclareLaunchArgument(
        'gz',
        default_value='false',
        description="Launch Gazebo GUI client ('true' / 'false').",
    )

    world_file = LaunchConfiguration('world_file')
    gz         = LaunchConfiguration('gz')

    # ── Gazebo Sim server (always headless; GUI is a separate process) ────────
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r  : run simulation immediately on start
            # -s  : server only (no GUI)
            # -v2 : verbosity level 2 (warnings + errors)
            'gz_args':        ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Gazebo GUI client (optional) ──────────────────────────────────────────
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':        '-g -v2 ',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gz),
    )

    # ── Global /clock bridge ──────────────────────────────────────────────────
    # Bridges Gazebo simulation time to ROS /clock.
    # Enables use_sim_time=true across all ROS nodes.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        world_file_arg,
        gz_arg,
        gz_server,
        gz_client,
        clock_bridge,
    ])
