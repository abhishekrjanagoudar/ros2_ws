#!/usr/bin/env python3
"""
followers.launch.py
===================
Modular launch file — Autonomous follower node launcher.

Starts one ``follower_node.py`` per follower robot (tb2 … tb<nBurger+1>).
The leader (tb1) is teleoperated and gets no follower node.

Each follower node:
  - Runs under its own ROS namespace (e.g. /tb2, /tb3)
  - Subscribes to /<ns>/scan        (LaserScan from ros_gz_bridge)
  - Publishes to  /<ns>/cmd_vel     (TwistStamped → ros_gz_bridge → Gazebo)
  - Loads parameters from config/follower_params.yaml

Uses OpaqueFunction for dynamic node count based on nBurger.

Args:
  nBurger     : int 1 or 2  (default: 2)
  use_sim_time: 'true' | 'false'  (default: 'true')

Standalone usage (requires Gazebo + robots already running)::

  ros2 launch multi_tb3_system followers.launch.py nBurger:=2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    """Return follower Node actions for tb2 … tb<nBurger+1>."""
    n_burgers    = int(LaunchConfiguration('nBurger').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    n_burgers = max(0, min(n_burgers, 2))   # clamp 0–2

    pkg_share   = get_package_share_directory('multi_tb3_system')
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    actions = []
    for i in range(2, n_burgers + 2):   # i = 2, 3 (tb2, tb3)
        ns = f'tb{i}'
        actions.append(
            Node(
                package='multi_tb3_system',
                executable='follower_node.py',
                name='follower_node',
                namespace=ns,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                output='screen',
                emulate_tty=True,   # preserve colour logging in terminal
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'nBurger',
            default_value='2',
            description='Number of follower robots (1 or 2).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description="Use simulation clock ('true') or wall clock ('false').",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
