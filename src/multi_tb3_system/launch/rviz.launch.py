#!/usr/bin/env python3
"""
rviz.launch.py
==============
Modular launch file — RViz2 visualization.

Starts RViz2 with the pre-configured multi_robot.rviz layout showing:
  🔴 /tb1/scan  — LaserScan red
  🟢 /tb2/scan  — LaserScan green
  🔵 /tb3/scan  — LaserScan blue
  + Odometry arrows and TF tree for all active robots

Args:
  use_sim_time: 'true' | 'false'  (default: 'true')

Standalone usage::

  ros2 launch multi_tb3_system rviz.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('multi_tb3_system')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Use simulation clock ('true') or wall clock ('false').",
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
    ])
