#!/usr/bin/env python3
"""
world_empty.launch.py
=====================
Launches the Multi-TurtleBot3 convoy in the empty world (no obstacles).
Best for initial convoy formation testing.

Usage:
  ros2 launch multi_tb3_system world_empty.launch.py
  ros2 launch multi_tb3_system world_empty.launch.py use_rviz:=true
  ros2 launch multi_tb3_system world_empty.launch.py use_gui:=false
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('multi_tb3_system')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gazebo GUI',
    )

    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'multi_robot.launch.py')
        ),
        launch_arguments={
            'world':    'empty',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_gui':  LaunchConfiguration('use_gui'),
        }.items(),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_gui_arg,
        multi_robot_launch,
    ])
