#!/usr/bin/env python3
"""localization.launch.py — stub for future AMCL / EKF localization stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_file',     default_value=''),
    ])
