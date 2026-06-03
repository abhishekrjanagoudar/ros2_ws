#!/usr/bin/env python3
"""mapping.launch.py — stub for future slam_toolbox / cartographer SLAM stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_robot',   default_value='tb1'),
    ])
