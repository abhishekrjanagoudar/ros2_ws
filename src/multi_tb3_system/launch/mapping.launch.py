#!/usr/bin/env python3
"""
mapping.launch.py
=================
Reserved modular launch file — SLAM / Mapping layer.

Placeholder for future SLAM integration (e.g. slam_toolbox, cartographer).

Currently a NO-OP stub that accepts all anticipated args for forward
compatibility.  Extend this file when adding SLAM capabilities.

Future responsibilities:
  - Start slam_toolbox or cartographer_ros
  - Configure map frame / odom frame linkage per robot
  - Load SLAM parameters from config/slam_params.yaml

Args (reserved, not yet used):
  use_sim_time : 'true' | 'false'
  slam_robot   : namespace of robot performing SLAM (e.g. 'tb1')
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import logging

logger = logging.getLogger(__name__)


def generate_launch_description() -> LaunchDescription:
    logger.info(
        '[mapping.launch.py] STUB — no SLAM nodes started. '
        'Implement when slam_toolbox or cartographer is needed.'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Reserved for future SLAM node configuration.',
        ),
        DeclareLaunchArgument(
            'slam_robot',
            default_value='tb1',
            description='Reserved: namespace of the SLAM robot.',
        ),
    ])
