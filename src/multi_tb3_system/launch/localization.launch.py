#!/usr/bin/env python3
"""
localization.launch.py
======================
Reserved modular launch file — Localization stack.

Placeholder for future localization integration (e.g. AMCL, EKF,
robot_localization, or Nav2 AMCL).

Currently a NO-OP stub.  Extend when adding localization support.

Future responsibilities:
  - Start robot_localization EKF node (fuses odom + IMU)
  - Start AMCL for map-based localization
  - Configure per-robot localization parameters

Args (reserved, not yet used):
  use_sim_time : 'true' | 'false'
  map_file     : path to pre-built map YAML (for AMCL)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import logging

logger = logging.getLogger(__name__)


def generate_launch_description() -> LaunchDescription:
    logger.info(
        '[localization.launch.py] STUB — no localization nodes started. '
        'Implement when AMCL / EKF / Nav2 localization is needed.'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Reserved for future localization node configuration.',
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Reserved: path to map YAML for AMCL.',
        ),
    ])
