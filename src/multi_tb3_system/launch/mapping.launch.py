#!/usr/bin/env python3
"""
mapping.launch.py — SLAM mapping using slam_toolbox (online_async mode).

Supports two modes:
  • **Single-robot mapping:** Run slam_toolbox for one robot (default: tb1).
    Produces one occupancy grid map from a single robot's LiDAR.

  • **Multi-robot mapping:** Run one slam_toolbox instance per robot.
    Each instance builds its own map under its namespace. Maps can later
    be merged using slam_toolbox's merge_maps utility.

The launch file uses slam_toolbox's `online_async_launch.py` internally and
passes a custom params file tuned for TurtleBot3 Burger (3.5m LDS-01 LiDAR).

It also starts a `nav2_map_server/map_saver_server` so you can save the map
at any time via:
    ros2 service call /map_save nav2_msgs/srv/SaveMap "{}"
    # or: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

Args:
  mapping_mode  : 'online_async' (default) — future-proof for adding other modes
  slam_robot    : 'tb1' (default) | 'all' — which robot(s) run SLAM
  use_sim_time  : 'true' (default) | 'false'
  autostart     : 'true' (default) — auto-configure the slam_toolbox lifecycle node

Usage:
  # Single robot (tb1 builds the map while you teleoperate it)
  ros2 launch multi_tb3_system mapping.launch.py slam_robot:=tb1

  # Multi-robot (all robots contribute to separate maps)
  ros2 launch multi_tb3_system mapping.launch.py slam_robot:=all

  # Standalone (system must already be running via robot.launch.py)
  ros2 launch multi_tb3_system mapping.launch.py

  # Integrated (launched from robot.launch.py with enable_mapping:=true)
  ros2 launch multi_tb3_system robot.launch.py enable_mapping:=true ros_ui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_slam_actions(context, *args, **kwargs):
    """
    Build slam_toolbox + map_saver actions for the requested robot(s).

    Single-robot mode (slam_robot=tb1):
      - Remaps scan_topic to /tb1/scan
      - Remaps odom_frame to tb1/odom
      - Publishes map → /map (global)
      - TF: map → tb1/odom

    Multi-robot mode (slam_robot=all):
      - One slam_toolbox per robot under /tbX namespace
      - Each publishes /tbX/map
      - TF: tbX/map → tbX/odom
    """
    slam_robot    = LaunchConfiguration('slam_robot').perform(context)
    use_sim_time  = LaunchConfiguration('use_sim_time').perform(context)
    autostart     = LaunchConfiguration('autostart').perform(context)
    mapping_mode  = LaunchConfiguration('mapping_mode').perform(context)

    pkg_share = get_package_share_directory('multi_tb3_system')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Resolve params file based on mapping_mode
    params_file = os.path.join(pkg_share, 'config', f'mapping_{mapping_mode}.yaml')
    if not os.path.isfile(params_file):
        # Fallback to slam_toolbox defaults
        params_file = os.path.join(
            slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml'
        )

    # Resolve slam_toolbox launch file based on mode
    mode_to_launch = {
        'online_async': 'online_async_launch.py',
        'online_sync':  'online_sync_launch.py',
        'offline':      'offline_launch.py',
        'localization': 'localization_launch.py',
    }
    slam_launch_file = os.path.join(
        slam_toolbox_dir, 'launch',
        mode_to_launch.get(mapping_mode, 'online_async_launch.py')
    )

    # Determine which robots to map
    if slam_robot == 'all':
        # Import here to avoid circular dependency at module level
        from multi_tb3_system.launch_common import clamp_followers
        # Assume max (leader + 2 followers = 3 robots)
        # In practice this is fine — unused slam instances just won't receive data
        robot_namespaces = ['tb1', 'tb2', 'tb3']
    else:
        robot_namespaces = [slam_robot]

    all_actions = []

    for ns in robot_namespaces:
        # Frame names for this robot
        odom_frame = f'{ns}/odom'
        base_frame = f'{ns}/base_footprint'
        scan_topic = f'/{ns}/scan'

        # For multi-robot: each gets its own map frame to avoid TF conflicts
        if len(robot_namespaces) > 1:
            map_frame = f'{ns}/map'
        else:
            map_frame = 'map'

        # Launch slam_toolbox node directly with per-robot params.
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns if len(robot_namespaces) > 1 else '',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time == 'true',
                    'odom_frame': odom_frame,
                    'map_frame': map_frame,
                    'base_frame': base_frame,
                    'scan_topic': scan_topic,
                },
            ],
            remappings=[
                ('/map', f'/{ns}/map') if len(robot_namespaces) > 1 else ('/map', '/map'),
                ('/map_metadata', f'/{ns}/map_metadata') if len(robot_namespaces) > 1 else ('/map_metadata', '/map_metadata'),
            ],
        )

        all_actions.append(slam_node)

    # Map saver server (always one instance on global namespace)
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        output='screen',
        parameters=[
            {'autostart': autostart == 'true'},
            {'node_names': ['map_saver']},
        ],
    )

    all_actions.extend([map_saver, lifecycle_manager])

    return all_actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mapping_mode', default_value='online_async',
            description="SLAM mode: 'online_async' (default), 'online_sync', 'offline', 'localization'.",
        ),
        DeclareLaunchArgument(
            'slam_robot', default_value='tb1',
            description="Which robot runs SLAM: 'tb1', 'tb2', 'tb3', or 'all' for multi-robot mapping.",
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="'true' = Gazebo clock, 'false' = wall clock.",
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description="Auto-configure slam_toolbox lifecycle node on startup.",
        ),
        OpaqueFunction(function=_build_slam_actions),
    ])
