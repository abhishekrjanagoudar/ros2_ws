#!/usr/bin/env python3
"""
followers.launch.py — Path-Based Convoy.

Starts:
  * convoy_publisher on the leader (tb1): publishes /tb1/convoy_path.
  * one Pure-Pursuit follower_node.py per follower robot (tb2, tb3, ...).

Each follower subscribes to the shared leader path and tracks it while holding
a configurable gap (convoy_spacing). LiDAR is used only for safety.

Nodes start AFTER their robot has spawned + an init buffer so Gazebo and the
bridge settle before cmd_vel flows. Spawn/start timing and spawn geometry come
from ``multi_tb3_system.launch_common`` so this stays in lock-step with
spawn_robots.launch.py.

Args:
  nBurger       : follower count 1–2 (default 2)
  use_sim_time  : 'true' (default) | 'false'
  convoy_spacing: gap per convoy slot in metres (default 1.0)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from multi_tb3_system.launch_common import (
    SPAWN_Y,
    clamp_followers,
    follower_start_delay,
    spawn_x,
)

_LEADER_NS = 'tb1'


def _launch_setup(context, *args, **kwargs):
    n_burgers      = clamp_followers(int(LaunchConfiguration('nBurger').perform(context)))
    use_sim_time   = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    convoy_spacing = float(LaunchConfiguration('convoy_spacing').perform(context))

    pkg_share   = get_package_share_directory('multi_tb3_system')
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    actions = []

    # ── Leader trajectory publisher (tb1) ────────────────────────────────────
    convoy_pub = Node(
        package='multi_tb3_system',
        executable='convoy_publisher.py',
        name='convoy_publisher',
        namespace=_LEADER_NS,
        parameters=[{
            'use_sim_time':   use_sim_time,
            'path_frame':     'world',
            'spawn_offset_x': spawn_x(1),   # leader spawn x (0.0)
            'spawn_offset_y': SPAWN_Y,
        }],
        output='screen',
        emulate_tty=True,
    )
    # Leader spawns at t=0; small delay lets odom + bridge come up first.
    actions.append(TimerAction(period=2.0, actions=[convoy_pub]))

    # ── Pure-Pursuit followers (tb2, tb3, ...) ───────────────────────────────
    for i in range(2, n_burgers + 2):
        ns = f'tb{i}'
        node = Node(
            package='multi_tb3_system',
            executable='follower_node.py',
            name='follower_node',
            namespace=ns,
            parameters=[
                params_file,
                {
                    'use_sim_time':   use_sim_time,
                    'leader_ns':      _LEADER_NS,
                    'convoy_spacing': convoy_spacing,
                    'spawn_offset_x': spawn_x(i),   # this robot's world offset
                    'spawn_offset_y': SPAWN_Y,
                },
            ],
            output='screen',
            emulate_tty=True,
        )
        actions.append(TimerAction(period=follower_start_delay(i), actions=[node]))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('nBurger',        default_value='2',
                              description='Follower count (1–2).'),
        DeclareLaunchArgument('use_sim_time',   default_value='true',
                              description="'true' = Gz clock, 'false' = wall clock."),
        DeclareLaunchArgument('convoy_spacing', default_value='0.6',
                              description='Gap per convoy slot in metres.'),
        OpaqueFunction(function=_launch_setup),
    ])
