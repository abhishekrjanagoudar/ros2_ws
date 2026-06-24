#!/usr/bin/env python3
"""
followers.launch.py — Path-Based Convoy.

Starts:
  * convoy_publisher on the leader (tb1): publishes /tb1/convoy_path.
  * one costmap_generator.py per robot (tb1, tb2, tb3, ...): consumes the
    robot's /scan and publishes a per-robot ``local_costmap`` for visualization.
    Publishing is disabled by default (enable_costmap_viz=false) and enabled
    automatically when rviz:=true so headless runs waste no CPU.
  * one Pure-Pursuit follower_node.py per follower robot (tb2, tb3, ...).

Each follower subscribes to the shared leader path and tracks it while holding
a configurable gap (convoy_spacing). LiDAR is used only for safety.

Nodes start AFTER their robot has spawned + an init buffer so Gazebo and the
bridge settle before cmd_vel flows. Spawn/start timing and spawn geometry come
from ``multi_tb3_system.launch_common`` so this stays in lock-step with
spawn_robots.launch.py.

Args:
  nBurger       : follower count 1-2 (default 2)
  use_sim_time  : 'true' (default) | 'false'
  convoy_spacing: gap per convoy slot in metres (default 0.5)
  rviz          : 'true' | 'false' (default) — enables costmap_viz publishing
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
    spawn_x,
)

_LEADER_NS = 'tb1'


def _launch_setup(context, *args, **kwargs):
    n_burgers      = clamp_followers(int(LaunchConfiguration('nBurger').perform(context)))
    use_sim_time   = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    convoy_spacing = float(LaunchConfiguration('convoy_spacing').perform(context))
    enable_viz     = LaunchConfiguration('rviz').perform(context) == 'true'

    pkg_share   = get_package_share_directory('multi_tb3_system')
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    actions = []

    # ── Startup timeline (all times relative to when this launch file is invoked,
    #    which is AFTER Gazebo + all robots are already spawned and bridges are up)
    #
    #   t=0.0s  : convoy_publisher starts on tb1 (leader recording begins immediately)
    #   t=0.75s : costmap_generator nodes start (scan stream is live by this point)
    #   t=2.5s  : follower_node tb2 starts (Follower 1 begins tracking)
    #   t=3.5s  : follower_node tb3 starts (Follower 2 begins tracking, 1s after tb2)

    CONVOY_PUB_START  = 0.0    # leader path recording — start immediately
    COSTMAP_START     = 0.75   # per-robot costmap generator — after scan is live
    FOLLOWER1_START   = 1.25   # tb2 follower node
    FOLLOWER2_START   = 1.75   # tb3 follower node — 0.5s after tb2

    # ── Leader trajectory publisher (tb1) ────────────────────────────────────
    convoy_pub = Node(
        package='multi_tb3_system',
        executable='convoy_publisher.py',
        name='convoy_publisher',
        namespace=_LEADER_NS,
        parameters=[{
            'use_sim_time':   use_sim_time,
            'path_frame':     'world',
            'spawn_offset_x': spawn_x(1),
            'spawn_offset_y': SPAWN_Y,
        }],
        output='screen',
        emulate_tty=True,
    )
    actions.append(TimerAction(period=CONVOY_PUB_START, actions=[convoy_pub]))

    # ── Per-robot costmap_generator (tb1, tb2, tb3, ...) ─────────────────────
    for i in range(1, n_burgers + 2):
        ns = f'tb{i}'
        cg_node = Node(
            package='multi_tb3_system',
            executable='costmap_generator.py',
            name='costmap_generator',
            namespace=ns,
            parameters=[
                params_file,
                {
                    'use_sim_time':       use_sim_time,
                    'enable_costmap_viz': enable_viz,
                },
            ],
            output='screen',
            emulate_tty=True,
        )
        actions.append(TimerAction(period=COSTMAP_START, actions=[cg_node]))

    # ── Pure-Pursuit followers (tb2, tb3, ...) ───────────────────────────────
    # Follower start times are fixed regardless of how many followers there are:
    #   tb2 → t=5.0s, tb3 → t=7.0s
    follower_delays = {2: FOLLOWER1_START, 3: FOLLOWER2_START}

    for i in range(2, n_burgers + 2):
        ns = f'tb{i}'
        delay = follower_delays.get(i, FOLLOWER1_START + (i - 2) * 2.0)
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
                    'spawn_offset_x': spawn_x(i),
                    'spawn_offset_y': SPAWN_Y,
                },
            ],
            remappings=[
                ('convoy_path', f'/{_LEADER_NS}/convoy_path'),
            ],
            output='screen',
            emulate_tty=True,
        )
        actions.append(TimerAction(period=delay, actions=[node]))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('nBurger',        default_value='2',
                              description='Follower count (1-2).'),
        DeclareLaunchArgument('use_sim_time',   default_value='true',
                              description="'true' = Gz clock, 'false' = wall clock."),
        DeclareLaunchArgument('convoy_spacing', default_value='0.6',
                              description='Gap per convoy slot in metres (must match SPAWN_X_STEP=0.6m).'),
        DeclareLaunchArgument('rviz',           default_value='false',
                              description="'true' = enable costmap_viz publishing for RViz."),
        OpaqueFunction(function=_launch_setup),
    ])
