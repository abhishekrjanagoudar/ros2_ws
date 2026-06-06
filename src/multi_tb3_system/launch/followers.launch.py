#!/usr/bin/env python3
"""
followers.launch.py — starts one follower_node.py per follower robot.

Nodes start AFTER their robot has spawned + an init buffer to let Gazebo
and the bridge settle before cmd_vel flows.
  tb2: spawns at 3s → follower starts at 5s
  tb3: spawns at 6s → follower starts at 8s

Spawn/start timing comes from ``multi_tb3_system.launch_common`` so it stays
in lock-step with spawn_robots.launch.py.

Args:
  nBurger     : follower count 1–2 (default 2)
  use_sim_time: 'true' (default) | 'false'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from multi_tb3_system.launch_common import clamp_followers, follower_start_delay


def _launch_setup(context, *args, **kwargs):
    n_burgers    = clamp_followers(int(LaunchConfiguration('nBurger').perform(context)))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    pkg_share   = get_package_share_directory('multi_tb3_system')
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    actions = []
    
    # Start the convoy path publisher on the leader (tb1)
    convoy_pub_node = Node(
        package='multi_tb3_system',
        executable='convoy_publisher.py',
        name='convoy_publisher',
        namespace='tb1',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )
    # The leader spawns at t=0, so publisher can start right away (or with slight delay)
    actions.append(TimerAction(period=2.0, actions=[convoy_pub_node]))

    for i in range(2, n_burgers + 2):   # tb2, tb3, ...
        ns        = f'tb{i}'
        leader_ns = 'tb1'               # In Convoy architecture, everyone follows tb1's path
        node = Node(
            package='multi_tb3_system',
            executable='follower_node.py',
            name='follower_node',
            namespace=ns,
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'leader_ns':    leader_ns,   # Convoy path is /tb1/convoy_path
                },
            ],
            output='screen',
            emulate_tty=True,
        )
        actions.append(TimerAction(period=follower_start_delay(i), actions=[node]))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('nBurger',      default_value='2',
                              description='Follower count (1–2).'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description="'true' = Gz clock, 'false' = wall clock."),
        OpaqueFunction(function=_launch_setup),
    ])
