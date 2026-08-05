#!/usr/bin/env python3
"""
followers.launch.py — Path-Based Convoy.
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
    enable_amcl    = LaunchConfiguration('enable_amcl').perform(context) == 'true'

    pkg_share   = get_package_share_directory('multi_tb3_system')
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')
    amcl_params_file = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    actions = []

    #  Startup timeline (all times relative to when this launch file is invoked,

    CONVOY_PUB_START  = 0.0    # leader path recording — start immediately
    COSTMAP_START     = 0.38   # per-robot costmap generator — after scan is live
    FOLLOWER1_START   = 1.00   # tb2 follower node
    FOLLOWER2_START   = 1.25   # tb3 follower node — 0.5s after tb2

    # Per-robot costmap_generator (tb1, tb2, tb3, ...)
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

    # Leader trajectory publisher (only the global leader tb1 publishes the path)
    ns = _LEADER_NS
    convoy_pub = Node(
        package='multi_tb3_system',
        executable='convoy_publisher.py',
        name='convoy_publisher',
        namespace=ns,
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

    # Pure-Pursuit followers (tb2, tb3, ...)
    follower_delays = {2: FOLLOWER1_START, 3: FOLLOWER2_START}

    for i in range(2, n_burgers + 2):
        ns = f'tb{i}'
        leader_ns = f'tb{i-1}'
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
                    'leader_ns':      leader_ns,
                    'convoy_spacing': convoy_spacing,
                    'spawn_offset_x': spawn_x(i),
                    'spawn_offset_y': SPAWN_Y,
                    'enable_costmap_viz': enable_viz,
                },
            ],
            remappings=[
                ('convoy_path', f'/{_LEADER_NS}/convoy_path'),
            ],
            output='screen',
            emulate_tty=True,
        )
        actions.append(TimerAction(period=delay, actions=[node]))

        if enable_amcl:
            amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=ns,
                parameters=[
                    amcl_params_file,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    ('map', '/map'),
                ],
                output='screen',
            )
            amcl_lifecycle = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_amcl',
                namespace=ns,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['amcl']}
                ],
                output='screen',
            )
            actions.append(TimerAction(period=delay + 0.1, actions=[amcl_node, amcl_lifecycle]))



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
        DeclareLaunchArgument('enable_amcl',    default_value='true',
                              description="Enable AMCL localization for followers."),
        OpaqueFunction(function=_launch_setup),
    ])
