#!/usr/bin/env python3
"""
robot.launch.py — main entry point for the multi-TurtleBot3 convoy system.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _resolve_ui_flags(context, *args, **kwargs):
    """Resolve ros_ui override, then include sub-launches."""
    ros_ui         = LaunchConfiguration('ros_ui').perform(context)
    gz_flag        = LaunchConfiguration('gz').perform(context)
    rviz_flag      = LaunchConfiguration('rviz').perform(context)
    nBurger        = LaunchConfiguration('nBurger').perform(context)
    world          = LaunchConfiguration('world').perform(context)
    use_sim_time   = LaunchConfiguration('use_sim_time').perform(context)

    # ros_ui=true → both GUIs on. ros_ui=false (default) → respect individual gz/rviz flags.
    if ros_ui == 'true':
        effective_gz   = 'true'
        effective_rviz = 'true'
    else:
        effective_gz   = gz_flag
        effective_rviz = rviz_flag

    pkg = get_package_share_directory('multi_tb3_system')

    def _include(filename, extra=None):
        args = {'use_sim_time': use_sim_time}
        if extra:
            args.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', filename)),
            launch_arguments=args.items(),
        )

    actions = [
        _include('worlds.launch.py',       {'world': world, 'gz': effective_gz}),
        _include('spawn_robots.launch.py', {'nBurger': nBurger}),
        _include('followers.launch.py',    {'nBurger': nBurger, 'rviz': effective_rviz}),
    ]
    if effective_rviz == 'true':
        actions.append(_include('rviz.launch.py'))

    return actions


def generate_launch_description() -> LaunchDescription:
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('world',        default_value='empty',
                              description="Gazebo world: 'empty', 'pillars', or 'office'."),
        DeclareLaunchArgument('nBurger',      default_value='2',
                              description='Follower count (1–2). Total robots = nBurger + 1.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description="'true' = Gz sim clock, 'false' = wall clock."),
        DeclareLaunchArgument('gz',           default_value='false',
                              description="Show Gazebo GUI. Overridden by ros_ui."),
        DeclareLaunchArgument('rviz',         default_value='false',
                              description="Show RViz2. Overridden by ros_ui."),
        DeclareLaunchArgument('ros_ui',       default_value='false',
                              description="'true' → gz=true + rviz=true. Overrides gz and rviz."),

        # Expose TurtleBot3 mesh assets to Gazebo
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(tb3_gazebo_pkg, 'models'),
        ),

        OpaqueFunction(function=_resolve_ui_flags),
    ])
