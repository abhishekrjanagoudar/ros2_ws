#!/usr/bin/env python3
"""
robot.launch.py — main entry point for the multi-TurtleBot3 convoy system.

Args:
  world          : 'empty' (default) | 'pillars' | 'office'
  nBurger        : follower count, 1–2 (default 2 → 3 robots total)
  use_sim_time   : 'true' (default) | 'false'
  gz             : show Gazebo GUI  'true' | 'false' (default)
  rviz           : show RViz2       'true' | 'false' (default)
  ros_ui         : shortcut — 'true' sets gz=true + rviz=true (overrides both)
  enable_mapping : 'true' | 'false' (default) — start slam_toolbox SLAM
  mapping_mode   : 'online_async' (default) | 'online_sync'
  slam_robot     : 'tb1' (default) | 'all' — which robot(s) build the map

Usage:
  ros2 launch multi_tb3_system robot.launch.py world:=pillars ros_ui:=true
  ros2 launch multi_tb3_system robot.launch.py nBurger:=1
  ros2 launch multi_tb3_system robot.launch.py enable_mapping:=true slam_robot:=tb1 ros_ui:=true
  ros2 launch multi_tb3_system robot.launch.py enable_mapping:=true slam_robot:=all
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
    enable_mapping = LaunchConfiguration('enable_mapping').perform(context)
    mapping_mode   = LaunchConfiguration('mapping_mode').perform(context)
    slam_robot     = LaunchConfiguration('slam_robot').perform(context)

    # ros_ui=true → both GUIs on; ros_ui=false → both off
    effective_gz   = 'true' if ros_ui == 'true' else ('false' if ros_ui == 'false' else gz_flag)
    effective_rviz = 'true' if ros_ui == 'true' else ('false' if ros_ui == 'false' else rviz_flag)

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
        _include('followers.launch.py',    {'nBurger': nBurger}),
    ]
    if effective_rviz == 'true':
        actions.append(_include('rviz.launch.py'))

    # Conditionally start SLAM mapping
    if enable_mapping == 'true':
        actions.append(_include('mapping.launch.py', {
            'mapping_mode': mapping_mode,
            'slam_robot':   slam_robot,
            'autostart':    'true',
        }))

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
        DeclareLaunchArgument('enable_mapping', default_value='false',
                              description="'true' → start slam_toolbox for SLAM mapping."),
        DeclareLaunchArgument('mapping_mode',   default_value='online_async',
                              description="SLAM mode: 'online_async', 'online_sync'."),
        DeclareLaunchArgument('slam_robot',     default_value='tb1',
                              description="Which robot runs SLAM: 'tb1', 'tb2', 'tb3', or 'all'."),

        # Expose TurtleBot3 mesh assets to Gazebo
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(tb3_gazebo_pkg, 'models'),
        ),

        OpaqueFunction(function=_resolve_ui_flags),
    ])
