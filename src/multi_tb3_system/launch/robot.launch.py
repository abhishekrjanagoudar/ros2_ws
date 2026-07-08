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
from launch_ros.actions import Node


def _resolve_ui_flags(context, *args, **kwargs):
    """Resolve ros_ui override, then include sub-launches."""
    ros_ui         = LaunchConfiguration('ros_ui').perform(context)
    gz_flag        = LaunchConfiguration('gz').perform(context)
    rviz_flag      = LaunchConfiguration('rviz').perform(context)
    nBurger        = LaunchConfiguration('nBurger').perform(context)
    world          = LaunchConfiguration('world').perform(context)
    use_sim_time   = LaunchConfiguration('use_sim_time').perform(context)
    enable_followers = LaunchConfiguration('enable_followers').perform(context)
    enable_rf2o    = LaunchConfiguration('enable_rf2o').perform(context)
    slam           = LaunchConfiguration('slam').perform(context)
    enable_amcl    = LaunchConfiguration('enable_amcl').perform(context)

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
        _include('spawn_robots.launch.py', {'nBurger': nBurger, 'enable_rf2o': enable_rf2o}),
    ]
    if enable_followers == 'true':
        actions.append(_include('followers.launch.py', {
            'nBurger': nBurger,
            'rviz': effective_rviz,
            'enable_amcl': enable_amcl
        }))
    
    if effective_rviz == 'true':
        actions.append(_include('rviz.launch.py'))

    if slam == 'true':
        slam_params_file = os.path.join(pkg, 'config', 'slam_params.yaml')
        actions.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='tb1',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time == 'true'}],
            output='screen',
            remappings=[
                ('/map', '/tb1/map'),
                ('/map_metadata', '/tb1/map_metadata'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        ))

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
        DeclareLaunchArgument('enable_followers', default_value='true',
                              description="Include followers.launch.py (Costmaps, Convoy Publisher, Followers)."),
        DeclareLaunchArgument('enable_rf2o', default_value='true',
                              description="Enable RF2O laser odometry nodes."),
        DeclareLaunchArgument('slam', default_value='true',
                              description="Enable SLAM for tb1."),
        DeclareLaunchArgument('enable_amcl', default_value='true',
                              description="Enable AMCL localization for followers."),

        # Expose TurtleBot3 mesh assets to Gazebo
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(tb3_gazebo_pkg, 'models'),
        ),

        OpaqueFunction(function=_resolve_ui_flags),
    ])
