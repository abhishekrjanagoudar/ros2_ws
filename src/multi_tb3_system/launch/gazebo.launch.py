#!/usr/bin/env python3
"""
gazebo.launch.py — Gazebo Sim lifecycle (server + optional GUI + clock bridge).

GUI uses ExecuteProcess (not IncludeLaunchDescription) to inject
LIBGL_ALWAYS_SOFTWARE=1 per-process — fixes QGLXContext failure on WSL2.
GUI crash does NOT kill the server (on_exit_shutdown omitted for GUI).

Args:
  world_file  : absolute path to .world SDF
  gz          : 'true' | 'false' (default) — show Gazebo GUI
  use_sim_time: passed through, not used here directly
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file = LaunchConfiguration('world_file')
    gz         = LaunchConfiguration('gz')

    # Headless sim server — death kills whole launch (on_exit_shutdown=true)
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':         ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # GUI client — LIBGL_ALWAYS_SOFTWARE=1 bypasses broken WSL2 GLX/drisw path
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v2', '--force-version', '8'],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'QT_OPENGL': 'software'},
        condition=IfCondition(gz),
        output='screen',
    )

    # Bridge /clock (Gz sim time → ROS)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file',
                              description='Absolute path to the Gazebo world SDF.'),
        DeclareLaunchArgument('gz', default_value='false',
                              description="Launch Gazebo GUI client ('true'/'false')."),
        gz_server,
        gz_client,
        clock_bridge,
    ])
