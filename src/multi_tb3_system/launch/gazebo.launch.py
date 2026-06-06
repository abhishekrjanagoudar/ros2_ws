#!/usr/bin/env python3
"""
gazebo.launch.py — Gazebo Sim lifecycle (server + optional GUI + clock bridge).

GUI uses ExecuteProcess to inject GPU-accelerated rendering env vars for WSL2.
Uses GLX path (XWayland → WSLg → D3D12 → NVIDIA GPU) instead of EGL/software.
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

    # GUI client — WSL2 GPU path: GLX via XWayland → WSLg compositor → D3D12 → NVIDIA
    # DO NOT set LIBGL_ALWAYS_SOFTWARE or llvmpipe here — those force pure CPU rendering.
    # OGRE2 must use GLX (not EGL headless) because WSL2 has no /dev/dri render nodes;
    # EGL falls back to EGL_MESA_device_software (software rasterizer on CPU).
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v2', '--force-version', '8'],
        additional_env={
            # Force OGRE2 to use GLX context (GPU) instead of EGL headless (CPU fallback)
            'GZ_SIM_RENDER_ENGINE_API_BACKEND': 'opengl',
            # Ensure Mesa D3D12 picks NVIDIA, not any Intel iGPU
            'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA',
            # Disable VSync stall — WSLg does not need it and it causes frame latency
            '__GL_SYNC_TO_VBLANK': '0',
            # Disable DRI3 sync overhead on XWayland
            'LIBGL_DRI3_DISABLE': '1',
            # Use OGRE2 rendering engine
            'GZ_SIM_RENDER_ENGINE': 'ogre2',
            # Disable shadows — expensive on Mesa D3D12 translation layer
            'GZ_SIM_DISABLE_SHADOWS': '1',
        },
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
