#!/usr/bin/env python3
"""
worlds.launch.py — resolves world name → absolute .world path, starts Gazebo.

Args:
  world: 'empty' (default) | 'pillars' | 'office'
  gz   : 'true' | 'false' (default)

The 'office' world (CPR Office environment) uses a custom mesh model located
in config/cpr_office/models/. GZ_SIM_RESOURCE_PATH is extended automatically
so Gazebo can resolve the model://cpr_office/... URIs in the world file.
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


# World registry: name → (world file path, extra model dir or None)
# Paths are relative to the package share directory.
# extra_models: directory to append to GZ_SIM_RESOURCE_PATH, or None.
_WORLDS = {
    'empty':   ('worlds/empty.world',   None),
    'pillars': ('worlds/pillars.world', None),
    'office':  ('config/cpr_office/worlds/office_cpr.world',
                'config/cpr_office/models'),
}


def _launch_setup(context, *args, **kwargs):
    pkg_share  = get_package_share_directory('multi_tb3_system')
    world_name = LaunchConfiguration('world').perform(context)
    gz         = LaunchConfiguration('gz').perform(context)

    if world_name not in _WORLDS:
        raise ValueError(
            f"[worlds.launch] Unknown world '{world_name}'. "
            f"Valid: {list(_WORLDS.keys())}"
        )

    rel_world, rel_models = _WORLDS[world_name]
    world_file = os.path.join(pkg_share, rel_world)

    actions = []

    # For worlds with custom models, extend GZ_SIM_RESOURCE_PATH so Gazebo
    # can resolve model:// URIs (e.g. model://cpr_office/meshes/office.dae).
    if rel_models:
        actions.append(AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share, rel_models),
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world_file': world_file, 'gz': gz}.items(),
    ))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description="World name: 'empty', 'pillars', or 'office'."),
        DeclareLaunchArgument('gz',    default_value='false',
                              description="Show Gazebo GUI ('true'/'false')."),
        OpaqueFunction(function=_launch_setup),
    ])
