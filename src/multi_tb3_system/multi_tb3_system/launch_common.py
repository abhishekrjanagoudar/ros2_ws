#!/usr/bin/env python3
"""
launch_common.py — shared configuration and helpers for the launch stack.

This module is the single source of truth for convoy formation geometry,
staggered startup timing, and follower-count limits. Previously these values
were duplicated across ``spawn_robots.launch.py`` and ``followers.launch.py``
with a fragile "must match" comment — drift between the two would silently
break the spawn→follower startup ordering. Centralizing them here keeps the
two launch files in lock-step by construction.

It also provides:
  * ``read_burger_urdf()``      — shared TurtleBot3 Burger URDF loader.
  * ``make_legacy_world_launch()`` — factory for the thin world_* convenience
    launchers that delegate to the legacy ``multi_robot.launch.py``.

It is imported by launch files at launch time, exactly like
``generate_sdf.py`` already is.
"""

import os

from ament_index_python.packages import get_package_share_directory


# ─── Convoy formation geometry ─────────────────────────────────────────────────
# Robots are spawned in a line behind the leader: tb1 at x=0, tb2 at x=-1, ...
SPAWN_X_STEP = -1.0   # metres between successive robots along x
SPAWN_Y      =  0.0   # all robots share the same y
SPAWN_Z      =  0.01  # spawn slightly above ground to avoid clipping


# ─── Staggered startup timing (seconds) ────────────────────────────────────────
# Each robot spawns SPAWN_DELAY_STEP after the previous one; a follower then
# waits an extra FOLLOWER_INIT_BUFFER so Gazebo + the bridge can settle before
# cmd_vel starts flowing.
# 3s between spawns gives Gazebo enough time to initialize each robot entity
# (physics, sensors, DiffDrive) before the next is created, while keeping the
# convoy startup snappy. Followers then wait FOLLOWER_INIT_BUFFER after their
# robot spawns so the bridge can settle before cmd_vel flows.
SPAWN_DELAY_STEP      = 3.0
FOLLOWER_INIT_BUFFER  = 1.0


# ─── Follower-count limits ──────────────────────────────────────────────────────
MIN_FOLLOWERS = 0
MAX_FOLLOWERS = 2


def clamp_followers(n: int) -> int:
    """Clamp a requested follower count into the supported range."""
    return max(MIN_FOLLOWERS, min(n, MAX_FOLLOWERS))


def spawn_x(index: int) -> float:
    """X spawn position for robot *index* (1-based: tb1=0.0, tb2=-1.0, ...)."""
    # ``+ 0.0`` normalises the i=1 case from -0.0 to 0.0.
    return (index - 1) * SPAWN_X_STEP + 0.0


def spawn_delay(index: int) -> float:
    """Spawn delay (s) for robot *index* (1-based: tb1=0.0, tb2=3.0, ...)."""
    return (index - 1) * SPAWN_DELAY_STEP


def follower_start_delay(index: int) -> float:
    """Drive-start delay (s) for the follower on robot *index* (1-based)."""
    return spawn_delay(index) + FOLLOWER_INIT_BUFFER


def read_burger_urdf() -> str:
    """
    Load the TurtleBot3 Burger URDF and strip ``${namespace}`` so it can be
    used with robot_state_publisher's ``frame_prefix``.

    Prefers ``turtlebot3_description``; falls back to ``turtlebot3_gazebo``.
    """
    tb3_desc = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_burger.urdf')
    if not os.path.isfile(urdf_path):
        tb3_gz = get_package_share_directory('turtlebot3_gazebo')
        urdf_path = os.path.join(tb3_gz, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        return f.read().replace('${namespace}', '')


def make_legacy_world_launch(world: str):
    """
    Build a LaunchDescription that delegates to the legacy
    ``multi_robot.launch.py`` for a fixed *world*.

    Factored out of the otherwise-identical ``world_empty.launch.py`` and
    ``world_obstacles.launch.py`` convenience launchers.
    """
    # Imported lazily so plain-config consumers of this module don't pull in
    # the full launch machinery.
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration

    pkg_share = get_package_share_directory('multi_tb3_system')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description="Show RViz2 ('true'/'false')."),
        DeclareLaunchArgument('use_gui',  default_value='true',
                              description="Show Gazebo GUI ('true'/'false')."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'multi_robot.launch.py')
            ),
            launch_arguments={
                'world':    world,
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_gui':  LaunchConfiguration('use_gui'),
            }.items(),
        ),
    ])
