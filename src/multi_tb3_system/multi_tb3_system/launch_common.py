#!/usr/bin/env python3
"""
launch_common.py — shared configuration and helpers for the launch stack.
"""

import os

from ament_index_python.packages import get_package_share_directory


# Convoy formation geometry
SPAWN_X_STEP = -0.6   # metres between successive robots along x
SPAWN_Y      =  0.0   # all robots share the same y
SPAWN_Z      =  0.01  # spawn slightly above ground to avoid clipping



# Staggered startup timing (seconds)
SPAWN_DELAY_STEP      = 4.0   # was 3.0 — bumped to cover full Gazebo entity init
FOLLOWER_INIT_BUFFER  = 3.0   # was 1.0 — bumped so follower starts after first odom


# Follower-count limits
MIN_FOLLOWERS = 0
MAX_FOLLOWERS = 2


def clamp_followers(n: int) -> int:
    """Clamp a requested follower count into the supported range."""
    return max(MIN_FOLLOWERS, min(n, MAX_FOLLOWERS))


def spawn_x(index: int) -> float:
    """X spawn position for robot *index* (1-based: tb1=0.0, tb2=-0.5, tb3=-1.0)."""
    # ``+ 0.0`` normalises the i=1 case from -0.0 to 0.0.
    return (index - 1) * SPAWN_X_STEP + 0.0


def spawn_delay(index: int) -> float:
    """Spawn delay (s) for robot *index* (1-based: tb1=0.0, tb2=2.0, tb3=4.0)."""
    if index == 1:
        return 0.0
    elif index == 2:
        return 2.0
    elif index == 3:
        return 4.0
    return (index - 1) * 2.0


def follower_start_delay(index: int) -> float:
    """Drive-start delay (s) for the follower on robot *index* (1-based)."""
    if index == 2:
        return 5.0
    elif index == 3:
        return 7.0
    return spawn_delay(index) + FOLLOWER_INIT_BUFFER


def read_burger_urdf() -> str:
    """
Load the TurtleBot3 Burger URDF and strip ``${namespace}`` so it can be
"""
    tb3_desc = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_burger.urdf')
    if not os.path.isfile(urdf_path):
        tb3_gz = get_package_share_directory('turtlebot3_gazebo')
        urdf_path = os.path.join(tb3_gz, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        return f.read().replace('${namespace}', '')

