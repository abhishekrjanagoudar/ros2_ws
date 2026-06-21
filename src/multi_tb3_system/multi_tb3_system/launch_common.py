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
  * ``read_burger_urdf()`` — shared TurtleBot3 Burger URDF loader.

It is imported by launch files at launch time, exactly like
``generate_sdf.py`` already is.
"""

import os

from ament_index_python.packages import get_package_share_directory


# ─── Convoy formation geometry ─────────────────────────────────────────────────
# Robots are spawned in a line behind the leader:
#   tb1 at x= 0.0, tb2 at x=-0.5, tb3 at x=-1.0.
#
# ⚠️  KEEP IN SYNC — all three of these must equal the same absolute value:
#       1. SPAWN_X_STEP          (here, absolute value = 0.5 m)
#       2. convoy_spacing        in config/follower_params.yaml  (= 0.5 m)
#       3. convoy_spacing default in scripts/follower_node.py    (= 0.5 m)
#
# If they diverge, followers will spawn at the wrong initial gap relative to
# their Pure Pursuit target, causing an aggressive correction burst on startup.
SPAWN_X_STEP = -0.5   # metres between successive robots along x
SPAWN_Y      =  0.0   # all robots share the same y
SPAWN_Z      =  0.01  # spawn slightly above ground to avoid clipping



# ─── Staggered startup timing (seconds) ────────────────────────────────────────
# Each robot spawns SPAWN_DELAY_STEP after the previous one; a follower then
# waits an extra FOLLOWER_INIT_BUFFER so Gazebo + the bridge can settle before
# cmd_vel starts flowing.
#
# Observed startup timeline (Gazebo Harmonic on WSL2):
#   t=0      : tb1 spawns and its bridge is live within ~1.5s.
#   t=STEP   : tb2 spawn command fires; Gazebo entity creation takes ~2s,
#              the DiffDrive plugin begins publishing odom only AFTER that.
#   t=STEP+3 : bridge is confirmed live + first odom message received → safe
#              to start the follower control loop.
#
# SPAWN_DELAY_STEP=4 leaves 4 s for each robot's full Gazebo + bridge init
# before the next robot begins. FOLLOWER_INIT_BUFFER=3 adds 3 s on top of the
# robot's own spawn delay so the follower never fires before odom is available.
SPAWN_DELAY_STEP      = 4.0   # was 3.0 — bumped to cover full Gazebo entity init
FOLLOWER_INIT_BUFFER  = 3.0   # was 1.0 — bumped so follower starts after first odom


# ─── Follower-count limits ──────────────────────────────────────────────────────
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
    """Spawn delay (s) for robot *index* (1-based: tb1=0.0, tb2=4.0, tb3=8.0)."""
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

