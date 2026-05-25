#!/usr/bin/env python3
"""
robot.launch.py
===============
MAIN ENTRY POINT — Multi-TurtleBot3 convoy system orchestrator.

This is the single file users launch.  It composes all modular sub-launch
files and exposes a clean, consistent argument interface.

Sub-launch files included:
  worlds.launch.py       → Gazebo world selection + Gazebo server/client
  spawn_robots.launch.py → Dynamic robot spawning (N+1 robots)
  followers.launch.py    → Autonomous follower nodes
  rviz.launch.py         → RViz2 visualization (optional)
  mapping.launch.py      → SLAM layer (reserved stub)
  localization.launch.py → Localization layer (reserved stub)

────────────────────────────────────────────────────────────────────────────────
Launch Arguments
────────────────────────────────────────────────────────────────────────────────

  world        : World to simulate.
                 Allowed: 'empty' (default), 'pillars'

  nBurger      : Number of follower robots (AGVs) behind the leader.
                 1 → total = 2 robots (tb1 leader + tb2)
                 2 → total = 3 robots (tb1 leader + tb2 + tb3)   ← default
                 Maximum: 2

  use_sim_time : Time source for all ROS nodes.
                 'true'  → use Gazebo simulation clock  ← default
                 'false' → use wall clock (for real robot deployment)

  gz           : Show Gazebo 3D viewer window.
                 'false' ← default  (headless / WSL2 friendly)
                 'true'

  rviz         : Show RViz2 visualization.
                 'false' ← default
                 'true'

  ros_ui       : Convenience all-in-one UI toggle.  OVERRIDES gz and rviz.
                 'false' ← default  → forces gz=false, rviz=false
                 'true'             → forces gz=true,  rviz=true

────────────────────────────────────────────────────────────────────────────────
Usage Examples
────────────────────────────────────────────────────────────────────────────────

  # Minimal — 3 robots, empty world, headless, no RViz
  ros2 launch multi_tb3_system robot.launch.py

  # Full UI demo — Gazebo + RViz, 3 robots
  ros2 launch multi_tb3_system robot.launch.py ros_ui:=true

  # Obstacle world, full UI
  ros2 launch multi_tb3_system robot.launch.py world:=pillars ros_ui:=true

  # 2 robots only (leader + 1 follower), headless
  ros2 launch multi_tb3_system robot.launch.py nBurger:=1

  # RViz only (no Gazebo window), 3 robots, empty world
  ros2 launch multi_tb3_system robot.launch.py rviz:=true

  # Drive the leader robot (separate terminal):
  ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1

────────────────────────────────────────────────────────────────────────────────
Topic Architecture (after launch)
────────────────────────────────────────────────────────────────────────────────

  /tb1/scan        ← LaserScan  (Gz → ROS)
  /tb1/odom        ← Odometry   (Gz → ROS)
  /tb1/cmd_vel     → TwistStamped (ROS → Gz, from teleop)
  /tb1/joint_states← JointState (Gz → ROS)

  /tb2/scan        ← LaserScan  (Gz → ROS)
  /tb2/odom        ← Odometry   (Gz → ROS)
  /tb2/cmd_vel     → TwistStamped (ROS → Gz, from follower_node)
  /tb2/joint_states← JointState (Gz → ROS)

  /tb3/…           (same pattern, when nBurger=2)

  /tf              ← TFMessage  (Gz → ROS, all robots, frames prefixed ns/)
  /clock           ← Clock      (Gz → ROS, simulation time)
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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


# ─── Effective UI flag resolution ─────────────────────────────────────────────

def _resolve_ui_flags(context, *args, **kwargs):
    """
    OpaqueFunction: resolve ros_ui override and return gazebo/rviz sub-launches.

    ros_ui=true  → gz=true,  rviz=true   (full UI mode)
    ros_ui=false → gz=false, rviz=false  (headless mode, individual flags ignored)

    When ros_ui is active it OVERRIDES individual gz and rviz args.
    """
    ros_ui       = LaunchConfiguration('ros_ui').perform(context)
    gz_flag      = LaunchConfiguration('gz').perform(context)
    rviz_flag    = LaunchConfiguration('rviz').perform(context)
    nBurger      = LaunchConfiguration('nBurger').perform(context)
    world        = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # ros_ui overrides individual flags
    effective_gz   = 'true'  if ros_ui == 'true' else ('false' if ros_ui == 'false' else gz_flag)
    effective_rviz = 'true'  if ros_ui == 'true' else ('false' if ros_ui == 'false' else rviz_flag)

    pkg_share = get_package_share_directory('multi_tb3_system')

    def _include(filename, extra_args=None):
        args = {
            'use_sim_time': use_sim_time,
        }
        if extra_args:
            args.update(extra_args)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', filename)
            ),
            launch_arguments=args.items(),
        )

    actions = []

    # ── Gazebo (world + server + clock) ───────────────────────────────────────
    actions.append(_include('worlds.launch.py', {
        'world': world,
        'gz':    effective_gz,
    }))

    # ── Robot spawning (N+1 robots, staggered, with bridges) ─────────────────
    actions.append(_include('spawn_robots.launch.py', {
        'nBurger': nBurger,
    }))

    # ── Follower autonomy nodes ────────────────────────────────────────────────
    actions.append(_include('followers.launch.py', {
        'nBurger': nBurger,
    }))

    # ── RViz2 (conditional) ───────────────────────────────────────────────────
    if effective_rviz == 'true':
        actions.append(_include('rviz.launch.py'))

    # ── Reserved: mapping + localization stubs ────────────────────────────────
    # Uncomment when ready to use:
    # actions.append(_include('mapping.launch.py'))
    # actions.append(_include('localization.launch.py'))

    return actions


# ─── Generate Launch Description ──────────────────────────────────────────────

def generate_launch_description() -> LaunchDescription:
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    # Ensure Gazebo can find TurtleBot3 mesh assets
    set_gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_pkg, 'models'),
    )

    return LaunchDescription([
        # ── Argument declarations ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description="Gazebo world. Allowed: 'empty', 'pillars'.",
        ),
        DeclareLaunchArgument(
            'nBurger',
            default_value='2',
            description=(
                'Number of follower robots. '
                '1 → 2 robots total. 2 → 3 robots total. Max: 2.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description="Time source: 'true' = Gz sim clock, 'false' = wall clock.",
        ),
        DeclareLaunchArgument(
            'gz',
            default_value='false',
            description="Show Gazebo GUI window ('true' / 'false'). Overridden by ros_ui.",
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description="Show RViz2 ('true' / 'false'). Overridden by ros_ui.",
        ),
        DeclareLaunchArgument(
            'ros_ui',
            default_value='false',
            description=(
                "Convenience toggle. "
                "'true'  → forces gz=true,  rviz=true  (full UI). "
                "'false' → forces gz=false, rviz=false (headless)."
            ),
        ),

        # ── Environment setup ──────────────────────────────────────────────────
        set_gz_resource,

        # ── Dynamic composition ────────────────────────────────────────────────
        OpaqueFunction(function=_resolve_ui_flags),
    ])
