#!/usr/bin/env python3
"""
spawn_robots.launch.py
======================
Modular launch file — Dynamic multi-robot spawning.

Dynamically generates and spawns N+1 TurtleBot3 Burger robots based on
the ``nBurger`` argument (number of follower robots).

  nBurger=1 → spawns tb1 (leader) + tb2 (follower)  = 2 total
  nBurger=2 → spawns tb1 (leader) + tb2 + tb3        = 3 total

For each robot this file starts:
  1. ``ros_gz_sim create`` — spawn namespaced SDF into Gazebo world
  2. ``robot_state_publisher``  — publish URDF-based TF tree
  3. ``ros_gz_bridge parameter_bridge`` — bridge scan / odom / cmd_vel /
                                          joint_states between ROS ↔ Gz

Topic naming strategy
---------------------
The standard model.sdf uses bare (relative) DiffDrive topic names such
as ``cmd_vel``, which Gazebo Sim resolves globally — causing multi-robot
topic collisions.

Fix: ``generate_sdf.generate_robot_sdf(ns)`` patches all plugin topic
strings to absolute paths (``/tb1/cmd_vel``, ``/tb1/odom``, …) before
spawning.  The bridge then maps these Gz topics directly to matching ROS
topics in namespace ``/tb1`` with NO remapping needed.

Bridge direction symbols (ros_gz_bridge argument-style):
  ``[``  Gz → ROS
  ``]``  ROS → Gz
  ``@``  bidirectional

cmd_vel uses ``TwistStamped`` on the ROS side (compatible with the
DiffDrive plugin's ``gz.msgs.Twist`` on the Gazebo side).

Args:
  nBurger     : int  1 or 2  (default: 2)
  use_sim_time: 'true' | 'false'  (default: 'true')

Implementation note
-------------------
``OpaqueFunction`` is the standard ROS 2 launch mechanism for generating
a variable-length list of actions at runtime (i.e., based on argument
values that are only resolved in the launch context, not at parse time).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─── Spawn position helpers ───────────────────────────────────────────────────

# Robots start in a straight line along the -X axis.
# tb1 at X=0, tb2 at X=-1.0, tb3 at X=-2.0, …
SPAWN_X_STEP = -1.0   # metres between robots
SPAWN_Y      =  0.0
SPAWN_Z      =  0.01  # slightly above ground to avoid immediate contact

# Stagger delay between spawning each robot (seconds).
# Prevents Gazebo physics instability from simultaneous heavy model loading.
SPAWN_DELAY_STEP = 3.0   # seconds


# ─── URDF reader ──────────────────────────────────────────────────────────────

def _read_urdf() -> str:
    """Return TurtleBot3 Burger URDF string from turtlebot3_description."""
    tb3_desc = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_burger.urdf')
    if not os.path.isfile(urdf_path):
        # Fallback to turtlebot3_gazebo path (Jazzy installs differ)
        tb3_gz = get_package_share_directory('turtlebot3_gazebo')
        urdf_path = os.path.join(tb3_gz, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        urdf_str = f.read()
    
    # In Jazzy, turtlebot3_description URDFs are technically xacro files 
    # that contain literal ${namespace}. We strip it, allowing RSP's 
    # frame_prefix to handle namespace scoping properly.
    return urdf_str.replace('${namespace}', '')


# ─── Per-robot action factory ─────────────────────────────────────────────────

def _make_robot_actions(
    ns: str,
    x: float,
    urdf: str,
    use_sim_time: bool,
) -> list:
    """
    Return a list of launch Actions for one TurtleBot3 robot.

    Actions (order matters — spawn must come before bridge):
      - ros_gz_sim create   : spawn the namespaced model into Gazebo
      - robot_state_publisher
      - ros_gz_bridge parameter_bridge

    Args:
        ns:           Robot namespace, e.g. 'tb1'.
        x:            X-axis spawn position [m].
        urdf:         URDF file contents as string.
        use_sim_time: Whether nodes should use simulation clock.
    """
    from multi_tb3_system.generate_sdf import generate_robot_sdf

    # Generate per-robot namespaced SDF (written to /tmp)
    sdf_path = generate_robot_sdf(ns)

    # ── 1. Spawn into Gazebo ──────────────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        arguments=[
            '-file', sdf_path,
            '-name', ns,          # Gazebo entity name = namespace
            '-x',   str(x),
            '-y',   str(SPAWN_Y),
            '-z',   str(SPAWN_Z),
            '-Y',   '0.0',        # yaw = 0 (facing +X)
        ],
        output='screen',
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    # Publishes TF from URDF joints. Frame names are prefixed with ns/
    # to keep each robot's TF tree isolated from others.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': urdf,
            # frame_prefix scopes all URDF-based frames: ns/base_link, etc.
            'frame_prefix':      f'{ns}/',
        }],
        output='screen',
    )

    # ── 3. ros_gz_bridge ─────────────────────────────────────────────────────
    # Each robot's Gazebo plugins publish to absolute topics (/tbX/scan, etc.)
    # thanks to generate_sdf patching.  No remapping is needed here — the
    # Gz topic name EQUALS the desired ROS topic name.
    #
    # Bridge argument format (single string per topic):
    #   /ros_topic@ROS_TYPE[gz_type    →  Gz → ROS
    #   /ros_topic@ROS_TYPE]gz_type    →  ROS → Gz
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            # LaserScan: Gazebo → ROS
            f'/{ns}/scan'
            + '@sensor_msgs/msg/LaserScan'
            + '[gz.msgs.LaserScan',

            # Odometry: Gazebo → ROS
            f'/{ns}/odom'
            + '@nav_msgs/msg/Odometry'
            + '[gz.msgs.Odometry',

            # cmd_vel: ROS → Gazebo
            # DiffDrive expects gz.msgs.Twist, mapped to geometry_msgs/msg/Twist.
            f'/{ns}/cmd_vel'
            + '@geometry_msgs/msg/Twist'
            + ']gz.msgs.Twist',

            # JointStates: Gazebo → ROS
            f'/{ns}/joint_states'
            + '@sensor_msgs/msg/JointState'
            + '[gz.msgs.Model',

            # TF: Gazebo → ROS  (odom → base_footprint frame pair)
            # All robots share the global /tf topic — frame names are unique
            # because generate_sdf patches frame_id/child_frame_id with ns/.
            '/tf'
            + '@tf2_msgs/msg/TFMessage'
            + '[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    return [spawn, rsp, bridge]


# ─── OpaqueFunction body ──────────────────────────────────────────────────────

def _launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and return the full list of spawn actions."""
    n_burgers    = int(LaunchConfiguration('nBurger').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    # Clamp to supported range
    n_burgers = max(0, min(n_burgers, 2))
    total     = n_burgers + 1  # leader + followers

    urdf = _read_urdf()
    all_actions = []

    for i in range(1, total + 1):
        ns    = f'tb{i}'
        x_pos = (i - 1) * SPAWN_X_STEP   # 0.0, -1.0, -2.0

        robot_actions = _make_robot_actions(ns, x_pos, urdf, use_sim_time)

        if i == 1:
            # Leader: spawn immediately
            all_actions.extend(robot_actions)
        else:
            # Followers: stagger to avoid Gazebo physics load spikes
            delay = (i - 1) * SPAWN_DELAY_STEP
            all_actions.append(TimerAction(period=delay, actions=robot_actions))

    return all_actions


# ─── Launch Description ───────────────────────────────────────────────────────

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'nBurger',
            default_value='2',
            description=(
                'Number of follower robots behind the leader. '
                'nBurger=1 → 2 robots total. '
                'nBurger=2 → 3 robots total. '
                'Maximum: 2.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description="Use Gazebo simulation clock ('true') or wall clock ('false').",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
