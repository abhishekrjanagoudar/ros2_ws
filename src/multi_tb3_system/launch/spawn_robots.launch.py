#!/usr/bin/env python3
"""
spawn_robots.launch.py — spawns N+1 TurtleBot3 robots (leader + followers).

Each robot gets: Gazebo entity + robot_state_publisher + ros_gz_bridge +
a static transform publisher anchoring world → tbX/odom at spawn position.
All odom frames share `world` as a common TF root so RViz can render all
robots with a single fixed frame.
generate_sdf patches all plugin topics to absolute /tbX/... paths — no remapping needed.

Formation geometry and staggered-spawn timing live in
``multi_tb3_system.launch_common`` (shared with followers.launch.py).

Args:
  nBurger     : follower count 1–2 (default 2)
  use_sim_time: 'true' (default) | 'false'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from multi_tb3_system.launch_common import (
    SPAWN_Y,
    SPAWN_Z,
    clamp_followers,
    read_burger_urdf,
    spawn_delay,
    spawn_x,
)


def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool) -> list:
    """Return [spawn, rsp, bridge, static_tf] actions for one robot."""
    from multi_tb3_system.generate_sdf import generate_robot_sdf

    sdf_path = generate_robot_sdf(ns)

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        arguments=[
            '-file', sdf_path,
            '-name', ns,
            '-x', str(x), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z),
            '-Y', '0.0',
        ],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': urdf,
            'frame_prefix':      f'{ns}/',   # scopes TF frames: ns/base_link, etc.
        }],
        output='screen',
    )

    # Bridge format: /topic@ROS_TYPE[gz_type (Gz→ROS) or ]gz_type (ROS→Gz)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    # Anchor tbX/odom to shared world frame at spawn position so all TF trees share a root.
    # Without this, tb2/tb3 odom frames are disconnected from tb1/odom → RViz errors.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_world_{ns}_odom',
        arguments=[
            str(x), '0', '0',   # translation: spawn position
            '0', '0', '0', '1', # rotation: identity quaternion
            'world', f'{ns}/odom',
        ],
        output='screen',
    )

    return [spawn, rsp, bridge, static_tf]


def _launch_setup(context, *args, **kwargs):
    n_burgers    = clamp_followers(int(LaunchConfiguration('nBurger').perform(context)))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    total        = n_burgers + 1

    urdf = read_burger_urdf()
    all_actions = []

    for i in range(1, total + 1):
        ns      = f'tb{i}'
        actions = _make_robot_actions(ns, spawn_x(i), urdf, use_sim_time)

        if i == 1:
            all_actions.extend(actions)   # leader spawns immediately
        else:
            all_actions.append(TimerAction(period=spawn_delay(i), actions=actions))

    return all_actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('nBurger',      default_value='2',
                              description='Follower count (1–2). Total = nBurger + 1.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description="'true' = Gz clock, 'false' = wall clock."),
        OpaqueFunction(function=_launch_setup),
    ])
