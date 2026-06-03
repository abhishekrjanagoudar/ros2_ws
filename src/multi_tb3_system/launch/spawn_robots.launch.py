#!/usr/bin/env python3
"""
spawn_robots.launch.py — spawns N+1 TurtleBot3 robots (leader + followers).

Each robot gets: Gazebo entity + robot_state_publisher + ros_gz_bridge +
a static transform publisher anchoring world → tbX/odom at spawn position.
All odom frames share `world` as a common TF root so RViz can render all
robots with a single fixed frame.
generate_sdf patches all plugin topics to absolute /tbX/... paths — no remapping needed.

Args:
  nBurger     : follower count 1–2 (default 2)
  use_sim_time: 'true' (default) | 'false'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Spawn grid: tb1 at x=0, tb2 at x=-1.0, tb3 at x=-2.0
SPAWN_X_STEP     = -1.0   # m between robots
SPAWN_Y          =  0.0
SPAWN_Z          =  0.01  # slightly above ground
SPAWN_DELAY_STEP =  3.0   # seconds between successive spawns


def _read_urdf() -> str:
    """Load TurtleBot3 Burger URDF; strip ${namespace} for RSP frame_prefix."""
    tb3_desc = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_burger.urdf')
    if not os.path.isfile(urdf_path):
        tb3_gz = get_package_share_directory('turtlebot3_gazebo')
        urdf_path = os.path.join(tb3_gz, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        return f.read().replace('${namespace}', '')


def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool) -> list:
    """Return [spawn, rsp, bridge] actions for one robot."""
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
    n_burgers    = int(LaunchConfiguration('nBurger').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    n_burgers    = max(0, min(n_burgers, 2))
    total        = n_burgers + 1

    urdf = _read_urdf()
    all_actions = []

    for i in range(1, total + 1):
        ns      = f'tb{i}'
        x_pos   = (i - 1) * SPAWN_X_STEP
        actions = _make_robot_actions(ns, x_pos, urdf, use_sim_time)

        if i == 1:
            all_actions.extend(actions)   # leader spawns immediately
        else:
            delay = (i - 1) * SPAWN_DELAY_STEP
            all_actions.append(TimerAction(period=delay, actions=actions))

    return all_actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('nBurger',      default_value='2',
                              description='Follower count (1–2). Total = nBurger + 1.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description="'true' = Gz clock, 'false' = wall clock."),
        OpaqueFunction(function=_launch_setup),
    ])
