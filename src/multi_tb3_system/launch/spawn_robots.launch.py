#!/usr/bin/env python3
"""
spawn_robots.launch.py — spawns N+1 TurtleBot3 robots (leader + followers).
"""

from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
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


def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool, is_leader: bool) -> list:
    """Return [spawn, rsp, bridge, static_tf] actions for one robot."""
    from multi_tb3_system.generate_sdf import generate_robot_sdf

    sdf_path = generate_robot_sdf(ns)

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        arguments=[
            '-world', 'default',    # explicit world name — avoids world-discovery
            '-file', sdf_path,      # timeout that can fail silently at t=3s
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
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        ros_arguments=['--log-level', 'ERROR'],
        output='screen',
    )

    # Bridge format: /topic@ROS_TYPE[gz_type (Gz→ROS) or ]gz_type (ROS→Gz)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
    )

    actions = [spawn, rsp, bridge]

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{ns}',
        arguments=[str(x), str(SPAWN_Y), str(SPAWN_Z), '0', '0', '0', 'world', f'{ns}/odom'],
        output='screen'
    )
    actions.append(static_tf)



    # Mapless Laser Odometry
    # Launched with a delay so Gazebo/Bridge can spin up and /scan exists
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name=f'rf2o_{ns}',
        namespace=ns,
        parameters=[{
            'use_sim_time': use_sim_time,
            'laser_scan_topic': f'/{ns}/scan',
            'odom_topic': f'/{ns}/odom',
            'publish_tf': True,
            'base_frame_id': f'{ns}/base_footprint',
            'odom_frame_id': f'{ns}/odom',
            'init_pose_from_topic': '',
            'freq': 30.0
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        ros_arguments=['--log-level', 'ERROR'],
        output='screen',
    )
    actions.append(TimerAction(
        period=3.0,
        actions=[rf2o],
        condition=IfCondition(LaunchConfiguration('enable_rf2o'))
    ))

    return actions


def _launch_setup(context, *args, **kwargs):
    n_burgers    = clamp_followers(int(LaunchConfiguration('nBurger').perform(context)))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    total        = n_burgers + 1

    urdf = read_burger_urdf()
    all_actions = []

    for i in range(1, total + 1):
        ns      = f'tb{i}'
        actions = _make_robot_actions(ns, spawn_x(i), urdf, use_sim_time, is_leader=(i == 1))

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
        DeclareLaunchArgument('enable_rf2o',  default_value='true',
                              description='Enable RF2O laser odometry nodes.'),
        OpaqueFunction(function=_launch_setup),
    ])
