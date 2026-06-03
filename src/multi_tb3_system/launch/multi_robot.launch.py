#!/usr/bin/env python3
"""
multi_robot.launch.py — legacy monolithic launcher (3 hardcoded robots).

Prefer robot.launch.py for new usage — it supports dynamic nBurger count.
This file kept for backwards compatibility with world_empty/world_obstacles launchers.

Args:
  world   : 'empty' (default) | 'columns'
  use_rviz: 'true' | 'false' (default true)
  use_gui : 'true' | 'false' (default true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _read_urdf() -> str:
    pkg = get_package_share_directory('turtlebot3_gazebo')
    with open(os.path.join(pkg, 'urdf', 'turtlebot3_burger.urdf'), 'r') as f:
        return f.read()


def _sdf_path() -> str:
    pkg = get_package_share_directory('multi_tb3_system')
    return os.path.join(pkg, 'models', 'turtlebot3_burger', 'model.sdf')


def _make_robot_actions(ns, x, y, urdf, sdf, params, is_follower) -> list:
    """Return [spawn, rsp, bridge] + optional follower node for one robot."""

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        arguments=['-file', sdf, '-name', ns,
                   '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', '0.0'],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[{'use_sim_time': True, 'robot_description': urdf, 'frame_prefix': ''}],
        output='screen',
    )

    # Bridge Gz /model/tbX/* topics → ROS /tbX/* via remappings
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            f'/model/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/model/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            (f'/model/{ns}/scan',         f'/{ns}/scan'),
            (f'/model/{ns}/odom',         f'/{ns}/odom'),
            (f'/model/{ns}/tf',           '/tf'),
            (f'/model/{ns}/cmd_vel',      f'/{ns}/cmd_vel'),
            (f'/model/{ns}/joint_states', f'/{ns}/joint_states'),
        ],
        output='screen',
    )

    actions = [spawn, rsp, bridge]

    if is_follower:
        actions.append(Node(
            package='multi_tb3_system',
            executable='follower_node.py',
            name='follower_node',
            namespace=ns,
            parameters=[params, {'use_sim_time': True}],
            output='screen',
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    pkg_share      = get_package_share_directory('multi_tb3_system')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf        = _read_urdf()
    sdf         = _sdf_path()
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    world_name = LaunchConfiguration('world')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_gui    = LaunchConfiguration('use_gui')

    world_file = PathJoinSubstitution([
        FindPackageShare('multi_tb3_system'), 'worlds',
        PythonExpression(["'", world_name, "' + '.world'"]),
    ])

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_file],
                          'on_exit_shutdown': 'true'}.items(),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(use_gui),
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('world',    default_value='empty',
                              description="World: 'empty' or 'columns'."),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description="Show RViz2."),
        DeclareLaunchArgument('use_gui',  default_value='true',
                              description="Show Gazebo GUI."),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(tb3_gazebo_pkg, 'models')),
        gz_server,
        gz_gui,
        clock_bridge,
    ])

    # tb1 leader — immediate; tb2/tb3 followers — staggered
    for action in _make_robot_actions('tb1', 0.0,  0.0, urdf, sdf, params_file, False):
        ld.add_action(action)
    ld.add_action(TimerAction(period=3.0,
                              actions=_make_robot_actions('tb2', -1.0, 0.0,
                                                         urdf, sdf, params_file, True)))
    ld.add_action(TimerAction(period=6.0,
                              actions=_make_robot_actions('tb3', -2.0, 0.0,
                                                         urdf, sdf, params_file, True)))
    ld.add_action(rviz_node)

    return ld
