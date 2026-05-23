#!/usr/bin/env python3
"""
multi_robot.launch.py
=====================
Master launch file for the Multi-TurtleBot3 Burger convoy system.

Starts:
  1. Gazebo Sim server (headless) + optional client (GUI)
  2. Global clock bridge (/clock)
  3. Three TurtleBot3 Burger robots with staggered spawning:
       tb1 at t=0s  (x=0.0,  y=0.0)
       tb2 at t=3s  (x=-1.0, y=0.0)
       tb3 at t=6s  (x=-2.0, y=0.0)
  4. Per-robot nodes:
       - robot_state_publisher  (in namespace /tbX)
       - ros_gz_bridge          (scan, odom, cmd_vel, joint_states)
  5. Follower nodes for tb2 and tb3

Launch arguments:
  world    : 'empty' or 'columns' (default: empty)
  use_rviz : 'true'/'false'        (default: false)
  use_gui  : 'true'/'false'        (default: true)

Topic mapping after bridging:
  /tbX/scan      - LaserScan from Gazebo (/model/tbX/scan)
  /tbX/odom      - Odometry  from Gazebo (/model/tbX/odometry)
  /tbX/cmd_vel   - Velocity  to   Gazebo (/model/tbX/cmd_vel)
  /clock         - Sim clock from Gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ─── Constants ────────────────────────────────────────────────────────────────

# Gazebo world name is embedded inside the .world XML
# (world name="empty_convoy" / "columns_world") — but we load by file path.

# ─── Helpers ──────────────────────────────────────────────────────────────────

def read_urdf() -> str:
    """Read TurtleBot3 Burger URDF from the installed turtlebot3_gazebo package."""
    pkg = get_package_share_directory('turtlebot3_gazebo')
    path = os.path.join(pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(path, 'r') as f:
        return f.read()


def burger_sdf_path() -> str:
    """Path to the unmodified turtlebot3_burger model.sdf."""
    pkg = get_package_share_directory('turtlebot3_gazebo')
    return os.path.join(pkg, 'models', 'turtlebot3_burger', 'model.sdf')


def make_robot_actions(
    ns: str,
    x: float,
    y: float,
    urdf: str,
    sdf: str,
    params: str,
    is_follower: bool,
) -> list:
    """
    Return a list of launch actions for one robot (no timing — caller wraps).

    Actions:
      - ros_gz_sim create   : spawn the robot model in Gazebo
      - robot_state_publisher : publish TF from URDF
      - ros_gz_bridge         : bridge scan / odom / cmd_vel
      - follower_node (optional)
    """

    # ── Spawn ─────────────────────────────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        arguments=[
            '-file', sdf,
            '-name', ns,         # model name in Gazebo = namespace
            '-x', str(x),
            '-y', str(y),
            '-z', '0.01',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'use_sim_time':      True,
            'robot_description': urdf,
            'frame_prefix':      f'{ns}/',
        }],
        output='screen',
    )

    # ── ros_gz_bridge ─────────────────────────────────────────────────────────
    # When Gazebo spawns model named "tbX", it publishes:
    #   /model/tbX/scan            (LaserScan — from the sensor topic in model.sdf)
    #   /model/tbX/odometry        (Odometry)
    #   /model/tbX/cmd_vel         (receives Twist)
    #   /world/default/model/tbX/joint_state  (JointState)
    #   /model/tbX/tf              (TF)
    #
    # Note: The sensor topic in the SDF is "scan". Gazebo Sim prepends /model/<name>/
    # automatically when using the default (non-absolute) topic name in the SDF sensor.
    #
    # We bridge each Gz topic and remap to /tbX/* on the ROS side.

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            # LaserScan: Gazebo → ROS
            f'/model/{ns}/scan'
            + '@sensor_msgs/msg/LaserScan'
            + '[gz.msgs.LaserScan',

            # Odometry: Gazebo → ROS
            f'/model/{ns}/odometry'
            + '@nav_msgs/msg/Odometry'
            + '[gz.msgs.Odometry',

            # cmd_vel: ROS → Gazebo  (TwistStamped on ROS side, Twist on Gz side)
            f'/model/{ns}/cmd_vel'
            + '@geometry_msgs/msg/TwistStamped'
            + ']gz.msgs.Twist',

            # JointStates: Gazebo → ROS
            f'/world/default/model/{ns}/joint_state'
            + '@sensor_msgs/msg/JointState'
            + '[gz.msgs.Model',
        ],
        remappings=[
            # Map Gazebo namespaced topics to clean /tbX/* ROS topics
            (f'/model/{ns}/scan',                          f'/{ns}/scan'),
            (f'/model/{ns}/odometry',                      f'/{ns}/odom'),
            (f'/model/{ns}/cmd_vel',                       f'/{ns}/cmd_vel'),
            (f'/world/default/model/{ns}/joint_state',     f'/{ns}/joint_states'),
        ],
        output='screen',
    )

    actions = [spawn, rsp, bridge_node]

    # ── Follower node ─────────────────────────────────────────────────────────
    if is_follower:
        follower = Node(
            package='multi_tb3_system',
            executable='follower_node.py',
            name='follower_node',
            namespace=ns,
            parameters=[params],
            output='screen',
        )
        actions.append(follower)

    return actions


# ─── Launch Description ───────────────────────────────────────────────────────

def generate_launch_description() -> LaunchDescription:

    # Ensure TurtleBot3 model env is set
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')

    pkg_share      = get_package_share_directory('multi_tb3_system')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf        = read_urdf()
    sdf         = burger_sdf_path()
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    world_arg    = DeclareLaunchArgument('world',    default_value='empty',
                       description="World to load: 'empty' or 'columns'")
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                       description='Start RViz2')
    use_gui_arg  = DeclareLaunchArgument('use_gui',  default_value='true',
                       description='Start Gazebo GUI client')

    world_name = LaunchConfiguration('world')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_gui    = LaunchConfiguration('use_gui')

    # ── World file path (evaluated at launch time) ────────────────────────────
    world_file = PathJoinSubstitution([
        FindPackageShare('multi_tb3_system'),
        'worlds',
        PythonExpression(["'", world_name, "' + '.world'"]),
    ])

    # ── GZ_SIM_RESOURCE_PATH: so Gazebo finds TurtleBot3 meshes/models ───────
    set_gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_pkg, 'models'),
    )

    # ── Gazebo Sim server (headless) ──────────────────────────────────────────
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':        ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Gazebo GUI client (optional) ──────────────────────────────────────────
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':        '-g -v2 ',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(use_gui),
    )

    # ── Global clock bridge ───────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Robot 1: tb1 (leader) — spawned immediately ───────────────────────────
    tb1_actions = make_robot_actions(
        ns='tb1', x=0.0, y=0.0,
        urdf=urdf, sdf=sdf, params=params_file,
        is_follower=False,
    )

    # ── Robot 2: tb2 (follower 1) — spawned after 3 s ────────────────────────
    tb2_timer = TimerAction(
        period=3.0,
        actions=make_robot_actions(
            ns='tb2', x=-1.0, y=0.0,
            urdf=urdf, sdf=sdf, params=params_file,
            is_follower=True,
        ),
    )

    # ── Robot 3: tb3 (follower 2) — spawned after 6 s ────────────────────────
    tb3_timer = TimerAction(
        period=6.0,
        actions=make_robot_actions(
            ns='tb3', x=-2.0, y=0.0,
            urdf=urdf, sdf=sdf, params=params_file,
            is_follower=True,
        ),
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # ── Assemble ──────────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_gui_arg)

    ld.add_action(set_gz_resource)
    ld.add_action(gz_server)
    ld.add_action(gz_gui)
    ld.add_action(clock_bridge)

    # tb1 actions added directly (no timer)
    for action in tb1_actions:
        ld.add_action(action)

    # tb2 and tb3 wrapped in TimerAction
    ld.add_action(tb2_timer)
    ld.add_action(tb3_timer)

    ld.add_action(rviz_node)

    return ld
