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
       - robot_state_publisher  (in namespace /tbX, no frame_prefix)
       - ros_gz_bridge          (scan, odom, tf, cmd_vel, joint_states)
  5. Follower nodes for tb2 and tb3 (with use_sim_time=true)

Launch arguments:
  world    : 'empty' or 'columns' (default: empty)
  use_rviz : 'true'/'false'        (default: true)
  use_gui  : 'true'/'false'        (default: true)

Topic mapping after bridging:
  /tbX/scan        - LaserScan  from Gazebo (/model/tbX/scan)
  /tbX/odom        - Odometry   from Gazebo (/model/tbX/odom)
  /tf              - TF         from Gazebo (/model/tbX/tf)  [all robots → /tf]
  /tbX/cmd_vel     - Velocity   to   Gazebo (/model/tbX/cmd_vel)
  /tbX/joint_states- JointState from Gazebo (/model/tbX/joint_states)
  /clock           - Sim clock  from Gazebo

RECOMMENDED TELEOP (Burst-Mode):
  To teleoperate tb1 safely with "Hold-To-Move" functionality (stops immediately on release),
  run the custom teleop controller which natively publishes TwistStamped:
  ros2 run multi_tb3_system teleop_controller.py

TROUBLESHOOTING:
  - Check /cmd_vel types using: ros2 topic info /tb1/cmd_vel --verbose
  - Ensure teleop publishes geometry_msgs/msg/TwistStamped
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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ─── Helpers ──────────────────────────────────────────────────────────────────

def read_urdf() -> str:
    """Read TurtleBot3 Burger URDF from the installed turtlebot3_gazebo package."""
    pkg = get_package_share_directory('turtlebot3_gazebo')
    path = os.path.join(pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(path, 'r') as f:
        return f.read()


def burger_sdf_path() -> str:
    """Path to the locally modified turtlebot3_burger model.sdf for Gazebo Harmonic."""
    pkg = get_package_share_directory('multi_tb3_system')
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
      - ros_gz_sim create    : spawn the robot model in Gazebo
      - robot_state_publisher: publish TF from URDF joint states
      - ros_gz_bridge        : bridge scan / odom / tf / cmd_vel / joint_states
      - follower_node (optional, for tb2 and tb3 only)
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
    # NOTE: frame_prefix is intentionally left empty ('').
    # The TurtleBot3 Burger SDF DiffDrive plugin hardcodes:
    #   <frame_id>odom</frame_id>  <child_frame_id>base_footprint</child_frame_id>
    # Setting frame_prefix here would produce tb1/base_footprint etc., which
    # would NOT match the odom→base_footprint TF broadcast from Gazebo,
    # breaking the TF tree. All three robots share the same frame names in
    # Gazebo Sim; only the model namespaces differ for topics.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'use_sim_time':      True,
            'robot_description': urdf,
            'frame_prefix':      '',
        }],
        output='screen',
    )

    # ── ros_gz_bridge ─────────────────────────────────────────────────────────
    # When Gazebo spawns model named "tbX", it publishes:
    #   /model/tbX/scan         (LaserScan  — sensor topic "scan" in SDF)
    #   /model/tbX/odom         (Odometry   — odom_topic "odom" in SDF DiffDrive)
    #   /model/tbX/tf           (TFMessage  — tf_topic "/tf" in SDF, but scoped
    #                                         to model namespace in Gz Harmonic)
    #   /model/tbX/cmd_vel      (receives Twist from bridge)
    #   /model/tbX/joint_states (JointState — topic "joint_states" in SDF plugin)
    #
    # Bridge direction notation:
    #   @  = bidirectional
    #   [  = Gazebo → ROS only
    #   ]  = ROS → Gazebo only
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
            # FIX [1]: SDF odom_topic is "odom" → Gz publishes /model/tbX/odom
            #          (was incorrectly /model/tbX/odometry)
            f'/model/{ns}/odom'
            + '@nav_msgs/msg/Odometry'
            + '[gz.msgs.Odometry',

            # TF: Gazebo → ROS
            # FIX [3]: Added missing TF bridge — without this, no odom→base_footprint
            #          transform reaches ROS and RViz shows nothing.
            f'/model/{ns}/tf'
            + '@tf2_msgs/msg/TFMessage'
            + '[gz.msgs.Pose_V',

            # cmd_vel: ROS → Gazebo  (Twist on ROS side, Twist on Gz side)
            f'/model/{ns}/cmd_vel'
            + '@geometry_msgs/msg/Twist'
            + ']gz.msgs.Twist',

            # JointStates: Gazebo → ROS
            # FIX [2]: SDF joint_states topic is "joint_states" → Gz publishes
            #          /model/tbX/joint_states (was incorrectly /world/default/…)
            f'/model/{ns}/joint_states'
            + '@sensor_msgs/msg/JointState'
            + '[gz.msgs.Model',
        ],
        remappings=[
            # Map Gazebo namespaced topics to clean /tbX/* ROS topics
            (f'/model/{ns}/scan',         f'/{ns}/scan'),
            (f'/model/{ns}/odom',         f'/{ns}/odom'),          # FIX [1]
            (f'/model/{ns}/tf',           '/tf'),                   # FIX [3]: all robots → shared /tf
            (f'/model/{ns}/cmd_vel',      f'/{ns}/cmd_vel'),
            (f'/model/{ns}/joint_states', f'/{ns}/joint_states'),   # FIX [2]
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
            # FIX [8]: Added use_sim_time=True so the node uses /clock from Gazebo.
            # Without this, header.stamp is wall-clock time while the sim
            # runs on simulation time, causing QoS and timing mismatches.
            parameters=[params, {'use_sim_time': True}],
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
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true',
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
            'gz_args':          ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Gazebo GUI client (optional) ──────────────────────────────────────────
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':          '-g -v2 ',
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
