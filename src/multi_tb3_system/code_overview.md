# Folder Structure

`	ext
src/multi_tb3_system/
    package.xml
    CMakeLists.txt
    setup.py
    launch/
        world_obstacles.launch.py
        multi_robot.launch.py
        world_empty.launch.py
    multi_tb3_system/
        __init__.py
    src/
    models/
        turtlebot3_burger/
            model-1_4.sdf
            model.sdf
            model.config
    config/
        follower_params.yaml
        robot_ids.yaml
    worlds/
        empty.world
        columns.world
    rviz/
    include/
        multi_tb3_system/
    scripts/
        laser_processor.py
        teleop_controller.py
        safety_controller.py
        follower_node.py
`


# Source Code

## src/multi_tb3_system/CMakeLists.txt

`cmake
cmake_minimum_required(VERSION 3.8)
project(multi_tb3_system)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/follower_node.py
  scripts/teleop_controller.py
  scripts/laser_processor.py
  scripts/safety_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(DIRECTORY include/
  DESTINATION include
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
`

## src/multi_tb3_system/package.xml

`xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>multi_tb3_system</name>
  <version>0.1.0</version>
  <description>Multi-robot TurtleBot3 Burger leader-follower convoy system for ROS 2 Jazzy + Gazebo Sim</description>
  <maintainer email="student@h-da.de">Hochschule Darmstadt Student</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>ros_gz_bridge</depend>
  <depend>ros_gz_sim</depend>
  <depend>turtlebot3_gazebo</depend>
  <depend>turtlebot3_description</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>teleop_twist_keyboard</depend>
  <depend>rviz2</depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
`

## src/multi_tb3_system/setup.py

`py
from setuptools import setup, find_packages
package_name = 'multi_tb3_system'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hochschule Darmstadt Student',
    maintainer_email='student@h-da.de',
    description='Multi-robot TurtleBot3 Burger leader-follower convoy system',
    license='Apache-2.0',
    entry_points={},
)
`

## src/multi_tb3_system/launch/multi_robot.launch.py

`py
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
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            f'/model/{ns}/scan'
            + '@sensor_msgs/msg/LaserScan'
            + '[gz.msgs.LaserScan',
            f'/model/{ns}/odom'
            + '@nav_msgs/msg/Odometry'
            + '[gz.msgs.Odometry',
            f'/model/{ns}/tf'
            + '@tf2_msgs/msg/TFMessage'
            + '[gz.msgs.Pose_V',
            f'/model/{ns}/cmd_vel'
            + '@geometry_msgs/msg/Twist'
            + ']gz.msgs.Twist',
            f'/model/{ns}/joint_states'
            + '@sensor_msgs/msg/JointState'
            + '[gz.msgs.Model',
        ],
        remappings=[
            (f'/model/{ns}/scan',         f'/{ns}/scan'),
            (f'/model/{ns}/odom',         f'/{ns}/odom'),          # FIX [1]
            (f'/model/{ns}/tf',           '/tf'),                   # FIX [3]: all robots → shared /tf
            (f'/model/{ns}/cmd_vel',      f'/{ns}/cmd_vel'),
            (f'/model/{ns}/joint_states', f'/{ns}/joint_states'),   # FIX [2]
        ],
        output='screen',
    )
    actions = [spawn, rsp, bridge_node]
    if is_follower:
        follower = Node(
            package='multi_tb3_system',
            executable='follower_node.py',
            name='follower_node',
            namespace=ns,
            parameters=[params, {'use_sim_time': True}],
            output='screen',
        )
        actions.append(follower)
    return actions
def generate_launch_description() -> LaunchDescription:
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    pkg_share      = get_package_share_directory('multi_tb3_system')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    urdf        = read_urdf()
    sdf         = burger_sdf_path()
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')
    world_arg    = DeclareLaunchArgument('world',    default_value='empty',
                       description="World to load: 'empty' or 'columns'")
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true',
                       description='Start RViz2')
    use_gui_arg  = DeclareLaunchArgument('use_gui',  default_value='true',
                       description='Start Gazebo GUI client')
    world_name = LaunchConfiguration('world')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_gui    = LaunchConfiguration('use_gui')
    world_file = PathJoinSubstitution([
        FindPackageShare('multi_tb3_system'),
        'worlds',
        PythonExpression(["'", world_name, "' + '.world'"]),
    ])
    set_gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_pkg, 'models'),
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':          ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )
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
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    tb1_actions = make_robot_actions(
        ns='tb1', x=0.0, y=0.0,
        urdf=urdf, sdf=sdf, params=params_file,
        is_follower=False,
    )
    tb2_timer = TimerAction(
        period=3.0,
        actions=make_robot_actions(
            ns='tb2', x=-1.0, y=0.0,
            urdf=urdf, sdf=sdf, params=params_file,
            is_follower=True,
        ),
    )
    tb3_timer = TimerAction(
        period=6.0,
        actions=make_robot_actions(
            ns='tb3', x=-2.0, y=0.0,
            urdf=urdf, sdf=sdf, params=params_file,
            is_follower=True,
        ),
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )
    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_gui_arg)
    ld.add_action(set_gz_resource)
    ld.add_action(gz_server)
    ld.add_action(gz_gui)
    ld.add_action(clock_bridge)
    for action in tb1_actions:
        ld.add_action(action)
    ld.add_action(tb2_timer)
    ld.add_action(tb3_timer)
    ld.add_action(rviz_node)
    return ld
`

## src/multi_tb3_system/launch/world_empty.launch.py

`py
"""
world_empty.launch.py
=====================
Launches the Multi-TurtleBot3 convoy in the empty world (no obstacles).
Best for initial convoy formation testing.
Usage:
  ros2 launch multi_tb3_system world_empty.launch.py
  ros2 launch multi_tb3_system world_empty.launch.py use_rviz:=true
  ros2 launch multi_tb3_system world_empty.launch.py use_gui:=false
RECOMMENDED TELEOP (Burst-Mode):
  To teleoperate tb1 safely with "Hold-To-Move" functionality (stops immediately on release),
  run the custom teleop controller which natively publishes TwistStamped:
  ros2 run multi_tb3_system teleop_controller.py
TROUBLESHOOTING:
  - Check /cmd_vel types using: ros2 topic info /cmd_vel --verbose
  - Ensure teleop publishes geometry_msgs/msg/TwistStamped
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
def generate_launch_description():
    pkg_share = get_package_share_directory('multi_tb3_system')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 (set true to enable)',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gazebo GUI (set false for headless/WSL)',
    )
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'multi_robot.launch.py')
        ),
        launch_arguments={
            'world':    'empty',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_gui':  LaunchConfiguration('use_gui'),
        }.items(),
    )
    return LaunchDescription([
        use_rviz_arg,
        use_gui_arg,
        multi_robot_launch,
    ])
`

## src/multi_tb3_system/launch/world_obstacles.launch.py

`py
"""
world_obstacles.launch.py
=========================
Launches the Multi-TurtleBot3 convoy in the columns world (6 static pillars).
Used to verify that follower robots correctly reject static obstacles
and only track the moving leader robot ahead.
Usage:
  ros2 launch multi_tb3_system world_obstacles.launch.py
  ros2 launch multi_tb3_system world_obstacles.launch.py use_rviz:=true
  ros2 launch multi_tb3_system world_obstacles.launch.py use_gui:=false
RECOMMENDED TELEOP (Burst-Mode):
  To teleoperate tb1 safely with "Hold-To-Move" functionality (stops immediately on release),
  run the custom teleop controller which natively publishes TwistStamped:
  ros2 run multi_tb3_system teleop_controller.py
TROUBLESHOOTING:
  - Check /cmd_vel types using: ros2 topic info /cmd_vel --verbose
  - Ensure teleop publishes geometry_msgs/msg/TwistStamped
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
def generate_launch_description():
    pkg_share = get_package_share_directory('multi_tb3_system')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 (set true to enable)',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gazebo GUI (set false for headless/WSL)',
    )
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'multi_robot.launch.py')
        ),
        launch_arguments={
            'world':    'columns',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_gui':  LaunchConfiguration('use_gui'),
        }.items(),
    )
    return LaunchDescription([
        use_rviz_arg,
        use_gui_arg,
        multi_robot_launch,
    ])
`

## src/multi_tb3_system/models/turtlebot3_burger/model-1_4.sdf

`sdf
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="turtlebot3_burger">
    <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>

    <link name="base_footprint"/>

    <link name="base_link">

      <inertial>
        <pose>-0.032 0 0.070 0 0 0</pose>
        <inertia>
          <ixx>7.2397393e-01</ixx>
          <ixy>4.686399e-10</ixy>
          <ixz>-1.09525703e-08</ixz>
          <iyy>7.2397393e-01</iyy>
          <iyz>2.8582649e-09</iyz>
          <izz>6.53050163e-01</izz>
        </inertia>
        <mass>8.2573504e-01</mass>
      </inertial>

      <collision name="base_collision">
        <pose>-0.032 0 0.070 0 0 0</pose>
        <geometry>
          <box>
            <size>0.140 0.140 0.140</size>
          </box>
        </geometry>
      </collision>

      <visual name="base_visual">
        <pose>-0.032 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/burger_base.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
    </link>

    <link name="imu_link">
      <sensor name="tb3_imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
        <plugin name="turtlebot3_imu" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <!-- <namespace>/tb3</namespace> -->
            <remapping>~/out:=imu</remapping>
          </ros>
        </plugin>
      </sensor>
    </link>

    <link name="base_scan">
      <inertial>
        <pose>-0.020 0 0.161 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.114</mass>
      </inertial>

      <collision name="lidar_sensor_collision">
        <pose>-0.020 0 0.161 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.0508</radius>
            <length>0.055</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="lidar_sensor_visual">
        <pose>-0.032 0 0.171 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/lds.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>

      <sensor name="hls_lfcd_lds" type="ray">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <pose>-0.032 0 0.171 0 0 0</pose>
        <update_rate>5</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.120000</min>
            <max>3.5</max>
            <resolution>0.015000</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="turtlebot3_laserscan" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <!-- <namespace>/tb3</namespace> -->
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>base_scan</frame_name>
        </plugin>
      </sensor>
    </link>

    <link name="wheel_left_link">

      <inertial>
        <pose>0 0.08 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>1.8158194e-03</ixx>
          <ixy>-9.3392e-12</ixy>
          <ixz>1.04909e-11</ixz>
          <iyy>3.2922126e-03</iyy>
          <iyz>5.75694e-11</iyz>
          <izz>1.8158194e-03</izz>
        </inertia>
        <mass>2.8498940e-02</mass>
      </inertial>

      <collision name="wheel_left_collision">
        <pose>0 0.08 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_left_visual">
        <pose>0 0.08 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/tire.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
    </link>

    <link name="wheel_right_link">

      <inertial>
        <pose>0.0 -0.08 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>1.8158194e-03</ixx>
          <ixy>-9.3392e-12</ixy>
          <ixz>1.04909e-11</ixz>
          <iyy>3.2922126e-03</iyy>
          <iyz>5.75694e-11</iyz>
          <izz>1.8158194e-03</izz>
        </inertia>
        <mass>2.8498940e-02</mass>
      </inertial>

      <collision name="wheel_right_collision">
        <pose>0.0 -0.08 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_right_visual">
        <pose>0.0 -0.08 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/tire.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
    </link>

    <link name='caster_back_link'>
      <pose>-0.081 0 -0.004 -1.57 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.005000</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent>
      <child>base_link</child>
      <pose>0.0 0.0 0.010 0 0 0</pose>
    </joint>

    <joint name="wheel_left_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_left_link</child>
      <pose>0.0 0.08 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="wheel_right_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_right_link</child>
      <pose>0.0 -0.08 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name='caster_back_joint' type='ball'>
      <parent>base_link</parent>
      <child>caster_back_link</child>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
      <pose>-0.032 0 0.068 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>base_scan</child>
      <pose>-0.032 0 0.171 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">

      <ros>
        <!-- <namespace>/tb3</namespace> -->
      </ros>

      <update_rate>30</update_rate>

      <!-- wheels -->
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>0.160</wheel_separation>
      <wheel_diameter>0.066</wheel_diameter>

      <!-- limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <command_topic>cmd_vel</command_topic>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>

      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>

    </plugin>

    <plugin name="turtlebot3_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <!-- <namespace>/tb3</namespace> -->
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>wheel_left_joint</joint_name>
      <joint_name>wheel_right_joint</joint_name>
    </plugin>      
  </model>
</sdf>

`

## src/multi_tb3_system/models/turtlebot3_burger/model.config

`config
<?xml version="1.0"?>

<model>
  <name>TurtleBot3(Burger)</name>
  <version>2.0</version>
  <sdf version="1.4">model-1_4.sdf</sdf>
  <sdf version="1.5">model.sdf</sdf>

  <author>
    <name>Taehun Lim(Darby)</name>
    <email>thlim@robotis.com</email>
  </author>

  <description>
    TurtleBot3 Burger
  </description>
</model>

`

## src/multi_tb3_system/models/turtlebot3_burger/model.sdf

`sdf
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="turtlebot3_burger">
    <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>

    <link name="base_footprint"/>

    <link name="base_link">

      <inertial>
        <pose>-0.064 0 0.070 0 0 0</pose>
        <inertia>
          <ixx>1.9527e-02</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1.9527e-02</iyy>
          <iyz>0</iyz>
          <izz>1.9527e-02</izz>
        </inertia>
        <mass>8.2573504e-01</mass>
      </inertial>

      <collision name="base_collision">
        <pose>-0.032 0 0.070 0 0 0</pose>
        <geometry>
          <box>
            <size>0.14 0.14 0.14</size>
          </box>
        </geometry>
      </collision>

      <visual name="base_visual">
        <pose>-0.032 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/bases/burger_base.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1.0</ambient>
          <diffuse>0.3 0.3 0.3 1.0</diffuse>
        </material>
      </visual>
    </link>

    <link name="imu_link">
      <sensor name="tb3_imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <topic>imu</topic>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <link name="base_scan">
      <inertial>
        <pose>-0.020 0 0.161 0 0 0</pose>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.0001</iyy>
          <iyz>0.000</iyz>
          <izz>0.0001</izz>
        </inertia>
        <mass>0.114</mass>
      </inertial>

      <collision name="lidar_sensor_collision">
        <pose>-0.020 0 0.161 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.0508</radius>
            <length>0.055</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="lidar_sensor_visual">
        <pose>-0.032 0 0.171 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/sensors/lds.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient>
          <diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>

      <sensor name="hls_lfcd_lds" type="gpu_lidar">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <pose>-0.032 0 0.171 0 0 0</pose>
        <update_rate>5</update_rate>
        <topic>scan</topic>
        <gz_frame_id>base_scan</gz_frame_id>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.120000</min>
            <max>3.5</max>
            <resolution>0.015000</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <link name="wheel_left_link">

      <inertial>
        <pose>0 0.08 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>5.445e-05</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>5.445e-05</iyy>
          <iyz>0</iyz>
          <izz>1.089e-04</izz>
        </inertia>
        <mass>0.1</mass>
      </inertial>

      <collision name="wheel_left_collision">
        <pose>0 0.08 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_left_visual">
        <pose>0 0.08 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/wheels/left_tire.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient>
          <diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <link name="wheel_right_link">

      <inertial>
        <pose>0.0 -0.08 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>5.445e-05</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>5.445e-05</iyy>
          <iyz>0</iyz>
          <izz>1.089e-04</izz>
        </inertia>
        <mass>0.1</mass>
      </inertial>

      <collision name="wheel_right_collision">
        <pose>0.0 -0.08 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_right_visual">
        <pose>0.0 -0.08 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/wheels/right_tire.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient>
          <diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <link name='caster_back_link'>
      <pose>-0.081 0 -0.004 -1.57 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.00001</iyy>
          <iyz>0.000</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.005000</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent>
      <child>base_link</child>
      <pose>0.0 0.0 0.010 0 0 0</pose>
    </joint>

    <joint name="wheel_left_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_left_link</child>
      <pose>0.0 0.08 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <effort>20</effort>
        </limit>
      </axis>
    </joint>

    <joint name="wheel_right_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_right_link</child>
      <pose>0.0 -0.08 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <effort>20</effort>
        </limit>
      </axis>
    </joint>

    <joint name='caster_back_joint' type='ball'>
      <parent>base_link</parent>
      <child>caster_back_link</child>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
      <pose>-0.032 0 0.068 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>base_scan</child>
      <pose>-0.032 0 0.171 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">

      <!-- wheels -->
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>0.160</wheel_separation>
      <wheel_radius>0.033</wheel_radius>

      <!-- limits -->
      <max_linear_acceleration>1.0</max_linear_acceleration>



      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <odom_publisher_frequency>30</odom_publisher_frequency>

    </plugin>

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>wheel_left_joint</joint_name>
      <joint_name>wheel_right_joint</joint_name>
    </plugin>

  </model>
</sdf>

`

## src/multi_tb3_system/config/follower_params.yaml

`yaml
follower_node:
  ros__parameters:
    target_distance: 0.7
    safe_distance: 0.4
    kp_linear: 0.8
    kp_angular: 2.0
    max_linear_velocity: 0.22
    max_angular_velocity: 1.0
    front_angle_deg: 30.0
    cluster_distance: 0.20
    min_cluster_size: 2
    max_cluster_size: 40
`

## src/multi_tb3_system/config/robot_ids.yaml

`yaml
convoy:
  robots:
    - namespace: "tb1"
      role: "leader"
      description: "Teleoperated robot. Driven by keyboard input."
      initial_pose:
        x: 0.0
        y: 0.0
        yaw: 0.0
    - namespace: "tb2"
      role: "follower"
      follows: "tb1"
      description: "First autonomous follower. Follows tb1."
      initial_pose:
        x: -1.0
        y: 0.0
        yaw: 0.0
    - namespace: "tb3"
      role: "follower"
      follows: "tb2"
      description: "Second autonomous follower. Follows tb2."
      initial_pose:
        x: -2.0
        y: 0.0
        yaw: 0.0
`

## src/multi_tb3_system/worlds/columns.world

`xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="columns_world">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"     name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"     name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"         name="gz::sim::systems::Imu"/>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="column_1">
      <static>true</static>
      <pose>2.0 0.8 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="column_2">
      <static>true</static>
      <pose>2.0 -0.8 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="column_3">
      <static>true</static>
      <pose>3.5 0.6 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="column_4">
      <static>true</static>
      <pose>3.5 -0.6 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="column_5">
      <static>true</static>
      <pose>5.0 0.8 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="column_6">
      <static>true</static>
      <pose>5.0 -0.8 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>1.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
`

## src/multi_tb3_system/worlds/empty.world

`xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="empty_convoy">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"     name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"     name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"         name="gz::sim::systems::Imu"/>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
`

## src/multi_tb3_system/scripts/follower_node.py

`py
"""
follower_node.py
================
Autonomous LiDAR-based follower node for the Multi-TurtleBot3 convoy system.
This node is REUSABLE for both tb2 (follows tb1) and tb3 (follows tb2).
It is launched under the robot's own namespace so topic paths are resolved
correctly without any hard-coded robot names in this file.
Architecture:
  - Subscribes to: /scan  (LaserScan — relative to this robot's namespace)
  - Publishes to:  /cmd_vel (Twist — relative to this robot's namespace)
Algorithm (simple geometry, no temporal tracking):
  1. Receive LaserScan
  2. Convert ranges → Cartesian points
  3. Filter front-sector ± front_angle_deg
  4. Euclidean cluster segmentation
  5. Reject wall-like (large) and noise (tiny) clusters
  6. Select closest cluster → target (leader robot)
  7. PD control: drive toward target at target_distance
  8. Safety override: stop/steer if too close to anything
Parameters (loaded from follower_params.yaml via ros parameter server):
  target_distance      (float, default 0.7)
  safe_distance        (float, default 0.4)
  kp_linear            (float, default 0.8)
  kp_angular           (float, default 2.0)
  max_linear_velocity  (float, default 0.22)
  max_angular_velocity (float, default 1.0)
  front_angle_deg      (float, default 30.0)
  cluster_distance     (float, default 0.20)
  min_cluster_size     (int,   default 2)
  max_cluster_size     (int,   default 40)
Usage:
  ros2 run multi_tb3_system follower_node.py --ros-args -r __ns:=/tb2 \\
         --params-file /path/to/follower_params.yaml
"""
from __future__ import annotations
import math
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)
from laser_processor import process_scan, Cluster
from safety_controller import SafetyController
class FollowerNode(Node):
    """
    Autonomous follower node: subscribes to /scan, publishes /cmd_vel.
    When launched under namespace /tbX, this becomes /tbX/scan → /tbX/cmd_vel.
    """
    def __init__(self) -> None:
        super().__init__('follower_node')
        self.declare_parameter('target_distance',      0.7)
        self.declare_parameter('safe_distance',        0.4)
        self.declare_parameter('kp_linear',            0.8)
        self.declare_parameter('kp_angular',           2.0)
        self.declare_parameter('max_linear_velocity',  0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('front_angle_deg',      30.0)
        self.declare_parameter('cluster_distance',     0.20)
        self.declare_parameter('min_cluster_size',     2)
        self.declare_parameter('max_cluster_size',     40)
        self.target_distance     = self.get_parameter('target_distance').value
        self.safe_distance       = self.get_parameter('safe_distance').value
        self.kp_linear           = self.get_parameter('kp_linear').value
        self.kp_angular          = self.get_parameter('kp_angular').value
        self.max_linear_vel      = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel     = self.get_parameter('max_angular_velocity').value
        self.front_angle_deg     = self.get_parameter('front_angle_deg').value
        self.cluster_distance    = self.get_parameter('cluster_distance').value
        self.min_cluster_size    = self.get_parameter('min_cluster_size').value
        self.max_cluster_size    = self.get_parameter('max_cluster_size').value
        self.safety = SafetyController(
            safe_distance=self.safe_distance,
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel,
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',   # resolved to /tbX/scan via namespace
            self._scan_callback,
            scan_qos,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',  # resolved to /tbX/cmd_vel via namespace
            10,
        )
        self._no_target_count: int = 0   # frames without a target
        self.get_logger().info(
            f"FollowerNode started | "
            f"target_dist={self.target_distance}m | "
            f"front_cone=±{self.front_angle_deg}° | "
            f"safe_dist={self.safe_distance}m"
        )
    def _scan_callback(self, msg: LaserScan) -> None:
        """Process each incoming LaserScan and publish a velocity command."""
        target, all_clusters = process_scan(
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min if msg.range_min > 0 else 0.12,
            range_max=min(msg.range_max, 3.5),
            front_half_angle_deg=self.front_angle_deg,
            cluster_distance=self.cluster_distance,
            min_cluster_size=self.min_cluster_size,
            max_cluster_size=self.max_cluster_size,
        )
        linear_x, angular_z = self._compute_control(target)
        linear_x, angular_z = self.safety.check_and_modify(
            linear_x=linear_x,
            angular_z=angular_z,
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min if msg.range_min > 0 else 0.12,
        )
        self._publish_twist(linear_x, angular_z)
        if target:
            self._no_target_count = 0
            self.get_logger().debug(
                f"Target: dist={target.distance:.2f}m "
                f"angle={math.degrees(target.angle):.1f}° → "
                f"v={linear_x:.3f} ω={angular_z:.3f}"
            )
        else:
            self._no_target_count += 1
            if self._no_target_count % 20 == 1:
                self.get_logger().info(
                    f"No leader detected ({len(all_clusters)} clusters, "
                    f"none in front) — waiting..."
                )
    def _compute_control(self, target: Cluster | None) -> tuple[float, float]:
        """
        Proportional control law.
        When a target cluster is detected:
          Linear velocity  = Kp_lin * distance_error
            where distance_error = target.distance - target_distance
            (positive → too far  → move forward)
            (negative → too close → move backward / stop)
          Angular velocity = Kp_ang * angle_error
            where angle_error = target.angle (angle of centroid in robot frame)
            (positive angle → target is to the left → rotate left = +ω)
        When no target is detected:
          Stop (both velocities = 0).
        Returns:
            (linear_x, angular_z)
        """
        if target is None:
            return 0.0, 0.0
        distance_error = target.distance - self.target_distance
        angle_error    = target.angle
        linear_x  = self.kp_linear  * distance_error
        angular_z = self.kp_angular * angle_error
        if linear_x < 0.0:
            linear_x = max(linear_x, -0.05)   # slight backoff allowed
        return linear_x, angular_z
    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        """Publish a Twist message (required by Gazebo Sim bridge)."""
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_twist(0.0, 0.0)
        node.get_logger().info("FollowerNode shutting down — robot stopped.")
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
`

## src/multi_tb3_system/scripts/laser_processor.py

`py
"""
laser_processor.py
==================
Utility module for LiDAR data processing in the Multi-TurtleBot3 convoy system.
Responsibilities:
  - Convert LaserScan polar data to Cartesian points
  - Apply front-sector filtering (± angle_limit degrees)
  - Cluster points using Euclidean distance segmentation
  - Extract cluster centroids
  - Filter out wall-like (large) and noise (tiny) clusters
NO temporal tracking is performed here. This module is purely geometric.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
@dataclass
class Cluster:
    """Represents a group of LiDAR points that form a single detected object."""
    points: List[Tuple[float, float]]   # List of (x, y) in robot-local frame
    centroid_x: float                    # Mean x [m]
    centroid_y: float                    # Mean y [m]
    distance: float                      # Distance from robot origin [m]
    angle: float                         # Angle from robot heading [rad]
    size: int                            # Number of points in cluster
    def __repr__(self) -> str:
        return (f"Cluster(dist={self.distance:.2f}m, "
                f"angle={math.degrees(self.angle):.1f}°, "
                f"size={self.size})")
def scan_to_cartesian(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 3.5,
) -> List[Tuple[float, float]]:
    """
    Convert a LaserScan range array to a list of valid (x, y) Cartesian points
    in the robot's local coordinate frame.
    Robot convention:
      - x → forward
      - y → left
      - angle=0 → straight ahead
    Args:
        ranges:          Array of range measurements [m]
        angle_min:       Starting angle of the scan [rad]
        angle_increment: Angular step size per ray [rad]
        range_min:       Minimum valid range to accept [m]
        range_max:       Maximum valid range to accept [m]
    Returns:
        List of (x, y) tuples for valid points.
    """
    points = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        if r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((x, y))
    return points
def filter_front_sector(
    points: List[Tuple[float, float]],
    half_angle_deg: float = 30.0,
    min_x: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    Keep only points that are within the front-facing sector of the robot.
    The front sector is defined as:
      - x > min_x          (in front of the robot)
      - |angle| < half_angle_deg  (within ±half_angle_deg of straight ahead)
    Args:
        points:          List of (x, y) Cartesian points
        half_angle_deg:  Half-width of the front cone in degrees (default ±30°)
        min_x:           Minimum forward distance to include (default 0.0)
    Returns:
        Filtered list of (x, y) points in the front sector.
    """
    half_angle_rad = math.radians(half_angle_deg)
    filtered = []
    for (x, y) in points:
        if x < min_x:
            continue
        angle = math.atan2(y, x)
        if abs(angle) <= half_angle_rad:
            filtered.append((x, y))
    return filtered
def euclidean_cluster(
    points: List[Tuple[float, float]],
    cluster_distance: float = 0.20,
) -> List[List[Tuple[float, float]]]:
    """
    Segment a list of 2D points into clusters using Euclidean distance.
    A new cluster is started whenever the distance between consecutive points
    (in angle-sorted order) exceeds cluster_distance.
    Args:
        points:           List of (x, y) Cartesian points
        cluster_distance: Maximum gap between points in the same cluster [m]
    Returns:
        A list of clusters, where each cluster is a list of (x, y) tuples.
    """
    if not points:
        return []
    sorted_pts = sorted(points, key=lambda p: math.atan2(p[1], p[0]))
    clusters: List[List[Tuple[float, float]]] = []
    current_cluster: List[Tuple[float, float]] = [sorted_pts[0]]
    for i in range(1, len(sorted_pts)):
        prev = sorted_pts[i - 1]
        curr = sorted_pts[i]
        dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        if dist <= cluster_distance:
            current_cluster.append(curr)
        else:
            clusters.append(current_cluster)
            current_cluster = [curr]
    clusters.append(current_cluster)
    return clusters
def compute_centroid(cluster_points: List[Tuple[float, float]]) -> Tuple[float, float]:
    """
    Compute the mean (x, y) centroid of a cluster.
    Args:
        cluster_points: List of (x, y) point tuples.
    Returns:
        (cx, cy) centroid coordinates.
    """
    if not cluster_points:
        return (0.0, 0.0)
    xs = [p[0] for p in cluster_points]
    ys = [p[1] for p in cluster_points]
    return (sum(xs) / len(xs), sum(ys) / len(ys))
def make_clusters(
    raw_clusters: List[List[Tuple[float, float]]],
    min_cluster_size: int = 2,
    max_cluster_size: int = 40,
) -> List[Cluster]:
    """
    Convert raw point groups into Cluster objects, applying size filters.
    Filtering logic:
      - Clusters smaller than min_cluster_size → noise (rejected)
      - Clusters larger than max_cluster_size → wall / large surface (rejected)
      - Remaining clusters are converted to Cluster dataclass objects
    Args:
        raw_clusters:     List of point groups from euclidean_cluster()
        min_cluster_size: Minimum number of points to be a valid object
        max_cluster_size: Maximum number of points (above → wall/surface)
    Returns:
        List of Cluster objects representing valid detected objects.
    """
    result: List[Cluster] = []
    for pts in raw_clusters:
        n = len(pts)
        if n < min_cluster_size or n > max_cluster_size:
            continue
        cx, cy = compute_centroid(pts)
        dist = math.hypot(cx, cy)
        angle = math.atan2(cy, cx)
        result.append(Cluster(
            points=pts,
            centroid_x=cx,
            centroid_y=cy,
            distance=dist,
            angle=angle,
            size=n,
        ))
    return result
def select_target_cluster(clusters: List[Cluster]) -> Optional[Cluster]:
    """
    Select the best candidate cluster to follow.
    Strategy:
      - Among all valid clusters in the front sector, pick the closest one.
      - The assumption is that the robot being followed (leader) is the nearest
        object in the forward direction that isn't a wall (already filtered by
        max_cluster_size in make_clusters).
    Args:
        clusters: List of valid Cluster objects
    Returns:
        The closest Cluster, or None if no clusters exist.
    """
    if not clusters:
        return None
    return min(clusters, key=lambda c: c.distance)
def process_scan(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 3.5,
    front_half_angle_deg: float = 30.0,
    cluster_distance: float = 0.20,
    min_cluster_size: int = 2,
    max_cluster_size: int = 40,
) -> Tuple[Optional[Cluster], List[Cluster]]:
    """
    Full pipeline: raw LaserScan → target cluster.
    Steps:
      1. Convert scan to Cartesian points (filter invalid ranges)
      2. Filter points to front sector only
      3. Euclidean cluster segmentation
      4. Build Cluster objects (filter by size)
      5. Select closest cluster as target
    Args:
        ranges:               LaserScan.ranges array
        angle_min:            LaserScan.angle_min
        angle_increment:      LaserScan.angle_increment
        range_min:            Minimum range to accept
        range_max:            Maximum range to accept
        front_half_angle_deg: Front cone half-angle (degrees)
        cluster_distance:     Gap threshold for clustering
        min_cluster_size:     Minimum cluster size (noise filter)
        max_cluster_size:     Maximum cluster size (wall filter)
    Returns:
        (target_cluster, all_clusters)
        - target_cluster: The selected leader cluster, or None
        - all_clusters:   All valid front-sector clusters detected
    """
    points = scan_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max)
    front_points = filter_front_sector(points, half_angle_deg=front_half_angle_deg)
    raw_clusters = euclidean_cluster(front_points, cluster_distance=cluster_distance)
    clusters = make_clusters(raw_clusters, min_cluster_size, max_cluster_size)
    target = select_target_cluster(clusters)
    return target, clusters
`

## src/multi_tb3_system/scripts/safety_controller.py

`py
"""
safety_controller.py
====================
Safety layer for the Multi-TurtleBot3 convoy follower nodes.
Responsibilities:
  - Monitor the 180° front-half of the LaserScan for dangerously close obstacles
  - Issue emergency stop commands when something is within safe_distance
  - Apply gentle steering bias to avoid close obstacles on either side
  - Apply velocity limiting (clamp max linear and angular)
This module does NOT do any target tracking; it purely enforces hard limits.
"""
from __future__ import annotations
import math
from typing import List, Tuple
EMERGENCY_HALF_ANGLE_DEG = 45.0    # Check ±45° in front for emergency stop
STEER_HALF_ANGLE_DEG     = 60.0    # Check ±60° for steering bias
STEER_INFLUENCE_RANGE    = 0.8     # Obstacles within this range affect steering
class SafetyController:
    """
    Velocity safety layer.
    Use check_and_modify() to apply safety rules to any proposed velocity command
    before it is sent to the robot.
    """
    def __init__(
        self,
        safe_distance: float = 0.40,
        max_linear_vel: float = 0.22,
        max_angular_vel: float = 1.0,
    ) -> None:
        """
        Initialize the safety controller.
        Args:
            safe_distance:   Hard stop distance — if anything is closer than
                             this in front, set linear velocity to zero. [m]
            max_linear_vel:  Velocity clamp — never exceed this [m/s].
            max_angular_vel: Angular rate clamp [rad/s].
        """
        self.safe_distance   = safe_distance
        self.max_linear_vel  = max_linear_vel
        self.max_angular_vel = max_angular_vel
    def check_and_modify(
        self,
        linear_x: float,
        angular_z: float,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_min: float = 0.12,
    ) -> Tuple[float, float]:
        """
        Apply safety rules and return the (possibly modified) velocity pair.
        Rules applied in order:
          1. Emergency stop: if any obstacle in ±EMERGENCY_HALF_ANGLE_DEG
             is closer than safe_distance → stop linear motion, allow recovery.
          2. Steering bias: if obstacles are close on one side, nudge away.
          3. Velocity clamping: ensure |linear| ≤ max_linear_vel
                                and |angular| ≤ max_angular_vel.
        Args:
            linear_x:         Proposed linear velocity [m/s]
            angular_z:        Proposed angular velocity [rad/s]
            ranges:           LaserScan.ranges array
            angle_min:        LaserScan.angle_min [rad]
            angle_increment:  LaserScan.angle_increment [rad/rad]
            range_min:        Minimum valid range [m]
        Returns:
            (safe_linear_x, safe_angular_z)
        """
        emergency_half = math.radians(EMERGENCY_HALF_ANGLE_DEG)
        steer_half     = math.radians(STEER_HALF_ANGLE_DEG)
        min_left  = float('inf')
        min_right = float('inf')
        emergency = False
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < range_min:
                continue
            angle = angle_min + i * angle_increment
            if abs(angle) <= emergency_half:
                if r < self.safe_distance:
                    emergency = True
            if abs(angle) <= steer_half and r < STEER_INFLUENCE_RANGE:
                if angle >= 0:
                    min_left  = min(min_left,  r)
                else:
                    min_right = min(min_right, r)
        if emergency:
            linear_x = 0.0
        else:
            if min_left < STEER_INFLUENCE_RANGE and min_left < min_right:
                bias = (STEER_INFLUENCE_RANGE - min_left) / STEER_INFLUENCE_RANGE
                angular_z -= 0.5 * bias
            elif min_right < STEER_INFLUENCE_RANGE and min_right < min_left:
                bias = (STEER_INFLUENCE_RANGE - min_right) / STEER_INFLUENCE_RANGE
                angular_z += 0.5 * bias
        linear_x  = max(-self.max_linear_vel,  min(linear_x,  self.max_linear_vel))
        angular_z = max(-self.max_angular_vel,  min(angular_z, self.max_angular_vel))
        return linear_x, angular_z
    def get_emergency_stop_twist(self) -> Tuple[float, float]:
        """Return zero-velocity command (full stop)."""
        return 0.0, 0.0
`

## src/multi_tb3_system/scripts/teleop_controller.py

`py
"""
teleop_controller.py
====================
Burst-Mode (Hold-To-Move) keyboard teleoperation wrapper.
Publishes TwistStamped to cmd_vel at a continuous fixed rate.
FEATURE: BURST / HOLD-TO-MOVE
Unlike standard teleop nodes that latch velocities, this node requires the user 
to hold the key down to move. 
- Single key press → small movement burst
- Holding key      → continuous movement
- Releasing key    → publishes zero velocity continuously
This ensures the robot does not run away and gives precise control. It natively
publishes geometry_msgs/msg/Twist to cmd_vel (required by Gazebo Harmonic bridge).
USAGE:
Run this node in the namespace of the robot you want to control.
Example for tb1:
  ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
Key bindings:
  i / w  → forward
  , / x  → backward
  j / a  → rotate left
  l / d  → rotate right
  k / s  → stop immediately
  q / z  → increase / decrease linear and angular speeds simultaneously
  e / c  → increase / decrease angular speed only
  Ctrl+C → exit
"""
from __future__ import annotations
import sys
import tty
import termios
import threading
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
MOVE_BINDINGS = {
    'i': ( 1,  0),   ',': (-1,  0),   'j': ( 0,  1),   'l': ( 0, -1),   'k': ( 0,  0),
    'w': ( 1,  0),   'x': (-1,  0),   'a': ( 0,  1),   'd': ( 0, -1),   's': ( 0,  0),
    'I': ( 1,  0),   '<': (-1,  0),   'J': ( 0,  1),   'L': ( 0, -1),   'K': ( 0,  0),
    'W': ( 1,  0),   'X': (-1,  0),   'A': ( 0,  1),   'D': ( 0, -1),   'S': ( 0,  0),
}
SPEED_BINDINGS = {
    'q': (1.1,  1.1),   'z': (0.9,  0.9),   'e': (1.0,  1.1),   'c': (1.0,  0.9),
    'Q': (1.1,  1.1),   'Z': (0.9,  0.9),   'E': (1.0,  1.1),   'C': (1.0,  0.9),
}
MSG = """
╔════════════════════════════════════════════════════╗
║  TurtleBot3 Convoy — Burst-Mode Teleop             ║
╠════════════════════════════════════════════════════╣
║  Movement (Hold to move, release to stop!):        ║
║    i / w        → forward                          ║
║    , / x        → backward                         ║
║    j / a        → rotate left                      ║
║    l / d        → rotate right                     ║
║    k / s        → force stop                       ║
╠════════════════════════════════════════════════════╣
║  Speed adjustment:                                 ║
║    q / z        → overall faster / slower          ║
║    e / c        → angular faster / slower          ║
╠════════════════════════════════════════════════════╣
║  CTRL+C         → quit                             ║
╚════════════════════════════════════════════════════╝
"""
SPEED_MSG = "\rLinear: {lin:.2f} m/s  |  Angular: {ang:.2f} rad/s    "
def get_key(timeout: float = 0.05) -> str:
    """Read a single keypress in raw mode (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        import select
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
class TeleopController(Node):
    """
    Burst-mode keyboard teleop node.
    Publishes Twist to cmd_vel at a continuous fixed rate (20 Hz).
    """
    def __init__(self) -> None:
        super().__init__('teleop_controller')
        self.declare_parameter('max_linear_velocity',  0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('linear_speed_step',    0.01)
        self.declare_parameter('angular_speed_step',   0.1)
        self.max_lin = self.get_parameter('max_linear_velocity').value
        self.max_ang = self.get_parameter('max_angular_velocity').value
        self.lin_step = self.get_parameter('linear_speed_step').value
        self.ang_step = self.get_parameter('angular_speed_step').value
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.linear_speed  = 0.10
        self.angular_speed = 0.50
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_key_time = time.time()
        self.running = True
        publish_rate_hz = 20.0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._timer_callback)
        self.get_logger().info("Teleop Controller Started. Publishing at 20Hz.")
    def _timer_callback(self) -> None:
        """Publish Twist continuously. Zero out targets if key released."""
        now = time.time()
        if (now - self.last_key_time) > 0.15:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
        self._publish_twist(self.target_linear_x, self.target_angular_z)
    def run(self) -> None:
        """Main loop: read keys and update internal target commands."""
        print(MSG)
        print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)
        try:
            while rclpy.ok() and self.running:
                key = get_key(timeout=0.05)
                if key in MOVE_BINDINGS:
                    lin_dir, ang_dir = MOVE_BINDINGS[key]
                    self.target_linear_x  = lin_dir * self.linear_speed
                    self.target_angular_z = ang_dir * self.angular_speed
                    self.last_key_time = time.time()
                elif key in SPEED_BINDINGS:
                    lin_mult, ang_mult = SPEED_BINDINGS[key]
                    self.linear_speed  = min(self.max_lin, self.linear_speed  * lin_mult)
                    self.angular_speed = min(self.max_ang, self.angular_speed * ang_mult)
                    if self.target_linear_x != 0:
                        self.target_linear_x = math.copysign(self.linear_speed, self.target_linear_x)
                    if self.target_angular_z != 0:
                        self.target_angular_z = math.copysign(self.angular_speed, self.target_angular_z)
                    self.last_key_time = time.time()
                    print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)
                elif key == '\x03':   # Ctrl+C
                    break
        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")
        finally:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            for _ in range(5):
                self._publish_twist(0.0, 0.0)
                time.sleep(0.05)
            self.get_logger().info("TeleopController stopped — robot halted.")
    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
if __name__ == '__main__':
    main()
`
