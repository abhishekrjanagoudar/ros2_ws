# ROS2 Multi-TurtleBot3 Project – Updated Progress Summary

## 1. Project Definition (Completed)
Defined system as a multi-robot TurtleBot3 Burger simulation in Gazebo Harmonic
Identified core system behavior:
* Robot 1 → manually controlled using teleoperation
* Robot 2 → autonomously follows Robot 1
* Robot 3 → autonomously follows Robot 2

Defined simulation environments:
* Empty world (baseline testing)
* Obstacle world with static columns/pillars

## 2. Core Technical Challenge Identified (Critical Requirement)

Follower robots must distinguish between:
* ✅ Dynamic robot target (leader TurtleBot)
* ❌ Static obstacles (pillars/columns/walls)

This became the central perception problem of the assignment.

The system therefore requires:
* LiDAR-based object detection
* Dynamic vs static object classification
* Motion consistency tracking
* Safe convoy distance maintenance
* Obstacle avoidance

## 3. Problem Statement (Completed)

A formal problem statement was created including:
* Multi-robot convoy behavior
* LiDAR-based environment perception
* Dynamic target tracking
* Static obstacle filtering
* Autonomous follower control
* Collision avoidance
* Safe inter-robot spacing
* Velocity and distance regulation

**Additional requirement added:**
The follower robots must identify whether detected LiDAR objects correspond to another TurtleBot3 robot or to static obstacles such as columns.

## 4. ROS2 Workspace & Package Architecture (Completed)
**Workspace Structure**
```text
ros2_ws/
 └── src/
     └── multi_tb3_system/
         ├── launch/
         ├── src/
         ├── scripts/
         ├── config/
         ├── worlds/
         ├── models/
         ├── rviz/
         ├── include/
         └── CMakeLists.txt
```

## 5. Planned Node Architecture
**Planned ROS Nodes**

| Node | Purpose |
| --- | --- |
| `teleop_controller` | Manual control of Robot 1 |
| `follower_node` | Leader-following control logic |
| `laser_processor` | LiDAR clustering + classification |
| `safety_controller` | Collision prevention + emergency stop |

## 6. ROS2 Package Design (Completed)
**Package Name**
`multi_tb3_system`

**Package Creation Command**
```bash
ros2 pkg create multi_tb3_system --build-type ament_cmake
```

## 7. Core ROS Dependencies Identified
**Required Dependencies**
* `rclcpp`
* `rclpy`
* `geometry_msgs`
* `sensor_msgs`
* `nav_msgs`
* `tf2`
* `tf2_ros`

## 8. ROS2 Environment Setup (Completed)
* **Operating System**: Ubuntu 24.04 LTS
* **ROS Distribution**: ROS 2 Jazzy Jalisco
* **Simulation Stack**: Gazebo Harmonic, `ros_gz_bridge`

## 9. TurtleBot3 Package Verification (Completed)

Verified available packages:
* `turtlebot3_gazebo`
* `turtlebot3_teleop`
* `turtlebot3_navigation2`
* `turtlebot3_cartographer`

## 10. Full Robotics Stack Installation (Completed)
**Installed Components**
```bash
sudo apt install ros-jazzy-desktop -y

sudo apt install -y \
ros-jazzy-turtlebot3 \
ros-jazzy-turtlebot3-simulations \
ros-jazzy-turtlebot3-gazebo \
ros-jazzy-turtlebot3-description \
ros-jazzy-turtlebot3-teleop \
ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup \
ros-jazzy-cartographer \
ros-jazzy-cartographer-ros \
ros-jazzy-teleop-twist-keyboard \
ros-jazzy-ros-gz \
ros-jazzy-ros-gz-bridge \
ros-jazzy-ros-gz-sim \
ros-jazzy-rviz2 \
ros-jazzy-tf2-tools \
ros-jazzy-xacro
```

## 11. Environment Configuration (Completed)
**TurtleBot3 Model Setup**
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

**Verify**
```bash
echo $TURTLEBOT3_MODEL
```
*Expected: `burger`*

## 12. Gazebo Simulation Launch (Working)
**Launch Command**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
*Simulation launches successfully.*

## 13. RViz Integration (Working)
**RViz Launch**
```bash
rviz2
```
OR
```bash
ros2 run rviz2 rviz2 \
-d $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/rviz/tb3_gazebo.rviz
```

## 14. ROS Topic Verification (Working)
**Important ROS Topics Confirmed**
* `/cmd_vel`
* `/scan`
* `/odom`
* `/tf`
* `/tf_static`
* `/joint_states`
* `/imu`

**Verification Command**
```bash
ros2 topic list
```

## 15. Major Debugging Discovery (Very Important)
**Critical ROS2 Jazzy + Gazebo Harmonic Issue**

Discovered that:
* `teleop` initially published: `geometry_msgs/msg/Twist`
* BUT Gazebo bridge expected: `geometry_msgs/msg/TwistStamped`

*Result:*
Robot did NOT move despite valid `/cmd_vel`.

## 16. Root Cause Analysis (Completed)
**Problem**
* `teleop_twist_keyboard` default behavior: `Twist`
* Gazebo bridge subscriber: `TwistStamped`

Message mismatch caused robot controller failure.

## 17. Final Teleop Fix (Solved)
**Correct Working Teleop Command**
We replaced the standard teleop node with a custom **Burst-Mode (Hold-to-Move)** teleop controller that natively publishes `geometry_msgs/msg/TwistStamped`:
```bash
ros2 run multi_tb3_system teleop_controller.py
```
This ensures safe convoy testing by stopping the robot immediately when a key is released.

## 18. Manual Motion Verification (Successful)
**Verified Robot Motion via Direct Command**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: ''
twist:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
"
```
**Confirmed:**
* Gazebo bridge functioning
* Robot controller functioning
* Simulation physics functioning

## 19. Important ROS Debugging Commands Learned
* **ROS Environment Check**: `echo $ROS_DISTRO`
* **Topic List**: `ros2 topic list`
* **Topic Info**: `ros2 topic info /cmd_vel --verbose`
* **Echo Topic**: `ros2 topic echo /cmd_vel`
* **Node List**: `ros2 node list`
* **Verify Model**: `echo $TURTLEBOT3_MODEL`

## 20. Current System Status

| Component | Status |
| --- | --- |
| ROS2 Jazzy | ✅ Working |
| Gazebo Harmonic | ✅ Working |
| TurtleBot3 Burger | ✅ Working |
| RViz2 | ✅ Working (Integrated in Launch) |
| Teleop | ✅ Working |
| cmd_vel Bridge | ✅ Working |
| LiDAR Topics | ✅ Working |
| Odometry | ✅ Working |
| Multi-Robot System | ✅ Working (Spawning & Bridge Setup) |
| Follower Algorithm | ⏳ In Progress |
| Object Classification | ⏳ Not Started |

## 21. Next Development Phase
**Completed Tasks**
* ✅ Spawn 3 TurtleBot3 robots
* ✅ Add namespaces: `/tb1`, `/tb2`, `/tb3`
* ✅ Separate topics: `/scan`, `/cmd_vel`, `/odom`
* ✅ Enable RViz2 automatically in launch files
* ✅ Fix WSL Gazebo GUI rendering issues (`use_gui:=false`)

**Immediate Next Tasks**

**Leader-Follower System**
* Robot 2 follows Robot 1
* Robot 3 follows Robot 2

**LiDAR Processing**
* Object clustering
* Dynamic object tracking
* Static obstacle filtering

**Safety Layer**
* Distance maintenance
* Collision avoidance
* Emergency stop logic

## 📌 Current Overall Status

| Area | Status |
| --- | --- |
| Problem Definition | ✅ Complete |
| System Architecture | ✅ Complete |
| ROS2 Installation | ✅ Complete |
| Gazebo Integration | ✅ Complete |
| TurtleBot3 Integration | ✅ Complete |
| Teleoperation | ✅ Complete |
| Debugging & Validation | ✅ Complete |
| Multi-Robot Implementation | ✅ Complete |
| Autonomous Following Logic | ⏳ In Progress |
| LiDAR Classification System | ⏳ Pending |
