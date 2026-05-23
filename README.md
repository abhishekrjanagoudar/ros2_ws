# Multi-Robot TurtleBot3 Leader–Follower System

<div align="center">
  <h3>Multi-Robot TurtleBot3 Burger Leader–Follower System with LiDAR-Based Target and Obstacle Differentiation</h3>
  <p>
    <strong>Hochschule Darmstadt - University of Applied Sciences</strong><br>
    <em>Under the guidance of Prof. Dr.-Ing. Karl Kleinmann</em>
  </p>
</div>

<hr>

## 🎯 Overview
This project implements a ROS 2 based multi-robot system using **TurtleBot3 Burger** robots in a **Gazebo simulation environment**.
The system consists of multiple robots operating in a coordinated convoy formation. The first robot is manually controlled (teleoperated), while the subsequent robots autonomously follow the preceding robot using **LiDAR (LaserScan)** data.

A key feature of this project is the follower robots' ability to reliably distinguish between the leading TurtleBot3 Burger (a dynamic moving target) and static environmental objects (such as columns or pillars), using only onboard sensor data.

## ✨ Key Features
- **Teleoperated Leader:** Robot 1 is controlled manually.
- **Autonomous Followers:** Robots 2 and 3 dynamically track and follow the robot immediately ahead.
- **LiDAR-based Target Classification:** Differentiates between moving targets (leader) and static obstacles (walls, columns) using LaserScan data.
- **Safe Convoy Navigation:** Maintains safe inter-robot spacing and velocity control to prevent collisions.
- **Custom Gazebo Environments:** Includes an empty world for baseline testing and an obstacle world with static columns.

## 📋 Prerequisites
- **OS:** Ubuntu 24.04
- **ROS 2:** Jazzy Jalisco
- **Gazebo:** Compatible simulator for ROS 2 Jazzy

### Install Dependencies
Install the required TurtleBot3 packages for ROS 2 Jazzy:
```bash
sudo apt update
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-teleop ros-jazzy-turtlebot3-gazebo
```

Ensure your TurtleBot3 model is set to Burger:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 🛠️ Build & Installation
1. Clone the repository into your ROS 2 workspace (e.g., `~/ros2_ws/src`):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone your repo here
```
2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select multi_tb3_system
```
3. Source the setup file:
```bash
source install/setup.bash
```

## 🚀 Quick Start / Usage

### 1. Launch the Simulation Environment
You can launch the system in either an empty world or a world with obstacles. **Note: RViz2 will start automatically by default.**

**Option A: Empty World (Baseline Formation Testing)**
```bash
ros2 launch multi_tb3_system world_empty.launch.py
```

**Option B: Obstacle World (Columns Environment)**
```bash
ros2 launch multi_tb3_system world_obstacles.launch.py
```

**Option C: Main Multi-Robot Launch**
```bash
ros2 launch multi_tb3_system multi_robot.launch.py
```

**Running Headless (WSL 2 or No Display)**
If you are running in WSL or a headless environment and experience OpenGL/GUI errors, you can disable the Gazebo 3D GUI while keeping the simulation and RViz running:
```bash
ros2 launch multi_tb3_system world_empty.launch.py use_gui:=false
```

### 2. Teleoperate the Leader Robot
In a new terminal, source your workspace and run the custom **Burst-Mode** teleop controller to move the leader robot (Robot 1):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_tb3_system teleop_controller.py
```

> **💡 Burst / Hold-to-Move Mode:**
> Unlike the standard `teleop_twist_keyboard` (which causes continuous movement after a single key press), this project uses a custom teleop node. The robot will **only move while the key is held down** and will immediately stop when the key is released. This provides much finer control and safety when testing convoy behaviors. It also natively publishes the `geometry_msgs/msg/TwistStamped` messages required by Gazebo Harmonic.

### 3. Run Follower Logic (If not automatically started)
If you need to manually start the follower and safety nodes:
```bash
ros2 run multi_tb3_system laser_processor.py
ros2 run multi_tb3_system follower_node.py
ros2 run multi_tb3_system safety_controller.py
```

## ⚠️ Troubleshooting

**Robot Not Moving in Gazebo?**
If the teleoperated robot refuses to move, it is likely a message type mismatch. Gazebo Harmonic expects `geometry_msgs/msg/TwistStamped`, but default teleop nodes use `geometry_msgs/msg/Twist`.

1. Check the `/cmd_vel` message type:
   ```bash
   ros2 topic info /cmd_vel --verbose
   ```
2. Ensure your teleop node publishes `geometry_msgs/msg/TwistStamped`. The included `teleop_controller.py` handles this automatically, but if you attempt to use the standard `teleop_twist_keyboard`, you must append `--ros-args -p stamped:=true`.

## ⚙️ Configuration
The system behavior can be tuned via the configuration files located in the `src/multi_tb3_system/config/` directory.

### `config/follower_params.yaml`
Adjusts the tracking parameters, safe distance, and velocity limits for the follower robots.
```yaml
follower_node:
  ros__parameters:
    safe_distance: 0.5      # Target distance to maintain from the leader (meters)
    max_velocity: 0.22      # Maximum linear velocity (m/s)
    cluster_tolerance: 0.1  # Distance tolerance for LiDAR clustering
```

### `config/robot_ids.yaml`
Configures the namespaces and relationships between the robots in the convoy.
```yaml
robots:
  - id: "tb1"
    role: "leader"
  - id: "tb2"
    role: "follower"
    target: "tb1"
  - id: "tb3"
    role: "follower"
    target: "tb2"
```

## 🏗️ System Architecture (Nodes)
- **`teleop_controller.py`**: Manual control node for the leader robot.
- **`follower_node.py`**: Core logic for autonomous following, computing command velocities based on the identified target.
- **`laser_processor.py`**: Processes raw LiDAR (`/scan`) data, performs clustering, and classifies points as either dynamic targets (leader) or static obstacles.
- **`safety_controller.py`**: Monitors distances and enforces velocity limits to prevent collisions.
