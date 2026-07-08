<div align="center">

# 🤖 Multi-TurtleBot3 Convoy System & RF2O Odometry

**Autonomous multi-robot leader–follower convoy with high-precision laser odometry**

*ROS 2 Jazzy · Gazebo Sim Harmonic · TurtleBot3 Burger*

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros) ![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square) ![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python) ![C++](https://img.shields.io/badge/C++-14-yellow?style=flat-square&logo=c%2B%2B) ![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat-square)

</div>

---

## Overview

This workspace contains a complete ROS 2 solution for deploying a **3-robot TurtleBot3 Burger convoy** in Gazebo Sim (Harmonic). The leader (`tb1`) is teleoperated and broadcasts its trajectory; followers (`tb2`, `tb3`) autonomously track the path using **Pure Pursuit** to maintain precise separation distances. Followers look ahead along the leader's path, making the system immune to visual occlusions and error accumulation.

To improve path-tracking accuracy, the workspace integrates a custom **RF2O Laser Odometry** package, decoupling the robots from wheel-slip-prone kinematic models in Gazebo and replacing it with high-speed (0.9ms) 2D LiDAR planar odometry.

## 📦 Workspace Packages

1. **`multi_tb3_system`**
   - The core Python package orchestrating the multi-robot convoy.
   - Handles staggered spawning, SDF patching, Pure Pursuit path tracking, and leader-follower logic.
   - Includes modular launch files (`robot.launch.py`) for single-command simulation startup.

2. **`rf2o_laser_odometry`**
   - A fast and precise C++ implementation of 2D laser odometry from planar laser scans.
   - Formulates the range flow constraint equation in terms of sensor velocity to estimate motion (ICRA 2016).
   - Fully integrated into the convoy system to enhance localization for all simulated TurtleBots.

---

## ✨ Key Features
- **Dynamic Convoy Size:** Launch with 1 or 2 followers (2-3 robots total).
- **Pure Pursuit Tracking:** Follows the path cleanly via `nav_msgs/Path`.
- **Laser-Based Odometry:** Enhanced localization via integrated RF2O.
- **Namespaced Bridging:** `ros_gz_bridge` maps `/tbX/*` topics directly without complex remapping.
- **Staggered Spawning:** Robots load sequentially to prevent physics instability.

---

## 🏗️ Architecture & Launch System

**Convoy Formation:** `tb1` (Leader) ◄── `tb2` (Follower, -1.0m) ◄── `tb3` (Follower, -2.0m)

**Launch Hierarchy:**
- `robot.launch.py` (Main Entry Point)
  - `worlds.launch.py` → `gazebo.launch.py` (Gazebo Server & GUI)
  - `spawn_robots.launch.py` (Patches SDFs, Spawns robots & RF2O nodes)
  - `followers.launch.py` (Starts autonomous path tracking for tb2/tb3)
  - `rviz.launch.py` (Optional visualization)

---

## 📋 Prerequisites & Build

**Requirements:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Gazebo Sim Harmonic, Python 3.12.

```bash
# Install Dependencies
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-gazebo ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-rviz2 ros-jazzy-tf2-tools

# Build Workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Quick Start

**1. Launch the Simulation**
```bash
# 3 robots, empty world (Headless)
ros2 launch multi_tb3_system robot.launch.py

# Full UI (Gazebo + RViz) with obstacles
ros2 launch multi_tb3_system robot.launch.py world:=pillars ros_ui:=true
```

**2. Teleoperate Leader (New Terminal)**
```bash
ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
```
*Controls:* `W/I` (forward), `X/,` (backward), `A/J` (left), `D/L` (right), `S/K` (stop). Hold to move, release to stop.

---

## 🎛️ Launch Arguments (`robot.launch.py`)

| Argument | Default | Description |
|---|---|---|
| `world` | `empty` | Gazebo world: `empty`, `pillars`, `office` |
| `nBurger` | `2` | Number of followers (1 or 2) |
| `use_sim_time`| `true` | Use simulation clock |
| `ros_ui` | `false` | Convenience flag to enable both Gazebo Viewer and RViz2 |

---