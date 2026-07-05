<div align="center">

# 🤖 Multi-TurtleBot3 Convoy System

*ROS 2 Jazzy · Gazebo Sim Harmonic · TurtleBot3 Burger*

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple?style=flat-square&logo=ubuntu)

> **University Assignment** — Hochschule Darmstadt · Prof. Dr.-Ing. Karl Kleinmann

</div>

---

## Overview

3-robot TurtleBot3 Burger convoy in Gazebo Sim. `tb1` is teleoperated; `tb2` and `tb3` follow using **Pure Pursuit** on a shared `nav_msgs/Path`. 

**Strictly Mapless Laser Odometry**: To combat Gazebo's simulated wheel slip without relying on a global SLAM map, this project integrates `rf2o_laser_odometry`. It performs Iterative Closest Point (ICP) scan-matching to perfectly sync the physical world with the universal `map` frame, allowing flawlessly accurate breadcrumb tracking across the convoy.

---

## Prerequisites

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-rviz2
```

Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Build

First, clone the laser odometry dependency into your workspace (if not already present):
```bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/ros2_ws
```

Then build the workspace:
```bash
colcon build --packages-select multi_tb3_system rf2o_laser_odometry
source install/setup.bash
```

---

## Quick Start

```bash
# Launch simulation
ros2 launch multi_tb3_system robot.launch.py world:=pillars ros_ui:=true

# In a second terminal — drive the leader
ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
```

Hold a key to move, release to stop. Followers start automatically.

---

## Launch Arguments

| Argument | Default | Description |
|---|:---:|---|
| `world` | `empty` | `empty`, `pillars`, `office` |
| `nBurger` | `2` | Follower count (1–2) |
| `convoy_spacing` | `0.6` | Gap per slot in metres |
| `ros_ui` | `false` | `true` = RViz + Gazebo GUI + costmap viz |
| `gz` | `false` | Gazebo GUI only |
| `rviz` | `false` | RViz + costmap viz only |
| `use_sim_time` | `true` | Sim clock or wall clock |

---

## Project Structure

```
multi_tb3_system/
├── scripts/                        # ROS executable nodes + pure-logic helpers
│   ├── convoy_publisher.py         # Publishes leader breadcrumb path (10 Hz)
│   ├── convoy_tracking.py          # Pure: goal-point walk, breadcrumb freshness
│   ├── costmap_generator.py        # Viz-only costmap node (gated: enable_costmap_viz)
│   ├── costmap_utils.py            # Pure: scan → occupancy grid, obstacle queries
│   ├── follower_node.py            # ROS shell: params, subs, pub, thin control loop
│   ├── follower_state.py           # Pure: state machine classify + command builders
│   ├── motion_controller.py        # Pure: Pure Pursuit + safety override (testable)
│   ├── safety_controller.py        # Pure: emergency stop / steering bias
│   └── teleop_controller.py        # Burst-mode keyboard teleop
├── multi_tb3_system/               # Importable Python package
│   ├── launch_common.py            # Convoy geometry constants (single source of truth)
│   └── generate_sdf.py             # Per-robot SDF generator
├── launch/
│   ├── robot.launch.py             # ⭐ Entry point
│   ├── followers.launch.py         # Starts convoy_publisher + costmap + followers
│   ├── spawn_robots.launch.py
│   └── rviz.launch.py
├── config/
│   └── follower_params.yaml        # All tunable parameters (single source of truth)
├── worlds/
│   ├── empty.world
│   └── pillars.world
└── models/turtlebot3_burger/
```

