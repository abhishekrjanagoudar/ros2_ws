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

3-robot TurtleBot3 Burger convoy in Gazebo Sim. `tb1` is teleoperated; `tb2` and `tb3` follow using **Pure Pursuit** on a shared `nav_msgs/Path`. LiDAR is used only for emergency stop.

---

## Prerequisites

```bash
sudo apt install -y \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
```

Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Build

```bash
colcon build --packages-select multi_tb3_system
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
| `ros_ui` | `false` | `true` = RViz + Gazebo GUI |
| `gz` | `false` | Gazebo GUI only |
| `rviz` | `false` | RViz only |
| `use_sim_time` | `true` | Sim clock or wall clock |
| `enable_mapping` | `false` | Start slam_toolbox |
| `slam_robot` | `tb1` | `tb1`, `tb2`, `tb3`, `all` |

---

## Project Structure

```
multi_tb3_system/
├── scripts/
│   ├── convoy_publisher.py    # Publishes leader path (10 Hz)
│   ├── follower_node.py       # Pure Pursuit follower (50 Hz)
│   ├── safety_controller.py   # Emergency stop
│   └── teleop_controller.py   # Keyboard teleop
├── launch/
│   ├── robot.launch.py        # ⭐ Entry point
│   ├── followers.launch.py
│   ├── spawn_robots.launch.py
│   ├── gazebo.launch.py
│   ├── mapping.launch.py
│   └── rviz.launch.py
├── config/
│   ├── follower_params.yaml
│   └── mapping_online_async.yaml
├── worlds/
│   ├── empty.world
│   └── pillars.world
└── models/turtlebot3_burger/
```

---

## SLAM Mapping

```bash
# Launch with mapping
ros2 launch multi_tb3_system robot.launch.py enable_mapping:=true ros_ui:=true

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```
