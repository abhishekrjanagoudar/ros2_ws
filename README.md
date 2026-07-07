<div align="center">

# 🤖 Multi-TurtleBot3 Convoy System

**Autonomous multi-robot leader–follower convoy using Pure Pursuit path tracking**

*ROS 2 Jazzy · Gazebo Sim Harmonic · TurtleBot3 Burger*

---

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple?style=flat-square&logo=ubuntu)

</div>

---

## Overview

A production-grade modular ROS 2 package implementing a **3-robot TurtleBot3 Burger convoy** in Gazebo Sim (Harmonic). The leader robot (`tb1`) is teleoperated via keyboard and broadcasts its trajectory; follower robots (`tb2`, `tb3`) autonomously track the path using **Pure Pursuit** to maintain precise separation distances.

Followers look ahead along the leader's path, rendering the system completely immune to visual occlusions, obstacle interference, or daisy-chain error accumulation. LiDAR is strictly used for emergency local collision avoidance.

> **University Assignment** — Hochschule Darmstadt  
> Supervised by Prof. Dr.-Ing. Karl Kleinmann

---

## ✨ Features

| Capability | Details |
|---|---|
| **Dynamic convoy size** | Launch with `nBurger=1` or `nBurger=2` for 2–3 total robots; single entry point |
| **Pure pursuit path tracking** | Robust gap maintenance via nav_msgs/Path published by the orchestrator |
| **Convoy architecture** | All robots follow a single shared path, preventing error accumulation |
| **Burst-mode teleop** | Hold-to-move keyboard control with instant stop on release |
| **Modular launch system** | Six clean, independent launch files orchestrated by `robot.launch.py` |
| **Shared configuration** | Single source of truth for formation geometry and spawn timing via `launch_common.py` |
| **Zero topic collisions** | Per-robot SDF patching eliminates shared Gazebo transport topics |
| **Namespaced bridging** | `ros_gz_bridge` maps `/tbX/*` ROS topics ↔ Gazebo with no remapping |
| **Staggered spawning** | Robots load with 3-second gaps to prevent physics instability |
| **Synchronized follower start** | Followers begin driving 2 seconds after their robot spawns (bridge settle time) |

---

## 🏗️ System Architecture

### Convoy Formation

```
  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
  │     tb1      │   │     tb2      │   │     tb3      │
  │   (Leader)   │   │  (Follower)  │   │  (Follower)  │
  │              │   │              │   │              │
  │  Teleop KB   │◄──│  Pure Pursuit│◄──│ Pure Pursuit │
  │  → path pub  │   │  → cmd_vel   │   │  → cmd_vel   │
  └──────────────┘   └──────────────┘   └──────────────┘
      x = 0.0 m          x = -1.0 m         x = -2.0 m
```

### Launch System (Modular & Clean)

```
robot.launch.py  ⭐ ENTRY POINT (dynamic orchestrator)
│
├── worlds.launch.py ──────────────► gazebo.launch.py
│                                         │
│                                    ┌────┴─────────────────┐
│                                    │  Gazebo Sim Server   │
│                                    │  Optional GUI        │
│                                    │  /clock bridge       │
│                                    └──────────────────────┘
│
├── spawn_robots.launch.py (OpaqueFunction)
│        │
│        └── [per robot: tb1 … tbN]
│               ├── generate_sdf.py ── patches SDF topics per-robot
│               ├── ros_gz_sim create  spawn into Gazebo world
│               ├── robot_state_publisher
│               ├── ros_gz_bridge ──── /tbX/scan, /tbX/odom, /tbX/cmd_vel
│               └── static_transform_publisher ── /world → /tbX/odom TF anchor
│
├── followers.launch.py (OpaqueFunction)
│        └── [per follower: tb2 … tbN]
│               └── follower_node.py ◄── Pure pursuit path tracking
│
└── rviz.launch.py  (conditional on rviz=true / ros_ui:=true)
```
---

## 📋 Prerequisites

### System Requirements

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo Sim | Harmonic (`gz-sim 8.x`) |
| Python | 3.12 |
| TurtleBot3 Model | `burger` |

### Install Dependencies

```bash
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \

```

### Environment Setup

Add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## 🛠️ Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select multi_tb3_system
source install/setup.bash
```

Expected output:
```
Starting >>> multi_tb3_system
Finished <<< multi_tb3_system [~0.5s]
Summary: 1 package finished
```

---

## 🚀 Quick Start

### Step 1 — Launch the simulation

```bash
# Headless (no GUI windows) — 3 robots, empty world
ros2 launch multi_tb3_system robot.launch.py

# Full UI: Gazebo viewer + RViz
ros2 launch multi_tb3_system robot.launch.py ros_ui:=true

# Obstacle course + full UI
ros2 launch multi_tb3_system robot.launch.py world:=pillars ros_ui:=true
```

Wait for all three robots to appear (tb1 at t=0s, tb2 at t+5s, tb3 at t+10s, followers start driving at t+7s and t+12s).

### Step 2 — Teleoperate the leader

Open a **new terminal**, then:

```bash
ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
```

**Hold keys to move, release to stop immediately:**

| Key | Action |
|:---:|---|
| `w` / `i` | Move forward |
| `x` / `,` | Move backward |
| `a` / `j` | Rotate left |
| `d` / `l` | Rotate right |
| `s` / `k` | Full stop |

> **Why `--ros-args -r __ns:=/tb1`?**  
> The teleop node publishes to `cmd_vel` (relative). Remapping the namespace resolves this to `/tb1/cmd_vel`, which the bridge forwards to Gazebo. Without the namespace, no robot moves.

---

## 🎛️ Launch Arguments

All arguments are declared on `robot.launch.py` — the **single entry point** for the entire convoy system.

| Argument | Type | Default | Allowed Values | Description |
|---|:---:|:---:|---|---|
| `world` | string | `empty` | `empty`, `pillars`, `office` | Gazebo world to load |
| `nBurger` | int | `2` | `1`, `2` | Number of follower robots (total = nBurger + 1) |
| `use_sim_time` | bool | `true` | `true`, `false` | Use Gazebo simulation clock or wall clock |
| `gz` | bool | `false` | `true`, `false` | Show Gazebo 3D viewer window |
| `rviz` | bool | `false` | `true`, `false` | Show RViz2 visualization |
| `ros_ui` | bool | `false` | `true`, `false` | **Convenience override:** `true` → forces `gz=true` + `rviz=true` |
| `enable_rf2o` | bool | `true` | `true`, `false` | Enable RF2O laser odometry for high-precision tracking |



---

## 📡 ROS Topic Architecture

### Per-Robot Topics (X = 1, 2, 3)

| Topic | Type | Direction | Publisher | Subscriber |
|---|---|:---:|---|---|
| `/tbX/scan` | `sensor_msgs/LaserScan` | Gz → ROS | Gazebo LiDAR sensor | `follower_node` |
| `/tbX/odom` | `nav_msgs/Odometry` | Gz → ROS | Gazebo DiffDrive / `rf2o_laser_odometry` | `convoy_publisher` / `follower_node` |
| `/tbX/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gz | `teleop_controller` / `follower_node` | Gazebo DiffDrive |
| `/tbX/joint_states` | `sensor_msgs/JointState` | Gz → ROS | Gazebo JointStatePublisher | `robot_state_publisher` |

### Global Topics

| Topic | Type | Direction | Description |
|---|---|:---:|---|
| `/tf` | `tf2_msgs/TFMessage` | Gz → ROS | All robot transforms (frames prefixed `tbX/`) |
| `/clock` | `rosgraph_msgs/Clock` | Gz → ROS | Simulation time for `use_sim_time=true` |

### ros_gz_bridge Mapping

```
Gazebo Transport                         ROS 2
─────────────────────────────────────────────────────
/tb1/scan       (gz.msgs.LaserScan)  ──► /tb1/scan
/tb1/odom       (gz.msgs.Odometry)   ──► /tb1/odom
/tb1/cmd_vel    (gz.msgs.Twist)      ◄── /tb1/cmd_vel   [Twist]
/tb1/joint_states (gz.msgs.Model)    ──► /tb1/joint_states
/tf             (gz.msgs.Pose_V)     ──► /tf
/clock          (gz.msgs.Clock)      ──► /clock

[same pattern for /tb2/*, /tb3/*]
```

> **Twist type:**  
> ROS 2 Jazzy uses `geometry_msgs/Twist` for `cmd_vel` across the bridge. `ros_gz_bridge` automatically handles conversion to `gz.msgs.Twist` for the DiffDrive plugin. Both teleop and follower nodes publish `Twist`.

---

## 🗂️ Project Structure

```
ros2_ws/src/multi_tb3_system/
│
├── multi_tb3_system/               (Python package)
│   ├── __init__.py
│   ├── perception/
│   │   └── laser_processor.py      # Scan clustering library (future use)
│   ├── generate_sdf.py             # Per-robot SDF topic patching (multi-robot isolation)
│   └── launch_common.py            # ⭐ Shared config: spawn geometry, timing, helpers
│
├── rf2o_laser_odometry/            (C++ package)
│   ├── src/                        # Fast 2D laser odometry implementation
│   └── launch/                     # ROS 2 node launchers
│
├── scripts/                        (Executable ROS nodes)
│   ├── follower_node.py            # Autonomous follower (reusable for tb2, tb3, …)
│   ├── safety_controller.py        # Emergency stop + velocity limiter
│   └── teleop_controller.py        # Burst-mode keyboard teleop for tb1
│
├── launch/                         (ROS 2 launch files)
│   ├── robot.launch.py             # ⭐ ENTRY POINT (single orchestrator)
│   ├── worlds.launch.py            # World name → file path → gazebo.launch.py
│   ├── gazebo.launch.py            # Gz server + optional GUI + clock bridge
│   ├── spawn_robots.launch.py      # Dynamic N+1 robot spawning (OpaqueFunction)
│   ├── followers.launch.py         # Follower node launcher (OpaqueFunction)
│   └── rviz.launch.py              # RViz2 visualization
│
├── config/
│   ├── follower_params.yaml        # PD gains, distances, LiDAR thresholds
│   └── cpr_office/                 # CPR Office world with custom mesh & textures
│
├── worlds/
│   ├── empty.world                 # Flat ground plane
│   ├── columns.world               # Flat ground (alias for testing)
│   └── pillars.world               # 6 static cylindrical obstacles (slalom)
│
├── models/
│   └── turtlebot3_burger/
│       ├── model.sdf               # Local SDF template (patched at launch time)
│       ├── model-1_4.sdf           # Legacy SDF (SDF 1.4)
│       └── model.config            # Model metadata
│
├── rviz/
│   └── multi_robot.rviz            # Pre-configured: 3× LaserScan + Odometry + TF
│
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md                       # This file
├── DOCUMENTATION.md                # Detailed system documentation
└── LAUNCH_CLEANUP.md               # Refactoring changelog & migration guide
---

## ⚙️ Follower Parameters

Tunable in [`config/follower_params.yaml`](config/follower_params.yaml):

| Parameter | Default | Unit | Description |
|---|:---:|:---:|---|
| `target_distance` | `0.7` | m | Desired separation from the robot ahead |
| `safe_distance` | `0.4` | m | Emergency stop threshold |
| `kp_linear` | `0.8` | — | Linear proportional gain |
| `kp_angular` | `2.0` | — | Angular proportional gain |
| `max_linear_velocity` | `0.22` | m/s | Hard ceiling (TurtleBot3 Burger hardware limit) |
| `max_angular_velocity` | `1.0` | rad/s | Hard angular ceiling |
| `front_angle_deg` | `30.0` | ° | Half-width of forward detection cone (±30° from center) |
| `cluster_distance` | `0.20` | m | Max intra-cluster point gap for Euclidean clustering |
| `min_cluster_size` | `2` | pts | Below this → noise, rejected |
| `max_cluster_size` | `40` | pts | Above this → wall/floor, rejected (TB3 body ≈ 8–12 pts at 0.7m) |

---

## 🧩 Modular Sub-Launch Files

Each sub-launch file is independently usable for testing/debugging:

```bash
# Start only Gazebo with the pillars world
ros2 launch multi_tb3_system worlds.launch.py world:=pillars gz:=true

# Spawn robots only (Gazebo must already be running)
ros2 launch multi_tb3_system spawn_robots.launch.py nBurger:=2

# Start follower nodes only (robots must already be spawned and bridged)
ros2 launch multi_tb3_system followers.launch.py nBurger:=2

# Open RViz only (system must already be running)
ros2 launch multi_tb3_system rviz.launch.py
```

---

## 🔍 Monitoring & Debugging

```bash
# List all active topics
ros2 topic list

# Verify scan data is arriving at expected rate
ros2 topic hz /tb2/scan

# Watch follower commands in real-time
ros2 topic echo /tb2/cmd_vel

# Check bridge is running
ros2 node list | grep bridge

# Inspect TF tree for frame hierarchy
ros2 run tf2_tools view_frames

# Check simulation time (should increase steadily)
ros2 topic echo /clock --once

# Monitor follower_node logs
ros2 launch multi_tb3_system robot.launch.py ros_ui:=true
# Then in another terminal:
ros2 node list | grep follower
ros2 node info /tb2/follower_node
```

---

## 🛠️ Troubleshooting

### Robot does not move after teleop command

| Symptom | Likely cause | Fix |
|---|---|---|
| `/tb1/cmd_vel` not publishing | Wrong namespace remapping | Confirm `--ros-args -r __ns:=/tb1` |
| Topic exists but Gazebo ignores it | `Twist` type mismatch | Confirm bridge argument uses `geometry_msgs/msg/Twist]gz.msgs.Twist` |
| Robot moves then stops abruptly | `safe_distance` emergency stop triggered | Check `/tb1/scan` — obstacle within 0.4m; verify `cluster_distance` parameter |

### No `/tbX/scan` in `ros2 topic list`

```bash
# Check if bridge node is alive
ros2 node list | grep bridge_tb1

# Check Gazebo topic exists on Gz side
gz topic -l | grep scan

# Check for SDF patching errors in launch terminal
```

Likely cause: robot not yet spawned, or SDF patching failed. Check launch terminal output for errors from `generate_sdf.py`. Verify `/tmp/multi_tb3_tb1.sdf` exists and contains patched topic names.

### Follower orbiting or overshooting

Reduce `kp_angular` in `follower_params.yaml` (default: `2.0`). If robot oscillates longitudinally, reduce `kp_linear` (default: `0.8`). Test with incremental changes: try `kp_angular: 1.5` first.

### Gazebo topic mismatch / bridge error

```
[ERROR] [parameter_bridge]: bridge failed to create for topic /tb2/cmd_vel
```

The SDF patch did not apply — `/tb2/cmd_vel` does not exist in Gazebo. Check that `generate_sdf.py` ran successfully (look for `/tmp/multi_tb3_tb2.sdf` with correct topic names). Verify `multi_tb3_system/generate_sdf.py` and the SDF template exist.

### OpenGL / display error (WSL2)

```bash
# Launch without Gazebo GUI
ros2 launch multi_tb3_system robot.launch.py rviz:=true
# (do not use ros_ui:=true — that enables gz GUI which fails on WSL2 without GPU)
```

If Gazebo GUI still crashes, ensure `LIBGL_ALWAYS_SOFTWARE=1` is set in your environment.

### `use_sim_time` warnings at startup

```
[WARN] [rcl]: Buffer is not growing
[WARN] [rclpy]: Publish clock message before completing initialization
```

Normal during the first ~2 seconds while Gazebo initialises and `/clock` bridge starts publishing. Warnings clear automatically once clock begins flowing. Can be suppressed with `--ros-args --log-level clock_bridge:=ERROR` if desired.

### Followers not starting (no follower_node in `ros2 node list`)

Check the staggered timing:
- tb1 spawns at t=0s
- tb2 spawns at t=5s, follower starts at t=7s
- tb3 spawns at t=10s, follower starts at t=12s

Wait at least 12 seconds after launch before checking. If still no followers after 15s, check launch terminal for errors in `followers.launch.py` OpaqueFunction.

---

## 📚 Additional Documentation

- **[`DOCUMENTATION.md`](DOCUMENTATION.md)** — Detailed system architecture, data flow, component descriptions
- **[`LAUNCH_CLEANUP.md`](LAUNCH_CLEANUP.md)** — What was deleted in v0.2.0, migration guide, before/after examples
- **[`config/follower_params.yaml`](config/follower_params.yaml)** — Follower tuning parameters with defaults
- **`scripts/*.py`** — Source code with extensive docstrings

---

## 🗺️ Future Work

| Area | Description |
|---|---|
| **Nav2 integration** | Replace PD follower with Nav2 local planner for obstacle-aware path execution |
| **Localization** | Add AMCL localization using maps built by slam_toolbox |
| **Multi-robot map merging** | Automated merge of per-robot maps into a single global occupancy grid |
| **Real robot deployment** | Use `use_sim_time:=false` + real hardware bridges for physical TurtleBot3 units |
| **Multi-machine DDS** | Configure Fast-DDS discovery server for multi-PC convoy over LAN |
| **Dynamic follower count** | Extend `nBurger` beyond 2 with automatic URDF/SDF generation for 4+ robots |
| **Vision tracking** | Add camera-based detection to complement LiDAR for low-LiDAR-density scenarios |
| **Dynamic obstacle prediction** | Kalman filter for predicting moving obstacle trajectories |
| **Adaptive spacing** | Convoy gap control based on convoy speed for highway-style following |
| **Autonomous leader** | Replace teleop with waypoint navigation for leader robot |

---

## 📦 Package Info

| Field | Value |
|---|---|
| **Package name** | `multi_tb3_system` |
| **Version** | `0.3.0` |
| **Build system** | `ament_cmake` + `ament_cmake_python` |
| **License** | Apache 2.0 |
| **ROS distro** | Jazzy Jalisco |
| **Gazebo** | Harmonic (gz-sim 8.x) |
| **Python** | 3.12 |

### RF2O Laser Odometry Integration
The workspace now includes the `rf2o_laser_odometry` package. When `enable_rf2o:=true` is used:
- Each robot runs a dedicated `rf2o_laser_odometry_node`.
- Wheel odometry from Gazebo is augmented/replaced with high-frequency, high-precision laser-based planar odometry.
- This mitigates drift caused by wheel slip during Pure Pursuit sharp turns.

### What's New in v0.2.0

- **Modular launch system** — Single `robot.launch.py` entry point replacing legacy trio
- **Shared configuration module** — `launch_common.py` eliminates cross-file duplication and timing drift risk
- **Cleaned codebase** — Removed 7 obsolete/unimplemented files (legacy launchers, stubs, outdated docs)
- **New simulation world** — Added support for the `office` world (CPR Office) with automatic texture path injection
- **Dynamic robot count** — `nBurger=1` or `nBurger=2` at launch (previously hardcoded to 3)
- **Improved documentation** — This README + LAUNCH_CLEANUP.md migration guide

See [`LAUNCH_CLEANUP.md`](LAUNCH_CLEANUP.md) for details on what changed and how to update any custom scripts.

---

<div align="center">

*Built with ROS 2 Jazzy · Gazebo Sim Harmonic · TurtleBot3 Burger*  
*Hochschule Darmstadt — Robotics Engineering*

</div>
