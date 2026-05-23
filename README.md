# 🤖 multi_tb3_system

The `multi_tb3_system` package implements a **multi-robot TurtleBot3 Burger convoy** in Gazebo Sim (Harmonic) using ROS 2 Jazzy. Robot 1 is teleoperated; Robots 2 and 3 autonomously follow the robot ahead using LiDAR (LaserScan) clustering. A safety layer prevents collisions.

> **University Assignment** — Hochschule Darmstadt · Under guidance of Prof. Dr.-Ing. Karl Kleinmann

---

## 📋 Prerequisites

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (via `ros_gz`) |
| TurtleBot3 Model | `burger` |

```bash
sudo apt install -y \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-teleop \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-tf2-tools

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && source ~/.bashrc
```

---

## 🛠️ Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select multi_tb3_system
source install/setup.bash
```

---

## 🚀 Quick Start

### 1. Launch the Simulation

```bash
ros2 launch multi_tb3_system world_empty.launch.py
```

> Opens Gazebo with 3 TurtleBot3 robots (tb1 at origin, tb2 at −1 m, tb3 at −2 m). Follower nodes for tb2 and tb3 start automatically.

### 2. Teleoperate the Leader (tb1)

In a new terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
```

> **Hold a key** to move — releasing stops the robot immediately (burst-mode teleop).
>
> `i/w` → forward &nbsp;|&nbsp; `,/x` → backward &nbsp;|&nbsp; `j/a` → rotate left &nbsp;|&nbsp; `l/d` → rotate right &nbsp;|&nbsp; `k/s` → stop

---

## ⚙️ Launch Files

### `world_empty.launch.py` — Empty world (baseline)

```bash
ros2 launch multi_tb3_system world_empty.launch.py [args]
```

### `world_obstacles.launch.py` — Columns world (obstacle rejection test)

```bash
ros2 launch multi_tb3_system world_obstacles.launch.py [args]
```

### `multi_robot.launch.py` — Master launch (called by the above)

```bash
ros2 launch multi_tb3_system multi_robot.launch.py world:=empty [args]
```

---

## ⚙️ Launch Arguments

All three launch files accept the following arguments:

### 1. `world`  *(multi_robot.launch.py only)*

Selects the Gazebo world:

- `empty` *(default)*: Flat ground plane, no obstacles. Best for formation testing.
- `columns`: 6 static cylindrical pillars arranged as a slalom course. Tests obstacle rejection.

---

### 2. `use_gui`

Controls the Gazebo graphical client:

- `true` *(default)*: Launches the Gazebo GUI (3D viewer).
- `false`: Headless — Gazebo runs without a window. **Required for WSL 2 or servers without a display.**

---

### 3. `use_rviz`

Controls RViz2 visualization:

- `false` *(default)*: RViz is not launched.
- `true`: Launches RViz2 with pre-configured multi-robot display (LaserScan + Odometry for all 3 robots).

---

### 🧪 Examples

```bash
# Empty world, headless, no RViz
ros2 launch multi_tb3_system world_empty.launch.py use_gui:=false

# Obstacle world with RViz
ros2 launch multi_tb3_system world_obstacles.launch.py use_rviz:=true

# Full display: Gazebo GUI + RViz
ros2 launch multi_tb3_system world_empty.launch.py use_gui:=true use_rviz:=true
```

---

## 🗂️ Topic Map

| ROS Topic | Type | Direction | Description |
|---|---|---|---|
| `/tb1/cmd_vel` | `Twist` | → Gazebo | Leader velocity command |
| `/tb2/cmd_vel` | `Twist` | → Gazebo | Follower 1 velocity command |
| `/tb3/cmd_vel` | `Twist` | → Gazebo | Follower 2 velocity command |
| `/tbX/scan` | `LaserScan` | Gazebo → | Per-robot LiDAR data |
| `/tbX/odom` | `Odometry` | Gazebo → | Per-robot wheel odometry |
| `/tbX/joint_states` | `JointState` | Gazebo → | Per-robot wheel joint states |
| `/tf` | `TFMessage` | Gazebo → | All robot transforms (odom → base_footprint) |
| `/clock` | `Clock` | Gazebo → | Simulation time |

> ⚠️ The bridge uses standard `geometry_msgs/Twist` on the ROS side. Always publish `Twist` to `/tbX/cmd_vel`.

---

## 🏗️ Node Architecture

| Node | Script | Namespace | Role |
|---|---|---|---|
| `teleop_controller` | `teleop_controller.py` | `/tb1` | Keyboard control for the leader |
| `follower_node` | `follower_node.py` | `/tb2`, `/tb3` | Autonomous LiDAR-based following |
| `laser_processor` | `laser_processor.py` | *(library)* | LaserScan → Cartesian clustering |
| `safety_controller` | `safety_controller.py` | *(library)* | Velocity limits + emergency stop |

---

## 🔧 Configuration

### `config/follower_params.yaml`

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `target_distance` | `0.7` | m | Desired gap to robot ahead |
| `safe_distance` | `0.4` | m | Emergency stop threshold |
| `kp_linear` | `0.8` | — | Linear proportional gain |
| `kp_angular` | `2.0` | — | Angular proportional gain |
| `max_linear_velocity` | `0.22` | m/s | Hard velocity ceiling |
| `max_angular_velocity` | `1.0` | rad/s | Hard angular velocity ceiling |
| `front_angle_deg` | `30.0` | deg | Front cone half-angle for LiDAR filtering |
| `cluster_distance` | `0.20` | m | Max gap between points in same cluster |
| `min_cluster_size` | `2` | pts | Minimum cluster size (below = noise) |
| `max_cluster_size` | `40` | pts | Maximum cluster size (above = wall) |

---

## Notes

1. **TF frames**: All robots share the same `odom` and `base_footprint` frame names as published by Gazebo's DiffDrive plugin. This is by design — do not set `frame_prefix` in `robot_state_publisher`.
2. **Headless / WSL 2**: Always pass `use_gui:=false` if your system has no display or you encounter OpenGL errors.
3. **`robot_ids.yaml`**: This file is reference documentation only. It is not loaded at runtime — convoy configuration is in `multi_robot.launch.py`.
