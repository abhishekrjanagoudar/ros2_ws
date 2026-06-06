# multi_tb3_system — Technical Documentation

**ROS 2 Jazzy · Gazebo Sim Harmonic · TurtleBot3 Burger**
> University Assignment — Hochschule Darmstadt · Prof. Dr.-Ing. Karl Kleinmann

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Project Structure](#2-project-structure)
3. [Build & Setup](#3-build--setup)
4. [Launch Files](#4-launch-files)
5. [Nodes](#5-nodes)
6. [Python Modules](#6-python-modules)
7. [Config Files](#7-config-files)
8. [Worlds](#8-worlds)
9. [TF Tree](#9-tf-tree)
10. [Topic Architecture](#10-topic-architecture)
11. [Timing Sequence](#11-timing-sequence)
12. [Tuning Guide](#12-tuning-guide)
13. [Debugging](#13-debugging)

---

## 1. System Overview

Three TurtleBot3 Burger robots in a leader–follower convoy:

```
  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
  │     tb1      │   │     tb2      │   │     tb3      │
  │   (Leader)   │   │  (Follower)  │   │  (Follower)  │
  │  Keyboard    │◄──│  LiDAR scan  │◄──│  LiDAR scan  │
  │  → cmd_vel   │   │  → cmd_vel   │   │  → cmd_vel   │
  └──────────────┘   └──────────────┘   └──────────────┘
      x = 0.0 m          x = -1.0 m        x = -2.0 m
```

- **tb1** — teleoperated via keyboard
- **tb2** — autonomously follows tb1 using LiDAR clustering
- **tb3** — autonomously follows tb2 using LiDAR clustering
- No cameras, no Nav2, no odometry fusion — purely geometric LiDAR perception

---

## 2. Project Structure

```
ros2_ws/src/multi_tb3_system/
│
├── multi_tb3_system/
│   ├── __init__.py
│   └── generate_sdf.py          # Per-robot SDF topic patching
│
├── scripts/
│   ├── follower_node.py          # Autonomous follower node
│   ├── laser_processor.py        # LaserScan → clusters library
│   ├── safety_controller.py      # Emergency stop + velocity clamping
│   └── teleop_controller.py      # Burst-mode keyboard teleop
│
├── launch/
│   ├── robot.launch.py           # ⭐ ENTRY POINT
│   ├── worlds.launch.py          # World selection → gazebo.launch.py
│   ├── gazebo.launch.py          # Gz server + GUI + clock bridge
│   ├── spawn_robots.launch.py    # Dynamic robot spawning (N+1 robots)
│   ├── followers.launch.py       # Follower node launcher
│   ├── rviz.launch.py            # RViz2 visualization
│   ├── mapping.launch.py         # Stub (future: slam_toolbox)
│   ├── localization.launch.py    # Stub (future: AMCL/EKF)
│   ├── multi_robot.launch.py     # Legacy monolithic launcher
│   ├── world_empty.launch.py     # Convenience → multi_robot (empty)
│   └── world_obstacles.launch.py # Convenience → multi_robot (columns)
│
├── config/
│   ├── follower_params.yaml      # PD gains, distances, LiDAR thresholds
│   └── robot_ids.yaml            # Convoy hierarchy reference (not loaded at runtime)
│
├── worlds/
│   ├── empty.world               # Flat ground plane
│   └── pillars.world             # 6 cylindrical obstacles
│
├── models/
│   └── turtlebot3_burger/
│       └── model.sdf             # SDF template (patched at launch by generate_sdf.py)
│
├── rviz/
│   └── multi_robot.rviz          # Pre-configured RViz2 layout
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. Build & Setup

### Dependencies

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
  ros-jazzy-tf2-tools
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select multi_tb3_system
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## 4. Launch Files

### 4.1 `robot.launch.py` ⭐ Entry Point

**Use this file for all normal launches.**

```bash
ros2 launch multi_tb3_system robot.launch.py [args]
```

**Arguments:**

| Argument | Default | Description |
|---|:---:|---|
| `world` | `empty` | World name: `empty` or `pillars` |
| `nBurger` | `2` | Follower count (1–2). Total robots = nBurger + 1 |
| `use_sim_time` | `true` | `true` = Gazebo sim clock, `false` = wall clock |
| `gz` | `false` | Show Gazebo GUI window |
| `rviz` | `false` | Show RViz2 |
| `ros_ui` | `false` | Shortcut: `true` forces `gz=true` + `rviz=true` (overrides both) |

**Examples:**

```bash
# Headless — 3 robots, empty world
ros2 launch multi_tb3_system robot.launch.py

# Full UI — Gazebo + RViz, pillars world
ros2 launch multi_tb3_system robot.launch.py world:=pillars ros_ui:=true

# 2 robots only (leader + 1 follower)
ros2 launch multi_tb3_system robot.launch.py nBurger:=1 ros_ui:=true
```

**What it does:**

Uses `OpaqueFunction` (`_resolve_ui_flags`) to resolve `ros_ui` override, then includes:
1. `worlds.launch.py` — starts Gazebo with the selected world
2. `spawn_robots.launch.py` — spawns all robots
3. `followers.launch.py` — starts follower autonomy nodes
4. `rviz.launch.py` — conditionally, if `rviz=true`

---

### 4.2 `worlds.launch.py`

Resolves world name → absolute file path, then includes `gazebo.launch.py`.

```bash
ros2 launch multi_tb3_system worlds.launch.py world:=pillars gz:=true
```

**Arguments:** `world` (`empty`/`pillars`), `gz` (`true`/`false`)

Constructs the world file path via `PathJoinSubstitution`:
```
<pkg>/worlds/<world_name>.world
```

---

### 4.3 `gazebo.launch.py`

Starts Gazebo Sim server, optional GUI client, and clock bridge.

**Arguments:** `world_file` (absolute path), `gz` (`true`/`false`)

**Components launched:**

| Component | Type | Detail |
|---|---|---|
| Gz Server | `IncludeLaunchDescription` | `gz_sim.launch.py -r -s -v2 <world>`, `on_exit_shutdown=true` |
| Gz GUI | `ExecuteProcess` | `gz sim -g -v2`, with `LIBGL_ALWAYS_SOFTWARE=1` injected |
| Clock Bridge | `Node` | `parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` |

> **WSL2 note:** The GUI is launched as `ExecuteProcess` (not `IncludeLaunchDescription`) specifically to inject `LIBGL_ALWAYS_SOFTWARE=1` per-process. This bypasses the broken `QGLXContext` / `drisw` path on WSLg and forces CPU (`llvmpipe`) rendering. Using `IncludeLaunchDescription` does not support per-process environment injection.

---

### 4.4 `spawn_robots.launch.py`

Dynamically spawns N+1 robots using `OpaqueFunction`.

```bash
ros2 launch multi_tb3_system spawn_robots.launch.py nBurger:=2
```

**Arguments:** `nBurger` (1–2), `use_sim_time`

**Spawn grid constants:**

| Constant | Value | Description |
|---|:---:|---|
| `SPAWN_X_STEP` | `-1.0 m` | Spacing between robots along X axis |
| `SPAWN_Y` | `0.0 m` | All robots on Y=0 |
| `SPAWN_Z` | `0.01 m` | Slightly above ground |
| `SPAWN_DELAY_STEP` | `3.0 s` | Seconds between successive spawns |

**Per-robot actions** (via `_make_robot_actions()`):

1. **`generate_robot_sdf(ns)`** — patches `model.sdf` template with namespaced topics, writes to `/tmp/multi_tb3_<ns>.sdf`
2. **`ros_gz_sim create`** — spawns patched SDF into Gazebo at `(x_pos, 0, 0.01)`
3. **`robot_state_publisher`** — publishes URDF-based TF. `frame_prefix=ns/` scopes all frames to `tbX/base_link`, `tbX/base_footprint`, etc.
4. **`parameter_bridge`** — bridges `/tbX/scan`, `/tbX/odom`, `/tbX/cmd_vel`, `/tbX/joint_states`, `/tf`
5. **`static_transform_publisher`** — publishes `world → tbX/odom` at the robot's spawn position, connecting all TF trees to a common root

**Timing:**
- tb1: spawns at t=0
- tb2: spawns at t=3s (via `TimerAction`)
- tb3: spawns at t=6s (via `TimerAction`)

---

### 4.5 `followers.launch.py`

Starts one `follower_node.py` per follower robot with a startup delay matching spawn timing.

```bash
ros2 launch multi_tb3_system followers.launch.py nBurger:=2
```

**Arguments:** `nBurger` (1–2), `use_sim_time`

**Timing constants:**

| Constant | Value | Description |
|---|:---:|---|
| `_SPAWN_DELAY_STEP` | `3.0 s` | Must match `spawn_robots.launch.py` |
| `_FOLLOWER_INIT_BUFFER` | `2.0 s` | Extra time after spawn for Gz entity + bridge init |

**Follower start times:**
- tb2 follower: `(2-1)*3 + 2 = 5s`
- tb3 follower: `(3-1)*3 + 2 = 8s`

> **Why delayed?** Without the delay, follower nodes start at t=0 while their robots spawn at t=3/6s. When tb2 finally spawns, the node has been waiting and immediately chases tb1 at full speed — closing the gap to ~0.4m before tb3 spawns, making tb2 appear missing from Gazebo's viewport.

---

### 4.6 `rviz.launch.py`

Starts RViz2 with the pre-configured `rviz/multi_robot.rviz` layout.

```bash
ros2 launch multi_tb3_system rviz.launch.py
```

**Arguments:** `use_sim_time`

Loads the `.rviz` config which contains: 3× RobotModel, 3× LaserScan, 3× Odometry, TF display, Grid. Fixed frame: `world`.

---

### 4.7 `mapping.launch.py` / `localization.launch.py`

Stubs for future SLAM and localization integration. Currently no-ops — they accept reserved arguments (`use_sim_time`, `slam_robot` / `map_file`) but start no nodes.

---

### 4.8 `multi_robot.launch.py`

Legacy monolithic launcher. Hardcodes 3 robots, uses `/model/tbX/*` Gz topics with ROS remappings (older approach). Kept for backward compatibility with `world_empty.launch.py` and `world_obstacles.launch.py`.

> **Prefer `robot.launch.py`** for all new usage — it is modular, supports dynamic `nBurger`, and uses the cleaner `generate_sdf` approach (no remapping required).

---

### 4.9 `world_empty.launch.py` / `world_obstacles.launch.py`

Convenience launchers that delegate to `multi_robot.launch.py` with a preset world.

```bash
ros2 launch multi_tb3_system world_empty.launch.py use_rviz:=true
ros2 launch multi_tb3_system world_obstacles.launch.py use_gui:=false
```

**Arguments:** `use_rviz` (default `false`), `use_gui` (default `true`)

---

## 5. Nodes

### 5.1 `follower_node.py`

**Package:** `multi_tb3_system` | **Executable:** `follower_node.py`

Autonomous LiDAR-based follower. Namespace-agnostic — runs identically for tb2 and tb3. Topic paths resolve via ROS namespace.

**Subscriptions:**

| Topic | Type | QoS |
|---|---|---|
| `scan` (→ `/tbX/scan`) | `sensor_msgs/LaserScan` | Best Effort, Volatile, depth=5 |

**Publications:**

| Topic | Type | QoS |
|---|---|---|
| `cmd_vel` (→ `/tbX/cmd_vel`) | `geometry_msgs/Twist` | Reliable, depth=10 |

**Algorithm (executed on every scan callback):**

```
LaserScan (360°)
    │
    ▼  scan_to_cartesian()
    │  Filter valid ranges [range_min – 3.5m], convert polar → Cartesian (x,y)
    │
    ▼  filter_front_sector()
    │  Keep points within ±front_angle_deg of straight ahead (x > 0)
    │
    ▼  euclidean_cluster()
    │  Sort by angle, split into groups wherever gap > cluster_distance
    │
    ▼  make_clusters()
    │  Reject: < min_cluster_size pts (noise)
    │  Reject: > max_cluster_size pts (wall / large surface)
    │
    ▼  select_target_cluster()
    │  Pick closest valid cluster → assumed to be the robot ahead
    │
    ▼  _compute_control()
    │  linear_x  = kp_linear  × (target.distance − target_distance)
    │  angular_z = kp_angular × target.angle
    │  (negative linear_x clamped to -0.05 — slight backoff only)
    │
    ▼  SafetyController.check_and_modify()
    │  Emergency stop if anything < safe_distance in ±45° cone
    │  Steering bias if obstacle < 0.8m in ±60° cone
    │  Clamp to [−max_linear_vel, +max_linear_vel] and [−max_angular_vel, +max_angular_vel]
    │
    ▼  Publish Twist to cmd_vel
```

**Parameters** (from `follower_params.yaml`):

| Parameter | Default | Description |
|---|:---:|---|
| `target_distance` | `0.7 m` | Desired separation from robot ahead |
| `safe_distance` | `0.4 m` | Emergency stop threshold |
| `kp_linear` | `0.8` | Linear proportional gain |
| `kp_angular` | `2.0` | Angular proportional gain |
| `max_linear_velocity` | `0.22 m/s` | Hard ceiling (TB3 Burger limit) |
| `max_angular_velocity` | `1.0 rad/s` | Hard angular ceiling |
| `front_angle_deg` | `30.0°` | Half-width of forward detection cone |
| `cluster_distance` | `0.20 m` | Max intra-cluster point gap |
| `min_cluster_size` | `2 pts` | Below → noise, rejected |
| `max_cluster_size` | `40 pts` | Above → wall, rejected |

**Shutdown behaviour:** Publishes zero velocity before destruction. Guards against `rclpy` context invalidation during SIGINT cascade using `rclpy.ok()` checks.

---

### 5.2 `teleop_controller.py`

**Package:** `multi_tb3_system` | **Executable:** `teleop_controller.py`

Burst-mode (hold-to-move) keyboard teleoperation node. Runs at 20 Hz — publishes continuously, zero-ing out on key release.

```bash
ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
```

**Publications:**

| Topic | Type | QoS |
|---|---|---|
| `cmd_vel` (→ `/tb1/cmd_vel`) | `geometry_msgs/Twist` | Reliable, Volatile, depth=10 |

**Key bindings:**

| Key | Action |
|:---:|---|
| `w` / `i` | Forward |
| `x` / `,` | Backward |
| `a` / `j` | Rotate left |
| `d` / `l` | Rotate right |
| `s` / `k` | Force stop |
| `q` / `z` | Increase / decrease overall speed |
| `e` / `c` | Increase / decrease angular speed only |
| `Ctrl+C` | Quit |

**Implementation details:**
- `get_key()` reads the terminal in raw mode with a 50ms timeout (non-blocking)
- `_timer_callback` fires at 20 Hz from a `create_timer`. If no key registered for >150ms, velocities zero out
- ROS spin runs in a **background thread** so keyboard I/O on the main thread doesn't block the timer
- On exit, publishes zero 5× with 50ms gaps to ensure Gazebo receives the stop

---

## 6. Python Modules

### 6.1 `generate_sdf.py`

**Not a node.** Called at launch time (inside `OpaqueFunction`) to generate per-robot namespaced SDF files.

**Why needed:** The TurtleBot3 `model.sdf` uses bare plugin topic names (`cmd_vel`, `odom`, `joint_states`). Gazebo Sim publishes these globally — with multiple robots, all share the same topics, causing control collisions.

**What it does:**

1. Reads `models/turtlebot3_burger/model.sdf` template
2. Patches `<model name="...">` to match the namespace
3. Injects `<topic>`, `<odom_topic>`, `<tf_topic>` into the DiffDrive plugin block if absent
4. Replaces all bare topic/frame strings with absolute namespaced paths:

| Before | After (ns=tb2) |
|---|---|
| `<topic>scan</topic>` | `<topic>/tb2/scan</topic>` |
| `<topic>cmd_vel</topic>` | `<topic>/tb2/cmd_vel</topic>` |
| `<odom_topic>odom</odom_topic>` | `<odom_topic>/tb2/odom</odom_topic>` |
| `<frame_id>odom</frame_id>` | `<frame_id>tb2/odom</frame_id>` |
| `<child_frame_id>base_footprint</child_frame_id>` | `<child_frame_id>tb2/base_footprint</child_frame_id>` |

5. Writes patched SDF to `/tmp/multi_tb3_<ns>.sdf`
6. Returns the path for `ros_gz_sim create -file <path>`

**API:**
```python
from multi_tb3_system.generate_sdf import generate_robot_sdf
sdf_path = generate_robot_sdf('tb2')   # → '/tmp/multi_tb3_tb2.sdf'
```

---

### 6.2 `laser_processor.py`

Pure-geometry library. No ROS dependencies. Called by `follower_node.py` on every scan.

**Key types:**

```python
@dataclass
class Cluster:
    points:     List[Tuple[float, float]]  # (x, y) in robot frame
    centroid_x: float
    centroid_y: float
    distance:   float   # from robot origin [m]
    angle:      float   # from robot heading [rad]
    size:       int
```

**Functions:**

| Function | Input | Output | Description |
|---|---|---|---|
| `scan_to_cartesian()` | ranges, angle_min, angle_inc, range_min/max | `[(x,y)]` | Convert polar to Cartesian, filter invalid ranges |
| `filter_front_sector()` | points, half_angle_deg | `[(x,y)]` | Keep only points within ±angle and x>0 |
| `euclidean_cluster()` | points, cluster_distance | `[[(x,y)]]` | Sort by angle, split on gap > threshold |
| `compute_centroid()` | cluster_points | `(cx, cy)` | Mean x/y of cluster |
| `make_clusters()` | raw_clusters, min/max_size | `[Cluster]` | Build Cluster objects, apply size filters |
| `select_target_cluster()` | clusters | `Cluster\|None` | Return closest cluster |
| `process_scan()` | full scan params | `(target, all_clusters)` | Full pipeline in one call |

---

### 6.3 `safety_controller.py`

**Class:** `SafetyController`

Applied after the control law, before publishing. Modifies velocities to enforce hard safety limits.

```python
SafetyController(safe_distance=0.4, max_linear_vel=0.22, max_angular_vel=1.0)
safe_lin, safe_ang = controller.check_and_modify(linear_x, angular_z, ranges, ...)
```

**Rules applied in order:**

| Priority | Rule | Zone | Action |
|:---:|---|:---:|---|
| 1 | **Emergency stop** | ±45° front cone | If any obstacle < `safe_distance` → `linear_x = 0` (angular unchanged) |
| 2 | **Steering bias** | ±60° cone, within 0.8m | Nudge away from closer side: `angular_z ±= 0.5 × bias` |
| 3 | **Velocity clamp** | — | `linear_x` clamped to `±max_linear_vel`, `angular_z` to `±max_angular_vel` |

---

## 7. Config Files

### 7.1 `config/follower_params.yaml`

Loaded by `followers.launch.py` via `--params-file`. Applied to both tb2 and tb3.

```yaml
follower_node:
  ros__parameters:
    target_distance:      0.7    # maintain this gap from robot ahead [m]
    safe_distance:        0.4    # hard-stop if anything closer than this [m]
    kp_linear:            0.8    # linear_vel  = kp_linear  * (dist - target_distance)
    kp_angular:           2.0    # angular_vel = kp_angular * angle_to_target
    max_linear_velocity:  0.22   # TB3 Burger hardware limit [m/s]
    max_angular_velocity: 1.0    # [rad/s]
    front_angle_deg:      30.0   # ±deg cone ahead used for target detection
    cluster_distance:     0.20   # max gap between points in same cluster [m]
    min_cluster_size:     2      # fewer → noise, rejected
    max_cluster_size:     40     # more  → wall, rejected
```

**Tuning tips:**
- **Follower orbiting / oscillating:** reduce `kp_angular` (try 1.0–1.5)
- **Follower overshooting longitudinally:** reduce `kp_linear` (try 0.4–0.6)
- **Follower too far / too close:** adjust `target_distance`
- **Wall rejection failing:** reduce `max_cluster_size` (e.g. 20)
- **Robot not detected (too sparse):** reduce `min_cluster_size` to 1

---

### 7.2 `config/robot_ids.yaml`

Reference documentation only — **not loaded at runtime** by any node or launch file. Describes the convoy chain for human readers.

```yaml
convoy:
  robots:
    - namespace: "tb1"
      role: "leader"
      initial_pose: {x: 0.0, y: 0.0, yaw: 0.0}
    - namespace: "tb2"
      role: "follower"
      follows: "tb1"
      initial_pose: {x: -1.0, y: 0.0, yaw: 0.0}
    - namespace: "tb3"
      role: "follower"
      follows: "tb2"
      initial_pose: {x: -2.0, y: 0.0, yaw: 0.0}
```

---

## 8. Worlds

### `worlds/empty.world`

Flat ground plane with directional sun. No obstacles. Best for initial formation validation and PD gain tuning.

### `worlds/pillars.world`

Six static cylindrical pillars (`r=0.10m, h=0.50m`) in a staggered slalom pattern:

```
Direction of travel: ──────────────────────────────────────►

Y+ ●  P1(1.5, 0.5)     P3(3.5, 0.5)     P5(5.5, 0.5)

Y- ●      P2(2.5,-0.5)     P4(4.5,-0.5)     P6(6.5,-0.5)
```

Used to verify follower obstacle rejection. A pillar's cluster will have ~3–8 points (within `max_cluster_size`), so it WILL appear as a candidate — the follower must discriminate based on which is the moving robot. In practice, the leader is consistently the closest compact cluster in the forward cone.

---

## 9. TF Tree

Each robot has an isolated TF sub-tree. All sub-trees connect to a shared `world` root via static transforms published in `spawn_robots.launch.py`.

```
world                        ← static root
├── tb1/odom                 ← static_transform_publisher (world → tb1/odom @ x=0.0)
│   └── tb1/base_footprint   ← Gazebo DiffDrive via /tf bridge
│       └── tb1/base_link    ← robot_state_publisher (URDF joints, frame_prefix=tb1/)
│           ├── tb1/imu_link
│           ├── tb1/base_scan
│           └── tb1/wheel_*
│
├── tb2/odom                 ← static_transform_publisher (world → tb2/odom @ x=-1.0)
│   └── tb2/base_footprint
│       └── tb2/base_link
│           └── ...
│
└── tb3/odom                 ← static_transform_publisher (world → tb3/odom @ x=-2.0)
    └── tb3/base_footprint
        └── tb3/base_link
            └── ...
```

**RViz fixed frame: `world`**

Without the `world` → `tbX/odom` static transforms, tb2 and tb3 TF trees are isolated from tb1's tree. RViz (using a single fixed frame) cannot compute transforms across disconnected trees — hence the "No transform from [tb2/base_footprint] to [tb1/odom]" errors.

---

## 10. Topic Architecture

### Per-robot (X = 1, 2, 3)

| Topic | Type | Direction | Source → Sink |
|---|---|:---:|---|
| `/tbX/scan` | `sensor_msgs/LaserScan` | Gz → ROS | Gazebo LiDAR → `follower_node` |
| `/tbX/odom` | `nav_msgs/Odometry` | Gz → ROS | Gazebo DiffDrive → (RViz) |
| `/tbX/cmd_vel` | `geometry_msgs/Twist` | ROS → Gz | `teleop`/`follower_node` → Gazebo DiffDrive |
| `/tbX/joint_states` | `sensor_msgs/JointState` | Gz → ROS | Gazebo → `robot_state_publisher` |

### Global

| Topic | Type | Direction | Description |
|---|---|:---:|---|
| `/tf` | `tf2_msgs/TFMessage` | Gz → ROS | All robot odom→base_footprint transforms |
| `/clock` | `rosgraph_msgs/Clock` | Gz → ROS | Simulation clock |

### Bridge format (ros_gz_bridge argument style)

```
[  =  Gz → ROS
]  =  ROS → Gz

/tbX/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/tbX/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry
/tbX/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
/tbX/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model
/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

---

## 11. Timing Sequence

```
t=0s   Gazebo server starts, tb1 spawns (x=0), RSP + bridge + static_tf_world_tb1_odom start
t=3s   tb2 spawns (x=-1.0), RSP + bridge + static_tf_world_tb2_odom start
t=5s   tb2 follower_node starts
t=6s   tb3 spawns (x=-2.0), RSP + bridge + static_tf_world_tb3_odom start
t=8s   tb3 follower_node starts
```

The 2-second `_FOLLOWER_INIT_BUFFER` between spawn and follower start allows the Gazebo entity and ros_gz_bridge to fully initialize before `cmd_vel` messages start flowing.

---

## 12. Tuning Guide

### Adjusting Following Behaviour

All parameters live in `config/follower_params.yaml`. No rebuild needed — they are loaded at node start.

| Symptom | Parameter | Direction |
|---|---|---|
| Follower too close | `target_distance` | ↑ increase |
| Follower too far | `target_distance` | ↓ decrease |
| Follower oscillates (forward/back) | `kp_linear` | ↓ decrease |
| Follower steers wildly | `kp_angular` | ↓ decrease |
| Walls detected as targets | `max_cluster_size` | ↓ decrease |
| Leader not detected | `front_angle_deg` | ↑ increase |
| Follower crashes into leader | `safe_distance` | ↑ increase |

### Adjusting Robot Count

```bash
# 2 robots (leader + 1 follower)
ros2 launch multi_tb3_system robot.launch.py nBurger:=1

# 3 robots (leader + 2 followers) — default
ros2 launch multi_tb3_system robot.launch.py nBurger:=2
```

Maximum supported: `nBurger=2` (3 robots total).

---

## 13. Debugging

### Monitor topics

```bash
ros2 topic list                        # all active topics
ros2 topic hz /tb2/scan                # scan rate (~10 Hz expected)
ros2 topic echo /tb2/cmd_vel           # follower commands
ros2 topic echo /clock --once          # check sim clock
```

### Check nodes

```bash
ros2 node list                         # list all nodes
ros2 node list | grep bridge           # verify bridges
ros2 node list | grep follower         # verify follower nodes
```

### Inspect TF

```bash
ros2 run tf2_tools view_frames         # generates frames.pdf
ros2 run tf2_ros tf2_echo world tb2/base_footprint
```

### Verify SDF patch

```bash
grep -E "tb2|cmd_vel|odom" /tmp/multi_tb3_tb2.sdf
# Should show /tb2/cmd_vel, /tb2/odom, tb2/base_footprint etc.
```

### Common issues

| Error | Cause | Fix |
|---|---|---|
| `No transform from [tb2/base_footprint] to [world]` | Static TF not yet published | Wait for tb2 to spawn (t=3s) |
| `/tbX/scan` missing from topic list | Bridge not started / robot not spawned | Check `ros2 node list \| grep bridge_tbX` |
| Follower not moving | Node started before robot spawned | Check timing — follower starts at t=5s/8s |
| Gazebo GUI crash (WSLg) | GLX failure | `LIBGL_ALWAYS_SOFTWARE=1` already injected via `ExecuteProcess` in `gazebo.launch.py` |
| `rclpy RCLError` on shutdown | Context invalidated before `finally` | Already fixed — `rclpy.ok()` guard in `follower_node.py` |

---

## 14. Recent Fixes & Known Issues Resolved

During development, the following critical bugs were identified and successfully resolved:

### 14.1 Gazebo GUI WSLg Crash (`drisw` failure)
**Issue:** `gz sim -g` crashed immediately on Windows Subsystem for Linux (WSLg) with errors related to `QGLXContext` and `drisw screen creation`.
**Fix:** Modified `gazebo.launch.py` to use `ExecuteProcess` instead of `IncludeLaunchDescription` for the GUI. This allowed injecting the `LIBGL_ALWAYS_SOFTWARE=1` environment variable specifically for the GUI process, forcing stable `llvmpipe` software rendering and bypassing the broken WSLg GLX implementation.

### 14.2 RViz TF Tree Disconnection
**Issue:** RViz reported errors like `No transform from [tb2/base_footprint] to [tb1/odom]`. The robots functioned in Gazebo, but their TF trees were completely isolated from each other because they lacked a common root.
**Fix:** Added a `tf2_ros static_transform_publisher` to `spawn_robots.launch.py` for each robot. It publishes a static transform from the shared `world` frame to each robot's `tbX/odom` frame at its respective spawn position `(x, 0, 0)`. RViz's fixed frame was then set to `world` in `multi_robot.rviz`, unifying all three trees.

### 14.3 Missing Robot 2 (Race Condition)
**Issue:** At launch, `tb1` and `tb3` appeared, but `tb2` seemed missing. 
**Fix:** Identified as a timing race condition. `tb2` was spawning in Gazebo at `t=3s`, but its `follower_node.py` was starting at `t=0s`. The moment `tb2` spawned, the follower node instantly drove it forward into `tb1` (at full speed) before `tb3` even spawned, causing them to overlap visually. Fixed by adding a `TimerAction` in `followers.launch.py` to stagger the start of the follower nodes, ensuring they wait for their robot to fully spawn before sending `cmd_vel` messages.

### 14.4 `rclpy` Shutdown Exception
**Issue:** Shutting down the launch file with `Ctrl+C` caused a messy `rclpy.exceptions.RCLError: context is not valid` traceback from `follower_node.py`.
**Fix:** The node was trying to publish a zero-velocity Twist in the `finally` block after the ROS context had already been torn down by the launch manager. Wrapped the shutdown publish call in an `if rclpy.ok():` check and a `try/except` block to gracefully handle sudden context invalidation.
