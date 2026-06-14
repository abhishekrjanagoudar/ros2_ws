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
14. [Recent Fixes & Known Issues Resolved](#14-recent-fixes--known-issues-resolved)

---

## 1. System Overview

Three TurtleBot3 Burger robots form a leader–follower convoy using a **Path-Based Convoy with Pure Pursuit**:

```
  ┌──────────────────────┐   ┌──────────────────────┐   ┌──────────────────────┐
  │         tb1          │   │         tb2          │   │         tb3          │
  │       (Leader)       │   │      (Follower)      │   │      (Follower)      │
  │                      │   │                      │   │                      │
  │  Keyboard teleop     │   │  Pure Pursuit        │   │  Pure Pursuit        │
  │  convoy_publisher ───┼──►│  follower_node       │   │  follower_node       │
  │  /tb1/convoy_path    ├───┼──────────────────────┼───┼──► (same path)       │
  └──────────────────────┘   └──────────────────────┘   └──────────────────────┘
       x = 0.0 m                   x = -1.0 m                 x = -2.0 m
      gap = 0 m                   gap = 0.6 m                 gap = 1.2 m
```

**Key design principles:**

- **tb1** (leader) is teleoperated. `convoy_publisher` records every pose into a `nav_msgs/Path` published as `/tb1/convoy_path` in the `world` frame.
- **tb2 and tb3** each run `follower_node.py`. Both subscribe to the *same* shared path. Each finds a goal point a configurable arc-length (`gap`) back from the path end and tracks it with Pure Pursuit.
- **LiDAR** is used exclusively by `SafetyController` for emergency stop and steering bias. No LiDAR cluster detection, no daisy-chain leader tracking.
- **Error accumulation is eliminated**: every follower tracks the same ground-truth path, so position errors do not cascade down the convoy.

---

## 2. Project Structure

```
ros2_ws/src/multi_tb3_system/
│
├── multi_tb3_system/                   (Python package — imported by launch files)
│   ├── __init__.py
│   ├── generate_sdf.py                 # Per-robot SDF topic patching (called at launch time)
│   └── launch_common.py               # ⭐ Shared config: geometry, timing, helpers
│
├── scripts/                            (Executable ROS nodes)
│   ├── convoy_publisher.py             # Publishes leader trajectory as nav_msgs/Path (10 Hz)
│   ├── follower_node.py                # Pure Pursuit path follower (50 Hz timer)
│   ├── laser_processor.py             # Scan → Cartesian clustering library (NOT used by follower_node)
│   ├── safety_controller.py           # Emergency stop + steering bias (used by follower_node)
│   └── teleop_controller.py           # Burst-mode keyboard teleop for tb1
│
├── launch/
│   ├── robot.launch.py                # ⭐ ENTRY POINT
│   ├── worlds.launch.py               # World name → file path → gazebo.launch.py
│   ├── gazebo.launch.py               # Gz server + optional GUI + clock bridge
│   ├── spawn_robots.launch.py         # N+1 robot spawning with staggered timing
│   ├── followers.launch.py            # convoy_publisher + follower_node per follower
│   ├── mapping.launch.py              # slam_toolbox SLAM integration
│   └── rviz.launch.py                 # RViz2 visualization
│
├── config/
│   ├── follower_params.yaml           # Pure Pursuit gains, spacing, limits
│   ├── mapping_online_async.yaml      # slam_toolbox params (tuned for TB3 Burger)
│   └── cpr_office/                    # CPR Office world assets
│
├── worlds/
│   ├── empty.world                    # Flat ground plane (1 ms physics step)
│   ├── columns.world                  # Flat ground alias
│   └── pillars.world                  # 6 cylindrical obstacles (1 ms physics step)
│
├── models/
│   └── turtlebot3_burger/
│       ├── model.sdf                  # SDF template — patched at launch by generate_sdf.py
│       ├── model-1_4.sdf              # SDF 1.4 legacy variant
│       └── model.config
│
├── rviz/
│   └── multi_robot.rviz               # Pre-configured: RobotModel + LaserScan + TF × 3
│
├── .gitattributes                     # Forces LF endings on *.py, *.yaml, *.world, *.sdf
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
└── DOCUMENTATION.md                   # This file
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
  ros-jazzy-tf2-tools \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select multi_tb3_system
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Add to `~/.bashrc` to persist across sessions:
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## 4. Launch Files

### 4.1 `robot.launch.py` ⭐ Entry Point

```bash
ros2 launch multi_tb3_system robot.launch.py [args]
```

**Arguments:**

| Argument | Default | Description |
|---|:---:|---|
| `world` | `empty` | World name: `empty`, `pillars`, or `office` |
| `nBurger` | `2` | Follower count (1–2). Total robots = nBurger + 1 |
| `use_sim_time` | `true` | Gazebo sim clock (`true`) or wall clock (`false`) |
| `gz` | `false` | Show Gazebo GUI window |
| `rviz` | `false` | Show RViz2 |
| `ros_ui` | `false` | Shortcut: `true` sets `gz=true` + `rviz=true` |
| `convoy_spacing` | `0.6` | Gap per convoy slot in metres; forwarded to `followers.launch.py` |
| `enable_mapping` | `false` | Start slam_toolbox SLAM |
| `mapping_mode` | `online_async` | SLAM algorithm mode |
| `slam_robot` | `tb1` | Which robot(s) build the map: `tb1`, `tb2`, `tb3`, or `all` |

**Implementation — `_resolve_ui_flags` (OpaqueFunction):** resolves `ros_ui` override, then includes `worlds.launch.py`, `spawn_robots.launch.py`, `followers.launch.py`, and conditionally `rviz.launch.py` and `mapping.launch.py`.

---

### 4.2 `worlds.launch.py`

Resolves world name → absolute file path, delegates to `gazebo.launch.py`.

**Arguments:** `world`, `gz`, `use_sim_time`

---

### 4.3 `gazebo.launch.py`

Starts Gazebo Sim server, optional GUI client, and `/clock` bridge.

| Component | Type | Detail |
|---|---|---|
| Gz Server | `IncludeLaunchDescription` | `gz_sim.launch.py -r -s -v2 <world>`, `on_exit_shutdown=True` |
| Gz GUI | `ExecuteProcess` | `gz sim -g -v2` with `LIBGL_ALWAYS_SOFTWARE=1` injected per-process |
| Clock Bridge | `Node` | `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` |

> **WSL2:** GUI uses `ExecuteProcess` (not `IncludeLaunchDescription`) to inject `LIBGL_ALWAYS_SOFTWARE=1` — forces `llvmpipe` CPU rendering, bypassing the broken `QGLXContext`/`drisw` path on WSLg.

---

### 4.4 `spawn_robots.launch.py`

Spawns N+1 robots via `OpaqueFunction`. Geometry and timing from `launch_common`.

**Per-robot actions:**
1. `generate_robot_sdf(ns)` — patches topics, writes `/tmp/multi_tb3_<ns>.sdf`
2. `ros_gz_sim create` — spawns SDF at `(spawn_x(i), 0, 0.01)`
3. `robot_state_publisher` — URDF-based TF, `frame_prefix=tbX/`
4. `parameter_bridge` — bridges `/tbX/scan`, `/tbX/odom`, `/tbX/cmd_vel`, `/tbX/joint_states`, `/tf`
5. `static_transform_publisher` — `world → tbX/odom` at spawn position

**Spawn timing:**

| Robot | Delay |
|:---:|:---:|
| tb1 | 0 s |
| tb2 | 3 s |
| tb3 | 6 s |

---

### 4.5 `followers.launch.py`

Starts `convoy_publisher` on the leader and one `follower_node` per follower. All timing from `launch_common.follower_start_delay(i)`.

**Arguments:** `nBurger`, `use_sim_time`, `convoy_spacing`

**convoy_publisher (tb1):**
- Namespace: `/tb1`
- Params: `path_frame=world`, `spawn_offset_x=0.0`, `spawn_offset_y=0.0`
- Start: `TimerAction(period=2.0)`

**follower_node (tb2, tb3, ...):**
- Namespace: `/tbX`
- Params: loaded from `follower_params.yaml`, overridden with `convoy_spacing`, `spawn_offset_x=spawn_x(i)`, `spawn_offset_y=0.0`

**Follower start times:**

| Robot | Formula | Start time |
|:---:|---|:---:|
| tb2 | `spawn_delay(2) + FOLLOWER_INIT_BUFFER = 3 + 1` | 4 s |
| tb3 | `spawn_delay(3) + FOLLOWER_INIT_BUFFER = 6 + 1` | 7 s |

---

### 4.6 `rviz.launch.py`

Starts RViz2 with `rviz/multi_robot.rviz`. Fixed frame: `world`.

---

### 4.7 `mapping.launch.py`

Starts `slam_toolbox` online_async, `nav2_map_server` map_saver_server, and a lifecycle manager.

**Arguments:** `slam_robot` (`tb1`/`tb2`/`tb3`/`all`), `mapping_mode`, `autostart`, `use_sim_time`

`slam_robot=all` launches one slam_toolbox instance per robot in the appropriate namespace.

---

## 5. Nodes

### 5.1 `convoy_publisher.py`

**Namespace:** `/tb1`

Records the leader's odometry into a growing `nav_msgs/Path` and publishes it at 10 Hz in the `world` frame.

**Subscriptions:** `odom` → `/tb1/odom` (Best Effort, Volatile, depth=10)

**Publications:** `convoy_path` → `/tb1/convoy_path` (Reliable, depth=10)

**Parameters:**

| Parameter | Default | Description |
|---|:---:|---|
| `max_path_poses` | `2000` | Maximum poses to retain (ring-buffer) |
| `path_resolution` | `0.05` | Minimum distance (m) between consecutive appended poses |
| `path_frame` | `world` | Published path frame ID |
| `spawn_offset_x` | `0.0` | Leader's world X offset (set by launch) |
| `spawn_offset_y` | `0.0` | Leader's world Y offset |

**Algorithm:**
```
/tb1/odom arrives
    │  apply spawn offset: world_x = odom_x + spawn_offset_x
    │
    ▼  if path empty, or hypot(dx, dy) >= path_resolution:
    │      append PoseStamped to path_msg.poses
    │
    ▼  if len(poses) > max_path_poses: trim to last max_path_poses
    │
10 Hz timer → publish path_msg (frame_id=world, stamp=now)
```

---

### 5.2 `follower_node.py`

**Namespace:** `/tbX` (tb2 or tb3)

Pure Pursuit path follower. Runs on a fixed 50 Hz timer — completely decoupled from the 5 Hz LiDAR.

**Subscriptions:**

| Topic | Type | QoS |
|---|---|---|
| `odom` → `/tbX/odom` | `nav_msgs/Odometry` | Best Effort, Volatile, depth=10 |
| `/tb1/convoy_path` | `nav_msgs/Path` | Reliable, depth=10 |
| `scan` → `/tbX/scan` | `sensor_msgs/LaserScan` | Best Effort, Volatile, depth=10 |

**Publications:** `cmd_vel` → `/tbX/cmd_vel` (Reliable, depth=10)

**Convoy slot index:** derived from namespace. `tb2` → `_idx=2`, `_gap=1×convoy_spacing`. `tb3` → `_idx=3`, `_gap=2×convoy_spacing`.

**World pose reconstruction:** `odom_callback` adds `spawn_offset_x/y` to odometry so the robot's pose is in the `world` frame — matching the `/tb1/convoy_path` frame — without a TF lookup.

**Control algorithm (every 1/50 s):**

```
Guard: pose is None or path < 2 points → publish zero, return

Step 1 — Goal point
    Walk path from end backwards, accumulate arc length.
    Stop when accumulated >= _gap → goal_idx = i-1.
    If leader hasn't driven far enough: clamp goal_idx = 0 (path start).
    This ensures the follower closes the gap immediately on startup.

Step 2 — Closest path point
    Scan path[0..goal_idx], find point with minimum Euclidean distance to robot.

Step 3 — Pure Pursuit lookahead
    Walk forward from closest_idx, accumulate arc length.
    Stop at first point where accumulated >= lookahead_distance.
    Cap at goal if lookahead extends past it.

Step 4 — Transform to robot frame
    lx, ly = _to_robot_frame(lookahead_point, robot_pose)
    Ld = hypot(lx, ly)
    alpha = atan2(ly, lx)

Step 5 — Longitudinal control
    gx, _ = _to_robot_frame(goal, robot_pose)
    spacing_err = gx   (positive = goal ahead, negative = overshoot)

Step 6 — Compute velocities
    if spacing_err <= goal_tolerance:
        linear_x = 0.0, angular_z = 0.0           (hold position)
    else:
        linear_x  = kp_linear * spacing_err
        curvature = 2 * ly / Ld²
        angular_z = linear_x * curvature           (Pure Pursuit law)
        if |alpha| > 0.8 rad:                      (large misalignment)
            angular_z = kp_angular * alpha
            linear_x *= 0.3                        (creep, face path first)
        linear_x *= max(0.3, cos(alpha))           (slow in tight curves)

Step 7 — Safety override
    SafetyController.check_and_modify(linear_x, angular_z, scan)
    Emergency stop if obstacle < safe_distance in ±45° front cone.
    Steering bias away from closer side if obstacle < 0.8 m in ±60° cone.

Step 8 — Publish smoothed
    Clamp to velocity limits.
    Slew-rate limit against previous command (max_accel × dt).
    Publish Twist to cmd_vel.
```

**Shutdown:** Publishes zero-velocity Twist, guarded by `rclpy.ok()` and `try/except`.

---

### 5.3 `teleop_controller.py`

Burst-mode keyboard teleoperation. 20 Hz fixed-rate publish; zeros out on key release.

```bash
ros2 run multi_tb3_system teleop_controller.py --ros-args -r __ns:=/tb1
```

**Publications:** `cmd_vel` → `/tb1/cmd_vel` (Reliable, depth=10)

**Key bindings:** `w`/`i` forward, `x`/`,` backward, `a`/`j` left, `d`/`l` right, `s`/`k` stop.

**Implementation notes:**
- Non-blocking `get_key()` with 50 ms timeout in raw terminal mode
- 20 Hz timer zeroes velocity if no key for >150 ms (burst-mode hold)
- ROS spin runs in a background thread so keyboard I/O doesn't block the timer
- On exit, publishes zero 5× with 50 ms gaps

---

### 5.4 `safety_controller.py` (module)

Pure safety layer applied after the control law. No target tracking, no path following.

```python
ctrl = SafetyController(safe_distance=0.4, max_linear_vel=0.22, max_angular_vel=1.0)
safe_lin, safe_ang = ctrl.check_and_modify(linear_x, angular_z, ranges, ...)
```

**Rules (in order):**

| Priority | Rule | Zone | Action |
|:---:|---|:---:|---|
| 1 | Emergency stop | ±45° front | Obstacle < `safe_distance` → `linear_x = 0` (angular unchanged) |
| 2 | Steering bias | ±60°, within 0.8 m | Nudge away from closer side: `angular_z ±= 0.5 × bias` |
| 3 | Velocity clamp | — | Clamp to `±max_linear_vel` and `±max_angular_vel` |

---

## 6. Python Modules

### 6.1 `multi_tb3_system/generate_sdf.py`

Called at launch inside `spawn_robots.launch.py`'s `OpaqueFunction`. Patches the TurtleBot3 `model.sdf` template with per-robot namespaced topic names and writes it to `/tmp/multi_tb3_<ns>.sdf`.

**Why needed:** The TurtleBot3 SDF uses bare plugin topics (`cmd_vel`, `odom`) — with multiple robots, Gazebo broadcasts all data globally, causing control collisions.

**Patches applied (example ns=tb2):**

| Before | After |
|---|---|
| `<topic>scan</topic>` | `<topic>/tb2/scan</topic>` |
| `<topic>cmd_vel</topic>` | `<topic>/tb2/cmd_vel</topic>` |
| `<odom_topic>odom</odom_topic>` | `<odom_topic>/tb2/odom</odom_topic>` |
| `<frame_id>odom</frame_id>` | `<frame_id>tb2/odom</frame_id>` |
| `<child_frame_id>base_footprint</child_frame_id>` | `<child_frame_id>tb2/base_footprint</child_frame_id>` |

**API:** `generate_robot_sdf('tb2')` → `'/tmp/multi_tb3_tb2.sdf'`

---

### 6.2 `multi_tb3_system/launch_common.py`

Shared configuration module. The single source of truth for spawn geometry and timing. Imported by both `spawn_robots.launch.py` and `followers.launch.py` to prevent drift.

**Constants:**

| Constant | Value | Description |
|---|:---:|---|
| `SPAWN_X_STEP` | `-1.0 m` | X distance between robots |
| `SPAWN_Y` | `0.0 m` | All robots at Y=0 |
| `SPAWN_Z` | `0.01 m` | Spawn height above ground |
| `SPAWN_DELAY_STEP` | `3.0 s` | Time between successive spawns |
| `FOLLOWER_INIT_BUFFER` | `1.0 s` | Extra wait after spawn before follower drives |
| `MAX_FOLLOWERS` | `2` | Maximum supported follower count |

**Helper functions:**

| Function | Formula | Result |
|---|---|---|
| `spawn_x(i)` | `(i-1) × SPAWN_X_STEP` | tb1=0.0, tb2=-1.0, tb3=-2.0 |
| `spawn_delay(i)` | `(i-1) × SPAWN_DELAY_STEP` | tb2=3.0s, tb3=6.0s |
| `follower_start_delay(i)` | `spawn_delay(i) + FOLLOWER_INIT_BUFFER` | tb2=4.0s, tb3=7.0s |

---

### 6.3 `scripts/laser_processor.py`

Pure-geometry library. **Not used by `follower_node.py`** — left in the repository for potential future use. The follower passes raw scan data directly to `SafetyController`.

**Key type:** `Cluster` dataclass with `centroid_x`, `centroid_y`, `distance`, `angle`, `size`.

**Functions:** `scan_to_cartesian`, `filter_front_sector`, `euclidean_cluster`, `make_clusters`, `select_target_cluster`, `process_scan` (full pipeline).

---

## 7. Config Files

### 7.1 `config/follower_params.yaml`

Loaded by `followers.launch.py` as a params file for each `follower_node.py`. `convoy_spacing` and `spawn_offset_x/y` are overridden at launch time.

```yaml
follower_node:
  ros__parameters:
    leader_ns:            'tb1'

    # Convoy spacing
    convoy_spacing:       0.6     # gap per slot [m] (tb2=1×, tb3=2×)
    goal_tolerance:       0.10    # stop when forward distance to goal <= this [m]

    # Pure Pursuit
    lookahead_distance:   0.5     # lookahead arc-length along path [m]
    kp_linear:            0.8     # speed gain on spacing error
    kp_angular:           1.5     # heading gain (large-misalignment recovery)

    # Limits
    max_linear_velocity:  0.22    # [m/s]
    max_angular_velocity: 1.0     # [rad/s]
    safe_distance:        0.4     # LiDAR emergency stop [m]

    # Control loop
    control_frequency:    50.0    # Hz
    max_linear_accel:     1.0     # m/s²
    max_angular_accel:    3.0     # rad/s²
```

**Parameters injected at launch (not in YAML):**
- `spawn_offset_x` — from `launch_common.spawn_x(i)`
- `spawn_offset_y` — `SPAWN_Y = 0.0`

---

### 7.2 `config/mapping_online_async.yaml`

slam_toolbox configuration tuned for TurtleBot3 Burger LDS-01.

| Parameter | Default | Description |
|---|:---:|---|
| `resolution` | `0.05` | Map cell size (m/pixel) |
| `max_laser_range` | `3.5` | LDS-01 max range (m) |
| `min_laser_range` | `0.12` | LDS-01 min range (m) |
| `minimum_travel_distance` | `0.3` | Min motion (m) before new scan processed |
| `minimum_travel_heading` | `0.3` | Min rotation (rad) before new scan processed |
| `map_update_interval` | `3.0` | Seconds between map publications |
| `do_loop_closing` | `true` | Enable loop closure detection |

---

## 8. Worlds

### `worlds/empty.world`

Flat ground plane with directional sun. No obstacles. Best for formation validation and gain tuning. Physics step: **1 ms**.

### `worlds/pillars.world`

Six cylindrical pillars (`r=0.10 m`, `h=0.50 m`) in a staggered slalom pattern. Physics step: **1 ms**.

```
Direction of travel: ──────────────────────────────────────►

Y+ ●  P1(1.5, 0.5)     P3(3.5, 0.5)     P5(5.5, 0.5)

Y-       P2(2.5,-0.5)     P4(4.5,-0.5)     P6(6.5,-0.5)
```

> **Physics step:** Both worlds use `<step_size>0.001</step_size>` (1 ms). A 4 ms step caused bursty pose updates visible as GUI jitter and uneven path recording. 1 ms gives smooth, continuous odometry — see Section 14.3.

---

## 9. TF Tree

All three robots share a common `world` root via static transforms published in `spawn_robots.launch.py`.

```
world                              ← shared static root
│
├── tb1/odom                       ← static_transform_publisher @ (x=0.0, y=0, z=0)
│   └── tb1/base_footprint         ← Gazebo DiffDrive via /tf bridge
│       └── tb1/base_link          ← robot_state_publisher (URDF, frame_prefix=tb1/)
│           ├── tb1/imu_link
│           ├── tb1/base_scan
│           └── tb1/wheel_left_link, tb1/wheel_right_link
│
├── tb2/odom                       ← static_transform_publisher @ (x=-1.0, y=0, z=0)
│   └── tb2/base_footprint
│       └── tb2/base_link  ...
│
└── tb3/odom                       ← static_transform_publisher @ (x=-2.0, y=0, z=0)
    └── tb3/base_footprint
        └── tb3/base_link  ...
```

**RViz fixed frame:** `world`

**Follower world pose:** `follower_node` reconstructs world pose by adding `spawn_offset_x/y` to odometry. This matches the `/tb1/convoy_path` frame without a TF lookup at control time.

---

## 10. Topic Architecture

### Per-robot (X = 1, 2, 3)

| Topic | Type | Direction | Source → Sink |
|---|---|:---:|---|
| `/tbX/scan` | `sensor_msgs/LaserScan` | Gz → ROS | Gazebo LiDAR → `follower_node` (safety only) |
| `/tbX/odom` | `nav_msgs/Odometry` | Gz → ROS | Gazebo DiffDrive → `convoy_publisher` (tb1) / `follower_node` (tb2/3) |
| `/tbX/cmd_vel` | `geometry_msgs/Twist` | ROS → Gz | `teleop`/`follower_node` → Gazebo DiffDrive |
| `/tbX/joint_states` | `sensor_msgs/JointState` | Gz → ROS | Gazebo → `robot_state_publisher` |

### Global

| Topic | Type | Direction | Description |
|---|---|:---:|---|
| `/tb1/convoy_path` | `nav_msgs/Path` | ROS only | Leader trajectory; all followers subscribe |
| `/tf` | `tf2_msgs/TFMessage` | Gz → ROS | All odom → base_footprint transforms |
| `/clock` | `rosgraph_msgs/Clock` | Gz → ROS | Simulation clock |

### ros_gz_bridge format (`[` = Gz→ROS, `]` = ROS→Gz)

```
/tbX/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/tbX/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry
/tbX/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
/tbX/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model
/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

`/tb1/convoy_path` is a ROS-only topic — not bridged to Gazebo.

---

## 11. Timing Sequence

```
t = 0 s   Gazebo server starts
          tb1 spawns (x=0.0): RSP + bridge + static_tf world→tb1/odom
t = 2 s   convoy_publisher starts in /tb1 namespace
          (odom and bridge settled; path recording begins)
t = 3 s   tb2 spawns (x=-1.0): RSP + bridge + static_tf world→tb2/odom
t = 4 s   tb2/follower_node starts   [spawn_delay(2) + FOLLOWER_INIT_BUFFER = 3+1]
t = 6 s   tb3 spawns (x=-2.0): RSP + bridge + static_tf world→tb3/odom
t = 7 s   tb3/follower_node starts   [spawn_delay(3) + FOLLOWER_INIT_BUFFER = 6+1]
```

**Why the delays:** `FOLLOWER_INIT_BUFFER = 1.0 s` ensures the Gazebo entity, DiffDrive plugin, and `ros_gz_bridge` are fully initialized before `cmd_vel` flows. `convoy_publisher` waits 2 s so tb1's odom + bridge are live before path recording starts.

---

## 12. Tuning Guide

All follower parameters live in `config/follower_params.yaml`. No rebuild required. `convoy_spacing` can also be set at launch:

```bash
ros2 launch multi_tb3_system robot.launch.py convoy_spacing:=0.8
```

### convoy_spacing

Gap per slot: tb2 = 1×, tb3 = 2×. Recommended range: 0.5–1.5 m. Below 0.4 m risks safety-stop triggers.

### lookahead_distance

How far ahead on the path the robot steers toward.

| Effect | Adjustment |
|---|---|
| Cuts corners, path deviation | decrease (e.g., 0.3 m) |
| Oscillates left/right | increase (e.g., 0.7–1.0 m) |

### kp_linear

Gain on forward distance to goal point.

| Symptom | Fix |
|---|---|
| Oscillates fore/aft | reduce to 0.5–0.6 |
| Too slow to close gap | increase to 1.0–1.2 |

### kp_angular

Activates only in large-misalignment recovery (`|alpha| > 0.8 rad`). Normal following uses Pure Pursuit curvature.

| Symptom | Fix |
|---|---|
| Spins excessively on recovery | reduce to 1.0–1.2 |

### Slew rate limits

`max_linear_accel` (m/s²) and `max_angular_accel` (rad/s²) limit how fast commands ramp:

```
max_delta = max_accel × dt = max_accel / 50.0
```

Lower values → smoother but slower response. Defaults (1.0 and 3.0) are appropriate for TB3 in simulation.

### goal_tolerance

Follower holds position when forward distance to goal ≤ this. Increase to 0.15 m if follower oscillates at the hold point.

---

## 13. Debugging

```bash
# Convoy path being published?
ros2 topic hz /tb1/convoy_path

# Follower control rate (~50 Hz)
ros2 topic hz /tb2/cmd_vel
ros2 topic hz /tb3/cmd_vel

# Watch follower commands live
ros2 topic echo /tb2/cmd_vel

# Scan rate (~5 Hz)
ros2 topic hz /tb2/scan

# Nodes alive?
ros2 node list | grep convoy_publisher
ros2 node list | grep follower
ros2 node list | grep bridge

# Runtime parameters
ros2 param list /tb2/follower_node
ros2 param get /tb2/follower_node convoy_spacing
ros2 param get /tb2/follower_node spawn_offset_x

# TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo world tb2/base_footprint

# SDF patch verification
grep -E "/tb2|cmd_vel|odom|base_footprint" /tmp/multi_tb3_tb2.sdf
```

### Common issues

| Symptom | Cause | Fix |
|---|---|---|
| No follower nodes in `node list` | Timing (start at t=4s/7s) | Wait 8s; check `followers.launch.py` output for OpaqueFunction errors |
| `/tb1/convoy_path` absent | `convoy_publisher` not started | `ros2 node list \| grep convoy_publisher`; check `/tb1/odom` is flowing |
| Follower stationary after startup | Goal behind robot initially | Drive tb1 forward; follower closes gap once goal enters forward cone |
| Follower oscillates laterally | `kp_angular` too high / `lookahead_distance` too small | Reduce `kp_angular` to 1.0–1.2; increase `lookahead_distance` |
| `exit 127`, node dies silently | CRLF shebang (`python3\r`) | See Section 14.1; re-checkout after `.gitattributes` applied |
| Gazebo GUI crashes (WSLg) | `QGLXContext`/`drisw` failure | `LIBGL_ALWAYS_SOFTWARE=1` already injected; launch without `gz:=true` if still failing |
| `rclpy RCLError` on shutdown | Context invalidated before `finally` | Fixed — `rclpy.ok()` guard in `follower_node.py` |
| No `/tbX/scan` | Bridge not started / robot not spawned | Check `ros2 node list \| grep bridge_tbX`; verify `/tmp/multi_tb3_tbX.sdf` exists |

---

## 14. Recent Fixes & Known Issues Resolved

### 14.1 CRLF Line Endings Break Python Shebangs (Windows Git Mount)

**Issue:** When the repository was checked out via a Windows-mounted WSL2 filesystem with a Windows git client, Python scripts gained CRLF line endings. The shebang `#!/usr/bin/env python3` became `#!/usr/bin/env python3\r`. The Linux kernel looked for an interpreter literally named `python3<CR>`, failed, and the node exited with code 127 (command not found) — silently, with no output in the launch console.

**Fix:** Added `.gitattributes` at the repository root:
```
*.py    text eol=lf
*.yaml  text eol=lf
*.world text eol=lf
*.sdf   text eol=lf
```
After committing, existing checkouts must run `git checkout -- .` to re-normalize all files. Confirm with `file install/.../follower_node.py` — should say "ASCII text", not "ASCII text, with CRLF".

---

### 14.2 Launch Timing Mismatch — Follower Started Before Its Robot Spawned

**Issue:** `followers.launch.py` had its own hardcoded timing constants (`_SPAWN_DELAY_STEP = 5.0 s`) that had diverged from `launch_common.SPAWN_DELAY_STEP = 3.0 s`. This meant:
- tb2 follower started at `(2-1)*5 + 2 = 7 s`, but tb2 spawned at `3 s` — acceptable.
- tb3 follower started at `(3-1)*5 + 2 = 12 s` — far too late.

In an earlier variant, tb3 follower started *before* tb3 spawned, so `cmd_vel` messages were published before the bridge existed — the robot never moved.

**Fix:** Removed all hardcoded timing constants from `followers.launch.py`. Both spawn and follower launch files now derive everything from `launch_common`:
```python
from multi_tb3_system.launch_common import follower_start_delay, spawn_x
actions.append(TimerAction(period=follower_start_delay(i), actions=[node]))
```
Timing drift between the two files is now structurally impossible.

---

### 14.3 Gazebo Physics Step 4 ms Caused Bursty GUI Jitter

**Issue:** World files used `<step_size>0.004</step_size>` (4 ms / 250 Hz physics). At this rate, physics updates were applied in larger batches — odometry arrived unevenly, producing visible jitter in the Gazebo GUI and RViz robot models. The convoy path also recorded longer straight segments between pose updates, reducing path-following accuracy at corners.

**Fix:** Both world files changed to `<step_size>0.001</step_size>` (1 ms / 1000 Hz). This produces smooth, continuous pose updates with no GUI jitter. The simulation remains real-time on standard hardware.

---

### 14.4 Follower Hold-Position Bug on Startup

**Issue:** The original `follower_node` used arc-length between path indices as the longitudinal error metric. If the leader had not yet driven `_gap` metres, the goal search reached `path[0]` without satisfying the condition — the node returned with no goal and the follower sat still indefinitely. Additionally, Euclidean distance to the goal (rather than forward projection) caused overshoot oscillation.

**Fix (two changes):**

1. **Goal clamping:** When the arc-length walk reaches `path[0]` without satisfying `>= _gap`, set `goal_idx = 0` (path start) rather than returning. The follower immediately begins closing the gap toward the path start.

2. **Forward projection for longitudinal control:** `spacing_err` is now the X component (`gx`) of the goal transformed into robot frame:
```python
gx, _ = _to_robot_frame(goal[0], goal[1], rx, ry, ryaw)
spacing_err = gx
```
Positive `gx` = goal is ahead → drive forward. Negative `gx` = overshoot → slow to zero or slight reverse.

---

### 14.5 Gazebo GUI WSLg Crash — `drisw` Failure [v0.2.0]

**Issue:** `gz sim -g` crashed immediately on WSLg with `QGLXContext`/`drisw screen creation` errors.

**Fix:** `gazebo.launch.py` uses `ExecuteProcess` for the GUI, injecting `LIBGL_ALWAYS_SOFTWARE=1` per-process to force `llvmpipe` CPU rendering.

---

### 14.6 RViz TF Tree Disconnection [v0.2.0]

**Issue:** RViz reported `No transform from [tb2/base_footprint] to [world]` — the three robot TF trees had no common root.

**Fix:** `spawn_robots.launch.py` publishes a `static_transform_publisher` for each robot: `world → tbX/odom` at spawn position. RViz fixed frame set to `world`.

---

### 14.7 `rclpy` Shutdown Exception [v0.2.0]

**Issue:** `Ctrl+C` shutdown produced `rclpy.exceptions.RCLError: context is not valid` when the zero-velocity publish in the `finally` block ran after the ROS context was torn down.

**Fix:** The shutdown publish is wrapped in `if rclpy.ok(): try/except Exception` in `follower_node.py`.
