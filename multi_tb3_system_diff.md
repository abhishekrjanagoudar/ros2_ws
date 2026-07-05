`diff
diff --git a/src/multi_tb3_system/.gitignore b/src/multi_tb3_system/.gitignore
deleted file mode 100644
index d0bcb70..0000000
--- a/src/multi_tb3_system/.gitignore
+++ /dev/null
@@ -1,22 +0,0 @@
-# ── Windows metadata (created when files are downloaded on Windows) ────────────
-*Zone.Identifier
-*:Zone.Identifier
-
-# ── Python build artefacts ────────────────────────────────────────────────────
-__pycache__/
-*.py[cod]
-*.pyo
-
-# ── ROS 2 / colcon build output ───────────────────────────────────────────────
-build/
-install/
-log/
-
-
-# ── Editor / OS ───────────────────────────────────────────────────────────────
-.vscode/
-*.swp
-*.swo
-.DS_Store
-Thumbs.db
-ROS2/
diff --git a/src/multi_tb3_system/CMakeLists.txt b/src/multi_tb3_system/CMakeLists.txt
index e2612fd..1076ad1 100644
--- a/src/multi_tb3_system/CMakeLists.txt
+++ b/src/multi_tb3_system/CMakeLists.txt
@@ -31,6 +31,7 @@ install(PROGRAMS
   scripts/costmap_utils.py
   scripts/follower_node.py
   scripts/follower_state.py
+  scripts/local_planner.py
   scripts/motion_controller.py
   scripts/safety_controller.py
   scripts/teleop_controller.py
diff --git a/src/multi_tb3_system/README.md b/src/multi_tb3_system/README.md
index d0faa24..4404fbb 100644
--- a/src/multi_tb3_system/README.md
+++ b/src/multi_tb3_system/README.md
@@ -18,7 +18,9 @@
 
 ## Overview
 
-3-robot TurtleBot3 Burger convoy in Gazebo Sim. `tb1` is teleoperated; `tb2` and `tb3` follow using **Pure Pursuit** on a shared `nav_msgs/Path`. LiDAR is used for emergency stop and local costmap-based obstacle avoidance.
+3-robot TurtleBot3 Burger convoy in Gazebo Sim. `tb1` is teleoperated; `tb2` and `tb3` follow using **Pure Pursuit** on a shared `nav_msgs/Path`. 
+
+**Strictly Mapless Laser Odometry**: To combat Gazebo's simulated wheel slip without relying on a global SLAM map, this project integrates `rf2o_laser_odometry`. It performs Iterative Closest Point (ICP) scan-matching to perfectly sync the physical world with the universal `map` frame, allowing flawlessly accurate breadcrumb tracking across the convoy.
 
 ---
 
@@ -40,8 +42,16 @@ export TURTLEBOT3_MODEL=burger
 
 ## Build
 
+First, clone the laser odometry dependency into your workspace (if not already present):
+```bash
+cd ~/ros2_ws/src
+git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
+cd ~/ros2_ws
+```
+
+Then build the workspace:
 ```bash
-colcon build --packages-select multi_tb3_system
+colcon build --packages-select multi_tb3_system rf2o_laser_odometry
 source install/setup.bash
 ```
 
@@ -91,9 +101,7 @@ multi_tb3_system/
 │   └── teleop_controller.py        # Burst-mode keyboard teleop
 ├── multi_tb3_system/               # Importable Python package
 │   ├── launch_common.py            # Convoy geometry constants (single source of truth)
-│   ├── generate_sdf.py             # Per-robot SDF generator
-│   └── perception/
-│       └── laser_processor.py      # Scan clustering library (future use)
+│   └── generate_sdf.py             # Per-robot SDF generator
 ├── launch/
 │   ├── robot.launch.py             # ⭐ Entry point
 │   ├── followers.launch.py         # Starts convoy_publisher + costmap + followers
diff --git a/src/multi_tb3_system/config/follower_params.yaml b/src/multi_tb3_system/config/follower_params.yaml
index e6a0733..5b53132 100644
--- a/src/multi_tb3_system/config/follower_params.yaml
+++ b/src/multi_tb3_system/config/follower_params.yaml
@@ -10,15 +10,15 @@ follower_node:
     goal_tolerance:       0.12    # stop when within this distance of goal [m] (realistic for 20 Hz + slew limits)
 
     # Pure Pursuit
-    lookahead_distance:   0.25    # lookahead point distance along the path [m] — must be < gap/2 (gap = convoy_spacing × (slot-1))
+    lookahead_distance:   0.5     # lookahead point distance along the path [m] — increased to prevent overshoot arcs
     kp_linear:            0.6     # forward speed gain on spacing error
-    kp_angular:           1.8     # heading gain (large-misalignment recovery)
+    kp_angular:           2.2     # heading gain (large-misalignment recovery)
 
     # Limits
     max_linear_velocity:  0.22    # TB3 Burger hardware limit
     max_angular_velocity: 1.0
     # LiDAR hard-stop distance (SafetyController) [m]
-    safe_distance:        0.15
+    safe_distance:        0.20    # Emergency stop distance - doubled from 0.15m for safety margin
 
     # Convoy predecessor filtering
     predecessor_gap:      0.6     # must equal convoy_spacing [m]
@@ -37,10 +37,13 @@ follower_node:
 
     # Obstacle-avoidance state machine
     detour_forward_min_vel:      0.10  # forward creep maintained during DETOUR [m/s] (more assertive to clear narrow gaps)
-    stationary_deadlock_timeout: 3.5   # escalate to SEARCH after this many continuous stationary seconds [s] (give DETOUR time to work)
+    stationary_deadlock_timeout: 2.0   # escalate to SEARCH after this many continuous stationary seconds [s] (quicker recovery from stuck states)
     emergency_recovery_timeout:  0.5   # informational — resume window after emergency clears [s]
     search_angular_velocity:     0.9   # rotate-in-place rate during SEARCH [rad/s] (<= max_angular_velocity; faster sweep finds clear heading sooner)
 
+    # Local planner (Nav2-like obstacle avoidance)
+    enable_local_planner:        true  # Use DWA-inspired local planner for DETOUR state (set false for simple biased turning)
+
     # Persistent leader tracking
     breadcrumb_timeout:          12.0
 
@@ -54,3 +57,9 @@ costmap_generator:
     costmap_stale_timeout:       1.0
     costmap_frame:               ''
     range_min_override:          0.0
+
+convoy_publisher:
+  ros__parameters:
+    path_resolution: 0.01
+    yaw_resolution: 0.05
+
diff --git a/src/multi_tb3_system/launch/followers.launch.py b/src/multi_tb3_system/launch/followers.launch.py
index 0d04d6e..b6ff8a1 100644
--- a/src/multi_tb3_system/launch/followers.launch.py
+++ b/src/multi_tb3_system/launch/followers.launch.py
@@ -33,27 +33,26 @@ def _launch_setup(context, *args, **kwargs):
 
     #  Startup timeline (all times relative to when this launch file is invoked,
 
-    CONVOY_PUB_START  = 0.0    # leader path recording — start immediately
-    COSTMAP_START     = 0.38   # per-robot costmap generator — after scan is live
-    FOLLOWER1_START   = 1.00   # tb2 follower node
-    FOLLOWER2_START   = 1.25   # tb3 follower node — 0.5s after tb2
-
-    # Leader trajectory publisher (tb1)
-    convoy_pub = Node(
-        package='multi_tb3_system',
-        executable='convoy_publisher.py',
-        name='convoy_publisher',
-        namespace=_LEADER_NS,
-        parameters=[{
-            'use_sim_time':   use_sim_time,
-            'path_frame':     'world',
-            'spawn_offset_x': spawn_x(1),
-            'spawn_offset_y': SPAWN_Y,
-        }],
-        output='screen',
-        emulate_tty=True,
-    )
-    actions.append(TimerAction(period=CONVOY_PUB_START, actions=[convoy_pub]))
+    from multi_tb3_system.launch_common import spawn_delay
+
+    # Path publishers for all predecessors (tb1 ... tb[N])
+    # tb1 drops breadcrumbs for tb2. tb2 drops breadcrumbs for tb3, etc.
+    for i in range(1, n_burgers + 1):
+        ns = f'tb{i}'
+        convoy_pub = Node(
+            package='multi_tb3_system',
+            executable='convoy_publisher.py',
+            name='convoy_publisher',
+            namespace=ns,
+            parameters=[params_file, {
+                'use_sim_time':   use_sim_time,
+                'path_frame':     'map',
+            }],
+            output='screen',
+            emulate_tty=True,
+        )
+        delay = spawn_delay(i) + 4.0
+        actions.append(TimerAction(period=delay, actions=[convoy_pub]))
 
     # Per-robot costmap_generator (tb1, tb2, tb3, ...)
     for i in range(1, n_burgers + 2):
@@ -73,14 +72,14 @@ def _launch_setup(context, *args, **kwargs):
             output='screen',
             emulate_tty=True,
         )
-        actions.append(TimerAction(period=COSTMAP_START, actions=[cg_node]))
+        delay = spawn_delay(i) + 4.5
+        actions.append(TimerAction(period=delay, actions=[cg_node]))
 
     # Pure-Pursuit followers (tb2, tb3, ...)
-    follower_delays = {2: FOLLOWER1_START, 3: FOLLOWER2_START}
-
     for i in range(2, n_burgers + 2):
         ns = f'tb{i}'
-        delay = follower_delays.get(i, FOLLOWER1_START + (i - 2) * 2.0)
+        leader_ns = f'tb{i-1}'  # Daisy-chain: tb[i] tracks tb[i-1]
+        
         node = Node(
             package='multi_tb3_system',
             executable='follower_node.py',
@@ -90,18 +89,19 @@ def _launch_setup(context, *args, **kwargs):
                 params_file,
                 {
                     'use_sim_time':   use_sim_time,
-                    'leader_ns':      _LEADER_NS,
+                    'leader_ns':      leader_ns,
                     'convoy_spacing': convoy_spacing,
                     'spawn_offset_x': spawn_x(i),
                     'spawn_offset_y': SPAWN_Y,
                 },
             ],
             remappings=[
-                ('convoy_path', f'/{_LEADER_NS}/convoy_path'),
+                ('convoy_path', f'/{leader_ns}/convoy_path'),
             ],
             output='screen',
             emulate_tty=True,
         )
+        delay = spawn_delay(i) + 5.0
         actions.append(TimerAction(period=delay, actions=[node]))
 
     return actions
diff --git a/src/multi_tb3_system/launch/robot.launch.py b/src/multi_tb3_system/launch/robot.launch.py
index a0694f5..5a49b48 100644
--- a/src/multi_tb3_system/launch/robot.launch.py
+++ b/src/multi_tb3_system/launch/robot.launch.py
@@ -26,6 +26,8 @@ def _resolve_ui_flags(context, *args, **kwargs):
     nBurger        = LaunchConfiguration('nBurger').perform(context)
     world          = LaunchConfiguration('world').perform(context)
     use_sim_time   = LaunchConfiguration('use_sim_time').perform(context)
+    enable_followers = LaunchConfiguration('enable_followers').perform(context)
+    enable_rf2o    = LaunchConfiguration('enable_rf2o').perform(context)
 
     # ros_ui=true → both GUIs on. ros_ui=false (default) → respect individual gz/rviz flags.
     if ros_ui == 'true':
@@ -48,9 +50,11 @@ def _resolve_ui_flags(context, *args, **kwargs):
 
     actions = [
         _include('worlds.launch.py',       {'world': world, 'gz': effective_gz}),
-        _include('spawn_robots.launch.py', {'nBurger': nBurger}),
-        _include('followers.launch.py',    {'nBurger': nBurger, 'rviz': effective_rviz}),
+        _include('spawn_robots.launch.py', {'nBurger': nBurger, 'enable_rf2o': enable_rf2o}),
     ]
+    if enable_followers == 'true':
+        actions.append(_include('followers.launch.py', {'nBurger': nBurger, 'rviz': effective_rviz}))
+    
     if effective_rviz == 'true':
         actions.append(_include('rviz.launch.py'))
 
@@ -75,6 +79,10 @@ def generate_launch_description() -> LaunchDescription:
                               description="Show RViz2. Overridden by ros_ui."),
         DeclareLaunchArgument('ros_ui',       default_value='false',
                               description="'true' → gz=true + rviz=true. Overrides gz and rviz."),
+        DeclareLaunchArgument('enable_followers', default_value='true',
+                              description="Include followers.launch.py (Costmaps, Convoy Publisher, Followers)."),
+        DeclareLaunchArgument('enable_rf2o', default_value='true',
+                              description="Enable RF2O laser odometry nodes."),
 
         # Expose TurtleBot3 mesh assets to Gazebo
         AppendEnvironmentVariable(
diff --git a/src/multi_tb3_system/launch/spawn_robots.launch.py b/src/multi_tb3_system/launch/spawn_robots.launch.py
index e5b662d..24d3812 100644
--- a/src/multi_tb3_system/launch/spawn_robots.launch.py
+++ b/src/multi_tb3_system/launch/spawn_robots.launch.py
@@ -5,6 +5,7 @@ spawn_robots.launch.py — spawns N+1 TurtleBot3 robots (leader + followers).
 
 from launch import LaunchDescription
 from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
+from launch.conditions import IfCondition
 from launch.substitutions import LaunchConfiguration
 from launch_ros.actions import Node
 
@@ -18,7 +19,7 @@ from multi_tb3_system.launch_common import (
 )
 
 
-def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool) -> list:
+def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool, is_leader: bool) -> list:
     """Return [spawn, rsp, bridge, static_tf] actions for one robot."""
     from multi_tb3_system.generate_sdf import generate_robot_sdf
 
@@ -48,6 +49,10 @@ def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool) -> lis
             'robot_description': urdf,
             'frame_prefix':      f'{ns}/',   # scopes TF frames: ns/base_link, etc.
         }],
+        remappings=[
+            ('tf', '/tf'),
+            ('tf_static', '/tf_static'),
+        ],
         output='screen',
     )
 
@@ -58,29 +63,60 @@ def _make_robot_actions(ns: str, x: float, urdf: str, use_sim_time: bool) -> lis
         name=f'bridge_{ns}',
         arguments=[
             f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
-            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
             f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
             f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
-            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
         ],
         output='screen',
     )
 
-    # Anchor tbX/odom to shared world frame at spawn position so all TF trees share a root.
+    actions = [spawn, rsp, bridge]
+
+    # Anchor all robot odom frames to shared map frame at spawn position so all TF trees share a root.
     static_tf = Node(
         package='tf2_ros',
         executable='static_transform_publisher',
-        name=f'static_tf_world_{ns}_odom',
+        name=f'static_tf_map_{ns}_odom',
         arguments=[
             '--x', str(x), '--y', '0', '--z', '0',
             '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
-            '--frame-id', 'world',
+            '--frame-id', 'map',
             '--child-frame-id', f'{ns}/odom',
         ],
+        parameters=[{'use_sim_time': use_sim_time}],
+        output='screen',
+    )
+    actions.append(static_tf)
+
+    # Mapless Laser Odometry
+    # Launched with a delay so Gazebo/Bridge can spin up and /scan exists
+    rf2o = Node(
+        package='rf2o_laser_odometry',
+        executable='rf2o_laser_odometry_node',
+        name=f'rf2o_{ns}',
+        namespace=ns,
+        parameters=[{
+            'use_sim_time': use_sim_time,
+            'laser_scan_topic': f'/{ns}/scan',
+            'odom_topic': f'/{ns}/odom',
+            'publish_tf': True,
+            'base_frame_id': f'{ns}/base_footprint',
+            'odom_frame_id': f'{ns}/odom',
+            'init_pose_from_topic': '',
+            'freq': 30.0
+        }],
+        remappings=[
+            ('tf', '/tf'),
+            ('tf_static', '/tf_static'),
+        ],
         output='screen',
     )
+    actions.append(TimerAction(
+        period=3.0,
+        actions=[rf2o],
+        condition=IfCondition(LaunchConfiguration('enable_rf2o'))
+    ))
 
-    return [spawn, rsp, bridge, static_tf]
+    return actions
 
 
 def _launch_setup(context, *args, **kwargs):
@@ -93,7 +129,7 @@ def _launch_setup(context, *args, **kwargs):
 
     for i in range(1, total + 1):
         ns      = f'tb{i}'
-        actions = _make_robot_actions(ns, spawn_x(i), urdf, use_sim_time)
+        actions = _make_robot_actions(ns, spawn_x(i), urdf, use_sim_time, is_leader=(i == 1))
 
         if i == 1:
             all_actions.extend(actions)   # leader spawns immediately
@@ -109,5 +145,7 @@ def generate_launch_description() -> LaunchDescription:
                               description='Follower count (1–2). Total = nBurger + 1.'),
         DeclareLaunchArgument('use_sim_time', default_value='true',
                               description="'true' = Gz clock, 'false' = wall clock."),
+        DeclareLaunchArgument('enable_rf2o',  default_value='true',
+                              description='Enable RF2O laser odometry nodes.'),
         OpaqueFunction(function=_launch_setup),
     ])
diff --git a/src/multi_tb3_system/models/turtlebot3_burger/model-1_4.sdf b/src/multi_tb3_system/models/turtlebot3_burger/model-1_4.sdf
index 5923efd..281a573 100644
--- a/src/multi_tb3_system/models/turtlebot3_burger/model-1_4.sdf
+++ b/src/multi_tb3_system/models/turtlebot3_burger/model-1_4.sdf
@@ -375,8 +375,8 @@
       <right_joint>wheel_right_joint</right_joint>
 
       <!-- kinematics -->
-      <wheel_separation>0.160</wheel_separation>
-      <wheel_diameter>0.066</wheel_diameter>
+      <wheel_separation>0.1695</wheel_separation>
+      <wheel_diameter>0.0612</wheel_diameter>
 
       <!-- limits -->
       <max_wheel_torque>20</max_wheel_torque>
diff --git a/src/multi_tb3_system/models/turtlebot3_burger/model.sdf b/src/multi_tb3_system/models/turtlebot3_burger/model.sdf
index a82d9bb..37f4615 100644
--- a/src/multi_tb3_system/models/turtlebot3_burger/model.sdf
+++ b/src/multi_tb3_system/models/turtlebot3_burger/model.sdf
@@ -142,10 +142,10 @@
         <lidar>
           <scan>
             <horizontal>
-              <samples>360</samples>
+              <samples>240</samples>
               <resolution>1.000000</resolution>
-              <min_angle>0.000000</min_angle>
-              <max_angle>6.280000</max_angle>
+              <min_angle>-2.094395</min_angle>
+              <max_angle>2.094395</max_angle>
             </horizontal>
           </scan>
           <range>
@@ -380,15 +380,15 @@
       <right_joint>wheel_right_joint</right_joint>
 
       <!-- kinematics -->
-      <wheel_separation>0.160</wheel_separation>
-      <wheel_radius>0.033</wheel_radius>
+      <wheel_separation>0.1695</wheel_separation>
+      <wheel_radius>0.0306</wheel_radius>
 
       <!-- limits -->
       <max_linear_acceleration>1.0</max_linear_acceleration>
 
 
 
-      <frame_id>odom</frame_id>
+      <frame_id>odom_gz</frame_id>
       <child_frame_id>base_footprint</child_frame_id>
       <odom_publisher_frequency>30</odom_publisher_frequency>
 
diff --git a/src/multi_tb3_system/multi_tb3_system/generate_sdf.py b/src/multi_tb3_system/multi_tb3_system/generate_sdf.py
index 2f4709d..c07a96b 100644
--- a/src/multi_tb3_system/multi_tb3_system/generate_sdf.py
+++ b/src/multi_tb3_system/multi_tb3_system/generate_sdf.py
@@ -10,6 +10,13 @@ from ament_index_python.packages import get_package_share_directory
 
 logger = logging.getLogger(__name__)
 
+# Individual Wheel Calibrations
+WHEEL_CONFIGS = {
+    'tb1': {'wheel_radius': 0.0320, 'wheel_separation': 0.1779},
+    'tb2': {'wheel_radius': 0.0320, 'wheel_separation': 0.1617},
+    'tb3': {'wheel_radius': 0.0320, 'wheel_separation': 0.1779},
+}
+
 # Topic replacement map
 
 _TOPIC_PATCHES = [
@@ -27,10 +34,10 @@ _TOPIC_PATCHES = [
     (r'<tf_topic>/tf</tf_topic>',    '<tf_topic>/tf</tf_topic>'),   # keep global
 
     # Frame IDs (published in odometry message header)
-    (r'<frame_id>odom</frame_id>',
-     '<frame_id>{ns}/odom</frame_id>'),
+    (r'<frame_id>odom_gz</frame_id>',
+     '<frame_id>{ns}/odom_gz</frame_id>'),
     (r'<child_frame_id>base_footprint</child_frame_id>',
-     '<child_frame_id>{ns}/base_footprint</child_frame_id>'),
+     '<child_frame_id>{ns}/base_footprint_gz</child_frame_id>'),
     (r'<gz_frame_id>base_scan</gz_frame_id>',
      '<gz_frame_id>{ns}/base_scan</gz_frame_id>'),
 ]
@@ -56,7 +63,7 @@ Ensure the DiffDrive plugin block contains explicit <topic>, <odom_topic>,
         if '<topic>' not in body:
             injections.append(f'      <topic>/{ns}/cmd_vel</topic>')
         if '<odom_topic>' not in body:
-            injections.append(f'      <odom_topic>/{ns}/odom</odom_topic>')
+            injections.append(f'      <odom_topic>/{ns}/odom_wheels</odom_topic>')
         if '<tf_topic>' not in body:
             injections.append('      <tf_topic>/tf</tf_topic>')
 
@@ -97,6 +104,20 @@ Generate a namespaced TurtleBot3 Burger SDF for robot *ns*.
     # Inject DiffDrive topics if missing
     content = _inject_diffdrive_topics(content, ns)
 
+    # Inject individual wheel configurations
+    if ns in WHEEL_CONFIGS:
+        cfg = WHEEL_CONFIGS[ns]
+        content = re.sub(
+            r'<wheel_radius>[^<]*</wheel_radius>',
+            f'<wheel_radius>{cfg["wheel_radius"]}</wheel_radius>',
+            content
+        )
+        content = re.sub(
+            r'<wheel_separation>[^<]*</wheel_separation>',
+            f'<wheel_separation>{cfg["wheel_separation"]}</wheel_separation>',
+            content
+        )
+
     # Apply all remaining topic / frame patches
     for pattern, replacement in _TOPIC_PATCHES:
         replacement_str = replacement.replace('{ns}', ns)
diff --git a/src/multi_tb3_system/multi_tb3_system/perception/laser_processor.py b/src/multi_tb3_system/multi_tb3_system/perception/laser_processor.py
index c103dcd..cbcd043 100644
--- a/src/multi_tb3_system/multi_tb3_system/perception/laser_processor.py
+++ b/src/multi_tb3_system/multi_tb3_system/perception/laser_processor.py
@@ -21,11 +21,15 @@ class Cluster:
     distance: float                      # Distance from robot origin [m]
     angle: float                         # Angle from robot heading [rad]
     size: int                            # Number of points in cluster
+    physical_width: float                # Euclidean distance between edge points [m]
+    confidence: float = 0.0              # Sensor fusion confidence [0.0 to 1.0]
 
     def __repr__(self) -> str:
         return (f"Cluster(dist={self.distance:.2f}m, "
                 f"angle={math.degrees(self.angle):.1f}°, "
-                f"size={self.size})")
+                f"size={self.size}, "
+                f"width={self.physical_width:.2f}m, "
+                f"conf={self.confidence:.2f})")
 
 
 # Core processing functions
@@ -56,14 +60,22 @@ Convert a LaserScan range array to a list of valid (x, y) Cartesian points
 def filter_front_sector(
     points: List[Tuple[float, float]],
     half_angle_deg: float = 30.0,
+    center_angle_deg: float = 0.0,
     min_x: float = 0.0,
 ) -> List[Tuple[float, float]]:
-    """Keep only points within ±half_angle_deg of straight ahead."""
+    """Keep only points within ±half_angle_deg of the given center_angle_deg."""
     half_angle_rad = math.radians(half_angle_deg)
-    return [
-        (x, y) for (x, y) in points
-        if x >= min_x and abs(math.atan2(y, x)) <= half_angle_rad
-    ]
+    center_angle_rad = math.radians(center_angle_deg)
+    filtered = []
+    for (x, y) in points:
+        if x < min_x:
+            continue
+        angle = math.atan2(y, x)
+        diff = angle - center_angle_rad
+        wrapped_diff = math.atan2(math.sin(diff), math.cos(diff))
+        if abs(wrapped_diff) <= half_angle_rad:
+            filtered.append((x, y))
+    return filtered
 
 
 def euclidean_cluster(
@@ -100,6 +112,8 @@ def make_clusters(
     raw_clusters: List[List[Tuple[float, float]]],
     min_cluster_size: int = 2,
     max_cluster_size: int = 40,
+    min_physical_width: float = 0.05,
+    max_physical_width: float = 0.30,
 ) -> List[Cluster]:
     """Convert raw point groups into Cluster objects, filtering noise and walls."""
     result: List[Cluster] = []
@@ -107,6 +121,13 @@ def make_clusters(
         n = len(pts)
         if n < min_cluster_size or n > max_cluster_size:
             continue
+            
+        # Calculate physical width
+        first, last = pts[0], pts[-1]
+        width = math.hypot(last[0] - first[0], last[1] - first[1])
+        if width < min_physical_width or width > max_physical_width:
+            continue
+
         cx, cy = compute_centroid(pts)
         result.append(Cluster(
             points=pts,
@@ -115,6 +136,7 @@ def make_clusters(
             distance=math.hypot(cx, cy),
             angle=math.atan2(cy, cx),
             size=n,
+            physical_width=width,
         ))
     return result
 
@@ -122,19 +144,39 @@ def make_clusters(
 def select_target_cluster(
     clusters: List[Cluster],
     last_target_pos: Optional[Tuple[float, float]] = None,
+    expected_local_pos: Optional[Tuple[float, float]] = None,
     lock_radius: float = 0.4,
 ) -> Optional[Cluster]:
     """
-Select the best candidate cluster to follow.
-"""
+    Select the best candidate cluster to follow and calculate confidence.
+    """
     if not clusters:
         return None
-    if last_target_pos is not None:
-        lx, ly = last_target_pos
-        best = min(clusters, key=lambda c: math.hypot(c.centroid_x - lx, c.centroid_y - ly))
-        if math.hypot(best.centroid_x - lx, best.centroid_y - ly) <= lock_radius:
+
+    # Determine reference position for selection
+    ref_x, ref_y = None, None
+    if expected_local_pos is not None:
+        ref_x, ref_y = expected_local_pos
+    elif last_target_pos is not None:
+        ref_x, ref_y = last_target_pos
+
+    if ref_x is not None and ref_y is not None:
+        best = min(clusters, key=lambda c: math.hypot(c.centroid_x - ref_x, c.centroid_y - ref_y))
+        dist_to_ref = math.hypot(best.centroid_x - ref_x, best.centroid_y - ref_y)
+        
+        if dist_to_ref <= lock_radius:
+            # Calculate confidence based on distance and expected width (0.14m)
+            dist_conf = max(0.0, 1.0 - (dist_to_ref / lock_radius))
+            width_error = abs(best.physical_width - 0.14)
+            width_conf = max(0.0, 1.0 - (width_error / 0.15))
+            
+            best.confidence = dist_conf * width_conf
             return best
-    return min(clusters, key=lambda c: c.distance)
+
+    # Fallback to closest cluster if no reference is valid or nothing in radius
+    best = min(clusters, key=lambda c: c.distance)
+    best.confidence = 0.0  # Zero confidence for fallback to guarantee rejection
+    return best
 
 
 def process_scan(
@@ -143,18 +185,26 @@ def process_scan(
     angle_increment: float,
     range_min: float = 0.12,
     range_max: float = 3.5,
-    front_half_angle_deg: float = 30.0,
+    front_half_angle_deg: float = 60.0,
+    center_angle_deg: float = 0.0,
     cluster_distance: float = 0.20,
     min_cluster_size: int = 2,
     max_cluster_size: int = 40,
     last_target_pos: Optional[Tuple[float, float]] = None,
+    expected_local_pos: Optional[Tuple[float, float]] = None,
+    lock_radius: float = 0.4,
 ) -> Tuple[Optional[Cluster], List[Cluster]]:
     """
-Full pipeline: raw LaserScan → (target_cluster, all_clusters).
-"""
+    Full pipeline: raw LaserScan → (target_cluster, all_clusters).
+    """
     points      = scan_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max)
-    front       = filter_front_sector(points, half_angle_deg=front_half_angle_deg)
+    front       = filter_front_sector(points, half_angle_deg=front_half_angle_deg, center_angle_deg=center_angle_deg)
     raw         = euclidean_cluster(front, cluster_distance=cluster_distance)
     clusters    = make_clusters(raw, min_cluster_size, max_cluster_size)
-    target      = select_target_cluster(clusters, last_target_pos=last_target_pos)
+    target      = select_target_cluster(
+        clusters, 
+        last_target_pos=last_target_pos, 
+        expected_local_pos=expected_local_pos,
+        lock_radius=lock_radius
+    )
     return target, clusters
diff --git a/src/multi_tb3_system/package.xml b/src/multi_tb3_system/package.xml
index 0426d03..f8e90d5 100644
--- a/src/multi_tb3_system/package.xml
+++ b/src/multi_tb3_system/package.xml
@@ -17,6 +17,7 @@
 
   <!-- ── Core ROS 2 Python ────────────────────────────────────────────────── -->
   <depend>rclpy</depend>
+  <depend>ament_index_python</depend>
 
   <!-- ── Message types ────────────────────────────────────────────────────── -->
   <depend>geometry_msgs</depend>   <!-- TwistStamped for cmd_vel          -->
@@ -29,6 +30,7 @@
   <!-- ── TF2 ──────────────────────────────────────────────────────────────── -->
   <depend>tf2</depend>
   <depend>tf2_ros</depend>
+  <depend>tf2_ros_py</depend>
   <depend>tf2_geometry_msgs</depend>
 
   <!-- ── Gazebo / ROS bridge ──────────────────────────────────────────────── -->
@@ -45,11 +47,13 @@
   <!-- ── Visualization ────────────────────────────────────────────────────── -->
   <depend>rviz2</depend>
 
-
+  <!-- ── External nodes ───────────────────────────────────────────────────── -->
+  <exec_depend>rf2o_laser_odometry</exec_depend>
 
   <!-- ── Testing ──────────────────────────────────────────────────────────── -->
   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>
+  <test_depend>ament_cmake_pytest</test_depend>
   <test_depend>python3-hypothesis</test_depend>
 
   <export>
diff --git a/src/multi_tb3_system/rviz/multi_robot.rviz b/src/multi_tb3_system/rviz/multi_robot.rviz
index 40547ed..13dd1c9 100644
--- a/src/multi_tb3_system/rviz/multi_robot.rviz
+++ b/src/multi_tb3_system/rviz/multi_robot.rviz
@@ -522,7 +522,7 @@ Visualization Manager:
   Enabled: true
   Global Options:
     Background Color: 48; 48; 48
-    Fixed Frame: world
+    Fixed Frame: map
     Frame Rate: 30
   Name: root
   Tools:
diff --git a/src/multi_tb3_system/scripts/convoy_publisher.py b/src/multi_tb3_system/scripts/convoy_publisher.py
index ff8a18f..e52935b 100644
--- a/src/multi_tb3_system/scripts/convoy_publisher.py
+++ b/src/multi_tb3_system/scripts/convoy_publisher.py
@@ -7,9 +7,9 @@ import math
 
 import rclpy
 from rclpy.node import Node
-from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
-from nav_msgs.msg import Odometry, Path
+from nav_msgs.msg import Path
 from geometry_msgs.msg import PoseStamped
+import tf2_ros
 
 
 class ConvoyPublisher(Node):
@@ -18,64 +18,85 @@ class ConvoyPublisher(Node):
 
         self.declare_parameter('max_path_poses', 5000)
         self.declare_parameter('path_resolution', 0.01)
-        self.declare_parameter('path_frame', 'world')
-        self.declare_parameter('spawn_offset_x', 0.0)
-        self.declare_parameter('spawn_offset_y', 0.0)
-
-        self.max_poses  = self.get_parameter('max_path_poses').value
-        self.resolution = self.get_parameter('path_resolution').value
-        self.frame      = self.get_parameter('path_frame').value
-        self.off_x      = self.get_parameter('spawn_offset_x').value
-        self.off_y      = self.get_parameter('spawn_offset_y').value
+        self.declare_parameter('path_frame', 'map')
+        self.declare_parameter('base_frame', 'base_footprint')
+        self.declare_parameter('yaw_resolution', 0.05)  # rad; gates breadcrumbs during rotation-in-place
+
+        self.max_poses      = self.get_parameter('max_path_poses').value
+        self.resolution     = self.get_parameter('path_resolution').value
+        self.frame          = self.get_parameter('path_frame').value
+        self.base_frame     = self.get_parameter('base_frame').value
+        self.yaw_resolution = self.get_parameter('yaw_resolution').value
+        
+        # Resolve full base frame (e.g. 'tb1/base_footprint')
+        ns = self.get_namespace().strip('/')
+        self.full_base_frame = f"{ns}/{self.base_frame}" if ns else self.base_frame
 
         self.path_pub = self.create_publisher(Path, 'convoy_path', 10)
 
-        _sensor_qos = QoSProfile(
-            depth=10,
-            reliability=ReliabilityPolicy.BEST_EFFORT,
-            durability=DurabilityPolicy.VOLATILE,
-        )
-        self.odom_sub = self.create_subscription(
-            Odometry, 'odom', self.odom_callback, _sensor_qos,
-        )
+        # TF2 setup
+        self.tf_buffer = tf2_ros.Buffer()
+        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
 
         self.path_msg = Path()
         self.path_msg.header.frame_id = self.frame
 
-        # Publish path at 50 Hz to match follower control loops
-        self.timer = self.create_timer(0.02, self.publish_path)
+        # Publish path and lookup TF at 50 Hz
+        self.timer = self.create_timer(0.02, self.timer_callback)
 
         self.get_logger().info(
-            f"ConvoyPublisher | frame={self.frame} | "
-            f"offset=({self.off_x:.2f},{self.off_y:.2f}) | "
+            f"ConvoyPublisher | map_frame={self.frame} | base_frame={self.full_base_frame} | "
             f"resolution={self.resolution}m"
         )
 
-    def odom_callback(self, msg: Odometry):
-        pose = PoseStamped()
-        pose.header = msg.header
-        pose.header.frame_id = self.frame
-        pose.pose = msg.pose.pose
-        # Shift leader odom into the shared world frame.
-        pose.pose.position.x += self.off_x
-        pose.pose.position.y += self.off_y
-
-        if not self.path_msg.poses:
-            self.path_msg.poses.append(pose)
-        else:
-            last_pose = self.path_msg.poses[-1]
-            dx = pose.pose.position.x - last_pose.pose.position.x
-            dy = pose.pose.position.y - last_pose.pose.position.y
-            if math.hypot(dx, dy) >= self.resolution:
+    @staticmethod
+    def _yaw_delta(q_prev, q_cur) -> float:
+        """Shortest-path yaw difference between two orientations (2D, z/w only)."""
+        def yaw(q):
+            return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
+        d = yaw(q_cur) - yaw(q_prev)
+        return math.atan2(math.sin(d), math.cos(d))
+
+    def timer_callback(self):
+        now = self.get_clock().now()
+        
+        # 1. Look up current global pose
+        try:
+            trans = self.tf_buffer.lookup_transform(
+                self.frame,
+                self.full_base_frame,
+                rclpy.time.Time()
+            )
+            
+            pose = PoseStamped()
+            pose.header.stamp = trans.header.stamp
+            pose.header.frame_id = self.frame
+            pose.pose.position.x = trans.transform.translation.x
+            pose.pose.position.y = trans.transform.translation.y
+            pose.pose.position.z = trans.transform.translation.z
+            pose.pose.orientation = trans.transform.rotation
+
+            # 2. Append to path if moved or turned enough
+            if not self.path_msg.poses:
                 self.path_msg.poses.append(pose)
-
-        # Keep path size bounded
-        if len(self.path_msg.poses) > self.max_poses:
-            self.path_msg.poses = self.path_msg.poses[-self.max_poses:]
-
-    def publish_path(self):
-        # Persistent-publication contract (R5.1, R5.2, R5.5, R5.6): the path is
-        self.path_msg.header.stamp = self.get_clock().now().to_msg()
+            else:
+                last_pose = self.path_msg.poses[-1]
+                dx = pose.pose.position.x - last_pose.pose.position.x
+                dy = pose.pose.position.y - last_pose.pose.position.y
+                dyaw = self._yaw_delta(last_pose.pose.orientation, pose.pose.orientation)
+                if math.hypot(dx, dy) >= self.resolution or abs(dyaw) >= self.yaw_resolution:
+                    self.path_msg.poses.append(pose)
+
+            # Keep path size bounded
+            if len(self.path_msg.poses) > self.max_poses:
+                self.path_msg.poses = self.path_msg.poses[-self.max_poses:]
+                
+        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
+            self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=2.0)
+            pass
+
+        # 3. Publish path
+        self.path_msg.header.stamp = now.to_msg()
         self.path_msg.header.frame_id = self.frame
         self.path_pub.publish(self.path_msg)
 
diff --git a/src/multi_tb3_system/scripts/costmap_generator.py b/src/multi_tb3_system/scripts/costmap_generator.py
index 177cdf2..50effed 100644
--- a/src/multi_tb3_system/scripts/costmap_generator.py
+++ b/src/multi_tb3_system/scripts/costmap_generator.py
@@ -149,9 +149,11 @@ No-ops when ``enable_costmap_viz`` is false (headless mode) to avoid
         msg = OccupancyGrid()
         msg.header.stamp = self.get_clock().now().to_msg()
         # Prefer the explicit ``costmap_frame`` parameter; fall back to
+        ns = self.get_namespace().strip('/')
+        fallback_frame = f"{ns}/base_scan" if ns else 'base_scan'
         msg.header.frame_id = (
             self.costmap_frame
-            or (self._scan.header.frame_id if self._scan is not None else 'base_scan')
+            or (self._scan.header.frame_id if self._scan is not None else fallback_frame)
         )
         msg.info.resolution = float(cm.resolution)
         msg.info.width = int(cm.width)
diff --git a/src/multi_tb3_system/scripts/follower_node.py b/src/multi_tb3_system/scripts/follower_node.py
index d251a71..ce0145f 100644
--- a/src/multi_tb3_system/scripts/follower_node.py
+++ b/src/multi_tb3_system/scripts/follower_node.py
@@ -22,6 +22,7 @@ if _scripts_dir not in sys.path:
 from motion_controller import PursuitController, yaw_from_quaternion, slew
 from safety_controller import SafetyController
 from convoy_tracking import is_newer_breadcrumb
+from follower_state import FollowerState
 
 
 class FollowerNode(Node):
@@ -36,7 +37,7 @@ class FollowerNode(Node):
         self.declare_parameter('kp_angular', 1.5)
         self.declare_parameter('max_linear_velocity', 0.22)
         self.declare_parameter('max_angular_velocity', 1.0)
-        self.declare_parameter('safe_distance', 0.15)  # must be strictly < convoy_spacing (0.5 m) so the robot ahead at the nominal gap does not trigger an emergency stop
+        self.declare_parameter('safe_distance', 0.20)  # Emergency stop distance - must be < convoy_spacing (0.6m) to allow normal following
         self.declare_parameter('predecessor_gap', 0.6)
         self.declare_parameter('control_frequency', 20.0)  # Hz — must match follower_params.yaml
         self.declare_parameter('max_linear_accel', 1.0)
@@ -54,6 +55,7 @@ class FollowerNode(Node):
         self.declare_parameter('emergency_recovery_timeout', 0.5)
         self.declare_parameter('search_angular_velocity', 0.6)
         self.declare_parameter('breadcrumb_timeout', 10.0)
+        self.declare_parameter('enable_local_planner', True)
 
         gp = lambda n: self.get_parameter(n).value
 
@@ -106,7 +108,9 @@ class FollowerNode(Node):
             max_lin=self.max_lin,
             max_ang=self.max_ang,
             safety=safety,
+            enable_local_planner=bool(gp('enable_local_planner')),
         )
+        self._logged_state = None
 
         # Message caches (written by callbacks, read by control loop)
         self._pose: Optional[Tuple[float, float, float]] = None
@@ -125,6 +129,20 @@ class FollowerNode(Node):
         self._last_lin: float = 0.0
         self._last_ang: float = 0.0
         self._dt = 1.0 / float(self.control_frequency)
+        self._accum_x: float = 0.0
+        self._accum_y: float = 0.0
+
+        # TF2 Setup
+        import tf2_ros
+        self.tf_buffer = tf2_ros.Buffer()
+        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
+        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
+
+        # Resolve frames
+        ns = self.get_namespace().strip('/')
+        self.odom_frame = f"{ns}/odom" if ns else "odom"
+        self.base_frame = f"{ns}/base_footprint" if ns else "base_footprint"
+        self.map_frame = "map"
 
         # ROS wiring
         qos = QoSProfile(depth=10,
@@ -132,17 +150,15 @@ class FollowerNode(Node):
                          durability=DurabilityPolicy.VOLATILE)
 
         self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
-        self.create_subscription(Odometry,   'odom',
-                                 self._odom_cb, qos)
         self.create_subscription(Path,       'convoy_path',
                                  self._path_cb, 10)
         self.create_subscription(LaserScan,  'scan',
                                  self._scan_cb, qos)
         self.create_timer(self._dt, self._control_loop)
 
-        gap = (slot - 1) * float(gp('convoy_spacing'))
+        gap = float(gp('convoy_spacing'))
         self.get_logger().info(
-            f"Path-follower tb{slot} | gap={gap:.2f}m | "
+            f"Daisy-chain follower tb{slot} | gap={gap:.2f}m | "
             f"lookahead={gp('lookahead_distance'):.2f}m | "
             f"control={self.control_frequency:.0f}Hz"
         )
@@ -155,15 +171,6 @@ class FollowerNode(Node):
 
     # Callbacks: cache only
 
-    def _odom_cb(self, msg: Odometry) -> None:
-        p = msg.pose.pose.position
-        q = msg.pose.pose.orientation
-        self._pose = (
-            p.x + self.off_x,
-            p.y + self.off_y,
-            yaw_from_quaternion(q),
-        )
-
     def _path_cb(self, msg: Path) -> None:
         # Guard: ignore empty Path messages published during startup before the
         if not msg.poses:
@@ -185,6 +192,15 @@ class FollowerNode(Node):
         self._prev_count           = new_count
         self._prev_newest_stamp_ns = new_stamp_ns
 
+    def _odom_cb(self, msg: Odometry) -> None:
+        p = msg.pose.pose.position
+        q = msg.pose.pose.orientation
+        self._pose = (
+            p.x,
+            p.y,
+            yaw_from_quaternion(q),
+        )
+
     def _scan_cb(self, msg: LaserScan) -> None:
         self._scan         = msg
         self._scan_time_ns = self.get_clock().now().nanoseconds
@@ -192,6 +208,25 @@ class FollowerNode(Node):
     # Control loop
 
     def _control_loop(self) -> None:
+        now = self.get_clock().now()
+        now_ns = now.nanoseconds
+
+        # 1. Look up map -> base_footprint TF
+        try:
+            trans = self.tf_buffer.lookup_transform(
+                self.map_frame,
+                self.base_frame,
+                rclpy.time.Time()
+            )
+            self._pose = (
+                trans.transform.translation.x,
+                trans.transform.translation.y,
+                yaw_from_quaternion(trans.transform.rotation)
+            )
+        except Exception as e:
+            self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=2.0)
+            self._pose = None
+
         if self._pose is None or len(self._path) < 2:
             # Nothing to track yet — publish zero.
             if not self._has_ever_tracked:
@@ -209,14 +244,26 @@ class FollowerNode(Node):
             if self._scan_time_ns is not None else float('inf')
         )
 
-        linear_x, angular_z, _ = self._controller.step(
+        linear_x, angular_z, self._accum_x, self._accum_y = self._controller.step(
             pose=self._pose,
             path=self._path,
             scan=scan,
             scan_age_s=scan_age,
             now_ns=now_ns,
+            current_accum_x=self._accum_x,
+            current_accum_y=self._accum_y
         )
 
+        cur_state = self._controller.last_state
+        if cur_state != self._logged_state:
+            if cur_state == FollowerState.DETOUR:
+                leader_pos = self._path[-1] if self._path else None
+                self.get_logger().info(
+                    f"DETOUR triggered | pose={self._pose} | leader_last_breadcrumb={leader_pos}"
+                )
+            self.get_logger().info(f"State: {self._logged_state} -> {cur_state}")
+            self._logged_state = cur_state
+
         self._publish_smoothed(linear_x, angular_z)
 
         # Stationary timer uses the post-slew actual command (self._last_lin).
diff --git a/src/multi_tb3_system/scripts/local_planner.py b/src/multi_tb3_system/scripts/local_planner.py
new file mode 100644
index 0000000..6056706
--- /dev/null
+++ b/src/multi_tb3_system/scripts/local_planner.py
@@ -0,0 +1,306 @@
+#!/usr/bin/env python3
+"""
+Lightweight local planner for obstacle avoidance.
+Similar to DWA (Dynamic Window Approach) but simplified for convoy following.
+"""
+
+from __future__ import annotations
+
+import math
+from typing import List, Tuple
+from dataclasses import dataclass
+
+from costmap_utils import Costmap
+
+
+@dataclass
+class VelocityCandidate:
+    """A candidate velocity command with its evaluation score."""
+    linear: float
+    angular: float
+    score: float
+    collision: bool
+
+
+class LocalPlanner:
+    """
+    Lightweight local planner that generates collision-free velocities
+    toward a goal point using costmap-based obstacle avoidance.
+    
+    Similar to DWA (Dynamic Window Approach) but simplified for convoy use.
+    """
+    
+    def __init__(
+        self,
+        max_linear_vel: float = 0.22,
+        max_angular_vel: float = 1.0,
+        max_linear_acc: float = 2.0,
+        max_angular_acc: float = 3.0,
+        velocity_samples: int = 10,
+        angular_samples: int = 15,
+        predict_time: float = 1.5,
+        robot_radius: float = 0.25,
+        goal_weight: float = 1.0,
+        velocity_weight: float = 0.2,
+        obstacle_weight: float = 2.0,
+        control_period: float = 0.05,
+    ):
+        """
+        Initialize the local planner.
+        
+        Args:
+            max_linear_vel: Maximum linear velocity [m/s]
+            max_angular_vel: Maximum angular velocity [rad/s]
+            max_linear_acc: Maximum linear acceleration [m/s²]
+            max_angular_acc: Maximum angular acceleration [rad/s²]
+            velocity_samples: Number of linear velocity samples
+            angular_samples: Number of angular velocity samples
+            predict_time: Trajectory prediction horizon [s]
+            robot_radius: Robot radius for collision checking [m]
+            goal_weight: Weight for goal distance in scoring
+            velocity_weight: Weight for velocity preference (favor higher speeds)
+            obstacle_weight: Weight for obstacle clearance
+            control_period: Control loop period [s]
+        """
+        self.max_linear_vel = max_linear_vel
+        self.max_angular_vel = max_angular_vel
+        self.max_linear_acc = max_linear_acc
+        self.max_angular_acc = max_angular_acc
+        self.velocity_samples = velocity_samples
+        self.angular_samples = angular_samples
+        self.predict_time = predict_time
+        self.robot_radius = robot_radius
+        self.goal_weight = goal_weight
+        self.velocity_weight = velocity_weight
+        self.obstacle_weight = obstacle_weight
+        self.control_period = control_period
+        
+        # Current velocity for dynamic window calculation
+        self.current_linear = 0.0
+        self.current_angular = 0.0
+    
+    def update_current_velocity(self, linear: float, angular: float):
+        """Update the current velocity for dynamic window calculation."""
+        self.current_linear = linear
+        self.current_angular = angular
+    
+    def compute_velocity(
+        self,
+        goal_x: float,
+        goal_y: float,
+        costmap: Costmap,
+        current_yaw: float = 0.0,
+    ) -> Tuple[float, float]:
+        """
+        Compute collision-free velocity toward goal using costmap.
+        
+        Args:
+            goal_x: Goal x position in robot frame [m]
+            goal_y: Goal y position in robot frame [m]
+            costmap: Occupancy costmap for obstacle detection
+            current_yaw: Current robot yaw (for trajectory prediction) [rad]
+            
+        Returns:
+            (linear_vel, angular_vel): Best velocity command
+        """
+        # Generate velocity candidates within dynamic window
+        candidates = self._generate_candidates()
+        
+        # Evaluate each candidate
+        for candidate in candidates:
+            # Predict trajectory
+            trajectory = self._predict_trajectory(
+                candidate.linear, candidate.angular, current_yaw
+            )
+            
+            # Check collision
+            candidate.collision = self._check_collision(trajectory, costmap)
+            
+            # Compute score (only for collision-free trajectories)
+            if not candidate.collision:
+                candidate.score = self._evaluate_trajectory(
+                    candidate.linear, candidate.angular,
+                    trajectory, goal_x, goal_y, costmap
+                )
+            else:
+                candidate.score = -1e6  # Very negative score for collision
+        
+        # Select best candidate
+        best = max(candidates, key=lambda c: c.score)
+        
+        # If all candidates collide, stop
+        if best.collision:
+            return 0.0, 0.0
+        
+        return best.linear, best.angular
+    
+    def _generate_candidates(self) -> List[VelocityCandidate]:
+        """Generate velocity candidates within dynamic window."""
+        candidates = []
+        
+        # Dynamic window: reachable velocities from current velocity
+        # Allow backward motion for escape
+        min_linear = max(
+            0.0,  # Do not allow backward motion
+            self.current_linear - self.max_linear_acc * self.control_period
+        )
+        max_linear = min(
+            self.max_linear_vel,
+            self.current_linear + self.max_linear_acc * self.control_period
+        )
+        
+        min_angular = max(
+            -self.max_angular_vel,
+            self.current_angular - self.max_angular_acc * self.control_period
+        )
+        max_angular = min(
+            self.max_angular_vel,
+            self.current_angular + self.max_angular_acc * self.control_period
+        )
+        
+        # Sample velocities
+        if max_linear > min_linear:
+            linear_step = (max_linear - min_linear) / max(1, self.velocity_samples - 1)
+        else:
+            linear_step = 0.0
+        
+        if max_angular > min_angular:
+            angular_step = (max_angular - min_angular) / max(1, self.angular_samples - 1)
+        else:
+            angular_step = 0.0
+        
+        for i in range(self.velocity_samples):
+            linear = min_linear + i * linear_step
+            for j in range(self.angular_samples):
+                angular = min_angular + j * angular_step
+                candidates.append(VelocityCandidate(linear, angular, 0.0, False))
+        
+        return candidates
+    
+    def _predict_trajectory(
+        self,
+        linear: float,
+        angular: float,
+        current_yaw: float,
+    ) -> List[Tuple[float, float]]:
+        """
+        Predict trajectory for given velocities.
+        
+        Returns list of (x, y) positions in robot frame.
+        """
+        trajectory = []
+        dt = 0.1  # Prediction time step
+        steps = int(self.predict_time / dt)
+        
+        x, y, yaw = 0.0, 0.0, 0.0  # Start from robot origin (robot frame)
+        
+        for _ in range(steps):
+            # Simple motion model: constant velocity
+            x += linear * math.cos(yaw) * dt
+            y += linear * math.sin(yaw) * dt
+            yaw += angular * dt
+            trajectory.append((x, y))
+        
+        return trajectory
+    
+    def _check_collision(
+        self,
+        trajectory: List[Tuple[float, float]],
+        costmap: Costmap,
+    ) -> bool:
+        """Check if trajectory collides with obstacles in costmap."""
+        for x, y in trajectory:
+            # Check circle around predicted position
+            if self._is_occupied(x, y, costmap):
+                return True
+        return False
+    
+    def _is_occupied(self, x: float, y: float, costmap: Costmap) -> bool:
+        """Check if position (with robot radius) is occupied in costmap."""
+        # Convert to costmap coordinates
+        # Convert to costmap coordinates
+        cx = int((x - costmap.origin_x) / costmap.resolution)
+        cy = int((y - costmap.origin_y) / costmap.resolution)
+        
+        # Check cells within robot radius
+        radius_cells = int(self.robot_radius / costmap.resolution) + 1
+        
+        for dx in range(-radius_cells, radius_cells + 1):
+            for dy in range(-radius_cells, radius_cells + 1):
+                if dx*dx + dy*dy > radius_cells*radius_cells:
+                    continue
+                
+                check_x = cx + dx
+                check_y = cy + dy
+                
+                # Bounds check
+                if (0 <= check_x < costmap.width and 
+                    0 <= check_y < costmap.height):
+                    if costmap.data[check_y * costmap.width + check_x] > 50:
+                        return True
+        
+        return False
+    
+    def _evaluate_trajectory(
+        self,
+        linear: float,
+        angular: float,
+        trajectory: List[Tuple[float, float]],
+        goal_x: float,
+        goal_y: float,
+        costmap: Costmap,
+    ) -> float:
+        """
+        Evaluate trajectory quality.
+        
+        Higher score = better trajectory.
+        """
+        # 1. Goal distance: how close does trajectory end get to goal?
+        end_x, end_y = trajectory[-1] if trajectory else (0.0, 0.0)
+        dist_to_goal = math.hypot(goal_x - end_x, goal_y - end_y)
+        goal_score = -dist_to_goal  # Negative because closer is better
+        
+        # 2. Velocity preference: favor higher speeds (more progress)
+        velocity_score = linear
+        
+        # 3. Obstacle clearance: favor trajectories far from obstacles
+        min_clearance = float('inf')
+        for x, y in trajectory:
+            clearance = self._get_clearance(x, y, costmap)
+            min_clearance = min(min_clearance, clearance)
+        obstacle_score = min_clearance
+        
+        # Combined score
+        total_score = (
+            self.goal_weight * goal_score +
+            self.velocity_weight * velocity_score +
+            self.obstacle_weight * obstacle_score
+        )
+        
+        return total_score
+    
+    def _get_clearance(self, x: float, y: float, costmap: Costmap) -> float:
+        """Get minimum distance to nearest obstacle from position."""
+        cx = int((x - costmap.origin_x) / costmap.resolution)
+        cy = int((y - costmap.origin_y) / costmap.resolution)
+        
+        # Bounds check
+        if not (0 <= cx < costmap.width and 0 <= cy < costmap.height):
+            return 0.0
+        
+        # Search for nearest obstacle within reasonable radius
+        search_radius = 20  # cells
+        min_dist = float('inf')
+        
+        for dx in range(-search_radius, search_radius + 1):
+            for dy in range(-search_radius, search_radius + 1):
+                check_x = cx + dx
+                check_y = cy + dy
+                
+                if (0 <= check_x < costmap.width and 
+                    0 <= check_y < costmap.height):
+                    if costmap.data[check_y * costmap.width + check_x] > 50:
+                        dist = math.hypot(dx, dy) * costmap.resolution
+                        min_dist = min(min_dist, dist)
+        
+        return min_dist if min_dist != float('inf') else 10.0  # Max clearance
diff --git a/src/multi_tb3_system/scripts/motion_controller.py b/src/multi_tb3_system/scripts/motion_controller.py
index a796373..31e298c 100644
--- a/src/multi_tb3_system/scripts/motion_controller.py
+++ b/src/multi_tb3_system/scripts/motion_controller.py
@@ -25,6 +25,8 @@ from follower_state import (
     build_search_command,
 )
 from safety_controller import SafetyController
+from local_planner import LocalPlanner
+from multi_tb3_system.perception.laser_processor import process_scan
 
 
 # Geometry helpers
@@ -83,8 +85,14 @@ Stateful Pure Pursuit + state-machine convoy follower.
         max_lin: float,
         max_ang: float,
         safety: SafetyController,
+        enable_local_planner: bool = True,
     ) -> None:
-        self._gap                        = (convoy_slot - 1) * convoy_spacing
+        self._slot = convoy_slot
+        
+        # Daisy-chain tracking: Every follower tracks its direct predecessor's path.
+        # Therefore, the gap is ALWAYS exactly `convoy_spacing` from the end of the path.
+        self._gap = convoy_spacing
+
         self.lookahead_distance          = lookahead_distance
         self.kp_linear                   = kp_linear
         self.kp_angular                  = kp_angular
@@ -99,6 +107,25 @@ Stateful Pure Pursuit + state-machine convoy follower.
         self.max_lin                     = max_lin
         self.max_ang                     = max_ang
         self.safety                      = safety
+        self.enable_local_planner        = enable_local_planner
+
+        # Initialize local planner if enabled
+        self.local_planner: Optional[LocalPlanner] = None
+        if self.enable_local_planner:
+            self.local_planner = LocalPlanner(
+                max_linear_vel=max_lin,
+                max_angular_vel=max_ang,
+                max_linear_acc=2.0,
+                max_angular_acc=3.0,
+                velocity_samples=10,  # More samples for better paths
+                angular_samples=15,   # More angular samples for narrow gaps
+                predict_time=1.5,
+                robot_radius=0.25,
+                goal_weight=1.0,
+                velocity_weight=0.3,  # Favor moving forward
+                obstacle_weight=2.5,  # Strong obstacle avoidance
+                control_period=0.05,
+            )
 
         # Per-cycle mutable state
         self._last_closest_idx:  int            = 0
@@ -106,6 +133,7 @@ Stateful Pure Pursuit + state-machine convoy follower.
         self._emergency_since:   Optional[int]  = None   # wall-clock ns
         self._prev_emergency:    bool            = False
         self._state_for_timer_reset: FollowerState = FollowerState.TRACKING
+        self.last_state: FollowerState = FollowerState.TRACKING
         self._last_newest_time_ns: int          = 0      # ns since epoch
 
     # Breadcrumb-freshness update (called from _path_cb)
@@ -123,11 +151,51 @@ Stateful Pure Pursuit + state-machine convoy follower.
         scan,                               # sensor_msgs/LaserScan or None
         scan_age_s: float,                  # seconds since scan was received
         now_ns: int,                        # current wall-clock time [ns]
-    ) -> Tuple[float, float, bool]:
+        current_accum_x: float = 0.0,       # current map->odom TF x offset
+        current_accum_y: float = 0.0,       # current map->odom TF y offset
+    ) -> Tuple[float, float, float, float]:
+        """
+        Compute one control cycle.
+        Returns: (linear_x, angular_z, new_accum_x, new_accum_y)
         """
-Compute one control cycle.
-"""
         rx, ry, ryaw = pose
+        new_accum_x = current_accum_x
+        new_accum_y = current_accum_y
+
+        # 1. LiDAR Target Tracking
+        if scan is not None and len(path) > 0 and scan_age_s < 0.25:
+            expected_local_x, expected_local_y = to_robot_frame(path[-1][0], path[-1][1], rx, ry, ryaw)
+            rmin = scan.range_min if scan.range_min > 0 else 0.12
+            expected_bearing_deg = math.degrees(math.atan2(expected_local_y, expected_local_x))
+            
+            target_cluster, _ = process_scan(
+                ranges=list(scan.ranges),
+                angle_min=scan.angle_min,
+                angle_increment=scan.angle_increment,
+                range_min=rmin,
+                front_half_angle_deg=45.0,
+                center_angle_deg=expected_bearing_deg,
+                expected_local_pos=(expected_local_x, expected_local_y),
+                lock_radius=0.5
+            )
+
+            if target_cluster is not None and target_cluster.confidence > 0.5:
+                # Local physical error
+                err_x_local = target_cluster.centroid_x - expected_local_x
+                err_y_local = target_cluster.centroid_y - expected_local_y
+                
+                # Convert to global error (R_odom - R_true)
+                err_x_global = err_x_local * math.cos(ryaw) - err_y_local * math.sin(ryaw)
+                err_y_global = err_x_local * math.sin(ryaw) + err_y_local * math.cos(ryaw)
+                
+                # EMA filter to prevent violent swerves
+                alpha_ema = 0.1
+                new_accum_x = (1.0 - alpha_ema) * current_accum_x + alpha_ema * err_x_global
+                new_accum_y = (1.0 - alpha_ema) * current_accum_y + alpha_ema * err_y_global
+
+        # Shift the robot's perceived pose by the accumulated offset
+        rx -= new_accum_x
+        ry -= new_accum_y
 
         # Costmap
         scan_stale = scan is None or scan_age_s > self.costmap_stale_timeout
@@ -139,9 +207,24 @@ Compute one control cycle.
             scan_for_safety = None
         else:
             rmin = scan.range_min if scan.range_min > 0 else 0.12
+
+            # Compute predecessor bearing if we have a path
+            expected_bearing_deg = 0.0
+            if len(path) > 0:
+                pred_x, pred_y = path[-1]
+                dx = pred_x - rx
+                dy = pred_y - ry
+                local_dx = dx * math.cos(-ryaw) - dy * math.sin(-ryaw)
+                local_dy = dx * math.sin(-ryaw) + dy * math.cos(-ryaw)
+                expected_bearing_deg = math.degrees(math.atan2(local_dy, local_dx))
+
             filtered_ranges = self.safety.filter_predecessor_returns(
-                list(scan.ranges), scan.angle_min, scan.angle_increment,
+                ranges=list(scan.ranges),
+                angle_min=scan.angle_min,
+                angle_increment=scan.angle_increment,
+                expected_bearing_deg=expected_bearing_deg
             )
+
             cm = build_costmap(
                 filtered_ranges, scan.angle_min, scan.angle_increment,
                 rmin, scan.range_max,
@@ -162,12 +245,12 @@ Compute one control cycle.
 
         # Path-length guard (P4)
         path_too_short = (
-            goal_idx == 0 and len(path) > 1
-            and math.hypot(path[-1][0] - path[0][0],
-                           path[-1][1] - path[0][1]) < self._gap * 0.5
+            len(path) == 1 or
+            (goal_idx == 0 and math.hypot(path[-1][0] - path[0][0],
+                                          path[-1][1] - path[0][1]) < self._gap * 0.95)
         )
         if path_too_short:
-            return 0.0, 0.0, False
+            return 0.0, 0.0, new_accum_x, new_accum_y
 
         # Pure Pursuit
         search_start = max(0, self._last_closest_idx - 20)
@@ -275,18 +358,31 @@ Compute one control cycle.
                 self.search_angular_velocity, self.max_ang,
             )
         elif state == FollowerState.DETOUR:
-            base_linear, base_angular = build_detour_command(
-                cm, pursuit_linear,
-                self.max_lin, self.max_ang,
-                self.detour_forward_min_vel,
-            )
-            # Bias detour angular toward the goal when goal is more than 45°
-            goal_bearing = math.atan2(gy_local, gx_local)
-            if abs(goal_bearing) > math.radians(45):
-                goal_sign   = 1.0 if goal_bearing > 0 else -1.0
-                detour_sign = 1.0 if base_angular  > 0 else -1.0
-                if goal_sign != detour_sign:
-                    base_angular = -base_angular
+            # Use local planner if enabled, otherwise use simple detour
+            if self.enable_local_planner and self.local_planner is not None:
+                # Update local planner's current velocity for dynamic window
+                self.local_planner.update_current_velocity(
+                    self._pending_linear_for_stationary if hasattr(self, '_pending_linear_for_stationary') else 0.0,
+                    0.0
+                )
+                # Compute velocity toward goal using local planner
+                base_linear, base_angular = self.local_planner.compute_velocity(
+                    gx_local, gy_local, cm, ryaw
+                )
+            else:
+                # Simple detour: biased turning
+                base_linear, base_angular = build_detour_command(
+                    cm, pursuit_linear,
+                    self.max_lin, self.max_ang,
+                    self.detour_forward_min_vel,
+                )
+                # Bias detour angular toward the goal when goal is more than 45°
+                goal_bearing = math.atan2(gy_local, gx_local)
+                if abs(goal_bearing) > math.radians(45):
+                    goal_sign   = 1.0 if goal_bearing > 0 else -1.0
+                    detour_sign = 1.0 if base_angular  > 0 else -1.0
+                    if goal_sign != detour_sign:
+                        base_angular = -base_angular
         else:  # TRACKING
             base_linear, base_angular = pursuit_linear, pursuit_angular
 
@@ -317,7 +413,8 @@ Compute one control cycle.
         self._pending_linear_for_stationary = linear_x
 
         self._state_for_timer_reset = state
-        return linear_x, angular_z, safety_emergency_now
+        self.last_state = state
+        return float(linear_x), float(angular_z), new_accum_x, new_accum_y
 
     def update_stationary_timer(self, actual_linear: float, now_ns: int) -> None:
         """Call this AFTER slew-limiting so the timer reflects the wire command."""
diff --git a/src/multi_tb3_system/scripts/safety_controller.py b/src/multi_tb3_system/scripts/safety_controller.py
index 5f9a897..a667315 100644
--- a/src/multi_tb3_system/scripts/safety_controller.py
+++ b/src/multi_tb3_system/scripts/safety_controller.py
@@ -45,7 +45,8 @@ Initialize the safety controller.
         range_min: float,
     ) -> Tuple[float, float, bool]:
         """
-Returns
+Returns (min_left, min_right, is_emergency).
+Emergency detection now sees ALL obstacles including the leader - no predecessor filtering.
 """
         emergency_half = math.radians(EMERGENCY_HALF_ANGLE_DEG)
         steer_half     = math.radians(STEER_HALF_ANGLE_DEG)
@@ -60,20 +61,21 @@ Returns
             angle = angle_min + i * angle_increment
             angle = math.atan2(math.sin(angle), math.cos(angle))  # wrap to (-π, π]
 
-            PREDECESSOR_HALF_ANGLE = math.radians(15.0)
-            if (self.predecessor_gap > 0.0
-                    and abs(angle) <= PREDECESSOR_HALF_ANGLE):
-                lo = self.predecessor_gap * 0.3
-                hi = self.predecessor_gap * 1.2
-                if lo <= r <= hi:
-                    continue
-
-            # Emergency cone (±45°)
+            # Emergency cone (±45°) - NO FILTERING - detects all obstacles including leader
             if abs(angle) <= emergency_half and r < self.safe_distance:
                 is_emerg = True
 
-            # Steering cone (±60°)
+            # Steering cone (±60°) - still uses predecessor filter for gentle bias
             if abs(angle) <= steer_half and r < STEER_INFLUENCE_RANGE:
+                # Filter out predecessor for steering bias only
+                PREDECESSOR_HALF_ANGLE = math.radians(15.0)
+                if (self.predecessor_gap > 0.0
+                        and abs(angle) <= PREDECESSOR_HALF_ANGLE):
+                    lo = self.predecessor_gap * 0.3
+                    hi = self.predecessor_gap * 1.2
+                    if lo <= r <= hi:
+                        continue
+                
                 if angle >= 0:
                     min_left  = min(min_left,  r)
                 else:
@@ -170,6 +172,7 @@ Single-pass safety check — preferred in tight control loops.
         ranges: list,
         angle_min: float,
         angle_increment: float,
+        expected_bearing_deg: float = 0.0,
     ) -> list:
         """Return a copy of ranges with predecessor returns set to inf."""
         if self.predecessor_gap <= 0.0:
@@ -180,8 +183,9 @@ Single-pass safety check — preferred in tight control loops.
         filtered = list(ranges)
         for i, r in enumerate(filtered):
             angle = angle_min + i * angle_increment
-            angle = math.atan2(math.sin(angle), math.cos(angle))
-            if abs(angle) <= PREDECESSOR_HALF_ANGLE and lo <= r <= hi:
+            center_rad = math.radians(expected_bearing_deg)
+            ang_diff = math.atan2(math.sin(angle - center_rad), math.cos(angle - center_rad))
+            if abs(ang_diff) <= PREDECESSOR_HALF_ANGLE and lo <= r <= hi:
                 filtered[i] = float('inf')
         return filtered
 
diff --git a/src/multi_tb3_system/scripts/teleop_controller.py b/src/multi_tb3_system/scripts/teleop_controller.py
index 1783440..4f13a01 100644
--- a/src/multi_tb3_system/scripts/teleop_controller.py
+++ b/src/multi_tb3_system/scripts/teleop_controller.py
@@ -34,6 +34,19 @@ SPEED_BINDINGS = {
 
 MSG = """
 ╔════════════════════════════════════════════════════╗
+║             TB3 CONVOY TELEOP CONTROL              ║
+╠════════════════════════════════════════════════════╣
+║  Movement (Hold to move):                          ║
+║        W                      I                    ║
+║      A S D                  J K L                  ║
+║        X                      ,                    ║
+║                                                    ║
+║  Speed Control:                                    ║
+║    Q / Z : Increase/Decrease Max Speeds by 10%     ║
+║    E / C : Increase/Decrease Angular Speed by 10%  ║
+║                                                    ║
+║  CTRL-C to quit                                    ║
+╚════════════════════════════════════════════════════╝
 """
 
 SPEED_MSG = "\rLinear: {lin:.2f} m/s  |  Angular: {ang:.2f} rad/s    "
diff --git a/src/multi_tb3_system/test/test_laser_processor.py b/src/multi_tb3_system/test/test_laser_processor.py
index 92e2b64..52640be 100644
--- a/src/multi_tb3_system/test/test_laser_processor.py
+++ b/src/multi_tb3_system/test/test_laser_processor.py
@@ -43,7 +43,8 @@ class TestLaserProcessor(unittest.TestCase):
         self.assertIsNotNone(target)
         
         # Centroid of the points x1, y1, x2, y2, x3, y3
-        self.assertIsInstance(target.centroid, tuple)
+        self.assertIsInstance(target.centroid_x, float)
+        self.assertIsInstance(target.centroid_y, float)
 
 if __name__ == '__main__':
     unittest.main()
diff --git a/src/multi_tb3_system/worlds/pillars.world b/src/multi_tb3_system/worlds/pillars.world
index bd19197..18124d6 100644
--- a/src/multi_tb3_system/worlds/pillars.world
+++ b/src/multi_tb3_system/worlds/pillars.world
@@ -169,5 +169,39 @@
       </link>
     </model>
 
+    <!-- ── Enclosure Walls (To provide dense features for LiDAR scan-matching) ── -->
+    <model name="wall_north">
+      <static>true</static>
+      <pose>4.0 4.0 0.5 0 0 0</pose>
+      <link name="link">
+        <collision name="collision"><geometry><box><size>10.0 0.1 1.0</size></box></geometry></collision>
+        <visual name="visual"><geometry><box><size>10.0 0.1 1.0</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
+      </link>
+    </model>
+    <model name="wall_south">
+      <static>true</static>
+      <pose>4.0 -4.0 0.5 0 0 0</pose>
+      <link name="link">
+        <collision name="collision"><geometry><box><size>10.0 0.1 1.0</size></box></geometry></collision>
+        <visual name="visual"><geometry><box><size>10.0 0.1 1.0</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
+      </link>
+    </model>
+    <model name="wall_east">
+      <static>true</static>
+      <pose>9.0 0 0.5 0 0 0</pose>
+      <link name="link">
+        <collision name="collision"><geometry><box><size>0.1 8.0 1.0</size></box></geometry></collision>
+        <visual name="visual"><geometry><box><size>0.1 8.0 1.0</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
+      </link>
+    </model>
+    <model name="wall_west">
+      <static>true</static>
+      <pose>-1.0 0 0.5 0 0 0</pose>
+      <link name="link">
+        <collision name="collision"><geometry><box><size>0.1 8.0 1.0</size></box></geometry></collision>
+        <visual name="visual"><geometry><box><size>0.1 8.0 1.0</size></box></geometry><material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
+      </link>
+    </model>
+
   </world>
 </sdf>

`
