#!/usr/bin/env python3
"""
follower_node.py
================
Path-Based Convoy follower using Pure Pursuit with a costmap-driven
obstacle-avoidance state machine layered on top.

Architecture
------------
The leader (tb1) publishes its travelled trajectory as a nav_msgs/Path on
``/<leader_ns>/convoy_path`` (frame = ``world``). Every follower subscribes to
that single shared path and tracks it with a Pure Pursuit controller while
holding a configurable gap behind the robot ahead.

  * Lateral control : Pure Pursuit (lookahead point on the path).
  * Longitudinal     : proportional to remaining arc-length to the goal point,
                       so the convoy spacing is self-regulating.
  * LiDAR            : used by SafetyController for emergency stop / steer
                       bias AND by the in-process costmap builder for the
                       follower's TRACKING / DETOUR / EMERGENCY_STOP / SEARCH /
                       HOLD state machine.

Frames
------
Each robot's ``odom`` is anchored to ``world`` by a static transform at its
spawn position, with zero rotation. So the robot's world pose is simply
``odom + spawn_offset``. The spawn offset is supplied via parameters
(spawn_offset_x / spawn_offset_y) by the launch file.

Control runs on a fixed-rate timer (decoupled from the slow LiDAR), with
slew-rate limiting for smooth, continuous motion.

State machine (per cycle, command precedence)
---------------------------------------------
  1. Conditional-stop HOLD   (R7.2)  - elapsed > breadcrumb_timeout AND at final
  2. EMERGENCY_STOP / SEARCH (R3.x)  - SafetyController asserted last cycle
  3. DETOUR / HOLD / SEARCH  (R2.x)  - Goal_Point in costmap Forbidden_Zone
  4. TRACKING                (R6.x)  - Pure Pursuit fallback

The ``SafetyController`` hard override is applied AFTER the state machine
selects a base ``(linear, angular)``, exactly as before.

Parameters
----------
  leader_ns                    (str,   'tb1')
  convoy_spacing               (float, 0.5)   gap per convoy slot [m]
  lookahead_distance           (float, 0.5)   Pure Pursuit lookahead [m]
  kp_linear                    (float, 0.8)
  kp_angular                   (float, 1.5)
  max_linear_velocity          (float, 0.22)
  max_angular_velocity         (float, 1.0)
  safe_distance                (float, 0.35)
  control_frequency            (float, 50.0) Hz
  max_linear_accel             (float, 1.0)  m/s^2 command slew limit
  max_angular_accel            (float, 3.0)  rad/s^2 command slew limit
  goal_tolerance               (float, 0.4)  Euclidean stop threshold [m]
  spawn_offset_x/y             (float, 0.0)  odom->world translation
  costmap_size                 (float, 3.0)  square costmap span [m]
  costmap_resolution           (float, 0.05) cell size [m]
  costmap_publish_rate         (float, 10.0) informational; build runs each cycle
  costmap_stale_timeout        (float, 1.0)  clear costmap if scan older [s]
  detour_forward_min_vel       (float, 0.06) DETOUR creep [m/s]
  stationary_deadlock_timeout  (float, 2.0)  escalate to SEARCH after this [s]
  emergency_recovery_timeout   (float, 0.5)  informational [s]
  search_angular_velocity      (float, 0.6)  rotate-in-place rate [rad/s]
  breadcrumb_timeout           (float, 10.0) [0.1, 600] (R7.5)
"""

import math
import os
import sys
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from safety_controller import SafetyController
from costmap_utils import (
    Costmap,
    build_costmap,
    is_goal_blocked,
    free_counts_per_side,
)
from convoy_tracking import (
    compute_goal_point,
    is_newer_breadcrumb,
    should_hold,
)
from follower_state import (
    FollowerState,
    classify_state,
    build_detour_command,
    build_search_command,
)


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _to_robot_frame(px: float, py: float,
                    rx: float, ry: float, ryaw: float) -> Tuple[float, float]:
    """Transform a world point (px,py) into the robot's local frame."""
    dx = px - rx
    dy = py - ry
    local_x = dx * math.cos(ryaw) + dy * math.sin(ryaw)
    local_y = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
    return local_x, local_y


class FollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('follower_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('leader_ns', 'tb1')
        self.declare_parameter('convoy_spacing', 1.0)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('max_linear_velocity', 0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 3.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)

        # New parameters: local costmap, detour state machine, persistent
        # leader tracking. Defaults here are fallback-only; the canonical
        # values live in config/follower_params.yaml.
        self.declare_parameter('costmap_size', 3.0)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_publish_rate', 10.0)
        self.declare_parameter('costmap_stale_timeout', 1.0)
        self.declare_parameter('detour_forward_min_vel', 0.06)
        self.declare_parameter('stationary_deadlock_timeout', 2.0)
        self.declare_parameter('emergency_recovery_timeout', 0.5)
        self.declare_parameter('search_angular_velocity', 0.6)
        self.declare_parameter('breadcrumb_timeout', 10.0)

        gp = lambda n: self.get_parameter(n).value
        self.leader_ns          = gp('leader_ns')
        self.convoy_spacing     = gp('convoy_spacing')
        self.lookahead_distance = gp('lookahead_distance')
        self.kp_linear          = gp('kp_linear')
        self.kp_angular         = gp('kp_angular')
        self.max_lin            = gp('max_linear_velocity')
        self.max_ang            = gp('max_angular_velocity')
        self.control_frequency  = gp('control_frequency')
        self.max_lin_acc        = gp('max_linear_accel')
        self.max_ang_acc        = gp('max_angular_accel')
        self.goal_tol           = gp('goal_tolerance')
        self.off_x              = gp('spawn_offset_x')
        self.off_y              = gp('spawn_offset_y')

        self.costmap_size                = float(gp('costmap_size'))
        self.costmap_resolution          = float(gp('costmap_resolution'))
        self.costmap_publish_rate        = float(gp('costmap_publish_rate'))
        self.costmap_stale_timeout       = float(gp('costmap_stale_timeout'))
        self.detour_forward_min_vel      = float(gp('detour_forward_min_vel'))
        self.stationary_deadlock_timeout = float(gp('stationary_deadlock_timeout'))
        self.emergency_recovery_timeout  = float(gp('emergency_recovery_timeout'))
        self.search_angular_velocity     = float(gp('search_angular_velocity'))
        self.breadcrumb_timeout          = float(gp('breadcrumb_timeout'))

        # R7.5: validate breadcrumb_timeout against [0.1, 600]; clamp + warn.
        if not (0.1 <= self.breadcrumb_timeout <= 600.0):
            clamped = max(0.1, min(self.breadcrumb_timeout, 600.0))
            self.get_logger().warn(
                f"breadcrumb_timeout={self.breadcrumb_timeout} outside [0.1, 600]; "
                f"clamping to {clamped}"
            )
            self.breadcrumb_timeout = clamped

        self.safety = SafetyController(
            safe_distance=gp('safe_distance'),
            max_linear_vel=self.max_lin,
            max_angular_vel=self.max_ang,
        )

        # Convoy slot index from namespace (tb2 -> 2, tb3 -> 3)
        try:
            self._idx = int(self.get_namespace().strip('/').replace('tb', ''))
        except ValueError:
            self._idx = 2
        self._gap = (self._idx - 1) * self.convoy_spacing

        # ── State (written by callbacks, read by control loop) ───────────────
        self._pose: Optional[Tuple[float, float, float]] = None
        self._path: List[Tuple[float, float]] = []
        self._scan: Optional[LaserScan] = None
        self._scan_time = None  # rclpy.time.Time of latest scan receipt
        self._last_lin = 0.0
        self._last_ang = 0.0
        self._last_closest_idx = 0

        # Breadcrumb-freshness bookkeeping (R7.1, R7.6).
        self._prev_count = 0
        self._prev_newest_stamp_ns = 0
        self._last_newest_time = self.get_clock().now()

        # Anti-deadlock timers used by the state machine (R3.1, R3.5).
        # ``None`` means "not currently stationary / not currently in
        # emergency"; otherwise the value is the rclpy.time.Time at which
        # the run started.
        self._stationary_since = None
        self._emergency_since = None
        self._prev_emergency = False

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, qos)
        self.create_subscription(Path, f'/{self.leader_ns}/convoy_path',
                                 self._path_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos)

        self._dt = 1.0 / float(self.control_frequency)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"Path-follower tb{self._idx} | gap={self._gap:.2f}m | "
            f"lookahead={self.lookahead_distance:.2f}m | "
            f"control={self.control_frequency:.0f}Hz"
        )
        self.get_logger().info(
            f"State machine enabled | costmap={self.costmap_size:.1f}m@"
            f"{self.costmap_resolution:.2f}m | breadcrumb_timeout="
            f"{self.breadcrumb_timeout:.1f}s | "
            f"deadlock_timeout={self.stationary_deadlock_timeout:.1f}s"
        )

    # ── Callbacks: cache only ────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x + self.off_x, p.y + self.off_y, _yaw_from_quaternion(q))

    def _path_cb(self, msg: Path) -> None:
        self._path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

        # Breadcrumb-freshness bookkeeping (R7.1, R7.6). The newest
        # breadcrumb's stamp is the last pose's header stamp when poses
        # exist, else the message header stamp (so an empty path still
        # has a timestamp the freshness check can compare against).
        new_count = len(msg.poses)
        if msg.poses:
            h = msg.poses[-1].header
        else:
            h = msg.header
        new_stamp_ns = int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)

        if is_newer_breadcrumb(self._prev_count, self._prev_newest_stamp_ns,
                               new_count, new_stamp_ns):
            self._last_newest_time = self.get_clock().now()
        self._prev_count = new_count
        self._prev_newest_stamp_ns = new_stamp_ns

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_time = self.get_clock().now()

    # ── Control loop ─────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        if self._pose is None or len(self._path) < 2:
            # Nothing to track yet: hold zero, reset deadlock timers so a
            # transient pre-startup pause does not accumulate into a
            # spurious SEARCH escalation once the path arrives.
            self._stationary_since = None
            self._emergency_since = None
            self._prev_emergency = False
            self._publish_smoothed(0.0, 0.0)
            return

        rx, ry, ryaw = self._pose
        path = self._path
        now = self.get_clock().now()

        # ── In-process costmap (task 8.2) ─────────────────────────────
        # Build the local occupancy grid from the cached LaserScan. If no
        # scan has been received OR the last scan is older than
        # ``costmap_stale_timeout`` (R1.6, R1.7), feed empty ranges so the
        # builder produces an all-free grid of correct dimensions.
        scan_stale = (
            self._scan is None
            or self._scan_time is None
            or (now - self._scan_time).nanoseconds / 1e9
                > self.costmap_stale_timeout
        )
        if scan_stale:
            cm: Costmap = build_costmap(
                [], 0.0, 0.0, 0.0, 0.0,
                self.costmap_size, self.costmap_resolution,
            )
            scan_for_safety = None
        else:
            s = self._scan
            cm = build_costmap(
                s.ranges, s.angle_min, s.angle_increment,
                s.range_min, s.range_max,
                self.costmap_size, self.costmap_resolution,
            )
            scan_for_safety = s

        safety_emergency_now = False
        if scan_for_safety is not None:
            safety_emergency_now = self.safety.is_emergency(
                ranges=list(scan_for_safety.ranges),
                angle_min=scan_for_safety.angle_min,
                angle_increment=scan_for_safety.angle_increment,
                range_min=(
                    scan_for_safety.range_min
                    if scan_for_safety.range_min > 0 else 0.12
                ),
            )

        # ── Goal point: arc-length `gap` back from the newest path point ──
        # ``compute_goal_point`` interpolates along the breadcrumb path and
        # clamps to the oldest breadcrumb when the path is shorter than the
        # gap (R6.5). We still need an integer goal_idx for the existing
        # closest_idx / lookahead machinery below — recover it from the
        # same backward arc-length walk.
        goal = compute_goal_point(path, self._gap)
        goal_idx = 0
        acc = 0.0
        for i in range(len(path) - 1, 0, -1):
            acc += math.hypot(path[i][0] - path[i - 1][0],
                              path[i][1] - path[i - 1][1])
            if acc >= self._gap:
                goal_idx = i - 1
                break
        # If the loop did not break (total < gap), goal_idx stays at 0,
        # matching ``compute_goal_point``'s clamp to ``path[0]``.

        # ── Existing Pure Pursuit math (TRACKING base command) ────────
        # 2. Closest path point to the robot (start near the previous
        # closest point for an O(1) walk along the path).
        start_idx = min(self._last_closest_idx, goal_idx)
        closest_idx = start_idx
        best = float('inf')
        for i in range(start_idx, goal_idx + 1):
            d = math.hypot(path[i][0] - rx, path[i][1] - ry)
            if d < best:
                best = d
                closest_idx = i
            elif d > best + 0.5:
                break
        self._last_closest_idx = closest_idx

        # 3. Pure Pursuit lookahead point (capped at the goal).
        look = goal
        acc = 0.0
        for i in range(closest_idx, goal_idx):
            acc += math.hypot(path[i + 1][0] - path[i][0],
                              path[i + 1][1] - path[i][1])
            look = path[i + 1]
            if acc >= self.lookahead_distance:
                break

        lx, ly = _to_robot_frame(look[0], look[1], rx, ry, ryaw)
        Ld = max(math.hypot(lx, ly), 1e-3)
        alpha = math.atan2(ly, lx)

        # 4. Goal in robot-local frame (used for blocking test AND the
        # existing speed-scaling logic below).
        gx_local, gy_local = _to_robot_frame(goal[0], goal[1], rx, ry, ryaw)
        dist_to_goal = math.hypot(gx_local, gy_local)

        # 5. Pure Pursuit command (this is the TRACKING base; identical
        # math to the previous version of this file).
        if dist_to_goal <= self.goal_tol:
            pursuit_linear = 0.0
            pursuit_angular = 0.0
        else:
            forward_drive = max(0.0, gx_local)
            creep = self.kp_linear * dist_to_goal * 0.3
            pursuit_linear = self.kp_linear * max(forward_drive, creep)
            curvature = 2.0 * ly / (Ld * Ld)
            pursuit_angular = pursuit_linear * curvature
            if abs(alpha) > 0.8:
                pursuit_angular = self.kp_angular * alpha
                pursuit_linear *= 0.3
            pursuit_linear *= max(0.3, math.cos(alpha))

        # ── State-machine inputs ──────────────────────────────────────
        # Goal-blocked test in the robot-local costmap frame (R2.2, R3.3).
        blocked = is_goal_blocked(cm, gx_local, gy_local)
        left_free, right_free = free_counts_per_side(cm)
        both_blocked = (left_free == 0 and right_free == 0)

        # Distance to the goal in the world frame for persistent-pursuit
        # bookkeeping (R6.6 / R7.2 / R7.4).
        dist_to_goal_world = math.hypot(goal[0] - rx, goal[1] - ry)
        elapsed = (now - self._last_newest_time).nanoseconds / 1e9
        hold = should_hold(
            elapsed, self.breadcrumb_timeout,
            dist_to_goal_world, self.goal_tol,
        )
        has_unreached = dist_to_goal_world > self.goal_tol

        stationary_duration = (
            (now - self._stationary_since).nanoseconds / 1e9
            if self._stationary_since is not None else 0.0
        )

        if safety_emergency_now:
            if not self._prev_emergency or self._emergency_since is None:
                self._emergency_since = now
        else:
            self._emergency_since = None
        self._prev_emergency = safety_emergency_now

        emergency_duration = (
            (now - self._emergency_since).nanoseconds / 1e9
            if self._emergency_since is not None else 0.0
        )

        # ── Classify next state and pick the base command ─────────────
        state = classify_state(
            goal_blocked=blocked,
            both_sides_blocked=both_blocked,
            safety_emergency=safety_emergency_now,
            hold_active=hold,
            stationary_duration_s=stationary_duration,
            emergency_duration_s=emergency_duration,
            deadlock_timeout_s=self.stationary_deadlock_timeout,
            has_unreached_breadcrumbs=has_unreached,
        )

        if state == FollowerState.HOLD:
            base_linear, base_angular = 0.0, 0.0
        elif state == FollowerState.EMERGENCY_STOP:
            base_linear, base_angular = 0.0, 0.0
        elif state == FollowerState.SEARCH:
            base_linear, base_angular = build_search_command(
                self.search_angular_velocity, self.max_ang,
            )
        elif state == FollowerState.DETOUR:
            base_linear, base_angular = build_detour_command(
                cm, pursuit_linear,
                self.max_lin, self.max_ang,
                self.detour_forward_min_vel,
            )
        else:  # TRACKING
            base_linear, base_angular = pursuit_linear, pursuit_angular

        # ── SafetyController hard override (applied LAST) ─────────────
        linear_x, angular_z = base_linear, base_angular
        if scan_for_safety is not None:
            s = scan_for_safety
            linear_x, angular_z = self.safety.check_and_modify(
                linear_x=linear_x,
                angular_z=angular_z,
                ranges=list(s.ranges),
                angle_min=s.angle_min,
                angle_increment=s.angle_increment,
                range_min=s.range_min if s.range_min > 0 else 0.12,
            )

        # Slew/clamp the base command and record it as the new
        # ``self._last_lin`` / ``self._last_ang``.
        self._publish_smoothed(linear_x, angular_z)

        # ── Update stationary-duration tracking (R3.1) ────────────────
        # Use the post-publish ``self._last_lin`` (the actual command
        # sent on the wire). A near-zero magnitude starts the timer; any
        # forward / reverse motion clears it.
        if abs(self._last_lin) < 1e-6:
            if self._stationary_since is None:
                self._stationary_since = now
        else:
            self._stationary_since = None

    # ── Output smoothing ─────────────────────────────────────────────────────
    def _slew(self, cur: float, tgt: float, max_delta: float) -> float:
        if tgt > cur + max_delta:
            return cur + max_delta
        if tgt < cur - max_delta:
            return cur - max_delta
        return tgt

    def _publish_smoothed(self, linear_x: float, angular_z: float) -> None:
        # Clamp to limits, then slew-limit against the previous command.
        linear_x = max(-self.max_lin, min(float(linear_x), self.max_lin))
        angular_z = max(-self.max_ang, min(float(angular_z), self.max_ang))
        linear_x = self._slew(self._last_lin, linear_x, self.max_lin_acc * self._dt)
        angular_z = self._slew(self._last_ang, angular_z, self.max_ang_acc * self._dt)
        self._last_lin = linear_x
        self._last_ang = angular_z
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())   # stop on shutdown
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
