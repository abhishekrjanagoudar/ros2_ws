#!/usr/bin/env python3
"""
follower_node.py
================
ROS 2 node shell for the Path-Based Convoy follower.

This file owns **only**:
  * ROS parameter declarations and reads
  * Subscriber / publisher / timer creation
  * Message-cache callbacks (_odom_cb, _path_cb, _scan_cb)
  * Output smoothing and publishing (_publish_smoothed)
  * main()

All convoy algorithm logic lives in ``motion_controller.PursuitController``.
To tune control behaviour, edit motion_controller.py — not this file.

Architecture
------------
The leader (tb1) publishes its travelled trajectory as a nav_msgs/Path on
``/<leader_ns>/convoy_path`` (frame = ``world``). Every follower subscribes
to that single shared path and tracks it with a Pure Pursuit controller
while holding a configurable gap behind the robot ahead.

  * Lateral control : Pure Pursuit (lookahead point on the path).
  * Longitudinal     : proportional to remaining arc-length to the goal
                       point, so the convoy spacing is self-regulating.
  * LiDAR            : used by SafetyController for emergency stop / steer
                       bias AND by the in-process costmap builder for the
                       TRACKING / DETOUR / EMERGENCY_STOP / SEARCH / HOLD
                       state machine.

Frames
------
Each robot's ``odom`` is anchored to ``world`` by a static transform at its
spawn position, with zero rotation. So the robot's world pose is simply
``odom + spawn_offset``. The spawn offset is supplied via parameters
(spawn_offset_x / spawn_offset_y) by the launch file.

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
  control_frequency            (float, 50.0)  Hz
  max_linear_accel             (float, 1.0)   m/s^2 command slew limit
  max_angular_accel            (float, 3.0)   rad/s^2 command slew limit
  goal_tolerance               (float, 0.4)   Euclidean stop threshold [m]
  spawn_offset_x/y             (float, 0.0)   odom->world translation
  costmap_size                 (float, 3.0)   square costmap span [m]
  costmap_resolution           (float, 0.05)  cell size [m]
  costmap_publish_rate         (float, 10.0)  informational; build runs each cycle
  costmap_stale_timeout        (float, 1.0)   clear costmap if scan older [s]
  detour_forward_min_vel       (float, 0.06)  DETOUR creep [m/s]
  stationary_deadlock_timeout  (float, 2.0)   escalate to SEARCH after this [s]
  emergency_recovery_timeout   (float, 0.5)   informational [s]
  search_angular_velocity      (float, 0.6)   rotate-in-place rate [rad/s]
  breadcrumb_timeout           (float, 10.0)  [0.1, 600] (R7.5)
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

from motion_controller import PursuitController, yaw_from_quaternion, slew
from safety_controller import SafetyController
from convoy_tracking import is_newer_breadcrumb


class FollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('follower_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('leader_ns', 'tb1')
        self.declare_parameter('convoy_spacing', 0.6)  # must match SPAWN_X_STEP in launch_common.py and follower_params.yaml
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('max_linear_velocity', 0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safe_distance', 0.15)  # must be strictly < convoy_spacing (0.5 m) so the robot ahead at the nominal gap does not trigger an emergency stop
        self.declare_parameter('predecessor_gap', 0.6)
        self.declare_parameter('control_frequency', 20.0)  # Hz — must match follower_params.yaml
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 3.0)
        self.declare_parameter('goal_tolerance', 0.05)   # Euclidean stop threshold [m]; must match follower_params.yaml
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)

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
        self.max_lin            = gp('max_linear_velocity')
        self.max_ang            = gp('max_angular_velocity')
        self.control_frequency  = gp('control_frequency')
        self.max_lin_acc        = gp('max_linear_accel')
        self.max_ang_acc        = gp('max_angular_accel')
        self.off_x              = gp('spawn_offset_x')
        self.off_y              = gp('spawn_offset_y')

        breadcrumb_timeout = float(gp('breadcrumb_timeout'))
        # R7.5: validate breadcrumb_timeout against [0.1, 600]; clamp + warn.
        if not (0.1 <= breadcrumb_timeout <= 600.0):
            clamped = max(0.1, min(breadcrumb_timeout, 600.0))
            self.get_logger().warn(
                f"breadcrumb_timeout={breadcrumb_timeout} outside [0.1, 600]; "
                f"clamping to {clamped}"
            )
            breadcrumb_timeout = clamped

        # Convoy slot index from namespace (tb2 → 2, tb3 → 3)
        try:
            slot = int(self.get_namespace().strip('/').replace('tb', ''))
        except ValueError:
            slot = 2

        safety = SafetyController(
            safe_distance=gp('safe_distance'),
            max_linear_vel=self.max_lin,
            max_angular_vel=self.max_ang,
            predecessor_gap=float(gp('predecessor_gap')),
        )

        self._controller = PursuitController(
            convoy_slot=slot,
            convoy_spacing=float(gp('convoy_spacing')),
            lookahead_distance=float(gp('lookahead_distance')),
            kp_linear=float(gp('kp_linear')),
            kp_angular=float(gp('kp_angular')),
            goal_tol=float(gp('goal_tolerance')),
            costmap_size=float(gp('costmap_size')),
            costmap_resolution=float(gp('costmap_resolution')),
            costmap_stale_timeout=float(gp('costmap_stale_timeout')),
            detour_forward_min_vel=float(gp('detour_forward_min_vel')),
            stationary_deadlock_timeout=float(gp('stationary_deadlock_timeout')),
            search_angular_velocity=float(gp('search_angular_velocity')),
            breadcrumb_timeout=breadcrumb_timeout,
            max_lin=self.max_lin,
            max_ang=self.max_ang,
            safety=safety,
        )

        # ── Message caches (written by callbacks, read by control loop) ──────
        self._pose: Optional[Tuple[float, float, float]] = None
        self._path: List[Tuple[float, float]]            = []
        self._scan: Optional[LaserScan]                  = None
        self._scan_time_ns: Optional[int]                = None  # ns since epoch

        # Breadcrumb-freshness bookkeeping (R7.1, R7.6)
        self._prev_count: int            = 0
        self._prev_newest_stamp_ns: int  = 0

        # Tracks whether the control loop has successfully run at least once.
        # Used to gate pre-mission timer resets (see _control_loop).
        self._has_ever_tracked: bool = False

        # Output smoothing state
        self._last_lin: float = 0.0
        self._last_ang: float = 0.0
        self._dt = 1.0 / float(self.control_frequency)

        # ── ROS wiring ───────────────────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry,   'odom',
                                 self._odom_cb, qos)
        self.create_subscription(Path,       'convoy_path',
                                 self._path_cb, 10)
        self.create_subscription(LaserScan,  'scan',
                                 self._scan_cb, qos)
        self.create_timer(self._dt, self._control_loop)

        gap = (slot - 1) * float(gp('convoy_spacing'))
        self.get_logger().info(
            f"Path-follower tb{slot} | gap={gap:.2f}m | "
            f"lookahead={gp('lookahead_distance'):.2f}m | "
            f"control={self.control_frequency:.0f}Hz"
        )
        self.get_logger().info(
            f"State machine enabled | costmap={gp('costmap_size'):.1f}m@"
            f"{gp('costmap_resolution'):.2f}m | "
            f"breadcrumb_timeout={breadcrumb_timeout:.1f}s | "
            f"deadlock_timeout={gp('stationary_deadlock_timeout'):.1f}s"
        )

    # ── Callbacks: cache only ────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (
            p.x + self.off_x,
            p.y + self.off_y,
            yaw_from_quaternion(q),
        )

    def _path_cb(self, msg: Path) -> None:
        # Guard: ignore empty Path messages published during startup before the
        # leader's first odom arrives. Processing them would set _path=[] and
        # corrupt _last_newest_time_ns / breadcrumb freshness state (Prompt 3).
        if not msg.poses:
            return

        self._path = [(ps.pose.position.x, ps.pose.position.y)
                      for ps in msg.poses]

        # Breadcrumb-freshness bookkeeping (R7.1, R7.6).
        new_count = len(msg.poses)
        h = msg.poses[-1].header
        new_stamp_ns = int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)

        if is_newer_breadcrumb(self._prev_count, self._prev_newest_stamp_ns,
                               new_count, new_stamp_ns):
            now_ns = self.get_clock().now().nanoseconds
            self._controller.notify_newer_breadcrumb(now_ns)

        self._prev_count           = new_count
        self._prev_newest_stamp_ns = new_stamp_ns

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan         = msg
        self._scan_time_ns = self.get_clock().now().nanoseconds

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        if self._pose is None or len(self._path) < 2:
            # Nothing to track yet — publish zero.
            # Only reset deadlock timers during pre-mission startup (before
            # _has_ever_tracked is set). Once tracking has started, a transient
            # path dropout mid-mission must NOT clear the timers — doing so
            # would prevent SEARCH from escalating if the path disappears.
            if not self._has_ever_tracked:
                self._controller._stationary_since = None
                self._controller._emergency_since  = None
                self._controller._prev_emergency   = False
            self._publish_smoothed(0.0, 0.0)
            return

        self._has_ever_tracked = True
        now_ns    = self.get_clock().now().nanoseconds
        scan      = self._scan
        scan_age  = (
            (now_ns - self._scan_time_ns) / 1e9
            if self._scan_time_ns is not None else float('inf')
        )

        linear_x, angular_z, _ = self._controller.step(
            pose=self._pose,
            path=self._path,
            scan=scan,
            scan_age_s=scan_age,
            now_ns=now_ns,
        )

        self._publish_smoothed(linear_x, angular_z)

        # Stationary timer uses the post-slew actual command (self._last_lin).
        self._controller.update_stationary_timer(self._last_lin, now_ns)

    # ── Output smoothing ─────────────────────────────────────────────────────

    def _publish_smoothed(self, linear_x: float, angular_z: float) -> None:
        """Clamp then slew-limit against the previous command before publishing."""
        linear_x  = max(-self.max_lin, min(float(linear_x),  self.max_lin))
        angular_z = max(-self.max_ang, min(float(angular_z), self.max_ang))
        linear_x  = slew(self._last_lin, linear_x,  self.max_lin_acc * self._dt)
        angular_z = slew(self._last_ang, angular_z, self.max_ang_acc * self._dt)
        self._last_lin = linear_x
        self._last_ang = angular_z
        msg = Twist()
        msg.linear.x  = linear_x
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
