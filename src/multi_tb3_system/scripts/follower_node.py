#!/usr/bin/env python3
"""
follower_node.py
================
Path-Based Convoy follower using Pure Pursuit.

Architecture
------------
The leader (tb1) publishes its travelled trajectory as a nav_msgs/Path on
``/<leader_ns>/convoy_path`` (frame = ``world``). Every follower subscribes to
that single shared path and tracks it with a Pure Pursuit controller while
holding a configurable gap behind the robot ahead.

  * Lateral control : Pure Pursuit (lookahead point on the path).
  * Longitudinal     : proportional to remaining arc-length to the goal point,
                       so the convoy spacing is self-regulating.
  * LiDAR            : used ONLY by SafetyController for emergency stop / steer
                       bias. No leader detection via LiDAR clusters.

Frames
------
Each robot's ``odom`` is anchored to ``world`` by a static transform at its
spawn position, with zero rotation. So the robot's world pose is simply
``odom + spawn_offset``. The spawn offset is supplied via parameters
(spawn_offset_x / spawn_offset_y) by the launch file.

Control runs on a fixed-rate timer (decoupled from the slow LiDAR), with
slew-rate limiting for smooth, continuous motion.

Parameters
----------
  leader_ns            (str,   'tb1')  namespace publishing convoy_path
  convoy_spacing       (float, 1.0)    gap per convoy slot [m]
  lookahead_distance   (float, 0.5)    Pure Pursuit lookahead [m]
  kp_linear            (float, 0.8)    speed gain on spacing error
  kp_angular           (float, 1.5)    heading gain (large-misalignment mode)
  max_linear_velocity  (float, 0.22)
  max_angular_velocity (float, 1.0)
  safe_distance        (float, 0.4)
  control_frequency    (float, 50.0)   Hz
  max_linear_accel     (float, 1.0)    m/s^2 command slew limit
  max_angular_accel    (float, 3.0)    rad/s^2 command slew limit
  goal_tolerance       (float, 0.10)   stop when within this arc-length of goal
  spawn_offset_x/y     (float, 0.0)    odom->world translation for this robot
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
        self._last_lin = 0.0
        self._last_ang = 0.0

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

    # ── Callbacks: cache only ────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x + self.off_x, p.y + self.off_y, _yaw_from_quaternion(q))

    def _path_cb(self, msg: Path) -> None:
        self._path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg

    # ── Control loop ─────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        if self._pose is None or len(self._path) < 2:
            self._publish_smoothed(0.0, 0.0)
            return

        rx, ry, ryaw = self._pose
        path = self._path

        # 1. Goal point: arc-length `gap` back from the newest path point.
        #    If the leader hasn't travelled `gap` yet, clamp to the path start
        #    (goal_idx=0) instead of holding — so a follower that begins behind
        #    its slot closes the gap immediately rather than waiting for the
        #    leader to drive far ahead.
        goal_idx = 0
        acc = 0.0
        for i in range(len(path) - 1, 0, -1):
            acc += math.hypot(path[i][0] - path[i - 1][0],
                              path[i][1] - path[i - 1][1])
            if acc >= self._gap:
                goal_idx = i - 1
                break
        goal = path[goal_idx]

        # 2. Closest path point to the robot (search up to the goal).
        closest_idx = 0
        best = float('inf')
        for i in range(goal_idx + 1):
            d = math.hypot(path[i][0] - rx, path[i][1] - ry)
            if d < best:
                best = d
                closest_idx = i

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

        # 4. Longitudinal control: forward distance (robot frame) to the goal.
        #    Projecting onto the heading means the follower drives as soon as
        #    its target is ahead, and stops/backs off when it reaches the gap.
        gx, _gy = _to_robot_frame(goal[0], goal[1], rx, ry, ryaw)
        spacing_err = gx

        # 5. Compute velocities.
        if spacing_err <= self.goal_tol:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self.kp_linear * spacing_err
            # Pure Pursuit curvature -> angular velocity.
            curvature = 2.0 * ly / (Ld * Ld)
            angular_z = linear_x * curvature
            # Strong misalignment: rotate to face the path, creep forward.
            if abs(alpha) > 0.8:
                angular_z = self.kp_angular * alpha
                linear_x *= 0.3
            # Ease off speed in curves to avoid corner-cutting.
            linear_x *= max(0.3, math.cos(alpha))

        # 6. Safety override (LiDAR emergency stop + steering bias).
        if self._scan is not None:
            s = self._scan
            linear_x, angular_z = self.safety.check_and_modify(
                linear_x=linear_x,
                angular_z=angular_z,
                ranges=list(s.ranges),
                angle_min=s.angle_min,
                angle_increment=s.angle_increment,
                range_min=s.range_min if s.range_min > 0 else 0.12,
            )

        self._publish_smoothed(linear_x, angular_z)

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
