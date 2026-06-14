#!/usr/bin/env python3
"""
follower_node.py
================
Autonomous LiDAR-based follower node for the Multi-TurtleBot3 convoy system.

This node is REUSABLE for both tb2 (follows tb1) and tb3 (follows tb2).
It is launched under the robot's own namespace so topic paths are resolved
correctly without any hard-coded robot names in this file.

Architecture:
  - Subscribes to: /scan  (LaserScan — relative to this robot's namespace)
  - Publishes to:  /cmd_vel (Twist — relative to this robot's namespace)

Algorithm (simple geometry, no temporal tracking):
  1. Receive LaserScan
  2. Convert ranges → Cartesian points
  3. Filter front-sector ± front_angle_deg
  4. Euclidean cluster segmentation
  5. Reject wall-like (large) and noise (tiny) clusters
  6. Select closest cluster → target (leader robot)
  7. PD control: drive toward target at target_distance
  8. Safety override: stop/steer if too close to anything

Parameters (loaded from follower_params.yaml via ros parameter server):
  target_distance      (float, default 0.7)
  safe_distance        (float, default 0.4)
  kp_linear            (float, default 0.8)
  kp_angular           (float, default 2.0)
  max_linear_velocity  (float, default 0.22)
  max_angular_velocity (float, default 1.0)
  front_angle_deg      (float, default 30.0)
  cluster_distance     (float, default 0.20)
  min_cluster_size     (int,   default 2)
  max_cluster_size     (int,   default 40)

Usage:
  ros2 run multi_tb3_system follower_node.py --ros-args -r __ns:=/tb2 \\
         --params-file /path/to/follower_params.yaml
"""

from __future__ import annotations

import math
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Import sibling modules from scripts directory
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from laser_processor import process_scan, Cluster
from safety_controller import SafetyController


class FollowerNode(Node):
    """
    Autonomous follower node: subscribes to /scan, publishes /cmd_vel.

    When launched under namespace /tbX, this becomes /tbX/scan → /tbX/cmd_vel.
    """

    def __init__(self) -> None:
        super().__init__('follower_node')

        # ── Declare all parameters (with defaults) ──────────────────────────
        self.declare_parameter('target_distance',      0.7)
        self.declare_parameter('safe_distance',        0.4)
        self.declare_parameter('kp_linear',            0.8)
        self.declare_parameter('kp_angular',           2.0)
        self.declare_parameter('max_linear_velocity',  0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('front_angle_deg',      30.0)
        self.declare_parameter('cluster_distance',     0.20)
        self.declare_parameter('min_cluster_size',     2)
        self.declare_parameter('max_cluster_size',     40)

        # ── Fetch parameter values ──────────────────────────────────────────
        self.target_distance     = self.get_parameter('target_distance').value
        self.safe_distance       = self.get_parameter('safe_distance').value
        self.kp_linear           = self.get_parameter('kp_linear').value
        self.kp_angular          = self.get_parameter('kp_angular').value
        self.max_linear_vel      = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel     = self.get_parameter('max_angular_velocity').value
        self.front_angle_deg     = self.get_parameter('front_angle_deg').value
        self.cluster_distance    = self.get_parameter('cluster_distance').value
        self.min_cluster_size    = self.get_parameter('min_cluster_size').value
        self.max_cluster_size    = self.get_parameter('max_cluster_size').value

        # ── Safety controller instance ──────────────────────────────────────
        self.safety = SafetyController(
            safe_distance=self.safe_distance,
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel,
        )

        # ── QoS profile for LaserScan ───────────────────────────────────────
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # ── Subscriber: /scan ───────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',   # resolved to /tbX/scan via namespace
            self._scan_callback,
            scan_qos,
        )

        # ── Publisher: /cmd_vel ─────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',  # resolved to /tbX/cmd_vel via namespace
            10,
        )

        # ── State ───────────────────────────────────────────────────────────
        self._no_target_count: int = 0   # frames without a target

        self.get_logger().info(
            f"FollowerNode started | "
            f"target_dist={self.target_distance}m | "
            f"front_cone=±{self.front_angle_deg}° | "
            f"safe_dist={self.safe_distance}m"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Scan callback
    # ──────────────────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan) -> None:
        """Process each incoming LaserScan and publish a velocity command."""

        # ── Step 1-5: Full LiDAR processing pipeline ────────────────────────
        target, all_clusters = process_scan(
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min if msg.range_min > 0 else 0.12,
            range_max=min(msg.range_max, 3.5),
            front_half_angle_deg=self.front_angle_deg,
            cluster_distance=self.cluster_distance,
            min_cluster_size=self.min_cluster_size,
            max_cluster_size=self.max_cluster_size,
        )

        # ── Step 6: Compute control commands ────────────────────────────────
        linear_x, angular_z = self._compute_control(target)

        # ── Step 7: Safety override ──────────────────────────────────────────
        linear_x, angular_z = self.safety.check_and_modify(
            linear_x=linear_x,
            angular_z=angular_z,
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min if msg.range_min > 0 else 0.12,
        )

        # ── Step 8: Publish ──────────────────────────────────────────────────
        self._publish_twist(linear_x, angular_z)

        # ── Debug logging ────────────────────────────────────────────────────
        if target:
            self._no_target_count = 0
            self.get_logger().debug(
                f"Target: dist={target.distance:.2f}m "
                f"angle={math.degrees(target.angle):.1f}° → "
                f"v={linear_x:.3f} ω={angular_z:.3f}"
            )
        else:
            self._no_target_count += 1
            if self._no_target_count % 20 == 1:
                self.get_logger().info(
                    f"No leader detected ({len(all_clusters)} clusters, "
                    f"none in front) — waiting..."
                )

    # ──────────────────────────────────────────────────────────────────────────
    # Control law
    # ──────────────────────────────────────────────────────────────────────────

    def _compute_control(self, target: Cluster | None) -> tuple[float, float]:
        """
        Proportional control law.

        When a target cluster is detected:
          Linear velocity  = Kp_lin * distance_error
            where distance_error = target.distance - target_distance
            (positive → too far  → move forward)
            (negative → too close → move backward / stop)

          Angular velocity = Kp_ang * angle_error
            where angle_error = target.angle (angle of centroid in robot frame)
            (positive angle → target is to the left → rotate left = +ω)

        When no target is detected:
          Stop (both velocities = 0).

        Returns:
            (linear_x, angular_z)
        """
        if target is None:
            return 0.0, 0.0

        distance_error = target.distance - self.target_distance
        angle_error    = target.angle

        linear_x  = self.kp_linear  * distance_error
        angular_z = self.kp_angular * angle_error

        # Don't drive backward when too close — only stop
        if linear_x < 0.0:
            linear_x = max(linear_x, -0.05)   # slight backoff allowed

        return linear_x, angular_z

    # ──────────────────────────────────────────────────────────────────────────
    # Publisher helper
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        """Publish a Twist message (required by Gazebo Sim bridge)."""
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Guard against external kill (launch shutdown cascade) where
        # rclpy context is already invalid before finally block runs.
        try:
            if rclpy.ok():
                node._publish_twist(0.0, 0.0)
                node.get_logger().info("FollowerNode shutting down — robot stopped.")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
