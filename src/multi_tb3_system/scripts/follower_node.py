#!/usr/bin/env python3
"""
ROS 2 node shell for the Path-Based Convoy follower.
"""

import math
import os
import sys
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry, Path, OccupancyGrid
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

        # Parameters
        self.declare_parameter('leader_ns', 'tb1')
        self.declare_parameter('convoy_spacing', 0.6)  # must match SPAWN_X_STEP in launch_common.py and follower_params.yaml
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('max_linear_velocity', 0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safe_distance', 0.20)  # Emergency stop distance - must be < convoy_spacing (0.6m) to allow normal following
        self.declare_parameter('predecessor_gap', 0.6)
        self.declare_parameter('control_frequency', 20.0)  # Hz — must match follower_params.yaml
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 3.0)
        self.declare_parameter('goal_tolerance', 0.05)   # Euclidean stop threshold [m]; must match follower_params.yaml
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)

        self.declare_parameter('costmap_size', 3.0)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_stale_timeout', 1.0)
        self.declare_parameter('detour_forward_min_vel', 0.06)
        self.declare_parameter('stationary_deadlock_timeout', 2.0)
        self.declare_parameter('search_angular_velocity', 0.6)
        self.declare_parameter('breadcrumb_timeout', 10.0)
        self.declare_parameter('enable_costmap_viz', False)
        self.declare_parameter('costmap_frame', '')

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

        # Message caches (written by callbacks, read by control loop)
        self._pose: Optional[Tuple[float, float, float]] = None
        self._path: List[Tuple[float, float]]            = []
        self._scan: Optional[LaserScan]                  = None
        self._scan_time_ns: Optional[int]                = None  # ns since epoch

        # Breadcrumb-freshness bookkeeping (R7.1, R7.6)
        self._prev_count: int            = 0
        self._prev_newest_stamp_ns: int  = 0

        # Tracks whether the control loop has successfully run at least once.
        self._has_ever_tracked: bool = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_corr_x = 0.0
        self.tf_corr_y = 0.0

        # Output smoothing state
        self._last_lin: float = 0.0
        self._last_ang: float = 0.0
        self._dt = 1.0 / float(self.control_frequency)

        # ROS wiring
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
                                 
        self.enable_viz = bool(gp('enable_costmap_viz'))
        self.costmap_frame = str(gp('costmap_frame'))
        if self.enable_viz:
            costmap_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.costmap_pub = self.create_publisher(OccupancyGrid, 'local_costmap', costmap_qos)

        self.create_timer(self._dt, self._control_loop)

        gap = float(gp('convoy_spacing'))
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

    # Callbacks: cache only

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (
            p.x + self.off_x + self.tf_corr_x,
            p.y + self.off_y + self.tf_corr_y,
            yaw_from_quaternion(q),
        )

    def _path_cb(self, msg: Path) -> None:
        # Guard: ignore empty Path messages published during startup before the
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

    # Control loop

    def _control_loop(self) -> None:
        # Broadcast the dynamic world -> <namespace>/odom TF unconditionally
        # so RViz can see the robot even before tracking starts.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        ns = self.get_namespace().strip('/')
        t.child_frame_id = f'{ns}/odom'
        t.transform.translation.x = float(self.off_x + self.tf_corr_x)
        t.transform.translation.y = float(self.off_y + self.tf_corr_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        if self._pose is None or len(self._path) < 2:
            # Nothing to track yet — publish zero.
            if not self._has_ever_tracked:
                self._controller._stationary_since = None
                self._controller._emergency_since  = None
            self._publish_smoothed(0.0, 0.0)
            return

        self._has_ever_tracked = True
        now_ns    = self.get_clock().now().nanoseconds
        scan      = self._scan
        scan_age  = (
            (now_ns - self._scan_time_ns) / 1e9
            if self._scan_time_ns is not None else float('inf')
        )

        linear_x, angular_z = self._controller.step(
            pose=self._pose,
            path=self._path,
            scan=scan,
            scan_age_s=scan_age,
            now_ns=now_ns,
        )

        # Update TF correction if raw error is available
        if hasattr(self._controller, '_raw_err_x'):
            err_x = self._controller._raw_err_x
            err_y = self._controller._raw_err_y
            del self._controller._raw_err_x
            del self._controller._raw_err_y
            
            # Rotate local error to world frame
            ryaw = self._pose[2]
            err_world_x = err_x * math.cos(ryaw) - err_y * math.sin(ryaw)
            err_world_y = err_x * math.sin(ryaw) + err_y * math.cos(ryaw)
            
            # Accumulate with small gain (0.05) to avoid violent jumping
            self.tf_corr_x -= err_world_x * 0.05
            self.tf_corr_y -= err_world_y * 0.05

        self._publish_smoothed(linear_x, angular_z)
        
        if self.enable_viz and hasattr(self._controller, 'last_costmap'):
            self._publish_costmap(self._controller.last_costmap)

        # Stationary timer uses the post-slew actual command (self._last_lin).
        self._controller.update_stationary_timer(self._last_lin, now_ns)

    def _publish_costmap(self, cm) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.costmap_frame or (self._scan.header.frame_id if self._scan is not None else 'base_scan')
        msg.info.resolution = float(cm.resolution)
        msg.info.width = int(cm.width)
        msg.info.height = int(cm.height)
        msg.info.origin.position.x = float(cm.origin_x)
        msg.info.origin.position.y = float(cm.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = list(cm.data)
        self.costmap_pub.publish(msg)

    # Output smoothing

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
