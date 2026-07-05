#!/usr/bin/env python3
"""
VISUALIZATION-ONLY producer of ``nav_msgs/OccupancyGrid`` from a single
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Make sibling pure modules (e.g. ``costmap_utils``) importable when this
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from costmap_utils import build_costmap  # noqa: E402  (sys.path setup above)


class CostmapGenerator(Node):
    """
Visualization only — does not publish ``cmd_vel`` (R1.4, R1.5).
"""

    def __init__(self) -> None:
        super().__init__('costmap_generator')

        # Parameters
        self.declare_parameter('costmap_size', 3.0)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_publish_rate', 10.0)
        self.declare_parameter('costmap_stale_timeout', 1.0)
        self.declare_parameter('costmap_frame', '')
        self.declare_parameter('range_min_override', 0.0)
        # enable_costmap_viz: set false (default) in headless runs to avoid
        self.declare_parameter('enable_costmap_viz', False)
        # Backward-compatible aliases for direct/manual launches.
        self.declare_parameter('size_m', 0.0)
        self.declare_parameter('resolution', 0.0)
        self.declare_parameter('publish_rate', 0.0)
        self.declare_parameter('stale_timeout', 0.0)

        gp = lambda n: self.get_parameter(n).value
        self.size_m             = float(gp('size_m') or gp('costmap_size'))
        self.resolution         = float(gp('resolution') or gp('costmap_resolution'))
        self.publish_rate       = float(gp('publish_rate') or gp('costmap_publish_rate'))
        self.stale_timeout      = float(gp('stale_timeout') or gp('costmap_stale_timeout'))
        self.costmap_frame      = str(gp('costmap_frame'))
        self.range_min_override = float(gp('range_min_override'))
        self.enabled            = bool(gp('enable_costmap_viz'))

        # QoS profiles
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Costmap consumers (RViz Map display, follower diagnostics) want a
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions / publishers
        self._scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_cb, scan_qos,
        )
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, 'local_costmap', costmap_qos,
        )

        # Cached state (written by ``_scan_cb``, read by ``_publish``)
        self._scan = None        # type: LaserScan | None
        self._scan_time = None   # type: rclpy.time.Time | None
        # Most recently published Costmap; kept for inspection/tests.
        self._last_cm = None
        # Used to log a single info-level confirmation line on first publish.
        self._first_publish = True

        # Publish timer
        period = 1.0 / max(self.publish_rate, 1e-3)
        self._publish_timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"CostmapGenerator | size={self.size_m:.2f}m | "
            f"resolution={self.resolution:.3f}m | "
            f"publish_rate={self.publish_rate:.1f}Hz | "
            f"stale_timeout={self.stale_timeout:.2f}s | "
            f"costmap_frame='{self.costmap_frame}' | "
            f"range_min_override={self.range_min_override:.3f} | "
            f"enable_costmap_viz={self.enabled}"
        )
        if not self.enabled:
            self.get_logger().info(
                "CostmapGenerator: enable_costmap_viz=false — "
                "publish timer is active but will not publish. "
                "Set enable_costmap_viz:=true (or ros_ui:=true) to enable."
            )

    # Callbacks
    def _scan_cb(self, msg: LaserScan) -> None:
        """Cache the latest LaserScan and its receipt time."""
        self._scan = msg
        self._scan_time = self.get_clock().now()

    def _publish(self) -> None:
        """
No-ops when ``enable_costmap_viz`` is false (headless mode) to avoid
"""
        if not self.enabled:
            return

        # Decide what to feed into build_costmap
        if self._scan is None:
            # R1.6: no scan ever received → publish an all-free grid by
            cm = build_costmap(
                [], 0.0, 0.0, 0.0, 0.0, self.size_m, self.resolution,
            )
        else:
            elapsed = (
                self.get_clock().now() - self._scan_time
            ).nanoseconds / 1e9
            if elapsed > self.stale_timeout:
                # R1.7: cached scan is older than ``stale_timeout`` →
                cm = build_costmap(
                    [], 0.0, 0.0, 0.0, 0.0, self.size_m, self.resolution,
                )
            else:
                s = self._scan
                # ``range_min_override == 0.0`` disables the override;
                rmin = (
                    max(s.range_min, self.range_min_override)
                    if self.range_min_override > 0.0
                    else s.range_min
                )
                cm = build_costmap(
                    s.ranges, s.angle_min, s.angle_increment,
                    rmin, s.range_max, self.size_m, self.resolution,
                )

        # Cache the most recently computed costmap so tests / future
        self._last_cm = cm

        # Populate and publish the OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Prefer the explicit ``costmap_frame`` parameter; fall back to
        ns = self.get_namespace().strip('/')
        fallback_frame = f"{ns}/base_scan" if ns else 'base_scan'
        msg.header.frame_id = (
            self.costmap_frame
            or (self._scan.header.frame_id if self._scan is not None else fallback_frame)
        )
        msg.info.resolution = float(cm.resolution)
        msg.info.width = int(cm.width)
        msg.info.height = int(cm.height)
        msg.info.origin.position.x = float(cm.origin_x)
        msg.info.origin.position.y = float(cm.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # identity quaternion
        # Occupancy values are 0 or 100 (both fit in int8); rclpy
        msg.data = list(cm.data)
        self._costmap_pub.publish(msg)

        if self._first_publish:
            self.get_logger().info(
                f"CostmapGenerator publishing local_costmap "
                f"({msg.info.width}x{msg.info.height} @ "
                f"{msg.info.resolution:.3f}m, "
                f"frame_id='{msg.header.frame_id}')"
            )
            self._first_publish = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CostmapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
