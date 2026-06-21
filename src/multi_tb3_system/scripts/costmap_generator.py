#!/usr/bin/env python3
"""
costmap_generator.py
====================
VISUALIZATION-ONLY producer of ``nav_msgs/OccupancyGrid`` from a single
robot's ``sensor_msgs/LaserScan`` stream.

This node runs on **every** robot in the convoy (the leader ``tb1`` and
each follower ``tb2``, ``tb3``). For the leader the costmap exists purely
so the teleoperator can see nearby obstacles in RViz; for the followers
the published costmap is also visualization-only — follower detour
decisions consume ``costmap_utils.build_costmap`` *in process* to avoid
a serialization round-trip.

This node never publishes ``cmd_vel`` (R1.4, R1.5). The node:

* subscribes to ``scan`` (``sensor_msgs/LaserScan``) with BEST_EFFORT
  QoS, matching the rest of the stack (e.g. ``follower_node.py``);
* publishes ``local_costmap`` (``nav_msgs/OccupancyGrid``) with
  RELIABLE/VOLATILE QoS at ``publish_rate`` Hz;
* clears the costmap when no scan has been received yet (R1.6) or the
  latest cached scan is older than ``stale_timeout`` (R1.7).

The node sets up the parameters and the scan subscription / costmap
publisher (task 5.1), runs the publish-timer body that calls
``build_costmap`` and clears the grid on stale data (task 5.2), and
populates the ``OccupancyGrid`` header/info before publishing on
``local_costmap`` (task 5.3, validated against task 5.2's output).
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Make sibling pure modules (e.g. ``costmap_utils``) importable when this
# script is launched as an executable, mirroring the pattern used in
# ``follower_node.py``. The publish-timer body uses
# ``build_costmap`` / ``Costmap`` from ``costmap_utils``.
_scripts_dir = os.path.dirname(os.path.abspath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from costmap_utils import build_costmap  # noqa: E402  (sys.path setup above)


class CostmapGenerator(Node):
    """ROS 2 node that turns ``LaserScan`` into ``OccupancyGrid``.

    Visualization only — does not publish ``cmd_vel`` (R1.4, R1.5).
    """

    def __init__(self) -> None:
        super().__init__('costmap_generator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('costmap_size', 3.0)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_publish_rate', 10.0)
        self.declare_parameter('costmap_stale_timeout', 1.0)
        self.declare_parameter('costmap_frame', '')
        self.declare_parameter('range_min_override', 0.0)
        # enable_costmap_viz: set false (default) in headless runs to avoid
        # serializing 3×10×3600 int8 values/sec with no RViz consumer.
        # Set true by followers.launch.py when rviz=true or ros_ui=true.
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

        # ── QoS profiles ────────────────────────────────────────────────────
        # Match the BEST_EFFORT/VOLATILE sensor QoS used in follower_node.py
        # and convoy_publisher.py so we receive scans from the standard
        # Gazebo bridge / lidar driver.
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Costmap consumers (RViz Map display, follower diagnostics) want a
        # reliable single-slot stream so a missed message doesn't show a
        # stale grid.
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions / publishers ──────────────────────────────────────
        # NOTE: no ``cmd_vel`` publisher is created anywhere in this node
        # (R1.4, R1.5) — this is a visualization-only producer.
        self._scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_cb, scan_qos,
        )
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, 'local_costmap', costmap_qos,
        )

        # ── Cached state (written by ``_scan_cb``, read by ``_publish``) ────
        self._scan = None        # type: LaserScan | None
        self._scan_time = None   # type: rclpy.time.Time | None
        # Most recently published Costmap; kept for inspection/tests.
        self._last_cm = None
        # Used to log a single info-level confirmation line on first publish.
        self._first_publish = True

        # ── Publish timer ───────────────────────────────────────────────────
        # Period derived from publish_rate; clamped to a sane minimum so a
        # bad parameter value can't divide by zero.
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

    # ── Callbacks ───────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan) -> None:
        """Cache the latest LaserScan and its receipt time."""
        self._scan = msg
        self._scan_time = self.get_clock().now()

    def _publish(self) -> None:
        """Publish-timer body.

        No-ops when ``enable_costmap_viz`` is false (headless mode) to avoid
        serializing 3×10×3600 int8 values/sec with no RViz consumer.

        Builds the Local_Costmap from the cached ``LaserScan`` (or an
        all-free grid when no scan has been received yet — R1.6 — or
        when the cached scan is older than ``stale_timeout`` — R1.7) by
        delegating to ``costmap_utils.build_costmap``, then populates an
        ``OccupancyGrid`` with matching geometry and publishes it on
        ``local_costmap`` (R1.1, R1.2, R1.3, R2.1).
        """
        if not self.enabled:
            return

        # ── Decide what to feed into build_costmap ─────────────────────────
        if self._scan is None:
            # R1.6: no scan ever received → publish an all-free grid by
            # passing empty ranges. ``build_costmap`` initialises every
            # cell to 0 (free) and only flips cells when at least one
            # valid return maps in-bounds, so the unused angle/range
            # arguments are inert.
            cm = build_costmap(
                [], 0.0, 0.0, 0.0, 0.0, self.size_m, self.resolution,
            )
        else:
            elapsed = (
                self.get_clock().now() - self._scan_time
            ).nanoseconds / 1e9
            if elapsed > self.stale_timeout:
                # R1.7: cached scan is older than ``stale_timeout`` →
                # clear all Occupied_Cells by publishing the all-free
                # grid (same empty-ranges call as the no-scan branch).
                cm = build_costmap(
                    [], 0.0, 0.0, 0.0, 0.0, self.size_m, self.resolution,
                )
            else:
                s = self._scan
                # ``range_min_override == 0.0`` disables the override;
                # otherwise raise the lower bound so very-near returns
                # (e.g. from the robot's own chassis) are filtered out
                # via ``is_valid_return``.
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
        # tasks can inspect it without re-running the publish path.
        self._last_cm = cm

        # ── Populate and publish the OccupancyGrid ─────────────────────────
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Prefer the explicit ``costmap_frame`` parameter; fall back to
        # the cached scan's frame_id; final fallback ``base_scan`` keeps
        # RViz happy when no scan has been received yet.
        msg.header.frame_id = (
            self.costmap_frame
            or (self._scan.header.frame_id if self._scan is not None else 'base_scan')
        )
        msg.info.resolution = float(cm.resolution)
        msg.info.width = int(cm.width)
        msg.info.height = int(cm.height)
        msg.info.origin.position.x = float(cm.origin_x)
        msg.info.origin.position.y = float(cm.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # identity quaternion
        # Occupancy values are 0 or 100 (both fit in int8); rclpy
        # serializes a list[int] into the int8[] wire field.
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
