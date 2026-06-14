#!/usr/bin/env python3
"""
convoy_publisher.py — Publishes the leader's path for all followers to track.

This implements the Path-Based Convoy architecture. The leader (tb1) acts as
the orchestrator by publishing a single ground-truth nav_msgs/Path in the
``world`` frame. All followers subscribe to this shared path and track it with
Pure Pursuit, which eliminates the error accumulation of daisy-chained
follow-the-robot-ahead designs.

The leader's odom is anchored to ``world`` by a static transform at its spawn
position (zero rotation). The published path is therefore expressed in
``world`` coordinates by adding the leader's spawn offset to each odom pose.
For tb1 spawned at the origin the offset is (0, 0).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class ConvoyPublisher(Node):
    def __init__(self):
        super().__init__('convoy_publisher')

        self.declare_parameter('max_path_poses', 2000)
        self.declare_parameter('path_resolution', 0.05)
        self.declare_parameter('path_frame', 'world')
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)

        self.max_poses  = self.get_parameter('max_path_poses').value
        self.resolution = self.get_parameter('path_resolution').value
        self.frame      = self.get_parameter('path_frame').value
        self.off_x      = self.get_parameter('spawn_offset_x').value
        self.off_y      = self.get_parameter('spawn_offset_y').value

        self.path_pub = self.create_publisher(Path, 'convoy_path', 10)

        _sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, _sensor_qos,
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame

        # Publish path at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_path)

        self.get_logger().info(
            f"ConvoyPublisher | frame={self.frame} | "
            f"offset=({self.off_x:.2f},{self.off_y:.2f}) | "
            f"resolution={self.resolution}m"
        )

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.frame
        pose.pose = msg.pose.pose
        # Shift leader odom into the shared world frame.
        pose.pose.position.x += self.off_x
        pose.pose.position.y += self.off_y

        if not self.path_msg.poses:
            self.path_msg.poses.append(pose)
        else:
            last_pose = self.path_msg.poses[-1]
            dx = pose.pose.position.x - last_pose.pose.position.x
            dy = pose.pose.position.y - last_pose.pose.position.y
            if math.hypot(dx, dy) >= self.resolution:
                self.path_msg.poses.append(pose)

        # Keep path size bounded
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses = self.path_msg.poses[-self.max_poses:]

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = self.frame
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConvoyPublisher()
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
