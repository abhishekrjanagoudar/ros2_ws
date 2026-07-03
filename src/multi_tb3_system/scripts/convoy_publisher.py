#!/usr/bin/env python3
"""
convoy_publisher.py — Publishes the leader's path for all followers to track.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros


class ConvoyPublisher(Node):
    def __init__(self):
        super().__init__('convoy_publisher')

        self.declare_parameter('max_path_poses', 5000)
        self.declare_parameter('path_resolution', 0.01)
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.max_poses  = self.get_parameter('max_path_poses').value
        self.resolution = self.get_parameter('path_resolution').value
        self.frame      = self.get_parameter('path_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Resolve full base frame (e.g. 'tb1/base_footprint')
        ns = self.get_namespace().strip('/')
        self.full_base_frame = f"{ns}/{self.base_frame}" if ns else self.base_frame

        self.path_pub = self.create_publisher(Path, 'convoy_path', 10)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame

        # Publish path and lookup TF at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(
            f"ConvoyPublisher | map_frame={self.frame} | base_frame={self.full_base_frame} | "
            f"resolution={self.resolution}m"
        )

    def timer_callback(self):
        now = self.get_clock().now()
        
        # 1. Look up current global pose
        try:
            trans = self.tf_buffer.lookup_transform(
                self.frame,
                self.full_base_frame,
                rclpy.time.Time()
            )
            
            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = self.frame
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            # 2. Append to path if moved enough
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
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=2.0)
            pass

        # 3. Publish path
        self.path_msg.header.stamp = now.to_msg()
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
