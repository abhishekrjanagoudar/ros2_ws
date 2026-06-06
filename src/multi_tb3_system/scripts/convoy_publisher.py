#!/usr/bin/env python3
"""
convoy_publisher.py — Publishes the leader's path for all followers to track.

This implements the Convoy Architecture, replacing the daisy-chained
follower design. The leader (tb1) acts as the orchestrator by publishing
a single ground-truth nav_msgs/Path. All followers subscribe to this
shared path to eliminate error accumulation.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ConvoyPublisher(Node):
    def __init__(self):
        super().__init__('convoy_publisher')
        
        self.declare_parameter('max_path_poses', 500)
        self.declare_parameter('path_resolution', 0.05)
        
        self.max_poses = self.get_parameter('max_path_poses').value
        self.resolution = self.get_parameter('path_resolution').value
        
        self.path_pub = self.create_publisher(Path, 'convoy_path', 10)
        
        _sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            _sensor_qos,
        )
        
        self.path_msg = Path()
        
        # Publish path at 10Hz
        self.timer = self.create_timer(0.1, self.publish_path)

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
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
        self.path_msg.header.frame_id = 'world'
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
