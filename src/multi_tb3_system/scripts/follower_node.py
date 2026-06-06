#!/usr/bin/env python3
"""
follower_node.py
================
Autonomous convoy follower node.

Architecture
------------
CONVOY ARCHITECTURE:
This node receives the leader's exact historical path published as a
`nav_msgs/Path` topic. The follower computes its designated position in the 
convoy (e.g., 1.0m behind for tb2, 2.0m behind for tb3) and follows the path.

LiDAR is strictly used for emergency stopping and local collision avoidance
via the SafetyController. The follower does NOT use LiDAR to track the leader.

Parameters
----------
  target_distance           (float, 1.0 m)  ← Gap per convoy slot
  safe_distance             (float, 0.4 m)
  kp_linear                 (float, 0.8)
  kp_angular                (float, 2.0)
  max_linear_velocity       (float, 0.22 m/s)
  max_angular_velocity      (float, 1.0 rad/s)
  leader_ns                 (str,   'tb1')  ← The convoy orchestrator
"""

import math
import sys
import os
from typing import Optional, Tuple, List

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

def _world_to_local(world_x: float, world_y: float, robot_x: float, robot_y: float, robot_yaw: float) -> Tuple[float, float]:
    dx = world_x - robot_x
    dy = world_y - robot_y
    local_x =  dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
    local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
    return local_x, local_y

class FollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('follower_node')

        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_linear_velocity', 0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('leader_ns', 'tb1')

        self.target_distance = self.get_parameter('target_distance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.leader_ns = self.get_parameter('leader_ns').value

        self.safety = SafetyController(
            safe_distance=self.get_parameter('safe_distance').value,
            max_linear_vel=self.get_parameter('max_linear_velocity').value,
            max_angular_vel=self.get_parameter('max_angular_velocity').value,
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self._self_pose: Optional[Tuple[float, float, float]] = None
        self._convoy_path: List[Tuple[float, float]] = []

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.create_subscription(Odometry, 'odom', self._odom_callback, qos)
        self.create_subscription(Path, f'/{self.leader_ns}/convoy_path', self._path_callback, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, qos)

        try:
            self._my_idx = int(self.get_namespace().strip('/').replace('tb', ''))
        except ValueError:
            self._my_idx = 2
            
        self._my_offset_x = (self._my_idx - 1) * -1.0
        # The assigned gap from the leader (tb1)
        self._my_convoy_gap = (self._my_idx - 1) * self.target_distance
        
        self.get_logger().info(f"Convoy follower initialized. My idx: {self._my_idx}, Convoy gap: {self._my_convoy_gap}m")

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._self_pose = (p.x, p.y, _yaw_from_quaternion(q))

    def _path_callback(self, msg: Path) -> None:
        self._convoy_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._self_pose or not self._convoy_path:
            self._publish_twist(0.0, 0.0)
            return

        rx, ry, ryaw = self._self_pose
        # Shift follower's odom to the shared virtual world frame
        world_x = rx + self._my_offset_x
        world_y = ry
        world_yaw = ryaw

        # 1. Find gap waypoint: the point exactly _my_convoy_gap behind the leader
        gap_wp = self._convoy_path[-1]
        dist_from_leader = 0.0
        gap_idx = len(self._convoy_path) - 1
        path_is_long_enough = False

        for i in range(len(self._convoy_path) - 2, -1, -1):
            p1 = self._convoy_path[i]
            p2 = self._convoy_path[i + 1]
            dist_from_leader += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if dist_from_leader >= self._my_convoy_gap:
                gap_wp = p1
                gap_idx = i
                path_is_long_enough = True
                break

        # BUG-3 FIX: If path hasn't built up enough history, hold position
        if not path_is_long_enough:
            self._publish_twist(0.0, 0.0)
            return

        # 2. Find closest point on path (up to gap_idx) to current position
        closest_dist = float('inf')
        closest_idx = 0
        for i in range(gap_idx + 1):
            wx, wy = self._convoy_path[i]
            d = math.hypot(wx - world_x, wy - world_y)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        # 3. Pure pursuit lookahead point (for steering)
        lookahead = 0.4
        wp_wx, wp_wy = self._convoy_path[closest_idx]
        accum = 0.0
        for i in range(closest_idx, gap_idx):
            p1 = self._convoy_path[i]
            p2 = self._convoy_path[i + 1]
            accum += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if accum >= lookahead:
                wp_wx, wp_wy = p2
                break
        else:
            wp_wx, wp_wy = gap_wp

        # Convert lookahead waypoint to local frame (steering)
        local_x, local_y = _world_to_local(wp_wx, wp_wy, world_x, world_y, world_yaw)
        angle_error = math.atan2(local_y, local_x)

        # Convert gap waypoint to local frame (speed)
        gap_local_x, gap_local_y = _world_to_local(
            gap_wp[0], gap_wp[1], world_x, world_y, world_yaw
        )

        # BUG-2 FIX: Use SIGNED gap_local_x for speed (not unsigned hypot)
        # Dead zone: if within 0.15m of target, stop
        linear_x = 0.0
        angular_z = 0.0
        _dead_zone = 0.15

        if gap_local_x > _dead_zone:
            # Target is ahead: drive forward proportionally
            linear_x = self.kp_linear * gap_local_x
            angular_z = self.kp_angular * angle_error
        elif gap_local_x < -_dead_zone:
            # Target is behind: allow very slow reverse to correct overshoot
            linear_x = max(self.kp_linear * gap_local_x, -0.05)
            angular_z = self.kp_angular * angle_error
        # else: within dead zone, stay still (linear_x = angular_z = 0)

        # Safety override using local LiDAR
        linear_x, angular_z = self.safety.check_and_modify(
            linear_x=linear_x,
            angular_z=angular_z,
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min if msg.range_min > 0 else 0.12,
        )

        self._publish_twist(linear_x, angular_z)

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
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
                node._publish_twist(0.0, 0.0)
                print('FollowerNode shutting down — robot stopped.')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
