#!/usr/bin/env python3
"""
Square motion drift logger — executes a repeatable 2m×2m square
(4x rotate 90 CW + move 2m), logging both ROS2 odometry and Gazebo
ground-truth pose every 2 seconds AND at each phase boundary.

Usage:
    python3 drift_logger_square.py /tb1 tb1

Args:
    ros_ns       ROS2 namespace for the leader (e.g. /tb1)
    gz_model     Gazebo model name for the leader (e.g. tb1)
    --out        CSV output path (default /tmp/drift_square.csv)
"""

import argparse
import csv
import math
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(z: float, w: float) -> float:
    return math.degrees(2 * math.atan2(z, w))


class DriftLogger(Node):
    def __init__(self, ros_ns: str, gz_model: str, direction: str, out_path: str):
        super().__init__('drift_logger_square')

        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.ros_ns   = ros_ns
        self.gz_model = gz_model
        self.direction = direction
        self.out_path = out_path

        self.robots = ['tb1', 'tb2', 'tb3']
        self.all_odoms = {m: (None, None, None) for m in self.robots}
        
        # Background gz query state
        self.latest_gz = {m: (None, None, None) for m in self.robots}
        self.gz_thread_running = True
        self.gz_thread = threading.Thread(target=self._gz_query_loop, daemon=True)
        self.gz_thread.start()

        # Create odom subscriptions for all robots
        for m in self.robots:
            self.create_subscription(Odometry, f'/{m}/odom', 
                                     lambda msg, r=m: self._odom_cb(msg, r), 10)
            
        self.cmd_vel_pub = self.create_publisher(Twist, f'{ros_ns}/cmd_vel', 10)

        # 2-second periodic log
        self.create_timer(2.0, self._timer_log)

        self.csv_file = open(out_path, 'w', newline='')
        self.writer   = csv.writer(self.csv_file)
        
        header = ['t_sim', 'motion_phase']
        for m in self.robots:
            header.extend([f'{m}_odom_x', f'{m}_odom_y', f'{m}_odom_yaw_deg',
                           f'{m}_gz_x', f'{m}_gz_y', f'{m}_gz_yaw_deg',
                           f'{m}_pos_drift_m', f'{m}_yaw_drift_deg'])
        self.writer.writerow(header)
        self.csv_file.flush()

        self.start_sim_t = None
        self.get_logger().info(f'Logging to {out_path}. Square motion starting shortly...')

    # ---------- clock ----------

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self) -> float:
        if self.start_sim_t is None:
            self.start_sim_t = self._sim_now()
        return self._sim_now() - self.start_sim_t

    # ---------- odom / gz ----------

    def _odom_cb(self, msg: Odometry, robot_name: str):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.z, q.w)
        self.all_odoms[robot_name] = (p.x, p.y, yaw)

    def _gz_query_loop(self):
        """Background thread: continuously poll `gz model -m <name> -p` for all robots."""
        while self.gz_thread_running:
            for m in self.robots:
                if not self.gz_thread_running:
                    break
                try:
                    result = subprocess.run(
                        ['gz', 'model', '-m', m, '-p'],
                        capture_output=True, text=True, timeout=2.0,
                    )
                    text = result.stdout
                    groups = re.findall(r'\[([^\]]+)\]', text)
                    numeric = []
                    for g in groups:
                        parts = g.split()
                        if len(parts) == 3:
                            try:
                                numeric.append([float(v) for v in parts])
                            except ValueError:
                                pass
                    if len(numeric) >= 2:
                        xyz = numeric[-2]
                        rpy = numeric[-1]
                        self.latest_gz[m] = (xyz[0], xyz[1], math.degrees(rpy[2]))
                except Exception:
                    pass
            time.sleep(0.1)

    def _query_gazebo_poses(self):
        return self.latest_gz.copy()

    def _log_sample(self, phase: str):
        # Ensure we have odom for leader to drive
        leader_odom = self.all_odoms[self.gz_model]
        if leader_odom[0] is None:
            return
            
        gz = self._query_gazebo_poses()
        
        t = self._elapsed()
        row = [f'{t:.2f}', phase]
        log_str = f'{phase:20} t={t:6.1f}s | '
        
        for m in self.robots:
            o_x, o_y, o_yaw = self.all_odoms[m]
            g_x, g_y, g_yaw = gz[m]
            
            if o_x is not None and g_x is not None:
                pos_drift = math.hypot(g_x - o_x, g_y - o_y)
                yaw_drift = ((g_yaw - o_yaw + 180) % 360) - 180
                
                row.extend([
                    f'{o_x:.4f}', f'{o_y:.4f}', f'{o_yaw:.2f}',
                    f'{g_x:.4f}', f'{g_y:.4f}', f'{g_yaw:.2f}',
                    f'{pos_drift:.4f}', f'{yaw_drift:.2f}'
                ])
                log_str += f'{m}: gz=({g_x:6.3f},{g_y:6.3f},{g_yaw:6.1f}°) rviz=({o_x:6.3f},{o_y:6.3f},{o_yaw:6.1f}°) drift={pos_drift:.3f}m | '
            else:
                row.extend([''] * 8)
                log_str += f'{m}: waiting... | '

        self.writer.writerow(row)
        self.csv_file.flush()
        self.get_logger().info(log_str)

    def _timer_log(self):
        """Periodic update every 2 seconds."""
        if self.start_sim_t is not None:
            self._log_sample('timer_update')

    # ---------- motion primitives ----------

    def _rotate_cw_90(self):
        self.get_logger().info('Starting rotation (CW 90)...')
        while self.all_odoms[self.gz_model][2] is None:
            rclpy.spin_once(self, timeout_sec=0.02)

        start_yaw_rad = math.radians(self.all_odoms[self.gz_model][2])
        target_delta  = -math.pi / 2
        kp = 1.8; omega_max = 0.6; omega_min = 0.08
        tol = math.radians(0.5)
        zero = Twist()

        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            curr = math.radians(self.all_odoms[self.gz_model][2])
            delta     = (curr - start_yaw_rad + math.pi) % (2 * math.pi) - math.pi
            remaining = (target_delta - delta + math.pi) % (2 * math.pi) - math.pi
            if abs(remaining) <= tol:
                break
            omega = kp * remaining
            omega = max(-omega_max, min(-omega_min, omega)) if remaining < 0 \
                else max(omega_min, min(omega_max, omega))
            twist = Twist(); twist.angular.z = omega
            self.cmd_vel_pub.publish(twist)

        for _ in range(10):
            self.cmd_vel_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.15)
        rclpy.spin_once(self, timeout_sec=0.1)
        self._log_sample('rotate_cw_90_done')

    def _rotate_ccw_90(self):
        self.get_logger().info('Starting rotation (CCW 90)...')
        while self.all_odoms[self.gz_model][2] is None:
            rclpy.spin_once(self, timeout_sec=0.02)

        start_yaw_rad = math.radians(self.all_odoms[self.gz_model][2])
        target_delta  = math.pi / 2
        kp = 1.8; omega_max = 0.6; omega_min = 0.08
        tol = math.radians(0.5)
        zero = Twist()

        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            curr = math.radians(self.all_odoms[self.gz_model][2])
            delta     = (curr - start_yaw_rad + math.pi) % (2 * math.pi) - math.pi
            remaining = (target_delta - delta + math.pi) % (2 * math.pi) - math.pi
            if abs(remaining) <= tol:
                break
            omega = kp * remaining
            omega = max(-omega_max, min(-omega_min, omega)) if remaining < 0 \
                else max(omega_min, min(omega_max, omega))
            twist = Twist(); twist.angular.z = omega
            self.cmd_vel_pub.publish(twist)

        for _ in range(10):
            self.cmd_vel_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.15)
        rclpy.spin_once(self, timeout_sec=0.1)
        self._log_sample('rotate_ccw_90_done')

    def _move_forward_2m(self):
        self.get_logger().info('Starting forward motion (2m)...')
        while self.all_odoms[self.gz_model][0] is None:
            rclpy.spin_once(self, timeout_sec=0.02)

        start_x, start_y = self.all_odoms[self.gz_model][0], self.all_odoms[self.gz_model][1]
        kp = 1.5; v_max = 0.20; v_min = 0.05; tol = 0.01
        zero = Twist()

        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            travelled = math.hypot(self.all_odoms[self.gz_model][0] - start_x, self.all_odoms[self.gz_model][1] - start_y)
            remaining = 2.0 - travelled
            if remaining <= tol:
                break
            v = max(v_min, min(v_max, kp * remaining))
            twist = Twist(); twist.linear.x = v
            self.cmd_vel_pub.publish(twist)

        for _ in range(10):
            self.cmd_vel_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.15)
        rclpy.spin_once(self, timeout_sec=0.1)
        self._log_sample('move_forward_2m_done')

    # ---------- top-level run ----------

    def run(self):
        try:
            self.get_logger().info('Waiting for cmd_vel connection...')
            while self.cmd_vel_pub.get_subscription_count() == 0:
                rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(2.0)

            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)
            self._elapsed()
            self._log_sample('start')

            if self.direction == 'right':
                for side in range(1, 5):
                    self.get_logger().info(f'=== CW SIDE {side}/4 ===')
                    self._rotate_cw_90()
                    self._move_forward_2m()
            elif self.direction == 'left':
                for side in range(1, 5):
                    self.get_logger().info(f'=== CCW SIDE {side}/4 ===')
                    self._rotate_ccw_90()
                    self._move_forward_2m()

            self.get_logger().info('=== MOTION COMPLETE — settling ===')
            time.sleep(1.0)
            last = 0.0
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                now = self._elapsed()
                if now - last >= 1.0:
                    self._log_sample('final')
                    last = now
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        finally:
            self.gz_thread_running = False
            zero = Twist()
            try:
                if rclpy.ok():
                    self.cmd_vel_pub.publish(zero)
            except Exception:
                pass
            self.csv_file.close()
            self.get_logger().info(f'Saved log to {self.out_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Square motion drift logger: 4x (rotate 90 + move 2m)'
    )
    parser.add_argument('ros_ns',   help='ROS2 namespace, e.g. /tb1')
    parser.add_argument('gz_model', help='Gazebo model name, e.g. tb1')
    parser.add_argument('direction', choices=['left', 'right'], help='Direction to turn (left=CCW, right=CW)')
    parser.add_argument('--out', default='/tmp/drift_square.csv',
                        help='CSV output path')
    args = parser.parse_args()

    rclpy.init()
    node = DriftLogger(args.ros_ns, args.gz_model, args.direction, args.out)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
