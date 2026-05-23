#!/usr/bin/env python3
"""
teleop_controller.py
====================
Keyboard teleoperation wrapper for Robot 1 (tb1) in the Multi-TurtleBot3
convoy system.

This node reads keyboard input and publishes TwistStamped velocity commands
to /tb1/cmd_vel so that the leader robot can be driven manually.

Key bindings (same as teleop_twist_keyboard):
  w / x  → increase / decrease linear velocity
  a / d  → rotate left / right
  s      → stop immediately
  q / z  → increase / decrease linear and angular speeds simultaneously
  e / c  → increase / decrease angular speed only
  Ctrl+C → exit

Speed settings (adjustable at runtime):
  Linear speed step : 0.01 m/s
  Angular speed step: 0.1  rad/s
  Linear  max: 0.22 m/s (TurtleBot3 Burger limit)
  Angular max: 1.0  rad/s

Note:
  The node publishes TwistStamped (not Twist) because the Gazebo Sim bridge
  for DiffDrive in Jazzy expects geometry_msgs/msg/TwistStamped on the
  ROS side when using ros_gz_bridge.
"""

from __future__ import annotations

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


# ─── Key mapping ──────────────────────────────────────────────────────────────

MOVE_BINDINGS = {
    'w': ( 1,  0),   # forward
    'x': (-1,  0),   # backward
    'a': ( 0,  1),   # rotate left
    'd': ( 0, -1),   # rotate right
    's': ( 0,  0),   # stop
}

SPEED_BINDINGS = {
    'q': (1.1,  1.1),   # increase both
    'z': (0.9,  0.9),   # decrease both
    'e': (1.0,  1.1),   # increase angular
    'c': (1.0,  0.9),   # decrease angular
}

MSG = """
╔══════════════════════════════════════╗
║  TurtleBot3 Convoy — Teleop (tb1)   ║
╠══════════════════════════════════════╣
║  Movement:                           ║
║    w        → forward                ║
║    x        → backward               ║
║    a / d    → rotate left / right   ║
║    s        → stop                   ║
╠══════════════════════════════════════╣
║  Speed adjustment:                   ║
║    q / z    → faster / slower        ║
║    e / c    → angular faster/slower ║
╠══════════════════════════════════════╣
║  CTRL+C     → quit                   ║
╚══════════════════════════════════════╝
"""

SPEED_MSG = "\rLinear: {lin:.2f} m/s  |  Angular: {ang:.2f} rad/s    "


# ─── Keyboard reader ──────────────────────────────────────────────────────────

def get_key(timeout: float = 0.1) -> str:
    """Read a single keypress in raw mode (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        import select
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ─── Teleop Node ──────────────────────────────────────────────────────────────

class TeleopController(Node):
    """
    Keyboard teleop node for Robot 1 (Leader).

    Publishes TwistStamped to /tb1/cmd_vel.
    The namespace /tb1 is set via the launch file so this node publishes
    to cmd_vel which resolves to /tb1/cmd_vel.
    """

    def __init__(self) -> None:
        super().__init__('teleop_controller')

        self.declare_parameter('max_linear_velocity',  0.22)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('linear_speed_step',    0.01)
        self.declare_parameter('angular_speed_step',   0.1)

        self.max_lin = self.get_parameter('max_linear_velocity').value
        self.max_ang = self.get_parameter('max_angular_velocity').value
        self.lin_step = self.get_parameter('linear_speed_step').value
        self.ang_step = self.get_parameter('angular_speed_step').value

        # Publisher to /cmd_vel — namespace resolves to /tb1/cmd_vel
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Initial state
        self.linear_speed  = 0.10
        self.angular_speed = 0.50
        self.running = True

    def run(self) -> None:
        """Main loop: read keys and publish commands."""
        print(MSG)
        print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed))

        linear_x = 0.0
        angular_z = 0.0

        try:
            while rclpy.ok() and self.running:
                key = get_key(timeout=0.05)

                if key in MOVE_BINDINGS:
                    lin_dir, ang_dir = MOVE_BINDINGS[key]
                    linear_x  = lin_dir * self.linear_speed
                    angular_z = ang_dir * self.angular_speed

                elif key in SPEED_BINDINGS:
                    lin_mult, ang_mult = SPEED_BINDINGS[key]
                    self.linear_speed  = min(self.max_lin, self.linear_speed  * lin_mult)
                    self.angular_speed = min(self.max_ang, self.angular_speed * ang_mult)
                    # Keep current motion direction, update magnitude
                    if linear_x != 0:
                        linear_x = math.copysign(self.linear_speed,  linear_x)
                    if angular_z != 0:
                        angular_z = math.copysign(self.angular_speed, angular_z)
                    print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)

                elif key == '\x03':   # Ctrl+C
                    break

                elif key == '':
                    # No key pressed — hold current velocity
                    pass

                self._publish_twist(linear_x, angular_z)

        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")
        finally:
            self._publish_twist(0.0, 0.0)
            self.get_logger().info("TeleopController stopped — tb1 halted.")

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x  = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)


# ─── Entry point ──────────────────────────────────────────────────────────────

import math   # placed here to avoid circular import issues in module header

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopController()

    # Spin rclpy in a background thread so the main thread can do keyboard I/O
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
