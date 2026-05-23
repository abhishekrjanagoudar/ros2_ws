#!/usr/bin/env python3
"""
teleop_controller.py
====================
Burst-Mode (Hold-To-Move) keyboard teleoperation wrapper for Robot 1 (tb1).

FEATURE: BURST / HOLD-TO-MOVE
Unlike standard teleop nodes that latch velocities, this node requires the user 
to hold the key down to move. 
- Single key press → small movement burst
- Holding key      → continuous movement
- Releasing key    → publish zero velocity instantly (after a 0.3s timeout)

This ensures the robot does not run away and gives precise control. It natively
publishes geometry_msgs/msg/TwistStamped to /cmd_vel, which is required by 
the Gazebo Harmonic bridge.

Key bindings:
  i / w  → forward
  , / x  → backward
  j / a  → rotate left
  l / d  → rotate right
  k / s  → stop immediately
  q / z  → increase / decrease linear and angular speeds simultaneously
  e / c  → increase / decrease angular speed only
  Ctrl+C → exit
"""

from __future__ import annotations

import sys
import tty
import termios
import threading
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


# ─── Key mapping ──────────────────────────────────────────────────────────────

MOVE_BINDINGS = {
    'i': ( 1,  0),   ',': (-1,  0),   'j': ( 0,  1),   'l': ( 0, -1),   'k': ( 0,  0),
    'w': ( 1,  0),   'x': (-1,  0),   'a': ( 0,  1),   'd': ( 0, -1),   's': ( 0,  0),
    'I': ( 1,  0),   '<': (-1,  0),   'J': ( 0,  1),   'L': ( 0, -1),   'K': ( 0,  0),
    'W': ( 1,  0),   'X': (-1,  0),   'A': ( 0,  1),   'D': ( 0, -1),   'S': ( 0,  0),
}

SPEED_BINDINGS = {
    'q': (1.1,  1.1),   'z': (0.9,  0.9),   'e': (1.0,  1.1),   'c': (1.0,  0.9),
    'Q': (1.1,  1.1),   'Z': (0.9,  0.9),   'E': (1.0,  1.1),   'C': (1.0,  0.9),
}

MSG = """
╔════════════════════════════════════════════════════╗
║  TurtleBot3 Convoy — Burst-Mode Teleop (tb1)       ║
╠════════════════════════════════════════════════════╣
║  Movement (Hold to move, release to stop!):        ║
║    i / w        → forward                          ║
║    , / x        → backward                         ║
║    j / a        → rotate left                      ║
║    l / d        → rotate right                     ║
║    k / s        → force stop                       ║
╠════════════════════════════════════════════════════╣
║  Speed adjustment:                                 ║
║    q / z        → overall faster / slower          ║
║    e / c        → angular faster / slower          ║
╠════════════════════════════════════════════════════╣
║  CTRL+C         → quit                             ║
╚════════════════════════════════════════════════════╝
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
    Burst-mode keyboard teleop node for Robot 1 (Leader).
    Publishes TwistStamped to cmd_vel.
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
        print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)

        linear_x = 0.0
        angular_z = 0.0
        last_key_time = time.time()

        try:
            while rclpy.ok() and self.running:
                # Timeout must be shorter than the burst timeout
                key = get_key(timeout=0.05)
                now = time.time()

                if key in MOVE_BINDINGS:
                    lin_dir, ang_dir = MOVE_BINDINGS[key]
                    linear_x  = lin_dir * self.linear_speed
                    angular_z = ang_dir * self.angular_speed
                    last_key_time = now

                elif key in SPEED_BINDINGS:
                    lin_mult, ang_mult = SPEED_BINDINGS[key]
                    self.linear_speed  = min(self.max_lin, self.linear_speed  * lin_mult)
                    self.angular_speed = min(self.max_ang, self.angular_speed * ang_mult)
                    
                    # Apply new speed immediately if already moving
                    if linear_x != 0:
                        linear_x = math.copysign(self.linear_speed, linear_x)
                    if angular_z != 0:
                        angular_z = math.copysign(self.angular_speed, angular_z)
                    
                    last_key_time = now
                    print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)

                elif key == '\x03':   # Ctrl+C
                    break

                else:
                    # Burst timeout logic: if no valid key pressed for > 0.3s, stop.
                    # This overcomes the ~250ms OS typematic delay when a key is initially held.
                    if (now - last_key_time) > 0.3:
                        linear_x = 0.0
                        angular_z = 0.0

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
