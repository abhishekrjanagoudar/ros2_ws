#!/usr/bin/env python3
"""
Burst-Mode (Hold-To-Move) keyboard teleoperation wrapper.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist


# Key mapping

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
║             Burst-Mode Teleop Controller           ║
╠════════════════════════════════════════════════════╣
║                                                    ║
║  Moving around (Hold key to move):                 ║
║        w                   i                       ║
║   a    s    d      OR      j    k    l             ║
║        x                   ,                       ║
║                                                    ║
║  w/i : Forward                                     ║
║  x/, : Backward                                    ║
║  a/j : Turn Left                                   ║
║  d/l : Turn Right                                  ║
║  s/k : Force Stop                                  ║
║                                                    ║
║  Speed Control:                                    ║
║  q/z : Increase / Decrease all speeds by 10%       ║
║  e/c : Increase / Decrease angular speed by 10%    ║
║                                                    ║
║  CTRL-C to quit                                    ║
╚════════════════════════════════════════════════════╝
"""

SPEED_MSG = "\rLinear: {lin:.2f} m/s  |  Angular: {ang:.2f} rad/s    "


# Keyboard reader

def get_key(timeout: float = 0.05) -> str:
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


# Teleop Node

class TeleopController(Node):
    """
Burst-mode keyboard teleop node.
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

        # Explicit QoS: Reliable, Volatile, Depth 10. Matches ros_gz_bridge default.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        # Internal state
        self.linear_speed  = 0.10
        self.angular_speed = 0.50
        
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_key_time = time.time()
        self.running = True

        # Timer for fixed-rate continuous publishing (20 Hz)
        publish_rate_hz = 20.0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._timer_callback)

        self.get_logger().info("Teleop Controller Started. Publishing at 20Hz.")

    def _timer_callback(self) -> None:
        """Publish Twist continuously. Zero out targets if key released."""
        now = time.time()
        
        # Burst timeout logic: if no valid key pressed for > 0.15s, stop.
        if (now - self.last_key_time) > 0.15:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            
        self._publish_twist(self.target_linear_x, self.target_angular_z)

    def run(self) -> None:
        """Main loop: read keys and update internal target commands."""
        print(MSG)
        print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)

        try:
            while rclpy.ok() and self.running:
                key = get_key(timeout=0.05)

                if key in MOVE_BINDINGS:
                    lin_dir, ang_dir = MOVE_BINDINGS[key]
                    self.target_linear_x  = lin_dir * self.linear_speed
                    self.target_angular_z = ang_dir * self.angular_speed
                    self.last_key_time = time.time()

                elif key in SPEED_BINDINGS:
                    lin_mult, ang_mult = SPEED_BINDINGS[key]
                    self.linear_speed  = min(self.max_lin, self.linear_speed  * lin_mult)
                    self.angular_speed = min(self.max_ang, self.angular_speed * ang_mult)
                    
                    # Apply new speed immediately if already moving
                    if self.target_linear_x != 0:
                        self.target_linear_x = math.copysign(self.linear_speed, self.target_linear_x)
                    if self.target_angular_z != 0:
                        self.target_angular_z = math.copysign(self.angular_speed, self.target_angular_z)
                    
                    self.last_key_time = time.time()
                    print(SPEED_MSG.format(lin=self.linear_speed, ang=self.angular_speed), end='', flush=True)

                elif key == '\x03':   # Ctrl+C
                    break

        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")
        finally:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            # Publish 0 multiple times on shutdown to ensure receipt
            for _ in range(5):
                self._publish_twist(0.0, 0.0)
                time.sleep(0.05)
            self.get_logger().info("TeleopController stopped — robot halted.")

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)


# Entry point

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopController()

    # Spin rclpy in a background thread so the timer fires concurrently
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
