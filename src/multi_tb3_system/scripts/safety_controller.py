#!/usr/bin/env python3
"""
safety_controller.py
====================
Safety layer for the Multi-TurtleBot3 convoy follower nodes.

Responsibilities:
  - Monitor the 180° front-half of the LaserScan for dangerously close obstacles
  - Issue emergency stop commands when something is within safe_distance
  - Apply gentle steering bias to avoid close obstacles on either side
  - Apply velocity limiting (clamp max linear and angular)

This module does NOT do any target tracking; it purely enforces hard limits.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import TwistStamped, Twist


# ─── Constants ────────────────────────────────────────────────────────────────
EMERGENCY_HALF_ANGLE_DEG = 45.0    # Check ±45° in front for emergency stop
STEER_HALF_ANGLE_DEG     = 60.0    # Check ±60° for steering bias
STEER_INFLUENCE_RANGE    = 0.8     # Obstacles within this range affect steering


class SafetyController:
    """
    Velocity safety layer.

    Use check_and_modify() to apply safety rules to any proposed velocity command
    before it is sent to the robot.
    """

    def __init__(
        self,
        safe_distance: float = 0.40,
        max_linear_vel: float = 0.22,
        max_angular_vel: float = 1.0,
    ) -> None:
        """
        Initialize the safety controller.

        Args:
            safe_distance:   Hard stop distance — if anything is closer than
                             this in front, set linear velocity to zero. [m]
            max_linear_vel:  Velocity clamp — never exceed this [m/s].
            max_angular_vel: Angular rate clamp [rad/s].
        """
        self.safe_distance   = safe_distance
        self.max_linear_vel  = max_linear_vel
        self.max_angular_vel = max_angular_vel

    # ──────────────────────────────────────────────────────────────────────────

    def check_and_modify(
        self,
        linear_x: float,
        angular_z: float,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_min: float = 0.12,
    ) -> Tuple[float, float]:
        """
        Apply safety rules and return the (possibly modified) velocity pair.

        Rules applied in order:
          1. Emergency stop: if any obstacle in ±EMERGENCY_HALF_ANGLE_DEG
             is closer than safe_distance → stop linear motion, allow recovery.
          2. Steering bias: if obstacles are close on one side, nudge away.
          3. Velocity clamping: ensure |linear| ≤ max_linear_vel
                                and |angular| ≤ max_angular_vel.

        Args:
            linear_x:         Proposed linear velocity [m/s]
            angular_z:        Proposed angular velocity [rad/s]
            ranges:           LaserScan.ranges array
            angle_min:        LaserScan.angle_min [rad]
            angle_increment:  LaserScan.angle_increment [rad/rad]
            range_min:        Minimum valid range [m]

        Returns:
            (safe_linear_x, safe_angular_z)
        """
        emergency_half = math.radians(EMERGENCY_HALF_ANGLE_DEG)
        steer_half     = math.radians(STEER_HALF_ANGLE_DEG)

        # Accumulate min distances per side for steering
        min_left  = float('inf')
        min_right = float('inf')
        emergency = False

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < range_min:
                continue
            angle = angle_min + i * angle_increment

            # ─── Emergency zone (narrow front cone) ───
            if abs(angle) <= emergency_half:
                if r < self.safe_distance:
                    emergency = True

            # ─── Steering zone (wider front cone) ────
            if abs(angle) <= steer_half and r < STEER_INFLUENCE_RANGE:
                if angle >= 0:
                    min_left  = min(min_left,  r)
                else:
                    min_right = min(min_right, r)

        # ─── Apply rules ──────────────────────────────
        if emergency:
            # Hard stop on linear; allow angular so robot can recover
            linear_x = 0.0

        else:
            # Gentle steering bias away from close side-obstacles
            if min_left < STEER_INFLUENCE_RANGE and min_left < min_right:
                # Something close on the left → steer right
                bias = (STEER_INFLUENCE_RANGE - min_left) / STEER_INFLUENCE_RANGE
                angular_z -= 0.5 * bias

            elif min_right < STEER_INFLUENCE_RANGE and min_right < min_left:
                # Something close on the right → steer left
                bias = (STEER_INFLUENCE_RANGE - min_right) / STEER_INFLUENCE_RANGE
                angular_z += 0.5 * bias

        # ─── Velocity clamping ────────────────────────
        linear_x  = max(-self.max_linear_vel,  min(linear_x,  self.max_linear_vel))
        angular_z = max(-self.max_angular_vel,  min(angular_z, self.max_angular_vel))

        return linear_x, angular_z

    # ──────────────────────────────────────────────────────────────────────────

    def get_emergency_stop_twist(self) -> Tuple[float, float]:
        """Return zero-velocity command (full stop)."""
        return 0.0, 0.0
