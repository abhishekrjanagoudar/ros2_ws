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

Public API
----------
check_and_modify(linear_x, angular_z, ranges, ...)
    Apply safety rules; return (safe_linear_x, safe_angular_z).
    Preserved for callers that do not need the emergency flag.

check_and_modify_ex(linear_x, angular_z, ranges, ...)
    Single-pass variant; return (safe_linear_x, safe_angular_z, is_emergency).
    Preferred in follower_node._control_loop() to avoid scanning the LiDAR
    array twice per 50 Hz cycle (once for the state-machine input and once
    inside the hard override).

is_emergency(ranges, ...)
    Return True if an obstacle is inside the ±45° front cone.
"""

from __future__ import annotations

import math
from typing import List, Tuple


# ─── Constants ────────────────────────────────────────────────────────────────
EMERGENCY_HALF_ANGLE_DEG = 45.0    # Check ±45° in front for emergency stop
STEER_HALF_ANGLE_DEG     = 60.0    # Check ±60° for steering bias
STEER_INFLUENCE_RANGE    = 1.0     # Obstacles within this range affect steering (> convoy_spacing=0.5m)


class SafetyController:
    """
    Velocity safety layer.

    Use check_and_modify_ex() in tight control loops (single LiDAR pass).
    Use check_and_modify() where backward compatibility is required.
    """

    def __init__(
        self,
        safe_distance: float = 0.35,
        max_linear_vel: float = 0.22,
        max_angular_vel: float = 1.0,
    ) -> None:
        """
        Initialize the safety controller.

        Args:
            safe_distance:   Hard stop distance — if anything is closer than
                             this in front, set linear velocity to zero. [m]
                             Must be strictly less than convoy_spacing (0.5 m)
                             so the robot ahead at the nominal gap does not
                             trigger an emergency stop.
            max_linear_vel:  Velocity clamp — never exceed this [m/s].
            max_angular_vel: Angular rate clamp [rad/s].
        """
        self.safe_distance   = safe_distance
        self.max_linear_vel  = max_linear_vel
        self.max_angular_vel = max_angular_vel

    # ── Private helpers ───────────────────────────────────────────────────────

    def _scan_sides(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
    ) -> Tuple[float, float, bool]:
        """Single pass over the LaserScan array.

        Returns
        -------
        (min_left, min_right, is_emergency)
            min_left / min_right : closest valid range in the ±60° steering
                cone on each side; float('inf') if nothing is within
                STEER_INFLUENCE_RANGE.
            is_emergency : True if any valid return inside the ±45° front cone
                is closer than self.safe_distance.
        """
        emergency_half = math.radians(EMERGENCY_HALF_ANGLE_DEG)
        steer_half     = math.radians(STEER_HALF_ANGLE_DEG)

        min_left  = float('inf')
        min_right = float('inf')
        is_emerg  = False

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < range_min:
                continue
            angle = angle_min + i * angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # wrap to (-π, π]

            # ─── Emergency cone (±45°) ────────────────────────────────────
            if abs(angle) <= emergency_half and r < self.safe_distance:
                is_emerg = True

            # ─── Steering cone (±60°) ─────────────────────────────────────
            if abs(angle) <= steer_half and r < STEER_INFLUENCE_RANGE:
                if angle >= 0:
                    min_left  = min(min_left,  r)
                else:
                    min_right = min(min_right, r)

        return min_left, min_right, is_emerg

    def _apply_rules(
        self,
        linear_x: float,
        angular_z: float,
        min_left: float,
        min_right: float,
        is_emergency: bool,
    ) -> Tuple[float, float]:
        """Apply safety rules and clamp velocities given pre-computed side data."""
        if is_emergency:
            # Hard stop on linear; allow angular so robot can recover
            linear_x = 0.0
        else:
            # Gentle steering bias away from close side-obstacles
            if min_left < STEER_INFLUENCE_RANGE and min_left < min_right:
                bias = (STEER_INFLUENCE_RANGE - min_left) / STEER_INFLUENCE_RANGE
                angular_z -= 0.5 * bias
            elif min_right < STEER_INFLUENCE_RANGE and min_right < min_left:
                bias = (STEER_INFLUENCE_RANGE - min_right) / STEER_INFLUENCE_RANGE
                angular_z += 0.5 * bias

        # ─── Velocity clamping ────────────────────────────────────────────
        linear_x  = max(-self.max_linear_vel,  min(linear_x,  self.max_linear_vel))
        angular_z = max(-self.max_angular_vel,  min(angular_z, self.max_angular_vel))

        return linear_x, angular_z

    # ── Public API ────────────────────────────────────────────────────────────

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
          3. Velocity clamping: ensure |linear| <= max_linear_vel
                                and |angular| <= max_angular_vel.

        Args:
            linear_x:         Proposed linear velocity [m/s]
            angular_z:        Proposed angular velocity [rad/s]
            ranges:           LaserScan.ranges array
            angle_min:        LaserScan.angle_min [rad]
            angle_increment:  LaserScan.angle_increment [rad/rad]
            range_min:        Minimum valid range [m]

        Returns:
            (safe_linear_x, safe_angular_z)

        Note:
            Prefer check_and_modify_ex() in hot control loops to avoid
            scanning the LiDAR array twice per cycle.
        """
        min_left, min_right, is_emerg = self._scan_sides(
            ranges, angle_min, angle_increment, range_min,
        )
        return self._apply_rules(linear_x, angular_z, min_left, min_right, is_emerg)

    def check_and_modify_ex(
        self,
        linear_x: float,
        angular_z: float,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_min: float = 0.12,
    ) -> Tuple[float, float, bool]:
        """
        Single-pass safety check — preferred in tight control loops.

        Scans the LiDAR array exactly once to determine both the emergency flag
        (needed by the state machine) and the steering / clamping adjustments
        (the hard override), returning all three results together.

        Args:
            Same as check_and_modify().

        Returns:
            (safe_linear_x, safe_angular_z, is_emergency)
                safe_linear_x  : velocity after safety rules and clamping [m/s]
                safe_angular_z : angular rate after safety rules and clamping [rad/s]
                is_emergency   : True iff an obstacle was inside the ±45° front
                                 cone at a distance < safe_distance.
        """
        min_left, min_right, is_emerg = self._scan_sides(
            ranges, angle_min, angle_increment, range_min,
        )
        safe_lin, safe_ang = self._apply_rules(
            linear_x, angular_z, min_left, min_right, is_emerg,
        )
        return safe_lin, safe_ang, is_emerg

    def is_emergency(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_min: float = 0.12,
    ) -> bool:
        """Return True if an obstacle is inside the front emergency cone."""
        emergency_half = math.radians(EMERGENCY_HALF_ANGLE_DEG)
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < range_min:
                continue
            angle = angle_min + i * angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= emergency_half and r < self.safe_distance:
                return True
        return False

    # ──────────────────────────────────────────────────────────────────────────

    def get_emergency_stop_twist(self) -> Tuple[float, float]:
        """Return zero-velocity command (full stop)."""
        return 0.0, 0.0
