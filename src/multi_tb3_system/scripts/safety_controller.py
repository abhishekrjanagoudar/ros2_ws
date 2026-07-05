#!/usr/bin/env python3
"""
Safety layer for the Multi-TurtleBot3 convoy follower nodes.
"""

from __future__ import annotations

import math
from typing import List, Tuple


# Constants
EMERGENCY_HALF_ANGLE_DEG = 45.0    # unchanged — cone angle stays the same
STEER_HALF_ANGLE_DEG     = 60.0    # unchanged — cone angle stays the same
STEER_INFLUENCE_RANGE    = 0.5     # reduced 50%: only steer-bias obstacles within 0.5m


class SafetyController:
    """
Velocity safety layer.
"""

    def __init__(
        self,
        safe_distance: float = 0.35,
        max_linear_vel: float = 0.22,
        max_angular_vel: float = 1.0,
        predecessor_gap: float = 0.0,
    ) -> None:
        """
Initialize the safety controller.
"""
        self.safe_distance   = safe_distance
        self.max_linear_vel  = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.predecessor_gap = predecessor_gap

    # Private helpers

    def _scan_sides(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        expected_bearing_deg: float = 0.0,
    ) -> Tuple[float, float, bool]:
        """
Returns (min_left, min_right, is_emergency).
Emergency detection now sees ALL obstacles including the leader - no predecessor filtering.
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

            # Emergency cone (±45°) - NO FILTERING - detects all obstacles including leader
            if abs(angle) <= emergency_half and r < self.safe_distance:
                is_emerg = True

            # Steering cone (±60°) - still uses predecessor filter for gentle bias
            if abs(angle) <= steer_half and r < STEER_INFLUENCE_RANGE:
                # Filter out predecessor for steering bias only
                PREDECESSOR_HALF_ANGLE = math.radians(15.0)
                expected_bearing_rad = math.radians(expected_bearing_deg)
                diff = angle - expected_bearing_rad
                wrapped_diff = math.atan2(math.sin(diff), math.cos(diff))
                if (self.predecessor_gap > 0.0
                        and abs(wrapped_diff) <= PREDECESSOR_HALF_ANGLE):
                    lo = self.predecessor_gap * 0.3
                    hi = self.predecessor_gap * 1.2
                    if lo <= r <= hi:
                        continue
                
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

        # Velocity clamping
        linear_x  = max(-self.max_linear_vel,  min(linear_x,  self.max_linear_vel))
        angular_z = max(-self.max_angular_vel,  min(angular_z, self.max_angular_vel))

        return linear_x, angular_z

    # Public API

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
        expected_bearing_deg: float = 0.0,
    ) -> Tuple[float, float, bool]:
        """
Single-pass safety check — preferred in tight control loops.
"""
        min_left, min_right, is_emerg = self._scan_sides(
            ranges, angle_min, angle_increment, range_min, expected_bearing_deg,
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

    def filter_predecessor_returns(
        self,
        ranges: list,
        angle_min: float,
        angle_increment: float,
        expected_bearing_deg: float = 0.0,
    ) -> list:
        """Return a copy of ranges with predecessor returns set to inf."""
        if self.predecessor_gap <= 0.0:
            return ranges
        PREDECESSOR_HALF_ANGLE = math.radians(15.0)
        lo = self.predecessor_gap * 0.3
        hi = self.predecessor_gap * 1.2
        filtered = list(ranges)
        for i, r in enumerate(filtered):
            angle = angle_min + i * angle_increment
            center_rad = math.radians(expected_bearing_deg)
            ang_diff = math.atan2(math.sin(angle - center_rad), math.cos(angle - center_rad))
            if abs(ang_diff) <= PREDECESSOR_HALF_ANGLE and lo <= r <= hi:
                filtered[i] = float('inf')
        return filtered

    # 

    def get_emergency_stop_twist(self) -> Tuple[float, float]:
        """Return zero-velocity command (full stop)."""
        return 0.0, 0.0
