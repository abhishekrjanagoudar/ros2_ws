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
    ) -> Tuple[float, float, float, bool]:
        """
        Returns (min_left, min_right, min_front, is_emergency).
        Emergency detection now sees ALL obstacles including the leader - no predecessor filtering.
        """
        emergency_half = math.radians(EMERGENCY_HALF_ANGLE_DEG)
        steer_half     = math.radians(STEER_HALF_ANGLE_DEG)
        predecessor_half = math.radians(15.0)

        min_left  = float('inf')
        min_right = float('inf')
        min_front = float('inf')
        is_emerg  = False

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < range_min:
                continue
            angle = angle_min + i * angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # wrap to (-π, π]

            # Emergency cone (±45°) - NO FILTERING - detects all obstacles including leader
            if abs(angle) <= emergency_half and r < self.safe_distance:
                is_emerg = True

            # Front cone for ACC
            if abs(angle) <= predecessor_half:
                min_front = min(min_front, r)

            # Steering cone (±60°) - still uses predecessor filter for gentle bias
            if abs(angle) <= steer_half and r < STEER_INFLUENCE_RANGE:
                # Filter out predecessor for steering bias only
                if (self.predecessor_gap > 0.0
                        and abs(angle) <= predecessor_half):
                    lo = self.predecessor_gap * 0.3
                    hi = self.predecessor_gap * 1.2
                    if lo <= r <= hi:
                        continue
                
                if angle >= 0:
                    min_left  = min(min_left,  r)
                else:
                    min_right = min(min_right, r)

        return min_left, min_right, min_front, is_emerg

    def _apply_rules(
        self,
        linear_x: float,
        angular_z: float,
        min_left: float,
        min_right: float,
        min_front: float,
        is_emergency: bool,
    ) -> Tuple[float, float]:
        """Apply safety rules and clamp velocities given pre-computed side data."""
        if is_emergency:
            # Hard stop on forward linear; allow angular and reversing so robot can recover
            if linear_x > 0.0:
                linear_x = 0.0
        else:
            # Adaptive Cruise Control (ACC) based on front distance
            if self.predecessor_gap > 0.0 and min_front < self.predecessor_gap:
                if linear_x > 0.0:
                    if min_front <= self.safe_distance:
                        linear_x = 0.0
                    else:
                        ratio = (min_front - self.safe_distance) / (self.predecessor_gap - self.safe_distance)
                        linear_x *= ratio

            # Gentle steering bias away from close side-obstacles
            # Only apply when moving forward to avoid spinning in place when stopped behind a leader
            if linear_x > 0.01:
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
        """
        min_left, min_right, min_front, is_emerg = self._scan_sides(
            ranges, angle_min, angle_increment, range_min,
        )
        safe_lin, safe_ang = self._apply_rules(
            linear_x, angular_z, min_left, min_right, min_front, is_emerg,
        )
        return safe_lin, safe_ang, is_emerg

    def filter_predecessor_returns(
        self,
        ranges: list,
        angle_min: float,
        angle_increment: float,
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
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= PREDECESSOR_HALF_ANGLE and lo <= r <= hi:
                filtered[i] = float('inf')
        return filtered
