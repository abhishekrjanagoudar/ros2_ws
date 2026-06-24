#!/usr/bin/env python3
"""
Pure helpers for the LiDAR-derived local occupancy grid (Local_Costmap)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass
class Costmap:
    """
Attributes
"""

    data: list[int]
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


def is_valid_return(r: float, range_min: float, range_max: float) -> bool:
    """
A return is valid when its range value is finite (not NaN, not
"""
    return math.isfinite(r) and range_min <= r <= range_max


def local_to_cell(
    cm: Costmap, x: float, y: float
) -> Optional[tuple[int, int]]:
    """
The mapping uses the standard occupancy-grid convention:
"""
    col = math.floor((x - cm.origin_x) / cm.resolution)
    row = math.floor((y - cm.origin_y) / cm.resolution)
    if 0 <= row < cm.height and 0 <= col < cm.width:
        return (row, col)
    return None


def is_occupied(cm: Costmap, row: int, col: int) -> bool:
    """
Performs explicit bounds checking against ``cm.height`` and
"""
    if not (0 <= row < cm.height and 0 <= col < cm.width):
        return False
    return cm.data[row * cm.width + col] == 100


def build_costmap(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    size_m: float = 3.0,
    resolution: float = 0.05,
) -> Costmap:
    """
Constructs a square row-major occupancy grid centered on the robot
"""
    width = round(size_m / resolution)
    height = width
    origin_x = -size_m / 2.0
    origin_y = -size_m / 2.0
    data = [0] * (width * height)
    cm = Costmap(
        data=data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
    )
    for i, r in enumerate(ranges):
        if not is_valid_return(r, range_min, range_max):
            continue
        angle = angle_min + i * angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        cell = local_to_cell(cm, x, y)
        if cell is not None:
            row, col = cell
            data[row * width + col] = 100
    return cm


def segment_hits_occupied(
    cm: Costmap,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> bool:
    """
The segment is sampled at a spacing of at most ``resolution / 2`` so
"""
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    n = max(1, math.ceil(length / (cm.resolution / 2.0)))
    for k in range(n + 1):
        t = k / n
        x = x0 + t * dx
        y = y0 + t * dy
        cell = local_to_cell(cm, x, y)
        if cell is not None and is_occupied(cm, cell[0], cell[1]):
            return True
    return False


def is_goal_blocked(cm: Costmap, gx: float, gy: float) -> bool:
    """
The goal point is expressed in the robot's local costmap frame
"""
    cell = local_to_cell(cm, gx, gy)
    if cell is not None and is_occupied(cm, cell[0], cell[1]):
        return True
    return segment_hits_occupied(cm, 0.0, 0.0, gx, gy)


def free_counts_per_side(
    cm: Costmap, forward_only: bool = True
) -> tuple[int, int]:
    """
Walks every cell ``(row, col)`` of the grid and computes its center in
"""
    left = 0
    right = 0
    for row in range(cm.height):
        cy = cm.origin_y + (row + 0.5) * cm.resolution
        if cy == 0.0:
            continue
        for col in range(cm.width):
            cx = cm.origin_x + (col + 0.5) * cm.resolution
            if forward_only and cx <= 0.0:
                continue
            if is_occupied(cm, row, col):
                continue
            if cy > 0.0:
                left += 1
            else:  # cy < 0.0 (cy == 0.0 already skipped above)
                right += 1
    return (left, right)


def select_detour_bias(cm: Costmap, max_angular: float) -> float:
    """
The follower uses this to pick a detour heading when the Goal_Point is
"""
    left, right = free_counts_per_side(cm)
    if left == 0 and right == 0:
        return 0.0
    total = left + right
    imbalance = (left - right) / total
    bias = imbalance * max_angular
    if bias > max_angular:
        return max_angular
    if bias < -max_angular:
        return -max_angular
    return bias
