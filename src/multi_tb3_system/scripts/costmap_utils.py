#!/usr/bin/env python3
"""
costmap_utils.py
================
Pure helpers for the LiDAR-derived local occupancy grid (Local_Costmap)
used by both ``costmap_generator.py`` (visualization on every robot)
and ``follower_node.py`` (in-process detour decisions).

This module is intentionally side-effect free and must not import
``rclpy`` or any ROS message types so the logic can be exercised by
unit and property tests without a running ROS graph. It mirrors the
existing split between ``laser_processor.py`` (pure) and the nodes
that consume it.

Occupancy convention (matches ``nav_msgs/OccupancyGrid`` semantics
used by the design):

* ``0``   - free / non-occupied cell.
* ``100`` - occupied cell (an Occupied_Cell, part of the
  Forbidden_Zone).

The grid is row-major: cell ``(row, col)`` lives at
``data[row * width + col]``.

The local frame follows the standard robot convention (x forward,
y left). The grid is centered on the robot, so cell ``(0, 0)`` is at
the lower-left corner ``(origin_x, origin_y) = (-size_m/2,
-size_m/2)``.

Public API provided by this file:

  - ``Costmap`` dataclass
  - ``is_valid_return(r, range_min, range_max) -> bool``
  - ``local_to_cell(cm, x, y) -> Optional[tuple[int, int]]``
  - ``is_occupied(cm, row, col) -> bool``
  - ``build_costmap(ranges, angle_min, angle_increment, range_min,
    range_max, size_m=3.0, resolution=0.05) -> Costmap``
  - ``segment_hits_occupied(cm, x0, y0, x1, y1) -> bool``
  - ``is_goal_blocked(cm, gx, gy) -> bool``
  - ``free_counts_per_side(cm, forward_only=True) -> tuple[int, int]``
  - ``select_detour_bias(cm, max_angular) -> float``
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass
class Costmap:
    """Row-major occupancy grid centered on a robot.

    Attributes
    ----------
    data:
        Flat list of occupancy values of length ``width * height``,
        row-major. ``0`` means free, ``100`` means occupied.
    width:
        Number of cells along the x axis (columns).
    height:
        Number of cells along the y axis (rows).
    resolution:
        Cell side length in metres (e.g. ``0.05``).
    origin_x:
        Local-frame x of the lower-left corner of cell ``(0, 0)``.
        For a grid centered on the robot this is ``-size_m / 2``.
    origin_y:
        Local-frame y of the lower-left corner of cell ``(0, 0)``.
        For a grid centered on the robot this is ``-size_m / 2``.
    """

    data: list[int]
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


def is_valid_return(r: float, range_min: float, range_max: float) -> bool:
    """Return True iff ``r`` is a valid LiDAR return.

    A return is valid when its range value is finite (not NaN, not
    +/-inf) and lies within the inclusive interval
    ``[range_min, range_max]`` reported by the ``LaserScan`` message.

    Validates: Requirements 1.2, 2.1
    """
    return math.isfinite(r) and range_min <= r <= range_max


def local_to_cell(
    cm: Costmap, x: float, y: float
) -> Optional[tuple[int, int]]:
    """Map a local-frame point ``(x, y)`` to its ``(row, col)`` cell.

    The mapping uses the standard occupancy-grid convention:

    * ``col = floor((x - origin_x) / resolution)``
    * ``row = floor((y - origin_y) / resolution)``

    Returns ``None`` when the point falls outside the grid (i.e. the
    computed row/col is not in ``[0, height)`` / ``[0, width)``).

    Validates: Requirements 1.1, 2.1
    """
    col = math.floor((x - cm.origin_x) / cm.resolution)
    row = math.floor((y - cm.origin_y) / cm.resolution)
    if 0 <= row < cm.height and 0 <= col < cm.width:
        return (row, col)
    return None


def is_occupied(cm: Costmap, row: int, col: int) -> bool:
    """Return True iff cell ``(row, col)`` is marked occupied.

    Performs explicit bounds checking against ``cm.height`` and
    ``cm.width`` and returns False for any out-of-range index rather
    than raising, so callers can pass cells from upstream computations
    without an extra guard. Indexing into ``cm.data`` uses the
    row-major flattening ``row * width + col``.

    Validates: Requirements 1.1, 2.1
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
    """Build a Local_Costmap from a single ``LaserScan``-shaped sweep.

    Constructs a square row-major occupancy grid centered on the robot
    of side length ``size_m`` and cell size ``resolution`` metres.
    With the defaults this yields a 60x60 grid (3600 cells) whose
    lower-left corner is at ``(-1.5, -1.5)`` in the robot frame.

    Every cell starts ``0`` (free), so an empty or fully-invalid
    ``ranges`` iterable yields an all-free grid (R1.6). For each
    valid return (``is_valid_return`` is True; non-finite or
    out-of-range returns are skipped per R1.2/R2.1) the beam endpoint
    is computed in the robot frame as
    ``(r*cos(angle), r*sin(angle))`` with
    ``angle = angle_min + i * angle_increment`` (x forward, y left),
    mapped to a cell with ``local_to_cell``, and the cell is marked
    ``100`` (occupied) when it falls inside the grid. Endpoints that
    map outside the grid are silently dropped.

    Validates: Requirements 1.1, 1.2, 1.6, 2.1
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
    """Return True iff segment ``(x0, y0) -> (x1, y1)`` crosses an occupied cell.

    The segment is sampled at a spacing of at most ``resolution / 2`` so
    that no occupied cell along the segment can be skipped (a sample
    falls inside every cell the segment intersects). The number of
    sample points is

        n = max(1, ceil(hypot(x1 - x0, y1 - y0) / (resolution / 2)))

    and ``n + 1`` evenly spaced samples are taken at parameters
    ``t = k / n`` for ``k`` in ``range(n + 1)``. Each sample is mapped
    to a cell with ``local_to_cell``; samples that fall outside the
    grid contribute nothing (they cannot be occupied). The function
    returns True as soon as any in-grid sample lands in an occupied
    cell.

    This is a helper for ``is_goal_blocked`` and the follower's
    DETOUR / TRACKING transition: the design uses the same primitive
    to test whether the straight line from the robot origin to the
    Goal_Point intersects the Forbidden_Zone.

    Validates: Requirements 2.4, 3.3
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
    """Return True iff the goal at ``(gx, gy)`` is blocked.

    The goal point is expressed in the robot's local costmap frame
    (x forward, y left, robot at origin). It is *blocked* when either:

    * the goal cell itself is an Occupied_Cell (the goal sits inside
      the Forbidden_Zone), or
    * the straight segment from the robot origin ``(0, 0)`` to the
      goal ``(gx, gy)`` intersects an Occupied_Cell, as determined by
      ``segment_hits_occupied``.

    A goal that maps outside the grid is treated as not occupied at
    the cell test (``local_to_cell`` returns ``None``); the segment
    test still inspects every in-grid sample along the way to the
    goal, so an out-of-grid goal is blocked iff the segment crosses
    an occupied cell on the way out.

    Validates: Requirements 2.2, 3.3
    """
    cell = local_to_cell(cm, gx, gy)
    if cell is not None and is_occupied(cm, cell[0], cell[1]):
        return True
    return segment_hits_occupied(cm, 0.0, 0.0, gx, gy)


def free_counts_per_side(
    cm: Costmap, forward_only: bool = True
) -> tuple[int, int]:
    """Count non-occupied cells on the left (y > 0) and right (y < 0) sides.

    Walks every cell ``(row, col)`` of the grid and computes its center in
    the local frame as

    * ``cx = origin_x + (col + 0.5) * resolution``
    * ``cy = origin_y + (row + 0.5) * resolution``

    When ``forward_only`` is True (the default) cells with ``cx <= 0`` are
    skipped, so only the half-plane ahead of the robot contributes — this
    matches the design's "weigh the region ahead" choice for detour-bias
    selection. When ``forward_only`` is False the entire grid is considered.

    For each retained cell, ``is_occupied`` is consulted: a non-occupied
    (free) cell with ``cy > 0`` increments the left count, a non-occupied
    cell with ``cy < 0`` increments the right count. Cells exactly on the
    ``y == 0`` axis are ignored (they belong to neither side), as are
    occupied cells.

    Returns a tuple ``(left, right)`` of integer counts. The function is
    pure and never mutates ``cm``.

    Validates: Requirements 2.2
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
    """Return a signed angular bias toward the freer side of the costmap.

    The follower uses this to pick a detour heading when the Goal_Point is
    blocked: positive values point toward ``+y`` (turn left), negative
    values point toward ``-y`` (turn right), and the magnitude scales with
    the imbalance between the two sides so a strongly one-sided gap pulls
    harder than a marginally one-sided gap.

    The computation:

    1. ``left, right = free_counts_per_side(cm)`` (forward half only).
    2. If ``left == 0`` and ``right == 0`` the entire forward region is
       occupied; return ``0.0`` so the caller can map that to a HOLD
       command (R2.6).
    3. Otherwise let ``total = left + right`` and
       ``imbalance = (left - right) / total`` in ``[-1, 1]``.
    4. ``bias = imbalance * max_angular``, then clamp to
       ``[-max_angular, +max_angular]`` to enforce the angular limit
       (R2.5).

    Step 4's clamp is defensive: with ``imbalance`` in ``[-1, 1]`` and
    ``max_angular >= 0`` the unclamped value already lies inside the
    interval, but the clamp keeps the contract explicit for callers that
    may pass an unusual sign convention.

    Validates: Requirements 2.2, 2.5, 2.6
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
