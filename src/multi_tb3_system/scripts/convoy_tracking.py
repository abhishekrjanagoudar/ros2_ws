#!/usr/bin/env python3
"""
Pure helpers for convoy breadcrumb path tracking.
"""

from __future__ import annotations

import math


def compute_goal_point(
    path: list[tuple[float, float]],
    gap: float,
) -> tuple[float, float]:
    """
The newest breadcrumb is ``path[-1]``. The function walks the path
"""
    # Single-point path: nothing to walk along, return the lone point.
    if len(path) == 1:
        return path[0]

    accumulated = 0.0
    # Walk backwards: segment endpoints are (path[i], path[i-1])
    for i in range(len(path) - 1, 0, -1):
        x_new, y_new = path[i]
        x_old, y_old = path[i - 1]
        seg_len = math.hypot(x_new - x_old, y_new - y_old)

        # Skip zero-length segments (duplicate breadcrumbs) so we do
        if seg_len == 0.0:
            continue

        # If this segment is where the running sum crosses ``gap``,
        if accumulated + seg_len >= gap:
            remaining = gap - accumulated
            t = remaining / seg_len  # fraction from path[i] toward path[i-1]
            gx = x_new + t * (x_old - x_new)
            gy = y_new + t * (y_old - y_new)
            return (gx, gy)

        accumulated += seg_len

    # Total arc-length is strictly less than ``gap``: clamp to the
    return path[0]


def goal_reached(
    rx: float,
    ry: float,
    gx: float,
    gy: float,
    goal_tolerance: float,
) -> bool:
    """
The predicate compares the Euclidean distance between the Follower
"""
    return math.hypot(gx - rx, gy - ry) <= goal_tolerance


def is_newer_breadcrumb(
    prev_count: int,
    prev_stamp_ns: int,
    new_count: int,
    new_stamp_ns: int,
) -> bool:
    """
A newer Breadcrumb_Path is detected solely by a strict increase in the
"""
    return new_count > prev_count



def should_hold(
    elapsed_s: float,
    breadcrumb_timeout: float,
    dist_to_final: float,
    goal_tolerance: float,
) -> bool:
    """
The Follower must hold position only when *both* of the following
"""
    return elapsed_s > breadcrumb_timeout and dist_to_final <= goal_tolerance


def should_append(
    last_xy: tuple[float, float],
    new_xy: tuple[float, float],
    path_resolution: float,
) -> bool:
    """
The convoy publisher records a new Breadcrumb only when the Leader
"""
    return (
        math.hypot(new_xy[0] - last_xy[0], new_xy[1] - last_xy[1])
        >= path_resolution
    )
