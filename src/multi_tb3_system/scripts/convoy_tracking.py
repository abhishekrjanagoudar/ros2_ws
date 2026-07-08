#!/usr/bin/env python3
"""
Pure helpers for convoy breadcrumb path tracking.
"""

from __future__ import annotations

import math


def compute_goal_and_index(
    path: list[tuple[float, float]],
    gap: float,
) -> tuple[tuple[float, float], int]:
    """
    Returns the interpolated goal point at `gap` arc-length from the end of the path,
    and the index of the waypoint that immediately precedes this goal point.
    """
    if len(path) == 0:
        return (0.0, 0.0), 0
    if len(path) == 1 or gap <= 0.0:
        return path[0], 0

    accumulated = 0.0
    for i in range(len(path) - 1, 0, -1):
        x_new, y_new = path[i]
        x_old, y_old = path[i - 1]
        seg_len = math.hypot(x_new - x_old, y_new - y_old)

        if seg_len == 0.0:
            continue

        if accumulated + seg_len >= gap:
            remaining = gap - accumulated
            t = remaining / seg_len
            gx = x_new + t * (x_old - x_new)
            gy = y_new + t * (y_old - y_new)
            return (gx, gy), i - 1

        accumulated += seg_len

    return path[0], 0


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
