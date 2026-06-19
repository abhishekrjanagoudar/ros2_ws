#!/usr/bin/env python3
"""
convoy_tracking.py
==================
Pure helpers for convoy breadcrumb path tracking.

This module is intentionally side-effect free and must not import
``rclpy`` or any ROS message types so the logic can be exercised by
unit and property tests without a running ROS graph. It is consumed by
``follower_node.py`` to compute the per-cycle Goal_Point that Pure
Pursuit drives toward.

Public API:
  - compute_goal_point(path, gap) -> (gx, gy)
  - goal_reached(rx, ry, gx, gy, goal_tolerance) -> bool
  - is_newer_breadcrumb(prev_count, prev_stamp_ns, new_count, new_stamp_ns)
    -> bool
  - should_hold(elapsed_s, breadcrumb_timeout, dist_to_final, goal_tolerance)
    -> bool
  - should_append(last_xy, new_xy, path_resolution) -> bool
"""

from __future__ import annotations

import math


def compute_goal_point(
    path: list[tuple[float, float]],
    gap: float,
) -> tuple[float, float]:
    """Return the point at arc-length ``gap`` behind the newest breadcrumb.

    The newest breadcrumb is ``path[-1]``. The function walks the path
    *backwards* from there, accumulating segment arc-lengths until the
    running sum reaches ``gap``, then linearly interpolates the exact
    point at arc-length ``gap`` behind ``path[-1]`` along the segment
    where the running sum crossed ``gap``.

    If the path's total arc-length is strictly less than ``gap``, the
    result is clamped to the oldest breadcrumb ``path[0]`` (R6.5).

    Parameters
    ----------
    path:
        Ordered list of breadcrumb positions ``[(x, y), ...]`` from
        oldest (``path[0]``) to newest (``path[-1]``). The caller is
        expected to guarantee the list is non-empty.
    gap:
        Desired arc-length offset (in metres) behind the newest
        breadcrumb. Must be non-negative.

    Returns
    -------
    tuple[float, float]
        The interpolated goal point ``(gx, gy)`` in the same frame as
        ``path``.

    Notes
    -----
    Single-point paths trivially return that lone point. Zero-length
    segments (consecutive duplicate breadcrumbs) are skipped so they do
    not produce a divide-by-zero during interpolation.

    Validates: Requirements 6.3, 6.5
    """
    # Single-point path: nothing to walk along, return the lone point.
    if len(path) == 1:
        return path[0]

    accumulated = 0.0
    # Walk backwards: segment endpoints are (path[i], path[i-1])
    # for i = N-1 down to 1, where path[i] is the newer end and
    # path[i-1] is the older end of each segment.
    for i in range(len(path) - 1, 0, -1):
        x_new, y_new = path[i]
        x_old, y_old = path[i - 1]
        seg_len = math.hypot(x_new - x_old, y_new - y_old)

        # Skip zero-length segments (duplicate breadcrumbs) so we do
        # not divide by zero when interpolating.
        if seg_len == 0.0:
            continue

        # If this segment is where the running sum crosses ``gap``,
        # interpolate the exact crossing point and return.
        if accumulated + seg_len >= gap:
            remaining = gap - accumulated
            t = remaining / seg_len  # fraction from path[i] toward path[i-1]
            gx = x_new + t * (x_old - x_new)
            gy = y_new + t * (y_old - y_new)
            return (gx, gy)

        accumulated += seg_len

    # Total arc-length is strictly less than ``gap``: clamp to the
    # oldest breadcrumb (R6.5).
    return path[0]


def goal_reached(
    rx: float,
    ry: float,
    gx: float,
    gy: float,
    goal_tolerance: float,
) -> bool:
    """Return ``True`` when the Follower is at or within ``goal_tolerance``.

    The predicate compares the Euclidean distance between the Follower
    position ``(rx, ry)`` and the Goal_Point ``(gx, gy)`` against
    ``goal_tolerance``. A non-strict ``<=`` comparison is used so a
    Follower sitting exactly at the configured tolerance is treated as
    having reached the goal (R6.6).

    Parameters
    ----------
    rx, ry:
        Follower position in the same frame as the Goal_Point.
    gx, gy:
        Goal_Point position.
    goal_tolerance:
        Distance threshold (in metres) within which the Goal_Point is
        considered reached. Expected to be non-negative.

    Returns
    -------
    bool
        ``True`` if ``hypot(gx - rx, gy - ry) <= goal_tolerance``,
        else ``False``.

    Validates: Requirements 6.6
    """
    return math.hypot(gx - rx, gy - ry) <= goal_tolerance


def is_newer_breadcrumb(
    prev_count: int,
    prev_stamp_ns: int,
    new_count: int,
    new_stamp_ns: int,
) -> bool:
    """Return ``True`` when the latest Breadcrumb_Path is newer than the prior.

    A newer Breadcrumb_Path is detected either by an increase in the
    Breadcrumb count (a new pose was appended) or by a change in the
    newest-Breadcrumb timestamp (the publisher refreshed the message).
    Either condition is sufficient, matching R7.1's "count increases or
    timestamp of the newest Breadcrumb changes" criterion.

    The timestamp comparison uses ``!=`` rather than ``>`` so that any
    change is treated as a new arrival; this keeps the predicate robust
    to clock resets and to publishers that re-emit the same path with a
    refreshed header.

    Parameters
    ----------
    prev_count:
        Breadcrumb count from the most recently observed Breadcrumb_Path.
    prev_stamp_ns:
        Newest-Breadcrumb timestamp, in integer nanoseconds, from the
        most recently observed Breadcrumb_Path.
    new_count:
        Breadcrumb count of the just-received Breadcrumb_Path.
    new_stamp_ns:
        Newest-Breadcrumb timestamp, in integer nanoseconds, of the
        just-received Breadcrumb_Path.

    Returns
    -------
    bool
        ``True`` if the count strictly increased OR the newest-Breadcrumb
        timestamp differs from the previous one, else ``False``.

    Validates: Requirements 7.1
    """
    return new_count > prev_count or new_stamp_ns != prev_stamp_ns


def should_hold(
    elapsed_s: float,
    breadcrumb_timeout: float,
    dist_to_final: float,
    goal_tolerance: float,
) -> bool:
    """Return ``True`` when the conditional-stop HOLD precondition is met.

    The Follower must hold position only when *both* of the following
    hold simultaneously:

    1. The time since the most recent newer Breadcrumb_Path arrival,
       ``elapsed_s``, has exceeded the configured ``breadcrumb_timeout``
       (R7.2 — Leader is presumed lost).
    2. The Follower has reached the *final* Breadcrumb,
       ``dist_to_final <= goal_tolerance`` (R7.2 — there is no
       in-flight pursuit work left to do).

    When the timeout has elapsed but the Follower is still pursuing
    (i.e. ``dist_to_final > goal_tolerance``), the predicate returns
    ``False`` so the Follower keeps tracking the existing path until it
    reaches the final Breadcrumb (R7.3, R7.4 — persistent pursuit
    continues until completion). Receipt of a newer Breadcrumb_Path
    resets ``elapsed_s`` upstream in ``follower_node.py`` (R7.6), which
    flips this predicate back to ``False`` automatically without
    requiring the helper itself to track that event.

    Parameters
    ----------
    elapsed_s:
        Seconds since the most recent newer Breadcrumb_Path was
        observed (see ``is_newer_breadcrumb``). Non-negative.
    breadcrumb_timeout:
        Configured silence threshold (in seconds) before the Leader is
        treated as lost (R7.5 valid range ``[0.1, 600]``).
    dist_to_final:
        Euclidean distance (in metres) from the Follower's current
        position to the *final* Breadcrumb in the cached path.
    goal_tolerance:
        Distance threshold (in metres) within which the final
        Breadcrumb is considered reached.

    Returns
    -------
    bool
        ``True`` iff ``elapsed_s > breadcrumb_timeout`` *and*
        ``dist_to_final <= goal_tolerance``; ``False`` otherwise.

    Validates: Requirements 7.2, 7.3, 7.4, 7.6
    """
    return elapsed_s > breadcrumb_timeout and dist_to_final <= goal_tolerance


def should_append(
    last_xy: tuple[float, float],
    new_xy: tuple[float, float],
    path_resolution: float,
) -> bool:
    """Return ``True`` when a new Leader pose should be appended to the path.

    The convoy publisher records a new Breadcrumb only when the Leader
    has moved at least ``path_resolution`` metres from the most
    recently appended Breadcrumb. This guards against unbounded growth
    of the breadcrumb list while a stationary Leader continues to
    publish odometry, and produces an evenly-spaced trail at the
    configured resolution (R5.3).

    The comparison is non-strict (``>=``) so a Leader pose sitting
    exactly at the resolution boundary is appended; this matches the
    publisher's existing behaviour and keeps the path advancing in
    lock-step with the configured spacing.

    Parameters
    ----------
    last_xy:
        ``(x, y)`` of the most recently appended Breadcrumb. Caller
        guarantees a Breadcrumb has already been recorded; the very
        first Leader pose is appended unconditionally upstream so an
        empty path is published until then (R5.1).
    new_xy:
        ``(x, y)`` of the candidate Leader pose to potentially append.
    path_resolution:
        Minimum spacing (in metres) between successive Breadcrumbs.
        Expected to be non-negative.

    Returns
    -------
    bool
        ``True`` iff the Euclidean distance from ``last_xy`` to
        ``new_xy`` is at least ``path_resolution``, else ``False``.

    Notes
    -----
    The caller is responsible for capping the path at
    ``max_path_poses`` (5000) by trimming the oldest Breadcrumbs once
    appended (R5.4); this predicate only governs the *append* decision.

    Validates: Requirements 5.1, 5.3, 5.4
    """
    return (
        math.hypot(new_xy[0] - last_xy[0], new_xy[1] - last_xy[1])
        >= path_resolution
    )
