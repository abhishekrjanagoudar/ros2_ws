#!/usr/bin/env python3
"""
motion_controller.py
====================
Pure-logic convoy motion controller — no ROS imports.

Encapsulates everything that was previously embedded in
``FollowerNode._control_loop()``:

  * Geometry helpers  (_yaw_from_quaternion, _to_robot_frame, _slew)
  * In-process costmap build (delegates to costmap_utils)
  * Arc-length goal-point walk (delegates to convoy_tracking)
  * Pure Pursuit lateral/longitudinal law
  * State-machine classify + base-command selection (delegates to follower_state)
  * SafetyController hard override (delegates to safety_controller)

The node shell (follower_node.py) instantiates one PursuitController,
caches ROS messages in callbacks, then calls::

    lin, ang, emerg = controller.step(pose, path, scan, now_ns)

on every control-timer tick.  All algorithm changes live here; the node
shell never needs to be touched for tuning.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from costmap_utils import (
    Costmap,
    build_costmap,
    is_goal_blocked,
    free_counts_per_side,
)
from convoy_tracking import (
    compute_goal_point,
    should_hold,
)
from follower_state import (
    FollowerState,
    classify_state,
    build_detour_command,
    build_search_command,
)
from safety_controller import SafetyController


# ─── Geometry helpers ─────────────────────────────────────────────────────────

def yaw_from_quaternion(q) -> float:
    """Extract yaw angle from a ROS Quaternion message (or any object with
    x, y, z, w attributes)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def to_robot_frame(
    px: float, py: float,
    rx: float, ry: float, ryaw: float,
) -> Tuple[float, float]:
    """Transform world point (px, py) into the robot's local frame."""
    dx = px - rx
    dy = py - ry
    local_x =  dx * math.cos(ryaw) + dy * math.sin(ryaw)
    local_y = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
    return local_x, local_y


def slew(cur: float, tgt: float, max_delta: float) -> float:
    """Rate-limit *tgt* so it does not change by more than *max_delta* per step."""
    if tgt > cur + max_delta:
        return cur + max_delta
    if tgt < cur - max_delta:
        return cur - max_delta
    return tgt


# ─── Controller ───────────────────────────────────────────────────────────────

class PursuitController:
    """
    Stateful Pure Pursuit + state-machine convoy follower.

    Owns all algorithm state (timers, previous emergency flag, last closest
    index).  The ROS node shell owns only the message caches and the publisher.

    Usage::

        ctrl = PursuitController(convoy_slot=2, params=...)
        ...
        lin, ang, is_emerg = ctrl.step(pose, path, scan, now_ns)
    """

    def __init__(
        self,
        convoy_slot: int,           # tb2 → 2, tb3 → 3
        convoy_spacing: float,
        lookahead_distance: float,
        kp_linear: float,
        kp_angular: float,
        goal_tol: float,
        costmap_size: float,
        costmap_resolution: float,
        costmap_stale_timeout: float,
        detour_forward_min_vel: float,
        stationary_deadlock_timeout: float,
        search_angular_velocity: float,
        breadcrumb_timeout: float,
        max_lin: float,
        max_ang: float,
        safety: SafetyController,
    ) -> None:
        self._gap                        = (convoy_slot - 1) * convoy_spacing
        self.lookahead_distance          = lookahead_distance
        self.kp_linear                   = kp_linear
        self.kp_angular                  = kp_angular
        self.goal_tol                    = goal_tol
        self.costmap_size                = costmap_size
        self.costmap_resolution          = costmap_resolution
        self.costmap_stale_timeout       = costmap_stale_timeout
        self.detour_forward_min_vel      = detour_forward_min_vel
        self.stationary_deadlock_timeout = stationary_deadlock_timeout
        self.search_angular_velocity     = search_angular_velocity
        self.breadcrumb_timeout          = breadcrumb_timeout
        self.max_lin                     = max_lin
        self.max_ang                     = max_ang
        self.safety                      = safety

        # Per-cycle mutable state
        self._last_closest_idx:  int            = 0
        self._stationary_since:  Optional[int]  = None   # wall-clock ns
        self._emergency_since:   Optional[int]  = None   # wall-clock ns
        self._prev_emergency:    bool            = False
        self._last_newest_time_ns: int          = 0      # ns since epoch

    # ── Breadcrumb-freshness update (called from _path_cb) ────────────────────

    def notify_newer_breadcrumb(self, now_ns: int) -> None:
        """Record the wall-clock time of the latest breadcrumb arrival."""
        self._last_newest_time_ns = now_ns

    # ── Main entry point ──────────────────────────────────────────────────────

    def step(
        self,
        pose: Tuple[float, float, float],   # (rx, ry, ryaw) in world frame
        path: List[Tuple[float, float]],    # convoy breadcrumb path
        scan,                               # sensor_msgs/LaserScan or None
        scan_age_s: float,                  # seconds since scan was received
        now_ns: int,                        # current wall-clock time [ns]
    ) -> Tuple[float, float, bool]:
        """
        Compute one control cycle.

        Parameters
        ----------
        pose        : (rx, ry, ryaw) world-frame robot pose
        path        : convoy breadcrumb list [(x,y), ...]
        scan        : LaserScan message, or None if no scan yet
        scan_age_s  : seconds since the cached scan was stamped
        now_ns      : current time in nanoseconds (from rclpy clock)

        Returns
        -------
        (linear_x, angular_z, is_emergency)
        """
        rx, ry, ryaw = pose

        # ── Costmap ───────────────────────────────────────────────────────────
        scan_stale = scan is None or scan_age_s > self.costmap_stale_timeout
        if scan_stale:
            cm: Costmap = build_costmap(
                [], 0.0, 0.0, 0.0, 0.0,
                self.costmap_size, self.costmap_resolution,
            )
            scan_for_safety = None
        else:
            rmin = scan.range_min if scan.range_min > 0 else 0.12
            cm = build_costmap(
                scan.ranges, scan.angle_min, scan.angle_increment,
                rmin, scan.range_max,
                self.costmap_size, self.costmap_resolution,
            )
            scan_for_safety = scan

        # ── Goal point: arc-length `gap` back from path end ───────────────────
        # compute_goal_point interpolates and clamps when path < gap.
        goal = compute_goal_point(path, self._gap)
        goal_idx = 0
        acc = 0.0
        for i in range(len(path) - 1, 0, -1):
            acc += math.hypot(path[i][0] - path[i - 1][0],
                              path[i][1] - path[i - 1][1])
            if acc >= self._gap:
                goal_idx = i - 1
                break

        # ── Path-length guard (P4) ───────────────────────────────────────────
        # If the path is shorter than the required gap, the robot is not yet in
        # a valid convoy position. Hold in place rather than sprinting to path[0].
        path_too_short = (
            goal_idx == 0 and len(path) > 1
            and math.hypot(path[-1][0] - path[0][0],
                           path[-1][1] - path[0][1]) < self._gap * 0.5
        )
        if path_too_short:
            return 0.0, 0.0, False

        # ── Pure Pursuit ──────────────────────────────────────────────────────
        # 1. Closest path point (O(1) walk starting near last closest).
        start_idx   = min(self._last_closest_idx, goal_idx)
        closest_idx = start_idx
        best        = float('inf')
        for i in range(start_idx, goal_idx + 1):
            d = math.hypot(path[i][0] - rx, path[i][1] - ry)
            if d < best:
                best = d
                closest_idx = i
            elif d > best + 0.5:
                break
        self._last_closest_idx = closest_idx

        # 2. Lookahead point (capped at the goal).
        look = goal
        acc  = 0.0
        for i in range(closest_idx, goal_idx):
            acc += math.hypot(path[i + 1][0] - path[i][0],
                              path[i + 1][1] - path[i][1])
            if acc >= self.lookahead_distance:
                look = path[i + 1]
                break

        lx, ly = to_robot_frame(look[0], look[1], rx, ry, ryaw)
        Ld     = max(math.hypot(lx, ly), 1e-3)
        alpha  = math.atan2(ly, lx)

        # 3. Goal in robot-local frame (blocking test + speed scaling).
        gx_local, gy_local = to_robot_frame(goal[0], goal[1], rx, ry, ryaw)
        dist_to_goal        = math.hypot(gx_local, gy_local)

        # 4. Pure Pursuit base command.
        if dist_to_goal <= self.goal_tol:
            pursuit_linear  = 0.0
            pursuit_angular = 0.0
        else:
            forward_drive   = max(0.0, gx_local)
            if forward_drive <= 0.0:
                # Goal is behind the robot — stop forward motion, allow only
                # angular correction to turn back toward the goal.
                pursuit_linear  = 0.0
                pursuit_angular = self.kp_angular * math.atan2(gy_local, -gx_local + 1e-6)
            else:
                creep           = self.kp_linear * dist_to_goal * 0.3
                pursuit_linear  = self.kp_linear * max(forward_drive, creep)
                curvature       = 2.0 * ly / (Ld * Ld)
                pursuit_angular = pursuit_linear * curvature
                if abs(alpha) > 0.8:
                    pursuit_angular = self.kp_angular * alpha
                    pursuit_linear *= 0.3
                pursuit_linear *= max(0.3, math.cos(alpha))

        # ── State-machine inputs ──────────────────────────────────────────────
        blocked              = is_goal_blocked(cm, gx_local, gy_local)
        left_free, right_free = free_counts_per_side(cm)
        both_blocked         = (left_free == 0 and right_free == 0)

        dist_to_goal_world   = math.hypot(goal[0] - rx, goal[1] - ry)
        elapsed_s            = (now_ns - self._last_newest_time_ns) / 1e9
        hold                 = should_hold(
            elapsed_s, self.breadcrumb_timeout,
            dist_to_goal_world, self.goal_tol,
        )
        has_unreached        = dist_to_goal_world > self.goal_tol

        stationary_duration = (
            (now_ns - self._stationary_since) / 1e9
            if self._stationary_since is not None else 0.0
        )
        # emergency_duration uses the previous cycle's timer so the state
        # machine sees accumulated duration, not the instantaneous flag.
        emergency_duration = (
            (now_ns - self._emergency_since) / 1e9
            if self._emergency_since is not None else 0.0
        )

        # safety_emergency_now is resolved by check_and_modify_ex() below
        # (single LiDAR pass). Use False as a well-defined placeholder for
        # the state machine; classify_state() reads emergency_duration from
        # the previous cycle, not the raw flag.
        safety_emergency_now = False

        # ── Classify next state ───────────────────────────────────────────────
        state = classify_state(
            goal_blocked=blocked,
            both_sides_blocked=both_blocked,
            safety_emergency=safety_emergency_now,
            hold_active=hold,
            stationary_duration_s=stationary_duration,
            emergency_duration_s=emergency_duration,
            deadlock_timeout_s=self.stationary_deadlock_timeout,
            has_unreached_breadcrumbs=has_unreached,
        )

        if state == FollowerState.HOLD:
            base_linear, base_angular = 0.0, 0.0
        elif state == FollowerState.EMERGENCY_STOP:
            base_linear, base_angular = 0.0, 0.0
        elif state == FollowerState.SEARCH:
            base_linear, base_angular = build_search_command(
                self.search_angular_velocity, self.max_ang,
            )
        elif state == FollowerState.DETOUR:
            base_linear, base_angular = build_detour_command(
                cm, pursuit_linear,
                self.max_lin, self.max_ang,
                self.detour_forward_min_vel,
            )
        else:  # TRACKING
            base_linear, base_angular = pursuit_linear, pursuit_angular

        # ── SafetyController hard override (single LiDAR pass) ───────────────
        # check_and_modify_ex() returns corrected velocities AND the emergency
        # flag in one scan walk, eliminating the previous double-pass pattern.
        linear_x, angular_z = base_linear, base_angular
        if scan_for_safety is not None:
            s    = scan_for_safety
            rmin = s.range_min if s.range_min > 0 else 0.12
            linear_x, angular_z, safety_emergency_now = self.safety.check_and_modify_ex(
                linear_x=linear_x,
                angular_z=angular_z,
                ranges=list(s.ranges),
                angle_min=s.angle_min,
                angle_increment=s.angle_increment,
                range_min=rmin,
            )

        # ── Emergency-duration timer (updated with the real flag) ─────────────
        if safety_emergency_now:
            if not self._prev_emergency or self._emergency_since is None:
                self._emergency_since = now_ns
        else:
            self._emergency_since = None
        self._prev_emergency = safety_emergency_now

        # ── Stationary-duration timer ─────────────────────────────────────────
        # Updated from the CALLER's _last_lin (post-slew) — see note in node.
        # The controller exposes _update_stationary() for the node to call after
        # it applies slew limiting so the timer reflects the actual output.
        self._pending_linear_for_stationary = linear_x

        return linear_x, angular_z, safety_emergency_now

    def update_stationary_timer(self, actual_linear: float, now_ns: int) -> None:
        """Call this AFTER slew-limiting so the timer reflects the wire command."""
        if abs(actual_linear) < 1e-6:
            if self._stationary_since is None:
                self._stationary_since = now_ns
        else:
            self._stationary_since = None
