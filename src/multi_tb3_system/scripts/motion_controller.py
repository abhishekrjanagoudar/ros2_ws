#!/usr/bin/env python3
"""
Pure-logic convoy motion controller — no ROS imports.
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
from local_planner import LocalPlanner
from multi_tb3_system.perception.laser_processor import process_scan


# Geometry helpers

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


# Controller

class PursuitController:
    """
Stateful Pure Pursuit + state-machine convoy follower.
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
        enable_local_planner: bool = True,
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
        self.enable_local_planner        = enable_local_planner

        # Initialize local planner if enabled
        self.local_planner: Optional[LocalPlanner] = None
        if self.enable_local_planner:
            self.local_planner = LocalPlanner(
                max_linear_vel=max_lin,
                max_angular_vel=max_ang,
                max_linear_acc=2.0,
                max_angular_acc=3.0,
                velocity_samples=10,  # More samples for better paths
                angular_samples=15,   # More angular samples for narrow gaps
                predict_time=1.5,
                robot_radius=0.25,
                goal_weight=1.0,
                velocity_weight=0.3,  # Favor moving forward
                obstacle_weight=2.5,  # Strong obstacle avoidance
                control_period=0.05,
            )

        # Per-cycle mutable state
        self._last_closest_idx:  int            = 0
        self._stationary_since:  Optional[int]  = None   # wall-clock ns
        self._emergency_since:   Optional[int]  = None   # wall-clock ns
        self._prev_emergency:    bool            = False
        self._state_for_timer_reset: FollowerState = FollowerState.TRACKING
        self._last_newest_time_ns: int          = 0      # ns since epoch

    # Breadcrumb-freshness update (called from _path_cb)

    def notify_newer_breadcrumb(self, now_ns: int) -> None:
        """Record the wall-clock time of the latest breadcrumb arrival."""
        self._last_newest_time_ns = now_ns

    # Main entry point

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
"""
        rx, ry, ryaw = pose

        # Costmap
        scan_stale = scan is None or scan_age_s > self.costmap_stale_timeout
        if scan_stale:
            cm: Costmap = build_costmap(
                [], 0.0, 0.0, 0.0, 0.0,
                self.costmap_size, self.costmap_resolution,
            )
            scan_for_safety = None
        else:
            rmin = scan.range_min if scan.range_min > 0 else 0.12
            filtered_ranges = self.safety.filter_predecessor_returns(
                list(scan.ranges), scan.angle_min, scan.angle_increment,
            )
            cm = build_costmap(
                filtered_ranges, scan.angle_min, scan.angle_increment,
                rmin, scan.range_max,
                self.costmap_size, self.costmap_resolution,
            )
            scan_for_safety = scan

        # Goal point: arc-length `gap` back from path end
        goal = compute_goal_point(path, self._gap)
        goal_idx = 0
        acc = 0.0
        for i in range(len(path) - 1, 0, -1):
            acc += math.hypot(path[i][0] - path[i - 1][0],
                              path[i][1] - path[i - 1][1])
            if acc >= self._gap:
                goal_idx = i - 1
                break

        # Path-length guard (P4)
        path_too_short = (
            goal_idx == 0 and len(path) > 1
            and math.hypot(path[-1][0] - path[0][0],
                           path[-1][1] - path[0][1]) < self._gap * 0.5
        )
        if path_too_short:
            return 0.0, 0.0, False

        # Pure Pursuit
        search_start = max(0, self._last_closest_idx - 20)
        start_idx    = min(search_start, goal_idx)
        closest_idx  = start_idx
        best         = float('inf')
        for i in range(start_idx, goal_idx + 1):
            d = math.hypot(path[i][0] - rx, path[i][1] - ry)
            if d < best:
                best = d
                closest_idx = i
            elif d > best + 0.10:
                # Early-break only when distance is clearly growing.
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

        # LiDAR Target Tracking
        if scan_for_safety is not None and len(path) > 0:
            expected_local_x, expected_local_y = to_robot_frame(path[-1][0], path[-1][1], rx, ry, ryaw)
            rmin = scan_for_safety.range_min if scan_for_safety.range_min > 0 else 0.12
            target_cluster, _ = process_scan(
                ranges=list(scan_for_safety.ranges),
                angle_min=scan_for_safety.angle_min,
                angle_increment=scan_for_safety.angle_increment,
                range_min=rmin,
                front_half_angle_deg=90.0,
                expected_local_pos=(expected_local_x, expected_local_y)
            )
            if target_cluster is not None and target_cluster.confidence > 0.5:
                # Calculate physical error: where the leader actually is vs where Odometry thinks it is.
                err_x = target_cluster.centroid_x - expected_local_x
                err_y = target_cluster.centroid_y - expected_local_y
                
                # Apply an EMA filter to the offset to prevent violent swerves (alpha = 0.1)
                if not hasattr(self, '_ema_err_x'):
                    self._ema_err_x = err_x
                    self._ema_err_y = err_y
                else:
                    self._ema_err_x = 0.1 * err_x + 0.9 * self._ema_err_x
                    self._ema_err_y = 0.1 * err_y + 0.9 * self._ema_err_y
                
                # Shift the goal by the smoothed physical error to eliminate drift!
                gx_local += self._ema_err_x
                gy_local += self._ema_err_y

        # 4. Pure Pursuit base command.
        if dist_to_goal <= self.goal_tol:
            pursuit_linear  = 0.0
            pursuit_angular = 0.0
        else:
            forward_drive   = max(0.0, gx_local)
            if forward_drive <= 0.0:
                # Goal is behind the robot — stop forward motion, allow only
                pursuit_linear  = 0.0
                pursuit_angular = self.kp_angular * math.atan2(gy_local, -gx_local + 1e-6)
            else:
                creep          = self.kp_linear * dist_to_goal * 0.3
                pursuit_linear = self.kp_linear * max(forward_drive, creep)

                if abs(alpha) > 0.8:
                    # Large heading error: prioritize turning, slow forward motion.
                    pursuit_angular = self.kp_angular * alpha
                    pursuit_linear *= max(0.35, math.cos(alpha))
                else:
                    # Normal tracking: smooth arc via Pure Pursuit curvature.
                    curvature       = 2.0 * ly / (Ld * Ld)
                    pursuit_angular = pursuit_linear * curvature
                    # Mild speed scaling for small heading errors.
                    pursuit_linear *= max(0.7, math.cos(alpha))

        # State-machine inputs
        blocked              = is_goal_blocked(cm, gx_local, gy_local)
        left_free, right_free = free_counts_per_side(cm)
        both_blocked         = (left_free == 0 and right_free == 0)

        dist_to_goal_world   = math.hypot(goal[0] - rx, goal[1] - ry)
        dist_to_path_end     = math.hypot(path[-1][0] - rx, path[-1][1] - ry)
        elapsed_s            = (now_ns - self._last_newest_time_ns) / 1e9
        hold                 = should_hold(
            elapsed_s, self.breadcrumb_timeout,
            dist_to_path_end, self.goal_tol,
        )
        has_unreached        = dist_to_goal_world > self.goal_tol

        stationary_duration = (
            (now_ns - self._stationary_since) / 1e9
            if self._stationary_since is not None else 0.0
        )
        # emergency_duration uses the previous cycle's timer so the state
        emergency_duration = (
            (now_ns - self._emergency_since) / 1e9
            if self._emergency_since is not None else 0.0
        )

        # safety_emergency_now is resolved by check_and_modify_ex() below
        safety_emergency_now = False

        # Classify next state
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
            # Use local planner if enabled, otherwise use simple detour
            if self.enable_local_planner and self.local_planner is not None:
                # Update local planner's current velocity for dynamic window
                self.local_planner.update_current_velocity(
                    self._pending_linear_for_stationary if hasattr(self, '_pending_linear_for_stationary') else 0.0,
                    0.0
                )
                # Compute velocity toward goal using local planner
                base_linear, base_angular = self.local_planner.compute_velocity(
                    gx_local, gy_local, cm, ryaw
                )
            else:
                # Simple detour: biased turning
                base_linear, base_angular = build_detour_command(
                    cm, pursuit_linear,
                    self.max_lin, self.max_ang,
                    self.detour_forward_min_vel,
                )
                # Bias detour angular toward the goal when goal is more than 45°
                goal_bearing = math.atan2(gy_local, gx_local)
                if abs(goal_bearing) > math.radians(45):
                    goal_sign   = 1.0 if goal_bearing > 0 else -1.0
                    detour_sign = 1.0 if base_angular  > 0 else -1.0
                    if goal_sign != detour_sign:
                        base_angular = -base_angular
        else:  # TRACKING
            base_linear, base_angular = pursuit_linear, pursuit_angular

        # SafetyController hard override (single LiDAR pass)
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

        # Emergency-duration timer (updated with the real flag)
        if safety_emergency_now:
            if self._emergency_since is None:
                self._emergency_since = now_ns
        elif self._state_for_timer_reset == FollowerState.TRACKING:
            # Only reset when cleanly back in TRACKING, not on momentary
            self._emergency_since = None
        self._prev_emergency = safety_emergency_now

        # Stationary-duration timer
        self._pending_linear_for_stationary = linear_x

        self._state_for_timer_reset = state
        return linear_x, angular_z, safety_emergency_now

    def update_stationary_timer(self, actual_linear: float, now_ns: int) -> None:
        """Call this AFTER slew-limiting so the timer reflects the wire command."""
        if abs(actual_linear) < 1e-6:
            # Robot is stopped for any reason (emergency, detour suppressed,
            if self._stationary_since is None:
                self._stationary_since = now_ns
        else:
            # Only clear when the robot is genuinely moving.
            self._stationary_since = None
