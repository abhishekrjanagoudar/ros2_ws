#!/usr/bin/env python3
"""
Pure follower-state-machine helper module.
"""

from __future__ import annotations

from enum import Enum, auto

from costmap_utils import Costmap, select_detour_bias


class FollowerState(Enum):
    """Discrete state of the follower's obstacle-avoidance overlay."""

    TRACKING = auto()
    DETOUR = auto()
    EMERGENCY_STOP = auto()
    SEARCH = auto()
    HOLD = auto()


def should_escalate_to_search(
    stationary_duration_s: float,
    emergency_duration_s: float,
    deadlock_timeout_s: float,
    has_unreached_breadcrumbs: bool,
) -> bool:
    """
Escalation fires when *either* the stationary-duration timer or the
"""
    timed_out = (
        stationary_duration_s > deadlock_timeout_s
        or emergency_duration_s > deadlock_timeout_s
    )
    return timed_out and has_unreached_breadcrumbs


def classify_state(
    *,
    goal_blocked: bool,
    both_sides_blocked: bool,
    safety_emergency: bool,
    hold_active: bool,
    stationary_duration_s: float,
    emergency_duration_s: float,
    deadlock_timeout_s: float,
    has_unreached_breadcrumbs: bool,
) -> FollowerState:
    """
Inputs (all keyword-only to keep call sites self-documenting):
"""
    if hold_active:
        return FollowerState.HOLD

    if safety_emergency:
        if should_escalate_to_search(
            stationary_duration_s,
            emergency_duration_s,
            deadlock_timeout_s,
            has_unreached_breadcrumbs,
        ):
            return FollowerState.SEARCH
        return FollowerState.EMERGENCY_STOP

    if goal_blocked:
        if both_sides_blocked:
            if should_escalate_to_search(
                stationary_duration_s,
                emergency_duration_s,
                deadlock_timeout_s,
                has_unreached_breadcrumbs,
            ):
                return FollowerState.SEARCH
            return FollowerState.HOLD
        return FollowerState.DETOUR

    return FollowerState.TRACKING


def build_detour_command(
    cm: Costmap,
    pursuit_linear: float,
    max_linear: float,
    max_angular: float,
    detour_forward_min_vel: float,
) -> tuple[float, float]:
    """
The angular component comes from
"""
    angular = select_detour_bias(cm, max_angular)
    # Enforce a minimum turn magnitude. A weak bias (|angular| < 0.5 rad/s)
    min_detour_angular = min(0.5, max_angular)
    if angular == 0.0:
        angular = min_detour_angular
    elif 0.0 < angular < min_detour_angular:
        angular = min_detour_angular
    elif -min_detour_angular < angular < 0.0:
        angular = -min_detour_angular
    linear = max(detour_forward_min_vel, pursuit_linear)
    if linear < 0.0:
        linear = 0.0
    if linear > max_linear:
        linear = max_linear
    return (linear, angular)


def build_search_command(
    search_angular_velocity: float,
    max_angular: float,
    reverse_speed: float = -0.08,
) -> tuple[float, float]:
    """
Backing up while turning moves the robot away from convex
"""
    a = search_angular_velocity
    if a > max_angular:
        a = max_angular
    if a < -max_angular:
        a = -max_angular
    return (reverse_speed, a)
