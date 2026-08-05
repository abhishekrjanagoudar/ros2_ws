#!/usr/bin/env python3
"""
Pure follower-state-machine helper module.
"""

from __future__ import annotations

from enum import Enum, auto


class FollowerState(Enum):
    """Discrete state of the follower's obstacle-avoidance overlay."""

    TRACKING = auto()
    DETOUR = auto()
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

    if both_sides_blocked and should_escalate_to_search(
        stationary_duration_s,
        emergency_duration_s,
        deadlock_timeout_s,
        has_unreached_breadcrumbs,
    ):
        return FollowerState.SEARCH

    if goal_blocked:
        return FollowerState.DETOUR

    return FollowerState.TRACKING


def build_search_command(
    search_angular_velocity: float,
    max_angular: float,
    reverse_speed: float = 0.0,
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
