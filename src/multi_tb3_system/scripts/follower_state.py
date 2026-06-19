#!/usr/bin/env python3
"""
follower_state.py
=================
Pure follower-state-machine helper module.

This module is consumed by ``follower_node.py`` to layer the convoy
obstacle-avoidance state machine on top of the existing Pure Pursuit
breadcrumb tracker. It deliberately contains no ``rclpy`` imports and
no ROS message types so the decision logic is fully unit- and
property-testable without a running ROS graph, mirroring the existing
split between ``costmap_utils.py`` / ``convoy_tracking.py`` (pure) and
the nodes that consume them.

The state machine has five states:

* ``TRACKING``       - Pure Pursuit toward the breadcrumb Goal_Point.
* ``DETOUR``         - Goal blocked, steer toward the freer side.
* ``EMERGENCY_STOP`` - SafetyController reports a forward obstacle
                       inside ``safe_distance``.
* ``SEARCH``         - Anti-deadlock rotate-in-place when the
                       follower has been stationary too long while
                       breadcrumbs remain unreached.
* ``HOLD``           - Zero linear / zero angular. Two causes:
                       conditional-stop (R7.2) when at the final goal
                       and the breadcrumb timeout has elapsed, or a
                       momentary blocked-HOLD when both candidate
                       sides of the costmap are fully occupied (R2.6).

Command precedence (each cycle), per the design's
"Command precedence" section:

  1. Conditional-stop HOLD wins (R7.2) — exits immediately on a newer
     breadcrumb upstream.
  2. EMERGENCY_STOP next, with anti-deadlock escalation to SEARCH
     when the emergency persists past ``deadlock_timeout`` and there
     are still unreached breadcrumbs.
  3. Goal-blocked override: DETOUR / blocked-HOLD / SEARCH selected
     from the per-side free-cell counts and the stationary timer.
  4. TRACKING fallback (Pure Pursuit) when nothing else fires.

The ``SafetyController`` hard override is applied *after* this module
selects a base ``(linear, angular)`` command — it is not modeled here.

Public API:

  - ``FollowerState`` enum
  - ``should_escalate_to_search(stationary_duration_s,
    emergency_duration_s, deadlock_timeout_s,
    has_unreached_breadcrumbs) -> bool``
  - ``classify_state(*, goal_blocked, both_sides_blocked,
    safety_emergency, hold_active, stationary_duration_s,
    emergency_duration_s, deadlock_timeout_s,
    has_unreached_breadcrumbs) -> FollowerState``
  - ``build_detour_command(cm, pursuit_linear, max_linear, max_angular,
    detour_forward_min_vel) -> tuple[float, float]``
  - ``build_search_command(search_angular_velocity, max_angular)
    -> tuple[float, float]``
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
    """Return True iff the follower must escalate the current stop to SEARCH.

    Escalation fires when *either* the stationary-duration timer or the
    emergency-duration timer has exceeded the configured deadlock
    timeout *and* there are still unreached breadcrumbs to chase. With
    no unreached breadcrumbs there is nothing to search for, so the
    follower stays put (the conditional-stop HOLD path or a normal
    EMERGENCY_STOP handles that case).

    Validates: Requirements 3.1, 3.4, 3.5
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
    """Pick the next ``FollowerState`` per the design's command-precedence rules.

    Inputs (all keyword-only to keep call sites self-documenting):

    * ``goal_blocked`` - result of ``costmap_utils.is_goal_blocked`` for
      the current Goal_Point in the local costmap frame.
    * ``both_sides_blocked`` - True iff both candidate sides of the
      forward half of the costmap have zero free cells (i.e.
      ``select_detour_bias`` would return ``0.0``).
    * ``safety_emergency`` - True iff the SafetyController would assert
      its emergency-stop hard override this cycle.
    * ``hold_active`` - precomputed result of
      ``convoy_tracking.should_hold(...)``: True iff the conditional
      stop applies (R7.2).
    * ``stationary_duration_s`` / ``emergency_duration_s`` - time the
      follower has been stationary / in emergency-stop, used together
      with ``deadlock_timeout_s`` for the anti-deadlock SEARCH
      escalation (R3.1, R3.5).
    * ``has_unreached_breadcrumbs`` - whether there is still a
      breadcrumb to chase. SEARCH is meaningful only when there is.

    Precedence (matches design "Command precedence"):

      1. ``hold_active``                                  -> HOLD
         (conditional-stop wins; R7.2)
      2. ``safety_emergency``:
            ``should_escalate_to_search(...)``            -> SEARCH
            else                                          -> EMERGENCY_STOP
      3. ``goal_blocked``:
            ``both_sides_blocked``:
                ``should_escalate_to_search(...)``        -> SEARCH
                else                                      -> HOLD
                                                             (blocked-HOLD;
                                                              R2.6)
            else                                          -> DETOUR
      4. otherwise                                        -> TRACKING

    Validates: Requirements 2.4, 3.1, 3.3, 3.4, 3.5, 7.2
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
    """Compute the base ``(linear, angular)`` command for the DETOUR state.

    The angular component comes from
    ``costmap_utils.select_detour_bias(cm, max_angular)``, which already
    clamps to ``[-max_angular, +max_angular]`` and returns ``0.0`` when
    both candidate sides are fully occupied (R2.6). When the bias is
    ``0.0`` this builder returns ``(0.0, 0.0)`` so the caller can
    transition into HOLD / SEARCH per ``classify_state``.

    Otherwise the linear component is the larger of the Pure Pursuit
    desired velocity and ``detour_forward_min_vel`` so the follower
    keeps creeping forward even when Pure Pursuit alone would slow it
    to a near-stop (R2.3, R2.5). The result is clamped to
    ``[0, max_linear]`` so the detour never reverses and never exceeds
    the configured velocity ceiling.

    Validates: Requirements 2.2, 2.3, 2.5, 2.6
    """
    angular = select_detour_bias(cm, max_angular)
    if angular == 0.0:
        # classify_state() routes fully blocked costmaps to HOLD/SEARCH before
        # this builder is called. A zero bias here therefore means the free
        # space is tied; pick a deterministic turn so DETOUR does not degrade
        # into an accidental hold.
        angular = min(0.5, max_angular)
    linear = max(detour_forward_min_vel, pursuit_linear)
    if linear < 0.0:
        linear = 0.0
    if linear > max_linear:
        linear = max_linear
    return (linear, angular)


def build_search_command(
    search_angular_velocity: float,
    max_angular: float,
) -> tuple[float, float]:
    """Compute the base ``(linear, angular)`` command for the SEARCH state.

    SEARCH is a rotate-in-place sweep used when the follower has
    deadlocked on a stationary or emergency stop but still has
    breadcrumbs to chase. The linear component is always ``0.0``; the
    angular component is the configured ``search_angular_velocity``
    clamped to ``[-max_angular, +max_angular]`` so the rotation respects
    the global velocity limit.

    Validates: Requirements 3.4
    """
    a = search_angular_velocity
    if a > max_angular:
        a = max_angular
    if a < -max_angular:
        a = -max_angular
    return (0.0, a)
