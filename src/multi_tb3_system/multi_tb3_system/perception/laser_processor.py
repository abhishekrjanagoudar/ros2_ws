#!/usr/bin/env python3
"""
laser_processor.py
==================
Utility module for LiDAR data processing in the Multi-TurtleBot3 convoy system.

Responsibilities:
  - Convert LaserScan polar data to Cartesian points
  - Apply front-sector filtering (± angle_limit degrees)
  - Cluster points using Euclidean distance segmentation
  - Extract cluster centroids
  - Filter out wall-like (large) and noise (tiny) clusters

NO temporal tracking is performed here. This module is purely geometric.

Status
------
Not used by any running node — kept here for future work (e.g. a
LiDAR-based leader-detection follower that does not depend on the
breadcrumb path).  Import path: ``multi_tb3_system.perception.laser_processor``
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


# ─── Data structures ──────────────────────────────────────────────────────────

@dataclass
class Cluster:
    """Represents a group of LiDAR points that form a single detected object."""
    points: List[Tuple[float, float]]   # List of (x, y) in robot-local frame
    centroid_x: float                    # Mean x [m]
    centroid_y: float                    # Mean y [m]
    distance: float                      # Distance from robot origin [m]
    angle: float                         # Angle from robot heading [rad]
    size: int                            # Number of points in cluster

    def __repr__(self) -> str:
        return (f"Cluster(dist={self.distance:.2f}m, "
                f"angle={math.degrees(self.angle):.1f}°, "
                f"size={self.size})")


# ─── Core processing functions ─────────────────────────────────────────────────

def scan_to_cartesian(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 3.5,
) -> List[Tuple[float, float]]:
    """
    Convert a LaserScan range array to a list of valid (x, y) Cartesian points
    in the robot's local coordinate frame.

    Robot convention:
      - x → forward
      - y → left
      - angle=0 → straight ahead
    """
    points = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        if r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((x, y))
    return points


def filter_front_sector(
    points: List[Tuple[float, float]],
    half_angle_deg: float = 30.0,
    min_x: float = 0.0,
) -> List[Tuple[float, float]]:
    """Keep only points within ±half_angle_deg of straight ahead."""
    half_angle_rad = math.radians(half_angle_deg)
    return [
        (x, y) for (x, y) in points
        if x >= min_x and abs(math.atan2(y, x)) <= half_angle_rad
    ]


def euclidean_cluster(
    points: List[Tuple[float, float]],
    cluster_distance: float = 0.20,
) -> List[List[Tuple[float, float]]]:
    """Segment angle-sorted 2-D points into distance-gated clusters."""
    if not points:
        return []
    sorted_pts = sorted(points, key=lambda p: math.atan2(p[1], p[0]))
    clusters: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = [sorted_pts[0]]
    for i in range(1, len(sorted_pts)):
        prev, curr = sorted_pts[i - 1], sorted_pts[i]
        if math.hypot(curr[0] - prev[0], curr[1] - prev[1]) <= cluster_distance:
            current.append(curr)
        else:
            clusters.append(current)
            current = [curr]
    clusters.append(current)
    return clusters


def compute_centroid(cluster_points: List[Tuple[float, float]]) -> Tuple[float, float]:
    """Return mean (x, y) of a cluster."""
    if not cluster_points:
        return (0.0, 0.0)
    xs = [p[0] for p in cluster_points]
    ys = [p[1] for p in cluster_points]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def make_clusters(
    raw_clusters: List[List[Tuple[float, float]]],
    min_cluster_size: int = 2,
    max_cluster_size: int = 40,
) -> List[Cluster]:
    """Convert raw point groups into Cluster objects, filtering noise and walls."""
    result: List[Cluster] = []
    for pts in raw_clusters:
        n = len(pts)
        if n < min_cluster_size or n > max_cluster_size:
            continue
        cx, cy = compute_centroid(pts)
        result.append(Cluster(
            points=pts,
            centroid_x=cx,
            centroid_y=cy,
            distance=math.hypot(cx, cy),
            angle=math.atan2(cy, cx),
            size=n,
        ))
    return result


def select_target_cluster(
    clusters: List[Cluster],
    last_target_pos: Optional[Tuple[float, float]] = None,
    lock_radius: float = 0.4,
) -> Optional[Cluster]:
    """
    Select the best candidate cluster to follow.

    Prefers the cluster closest to the previous target position (target lock);
    falls back to the closest cluster to the robot.
    """
    if not clusters:
        return None
    if last_target_pos is not None:
        lx, ly = last_target_pos
        best = min(clusters, key=lambda c: math.hypot(c.centroid_x - lx, c.centroid_y - ly))
        if math.hypot(best.centroid_x - lx, best.centroid_y - ly) <= lock_radius:
            return best
    return min(clusters, key=lambda c: c.distance)


def process_scan(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 3.5,
    front_half_angle_deg: float = 30.0,
    cluster_distance: float = 0.20,
    min_cluster_size: int = 2,
    max_cluster_size: int = 40,
    last_target_pos: Optional[Tuple[float, float]] = None,
) -> Tuple[Optional[Cluster], List[Cluster]]:
    """
    Full pipeline: raw LaserScan → (target_cluster, all_clusters).

    Steps: polar→Cartesian → front-sector filter → cluster → size-filter → select.
    """
    points      = scan_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max)
    front       = filter_front_sector(points, half_angle_deg=front_half_angle_deg)
    raw         = euclidean_cluster(front, cluster_distance=cluster_distance)
    clusters    = make_clusters(raw, min_cluster_size, max_cluster_size)
    target      = select_target_cluster(clusters, last_target_pos=last_target_pos)
    return target, clusters
