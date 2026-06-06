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

    Args:
        ranges:          Array of range measurements [m]
        angle_min:       Starting angle of the scan [rad]
        angle_increment: Angular step size per ray [rad]
        range_min:       Minimum valid range to accept [m]
        range_max:       Maximum valid range to accept [m]

    Returns:
        List of (x, y) tuples for valid points.
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
    """
    Keep only points that are within the front-facing sector of the robot.

    The front sector is defined as:
      - x > min_x          (in front of the robot)
      - |angle| < half_angle_deg  (within ±half_angle_deg of straight ahead)

    Args:
        points:          List of (x, y) Cartesian points
        half_angle_deg:  Half-width of the front cone in degrees (default ±30°)
        min_x:           Minimum forward distance to include (default 0.0)

    Returns:
        Filtered list of (x, y) points in the front sector.
    """
    half_angle_rad = math.radians(half_angle_deg)
    filtered = []
    for (x, y) in points:
        if x < min_x:
            continue
        angle = math.atan2(y, x)
        if abs(angle) <= half_angle_rad:
            filtered.append((x, y))
    return filtered


def euclidean_cluster(
    points: List[Tuple[float, float]],
    cluster_distance: float = 0.20,
) -> List[List[Tuple[float, float]]]:
    """
    Segment a list of 2D points into clusters using Euclidean distance.

    A new cluster is started whenever the distance between consecutive points
    (in angle-sorted order) exceeds cluster_distance.

    Args:
        points:           List of (x, y) Cartesian points
        cluster_distance: Maximum gap between points in the same cluster [m]

    Returns:
        A list of clusters, where each cluster is a list of (x, y) tuples.
    """
    if not points:
        return []

    # Sort by angle to group angularly-adjacent points (most reliable ordering)
    sorted_pts = sorted(points, key=lambda p: math.atan2(p[1], p[0]))

    clusters: List[List[Tuple[float, float]]] = []
    current_cluster: List[Tuple[float, float]] = [sorted_pts[0]]

    for i in range(1, len(sorted_pts)):
        prev = sorted_pts[i - 1]
        curr = sorted_pts[i]
        dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        if dist <= cluster_distance:
            current_cluster.append(curr)
        else:
            clusters.append(current_cluster)
            current_cluster = [curr]

    clusters.append(current_cluster)
    return clusters


def compute_centroid(cluster_points: List[Tuple[float, float]]) -> Tuple[float, float]:
    """
    Compute the mean (x, y) centroid of a cluster.

    Args:
        cluster_points: List of (x, y) point tuples.

    Returns:
        (cx, cy) centroid coordinates.
    """
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
    """
    Convert raw point groups into Cluster objects, applying size filters.

    Filtering logic:
      - Clusters smaller than min_cluster_size → noise (rejected)
      - Clusters larger than max_cluster_size → wall / large surface (rejected)
      - Remaining clusters are converted to Cluster dataclass objects

    Args:
        raw_clusters:     List of point groups from euclidean_cluster()
        min_cluster_size: Minimum number of points to be a valid object
        max_cluster_size: Maximum number of points (above → wall/surface)

    Returns:
        List of Cluster objects representing valid detected objects.
    """
    result: List[Cluster] = []
    for pts in raw_clusters:
        n = len(pts)
        if n < min_cluster_size or n > max_cluster_size:
            continue
        cx, cy = compute_centroid(pts)
        dist = math.hypot(cx, cy)
        angle = math.atan2(cy, cx)
        result.append(Cluster(
            points=pts,
            centroid_x=cx,
            centroid_y=cy,
            distance=dist,
            angle=angle,
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

    Strategy:
      - If we have a previous target position, find the cluster closest to it
        (temporal tracking / target lock).
      - If no valid previous target, or if all clusters are outside the lock radius,
        pick the cluster closest to the robot.

    Args:
        clusters: List of valid Cluster objects
        last_target_pos: (x, y) centroid of target from previous scan
        lock_radius: Max distance a cluster can move between scans to keep lock [m]

    Returns:
        The closest Cluster, or None if no clusters exist.
    """
    if not clusters:
        return None

    if last_target_pos is not None:
        lx, ly = last_target_pos
        # Find the cluster closest to the last known position
        best_cluster = min(clusters, key=lambda c: math.hypot(c.centroid_x - lx, c.centroid_y - ly))
        dist_to_last = math.hypot(best_cluster.centroid_x - lx, best_cluster.centroid_y - ly)
        
        # If it hasn't jumped too far, maintain lock
        if dist_to_last <= lock_radius:
            return best_cluster

    # Fallback to the closest cluster to the robot
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
    Full pipeline: raw LaserScan → target cluster.

    Steps:
      1. Convert scan to Cartesian points (filter invalid ranges)
      2. Filter points to front sector only
      3. Euclidean cluster segmentation
      4. Build Cluster objects (filter by size)
      5. Select closest cluster as target

    Args:
        ranges:               LaserScan.ranges array
        angle_min:            LaserScan.angle_min
        angle_increment:      LaserScan.angle_increment
        range_min:            Minimum range to accept
        range_max:            Maximum range to accept
        front_half_angle_deg: Front cone half-angle (degrees)
        cluster_distance:     Gap threshold for clustering
        min_cluster_size:     Minimum cluster size (noise filter)
        max_cluster_size:     Maximum cluster size (wall filter)
        last_target_pos:      Previous known target (x,y) for target lock

    Returns:
        (target_cluster, all_clusters)
        - target_cluster: The selected leader cluster, or None
        - all_clusters:   All valid front-sector clusters detected
    """
    # Step 1: Polar → Cartesian
    points = scan_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max)

    # Step 2: Front sector filter
    front_points = filter_front_sector(points, half_angle_deg=front_half_angle_deg)

    # Step 3: Cluster segmentation
    raw_clusters = euclidean_cluster(front_points, cluster_distance=cluster_distance)

    # Step 4: Build Cluster objects with size filtering
    clusters = make_clusters(raw_clusters, min_cluster_size, max_cluster_size)

    # Step 5: Select target
    target = select_target_cluster(clusters, last_target_pos=last_target_pos)

    return target, clusters
