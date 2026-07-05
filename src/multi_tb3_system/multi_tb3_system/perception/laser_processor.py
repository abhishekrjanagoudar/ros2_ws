#!/usr/bin/env python3
"""
Utility module for LiDAR data processing in the Multi-TurtleBot3 convoy system.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


# Data structures

@dataclass
class Cluster:
    """Represents a group of LiDAR points that form a single detected object."""
    points: List[Tuple[float, float]]   # List of (x, y) in robot-local frame
    centroid_x: float                    # Mean x [m]
    centroid_y: float                    # Mean y [m]
    distance: float                      # Distance from robot origin [m]
    angle: float                         # Angle from robot heading [rad]
    size: int                            # Number of points in cluster
    physical_width: float                # Euclidean distance between edge points [m]
    confidence: float = 0.0              # Sensor fusion confidence [0.0 to 1.0]

    def __repr__(self) -> str:
        return (f"Cluster(dist={self.distance:.2f}m, "
                f"angle={math.degrees(self.angle):.1f}°, "
                f"size={self.size}, "
                f"width={self.physical_width:.2f}m, "
                f"conf={self.confidence:.2f})")


# Core processing functions

def scan_to_cartesian(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 3.5,
) -> List[Tuple[float, float]]:
    """
Convert a LaserScan range array to a list of valid (x, y) Cartesian points
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
    center_angle_deg: float = 0.0,
    min_x: float = 0.0,
) -> List[Tuple[float, float]]:
    """Keep only points within ±half_angle_deg of the given center_angle_deg."""
    half_angle_rad = math.radians(half_angle_deg)
    center_angle_rad = math.radians(center_angle_deg)
    filtered = []
    for (x, y) in points:
        if x < min_x:
            continue
        angle = math.atan2(y, x)
        diff = angle - center_angle_rad
        wrapped_diff = math.atan2(math.sin(diff), math.cos(diff))
        if abs(wrapped_diff) <= half_angle_rad:
            filtered.append((x, y))
    return filtered


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
    min_physical_width: float = 0.05,
    max_physical_width: float = 0.30,
) -> List[Cluster]:
    """Convert raw point groups into Cluster objects, filtering noise and walls."""
    result: List[Cluster] = []
    for pts in raw_clusters:
        n = len(pts)
        if n < min_cluster_size or n > max_cluster_size:
            continue
            
        # Calculate physical width
        first, last = pts[0], pts[-1]
        width = math.hypot(last[0] - first[0], last[1] - first[1])
        if width < min_physical_width or width > max_physical_width:
            continue

        cx, cy = compute_centroid(pts)
        result.append(Cluster(
            points=pts,
            centroid_x=cx,
            centroid_y=cy,
            distance=math.hypot(cx, cy),
            angle=math.atan2(cy, cx),
            size=n,
            physical_width=width,
        ))
    return result


def select_target_cluster(
    clusters: List[Cluster],
    last_target_pos: Optional[Tuple[float, float]] = None,
    expected_local_pos: Optional[Tuple[float, float]] = None,
    lock_radius: float = 0.4,
) -> Optional[Cluster]:
    """
    Select the best candidate cluster to follow and calculate confidence.
    """
    if not clusters:
        return None

    # 1. Try cross-cycle continuity first (tight physical track)
    if last_target_pos is not None:
        ref_x, ref_y = last_target_pos
        best = min(clusters, key=lambda c: math.hypot(c.centroid_x - ref_x, c.centroid_y - ref_y))
        dist_to_ref = math.hypot(best.centroid_x - ref_x, best.centroid_y - ref_y)
        
        if dist_to_ref <= lock_radius * 0.5:
            dist_conf = max(0.0, 1.0 - (dist_to_ref / lock_radius))
            width_error = abs(best.physical_width - 0.14)
            width_conf = max(0.0, 1.0 - (width_error / 0.15))
            best.confidence = dist_conf * width_conf
            return best

    # 2. Fall back to the open-loop breadcrumb prediction
    if expected_local_pos is not None:
        ref_x, ref_y = expected_local_pos
        best = min(clusters, key=lambda c: math.hypot(c.centroid_x - ref_x, c.centroid_y - ref_y))
        dist_to_ref = math.hypot(best.centroid_x - ref_x, best.centroid_y - ref_y)
        
        if dist_to_ref <= lock_radius:
            dist_conf = max(0.0, 1.0 - (dist_to_ref / lock_radius))
            width_error = abs(best.physical_width - 0.14)
            width_conf = max(0.0, 1.0 - (width_error / 0.15))
            best.confidence = dist_conf * width_conf
            return best

    # 3. Fallback to closest cluster if nothing is in radius (forces rejection via 0 confidence)
    best = min(clusters, key=lambda c: c.distance)
    best.confidence = 0.0
    return best


def process_scan(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 3.5,
    front_half_angle_deg: float = 60.0,
    center_angle_deg: float = 0.0,
    cluster_distance: float = 0.20,
    min_cluster_size: int = 2,
    max_cluster_size: int = 40,
    last_target_pos: Optional[Tuple[float, float]] = None,
    expected_local_pos: Optional[Tuple[float, float]] = None,
    lock_radius: float = 0.4,
) -> Tuple[Optional[Cluster], List[Cluster]]:
    """
    Full pipeline: raw LaserScan → (target_cluster, all_clusters).
    """
    points      = scan_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max)
    front       = filter_front_sector(points, half_angle_deg=front_half_angle_deg, center_angle_deg=center_angle_deg)
    raw         = euclidean_cluster(front, cluster_distance=cluster_distance)
    clusters    = make_clusters(raw, min_cluster_size, max_cluster_size)
    target      = select_target_cluster(
        clusters, 
        last_target_pos=last_target_pos, 
        expected_local_pos=expected_local_pos,
        lock_radius=lock_radius
    )
    return target, clusters
