#!/usr/bin/env python3
"""
Lightweight local planner for obstacle avoidance.
Similar to DWA (Dynamic Window Approach) but simplified for convoy following.
"""

from __future__ import annotations

import math
from typing import List, Tuple
from dataclasses import dataclass

from costmap_utils import Costmap


@dataclass
class VelocityCandidate:
    """A candidate velocity command with its evaluation score."""
    linear: float
    angular: float
    score: float
    collision: bool


class LocalPlanner:
    """
    Lightweight local planner that generates collision-free velocities
    toward a goal point using costmap-based obstacle avoidance.
    
    Similar to DWA (Dynamic Window Approach) but simplified for convoy use.
    """
    
    def __init__(
        self,
        max_linear_vel: float = 0.22,
        max_angular_vel: float = 1.0,
        max_linear_acc: float = 2.0,
        max_angular_acc: float = 3.0,
        velocity_samples: int = 10,
        angular_samples: int = 15,
        predict_time: float = 1.5,
        robot_radius: float = 0.25,
        goal_weight: float = 1.0,
        velocity_weight: float = 0.2,
        obstacle_weight: float = 2.0,
        control_period: float = 0.05,
    ):
        """
        Initialize the local planner.
        
        Args:
            max_linear_vel: Maximum linear velocity [m/s]
            max_angular_vel: Maximum angular velocity [rad/s]
            max_linear_acc: Maximum linear acceleration [m/s²]
            max_angular_acc: Maximum angular acceleration [rad/s²]
            velocity_samples: Number of linear velocity samples
            angular_samples: Number of angular velocity samples
            predict_time: Trajectory prediction horizon [s]
            robot_radius: Robot radius for collision checking [m]
            goal_weight: Weight for goal distance in scoring
            velocity_weight: Weight for velocity preference (favor higher speeds)
            obstacle_weight: Weight for obstacle clearance
            control_period: Control loop period [s]
        """
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.max_linear_acc = max_linear_acc
        self.max_angular_acc = max_angular_acc
        self.velocity_samples = velocity_samples
        self.angular_samples = angular_samples
        self.predict_time = predict_time
        self.robot_radius = robot_radius
        self.goal_weight = goal_weight
        self.velocity_weight = velocity_weight
        self.obstacle_weight = obstacle_weight
        self.control_period = control_period
        
        # Current velocity for dynamic window calculation
        self.current_linear = 0.0
        self.current_angular = 0.0
    
    def update_current_velocity(self, linear: float, angular: float):
        """Update the current velocity for dynamic window calculation."""
        self.current_linear = linear
        self.current_angular = angular
    
    def compute_velocity(
        self,
        goal_x: float,
        goal_y: float,
        costmap: Costmap,
        current_yaw: float = 0.0,
    ) -> Tuple[float, float]:
        """
        Compute collision-free velocity toward goal using costmap.
        
        Args:
            goal_x: Goal x position in robot frame [m]
            goal_y: Goal y position in robot frame [m]
            costmap: Occupancy costmap for obstacle detection
            current_yaw: Current robot yaw (for trajectory prediction) [rad]
            
        Returns:
            (linear_vel, angular_vel): Best velocity command
        """
        # Generate velocity candidates within dynamic window
        candidates = self._generate_candidates()
        
        # Evaluate each candidate
        for candidate in candidates:
            # Predict trajectory
            trajectory = self._predict_trajectory(
                candidate.linear, candidate.angular, current_yaw
            )
            
            # Check collision
            candidate.collision = self._check_collision(trajectory, costmap)
            
            # Compute score (only for collision-free trajectories)
            if not candidate.collision:
                candidate.score = self._evaluate_trajectory(
                    candidate.linear, candidate.angular,
                    trajectory, goal_x, goal_y, costmap
                )
            else:
                candidate.score = -1e6  # Very negative score for collision
        
        # Select best candidate
        best = max(candidates, key=lambda c: c.score)
        
        # If all candidates collide, stop
        if best.collision:
            return 0.0, 0.0
        
        return best.linear, best.angular
    
    def _generate_candidates(self) -> List[VelocityCandidate]:
        """Generate velocity candidates within dynamic window."""
        candidates = []
        
        # Dynamic window: reachable velocities from current velocity
        # Allow backward motion for escape
        min_linear = max(
            -0.10,  # Allow slow backward motion for escape
            self.current_linear - self.max_linear_acc * self.control_period
        )
        max_linear = min(
            self.max_linear_vel,
            self.current_linear + self.max_linear_acc * self.control_period
        )
        
        min_angular = max(
            -self.max_angular_vel,
            self.current_angular - self.max_angular_acc * self.control_period
        )
        max_angular = min(
            self.max_angular_vel,
            self.current_angular + self.max_angular_acc * self.control_period
        )
        
        # Sample velocities
        if max_linear > min_linear:
            linear_step = (max_linear - min_linear) / max(1, self.velocity_samples - 1)
        else:
            linear_step = 0.0
        
        if max_angular > min_angular:
            angular_step = (max_angular - min_angular) / max(1, self.angular_samples - 1)
        else:
            angular_step = 0.0
        
        for i in range(self.velocity_samples):
            linear = min_linear + i * linear_step
            for j in range(self.angular_samples):
                angular = min_angular + j * angular_step
                candidates.append(VelocityCandidate(linear, angular, 0.0, False))
        
        return candidates
    
    def _predict_trajectory(
        self,
        linear: float,
        angular: float,
        current_yaw: float,
    ) -> List[Tuple[float, float]]:
        """
        Predict trajectory for given velocities.
        
        Returns list of (x, y) positions in robot frame.
        """
        trajectory = []
        dt = 0.1  # Prediction time step
        steps = int(self.predict_time / dt)
        
        x, y, yaw = 0.0, 0.0, 0.0  # Start from robot origin (robot frame)
        
        for _ in range(steps):
            # Simple motion model: constant velocity
            x += linear * math.cos(yaw) * dt
            y += linear * math.sin(yaw) * dt
            yaw += angular * dt
            trajectory.append((x, y))
        
        return trajectory
    
    def _check_collision(
        self,
        trajectory: List[Tuple[float, float]],
        costmap: Costmap,
    ) -> bool:
        """Check if trajectory collides with obstacles in costmap."""
        for x, y in trajectory:
            # Check circle around predicted position
            if self._is_occupied(x, y, costmap):
                return True
        return False
    
    def _is_occupied(self, x: float, y: float, costmap: Costmap) -> bool:
        """Check if position (with robot radius) is occupied in costmap."""
        # Convert to costmap coordinates
        # Convert to costmap coordinates
        cx = int((x - costmap.origin_x) / costmap.resolution)
        cy = int((y - costmap.origin_y) / costmap.resolution)
        
        # Check cells within robot radius
        radius_cells = int(self.robot_radius / costmap.resolution) + 1
        
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx*dx + dy*dy > radius_cells*radius_cells:
                    continue
                
                check_x = cx + dx
                check_y = cy + dy
                
                # Bounds check
                if (0 <= check_x < costmap.width and 
                    0 <= check_y < costmap.height):
                    if costmap.data[check_y * costmap.width + check_x] > 50:
                        return True
        
        return False
    
    def _evaluate_trajectory(
        self,
        linear: float,
        angular: float,
        trajectory: List[Tuple[float, float]],
        goal_x: float,
        goal_y: float,
        costmap: Costmap,
    ) -> float:
        """
        Evaluate trajectory quality.
        
        Higher score = better trajectory.
        """
        # 1. Goal distance: how close does trajectory end get to goal?
        end_x, end_y = trajectory[-1] if trajectory else (0.0, 0.0)
        dist_to_goal = math.hypot(goal_x - end_x, goal_y - end_y)
        goal_score = -dist_to_goal  # Negative because closer is better
        
        # 2. Velocity preference: favor higher speeds (more progress)
        velocity_score = linear
        
        # 3. Obstacle clearance: favor trajectories far from obstacles
        min_clearance = float('inf')
        for x, y in trajectory:
            clearance = self._get_clearance(x, y, costmap)
            min_clearance = min(min_clearance, clearance)
        obstacle_score = min_clearance
        
        # Combined score
        total_score = (
            self.goal_weight * goal_score +
            self.velocity_weight * velocity_score +
            self.obstacle_weight * obstacle_score
        )
        
        return total_score
    
    def _get_clearance(self, x: float, y: float, costmap: Costmap) -> float:
        """Get minimum distance to nearest obstacle from position."""
        cx = int((x - costmap.origin_x) / costmap.resolution)
        cy = int((y - costmap.origin_y) / costmap.resolution)
        
        # Bounds check
        if not (0 <= cx < costmap.width and 0 <= cy < costmap.height):
            return 0.0
        
        # Search for nearest obstacle within reasonable radius
        search_radius = 20  # cells
        min_dist = float('inf')
        
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x = cx + dx
                check_y = cy + dy
                
                if (0 <= check_x < costmap.width and 
                    0 <= check_y < costmap.height):
                    if costmap.data[check_y * costmap.width + check_x] > 50:
                        dist = math.hypot(dx, dy) * costmap.resolution
                        min_dist = min(min_dist, dist)
        
        return min_dist if min_dist != float('inf') else 10.0  # Max clearance
