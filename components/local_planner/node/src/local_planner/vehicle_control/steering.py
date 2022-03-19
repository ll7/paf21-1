"""A module providing steering control implementations"""

from typing import Tuple, List
from dataclasses import dataclass
from math import atan, dist

import numpy as np

from local_planner.core import Vehicle
from local_planner.core.geometry import \
    points_to_vector, norm_angle, vector_len, vector_to_dir


@dataclass
class NaiveSteeringController:
    """Representing a very straight-forward implementation of a steering
    controller driving towards an aim point in front of the vehicle."""
    vehicle: Vehicle

    def compute_steering_angle(self, route: List[Tuple[float, float]]) -> float:
        """Compute the steering angle for driving towards an aim point"""
        if not self.vehicle.is_ready:
            return 0.0

        aim_point = self._get_aim_point(route)
        if not aim_point:
            return 0.0

        aim_vector = points_to_vector(self.vehicle.pos, aim_point)
        aim_angle_rad = vector_to_dir(aim_vector)
        steer_angle = norm_angle(aim_angle_rad - self.vehicle.orientation_rad)

        max_angle = self.vehicle.meta.max_steer_angle_rad
        steer_angle = min(max(steer_angle, -max_angle), max_angle)
        return steer_angle

    @property
    def min_dist_ahead(self) -> float:
        speed = self.vehicle.velocity_mps
        if speed < 45.0 / 3.6:
            return 6.0
        if speed < 55.0 / 3.6:
            return 8.0
        if speed < 65.0 / 3.6:
            return 15.0
        if speed < 75.0 / 3.6:
            return 17.0
        return 22.0

    def _get_aim_point(self, route: List[Tuple[float, float]]) -> Tuple[float, float] or None:
        if len(route) < 2:
            return None

        has_min_dist = lambda wp: dist(wp, self.vehicle.pos) >= self.min_dist_ahead
        wps_with_min_dist = [wp for wp in route[1:] if has_min_dist(wp)]
        aim_point = wps_with_min_dist[0] if len(wps_with_min_dist) > 0 else None

        return aim_point


@dataclass
class StanleySteeringController:
    """Representing a steering controller implementing the Stanley method."""
    vehicle: Vehicle
    curvature: float=2 
    eps: float=1e-6

    def compute_steering_angle(self, route: List[Tuple[float, float]]) -> float:
        """Compute the steering angle according to the Stanley method."""

        if not self.vehicle.is_ready or len(route) < 2:
            return 0.0

        prev_wp, next_wp = route[0], route[1]
        heading_error = self._heading_error(prev_wp, next_wp)
        cross_track_error = self._cross_track_error(prev_wp, next_wp)
        steer_angle = heading_error + cross_track_error

        max_angle = self.vehicle.meta.max_steer_angle_rad
        steer_angle = min(max(steer_angle, -max_angle), max_angle)
        return steer_angle

    def _heading_error(self, prev_wp: Tuple[float, float], next_wp: Tuple[float, float]) -> float:
        vec_traj = points_to_vector(prev_wp, next_wp)
        dir_traj = vector_to_dir(vec_traj)
        return norm_angle(dir_traj - self.vehicle.orientation_rad)

    def _cross_track_error(self, prev_wp: Tuple[float, float],
                           next_wp: Tuple[float, float]) -> float:
        prev_to_next = points_to_vector(prev_wp, next_wp)
        prev_to_vehicle = points_to_vector(prev_wp, self.vehicle.pos)
        cross_prod = -np.cross(prev_to_next, prev_to_vehicle)
        traj_len = vector_len(prev_to_next)
        e_t = cross_prod / traj_len

        arg = (self.curvature * e_t) / (self.eps + self.vehicle.velocity_mps)
        return atan(arg)
