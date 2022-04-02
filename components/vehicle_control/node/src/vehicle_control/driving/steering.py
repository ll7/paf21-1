"""A module providing steering control implementations"""
from typing import Tuple, List, Protocol
from dataclasses import dataclass, field
from math import dist, atan2, ceil

import numpy as np

from vehicle_control.core import Vehicle
from vehicle_control.core.geometry import \
    points_to_vector, norm_angle, vector_len, vector_to_dir


class SteeringController(Protocol):
    # pylint: disable=too-few-public-methods
    """Representing a generalization of a steering controller."""

    def compute_steering_angle(self, route: List[Tuple[float, float]]) -> float:
        """Compute the steering angle given the route waypoints."""
        ...


@dataclass
class StanleySteeringController:
    """Representing a steering controller implementing the Stanley method."""
    refresh_rate: int
    vehicle: Vehicle
    curvature_value: float = 2.5
    curvature: float = 0.0
    factor: float = 1
    eps: float = 1e-6
    last_pos: Tuple[float, float] = (0, 0)
    cross_track_errors: List[float] = field(default_factory=list)
    cte_count: int = 1
    init = False
    pred_weights: List[float] = field(default_factory=lambda: [0.3, 0.25, 0.2, 0.15, 0.1])

    # TODO: remove a majority of all the weird edge cases caused by unintended statefulness

    def __post_init__(self):
        self.cte_count = ceil(self.refresh_rate / 2)

    def compute_steering_angle(self, route: List[Tuple[float, float]]):
        """Compute the steering angle by combining multiple predictions
        of the Stanley method, looking at different amounts of wapoints ahead."""

        if not self.vehicle.is_ready or len(route) < 2:
            return 0.0

        pos = self.vehicle.pos
        theta = self.vehicle.orientation_rad
        velocity = self.vehicle.velocity_mps
        timestep = 1 / self.refresh_rate
        temp_steering_angle = self._vanilla_stanley(route, pos, theta)
        steering_angle = self.pred_weights[0] * temp_steering_angle

        for step in range(1, len(self.pred_weights)):
            x_n = pos[0] + velocity * np.cos(theta + temp_steering_angle) * timestep
            y_n = pos[1] + velocity * np.sin(theta + temp_steering_angle) * timestep
            theta = theta + ((velocity * np.tan(temp_steering_angle)) /
                              self.vehicle.meta.wheelbase) * timestep
            pos = (x_n, y_n)
            temp_steering_angle = self._vanilla_stanley(route, pos, theta)
            steering_angle += self.pred_weights[step] * temp_steering_angle

        self.vehicle.steer_angle = steering_angle
        return steering_angle

    def _vanilla_stanley(self, route: List[Tuple[float, float]],
                               position: Tuple[float, float], orientation: float) -> float:
        """Compute the steering angle according to the Stanley method."""

        prev_wp, next_wp = self._get_prev_and_next_point(route, position)

        traveled_distance = 0
        if self.last_pos and position:
            traveled_distance = dist(self.last_pos, position)
        self.last_pos = position

        if not prev_wp or not next_wp or prev_wp == next_wp:
            return 0.0

        heading_error = self._heading_error(prev_wp, next_wp, orientation)

        # reset the recent cross-track errors when the car is standing still
        # -> this fixes the issue of exploding cross-track errors
        if self.init:
            self.cross_track_errors.insert(0, self._cross_track_error(prev_wp, next_wp, position))
            if len(self.cross_track_errors) > self.cte_count:
                self.cross_track_errors.pop()
            cross_track_error = sum(self.cross_track_errors) / len(self.cross_track_errors)
            if self.vehicle.velocity_mps < 1:
                self.init = False
                self.cross_track_errors = []
        else:
            cross_track_error = self._cross_track_error(prev_wp, next_wp, position)
            if traveled_distance > 0 and self.vehicle.velocity_mps > 1:
                self.init = True
                self.curvature = self.curvature_value

        steer_angle = self.factor * heading_error + cross_track_error
        max_angle = self.vehicle.meta.max_steer_angle_rad
        steer_angle = min(max(steer_angle, -max_angle), max_angle)
        return steer_angle

    @property
    def min_dist_ahead(self) -> float:
        """Compute the horizon of waypoints ahead that are considered
        by the Standley controller to predict accurate steering angles.

        This computation relies on the vehicle's speed in a way that the faster
        the car drives, the more points ahead need to be considered."""
        speed = self.vehicle.velocity_mps
        if speed == 0:
            return 0
        time = dist(self.last_pos, self.vehicle.pos) / speed
        gate = ceil(speed * time)
        return gate

    def _get_prev_and_next_point(self, route: List[Tuple[float, float]],
                                 position: Tuple[float, float]) \
                                     -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if len(route) < 2:
            return None, None

        min_dist = self.min_dist_ahead
        for i in range(1, len(route)):
            if dist(route[i], position) < min_dist or \
                dist(route[i], self.vehicle.pos) < dist(position, self.vehicle.pos):
                continue
            return route[i], self._get_next_point(route, i)

        return route[-2], route[-1]

    def _get_next_point(self, route: List[Tuple[float, float]], index) -> Tuple[float, float]:
        if len(route) < 2:
            return None
        min_dist = self.min_dist_ahead
        for i in range(index + 1, len(route)):
            if dist(route[i], route[index]) >= min_dist:
                return route[i]
        return route[-1]

    def _heading_error(self, prev_wp: Tuple[float, float], next_wp: Tuple[float, float],
                       orientation: float) -> float:
        vec_traj = points_to_vector(prev_wp, next_wp)
        dir_traj = vector_to_dir(vec_traj)
        return norm_angle(dir_traj - orientation)

    def _cross_track_error(self, prev_wp: Tuple[float, float],
                           next_wp: Tuple[float, float], position: Tuple[float, float]) -> float:
        prev_to_next = points_to_vector(prev_wp, next_wp)
        prev_to_vehicle = points_to_vector(prev_wp, position)
        cross_prod = -np.cross(prev_to_next, prev_to_vehicle)
        traj_len = vector_len(prev_to_next)
        e_t = cross_prod / traj_len

        return atan2((self.curvature * e_t), self.eps + self.vehicle.velocity_mps)
