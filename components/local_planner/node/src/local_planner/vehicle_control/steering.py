"""A module providing steering control implementations"""
import math
from typing import Tuple, List, Protocol
from dataclasses import dataclass, field
from math import dist, atan2, ceil

import numpy as np

from local_planner.core import Vehicle
from local_planner.core.geometry import \
    points_to_vector, norm_angle, vector_len, vector_to_dir, rotate_vector, add_vector

from local_planner.vehicle_control.curve_detection import CurveDetection


class SteeringController(Protocol):
    # pylint: disable=too-few-public-methods
    """Representing a generalization of a steering controller."""

    def compute_steering_angle(self, route: List[Tuple[float, float]]) -> float:
        """Compute the steering angle given the route waypoints."""
        ...


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

        max_angle = self.vehicle.meta.max_steer_angle_rad / self.vehicle.velocity_mps
        steer_angle = min(max(steer_angle, -max_angle), max_angle)

        return steer_angle

    @property
    def min_dist_ahead(self) -> float:
        speed = self.vehicle.velocity_mps
        return ceil((speed * (dist(self.last_pos, self.vehicle.pos) / speed)))

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
    curvature_value: float = 2.8
    curvature: float = 0.0
    factor: float = 1
    eps: float = 1e-6
    last_pos: Tuple[float, float] = (0, 0)
    cross_track_errors: List[float] = field(default_factory=list)
    cte_count: int = 35
    init = False

    def predictive_stanley(self, route: List[Tuple[float, float]], n: int, k: List[float]):
        assert len(k) == n
        if not self.vehicle.is_ready or len(route) < 2:
            print('canceled 1', self.vehicle.is_ready)
            return 0.0
        pos = self.vehicle.pos
        theta = self.vehicle.orientation_rad
        velocity = self.vehicle.velocity_mps
        timestep = 1 / 40
        temp_steering_angle = self.compute_steering_angle(route, pos, theta)
        steering_angle = k[0] * temp_steering_angle
        for step in range(1, n):
            theta = theta + ((velocity * np.tan(temp_steering_angle)) /
                             self.vehicle.meta.wheelbase) * timestep
            x_n = pos[0] + velocity * np.cos(theta + temp_steering_angle) * timestep
            y_n = pos[1] + velocity * np.sin(theta + temp_steering_angle) * timestep
            pos = (x_n, y_n)
            temp_steering_angle = self.compute_steering_angle(route, pos, theta)
            print(temp_steering_angle, 'steering', step)
            steering_angle += k[step] * temp_steering_angle
        self.vehicle.steer_angle = steering_angle
        return steering_angle


    def compute_steering_angle(self, route: List[Tuple[float, float]],
                               position: Tuple[float, float], orientation: float) -> float:
        """Compute the steering angle according to the Stanley method."""

        prev_wp, next_wp = self._get_prev_and_next_point(route, position)

        traveled_distance = 0
        if (self.last_pos is not None) and (position is not None):
            traveled_distance = dist(self.last_pos, position)
            print(traveled_distance, 'Traveled_Distance')
        self.last_pos = position

        if not prev_wp or not next_wp or prev_wp == next_wp:
            # TODO: think of a better fallback case
            print(prev_wp, next_wp)
            print('canceled 2')
            return 0.0

        heading_error = self._heading_error(prev_wp, next_wp, orientation)

        if self.init:
            self.cross_track_errors.insert(0, self._cross_track_error(prev_wp, next_wp, position))
            if len(self.cross_track_errors) > self.cte_count:
                del self.cross_track_errors[-1]
            cross_track_error = sum(self.cross_track_errors) / len(self.cross_track_errors)
        else:
            cross_track_error = self._cross_track_error(prev_wp, next_wp, position)
            if traveled_distance > 0 and self.vehicle.velocity_mps > 1:
                self.init = True
                self.curvature = self.curvature_value
        print(self.vehicle.velocity_mps, 'speed')
        print('Errors', heading_error, cross_track_error)

        steer_angle = self.factor * heading_error + cross_track_error

        max_angle = self.vehicle.meta.max_steer_angle_rad
        steer_angle = min(max(steer_angle, -max_angle), max_angle)
        return steer_angle

    @property
    def min_dist_ahead(self) -> float:
        speed = self.vehicle.velocity_mps
        if speed == 0:
            gate = 0
        else:
            time = (dist(self.last_pos, self.vehicle.pos) / speed)
            speed = speed
            gate = ceil((speed * time))
        #print(gate, 'Ceiling')
        return gate

    def _get_prev_and_next_point(self, route: List[Tuple[float, float]],
                                 position: Tuple[float, float]) -> Tuple[float, float] or None:
        if len(route) < 2:
            return None, None
        min_dist = self.min_dist_ahead
        for i in range(1, len(route)):
            if dist(route[i], position) >= min_dist:
                if dist(route[i], self.vehicle.pos) < dist(position, self.vehicle.pos):
                    continue
                return route[i], self._get_next_point(route, i)
                # return route[i], route[i]
        return route[-2], route[-1]

    def _get_next_point(self, route: List[Tuple[float, float]], index) -> Tuple[
                                                                              float, float] or None:
        if len(route) < 2:
            return None
        min_dist = self.min_dist_ahead
        for i in range(index + 1, len(route)):
            if dist(route[i], route[index]) >= min_dist:
                #print('second', i)
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

        # print(self.vehicle.pos, pos_front_axle, 'Positions')
        # print(self.vehicle.orientation_rad, 'Orientation')
        #print(dist(self.last_pos, self.vehicle.pos), 'Traveled_Distance')

        prev_to_vehicle = points_to_vector(prev_wp, position)
        cross_prod = -np.cross(prev_to_next, prev_to_vehicle)
        traj_len = vector_len(prev_to_next)
        e_t = cross_prod / traj_len
        # print('cross_track_error', e_t)
        # if abs(e_t) > 1:
        # print('thrown off')
        # arg = (self.curvature * e_t) / (self.eps + self.vehicle.velocity_mps)
        return atan2((self.curvature * e_t), self.eps + self.vehicle.velocity_mps)


class CombinedSteering:
    # pylint: disable=too-few-public-methods
    """Combines Naive Controller for curves and Stanley for straights"""
    vehicle: Vehicle
    naive_controller: SteeringController = None
    stanley_controller: SteeringController = None

    def __post_init__(self):
        self.naive_controller = NaiveSteeringController(self.vehicle)
        self.stanley_controller = StanleySteeringController(self.vehicle)

    def compute_steering_angle(self, route: List[Tuple[float, float]]) -> float:
        if CurveDetection.find_next_curve(route).dist_until_curve < 5:
            return self.naive_controller.compute_steering_angle(route)
        return self.stanley_controller.compute_steering_angle(route)
