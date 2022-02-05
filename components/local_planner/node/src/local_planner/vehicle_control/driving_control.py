"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from typing import Tuple, List
from dataclasses import dataclass, field
from math import atan, radians as deg2rad, dist

import numpy as np

from local_planner.core import Vehicle
from local_planner.core.geometry import \
    points_to_vector, norm_angle, vector_len, vector_to_dir


@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float


@dataclass
class DrivingController:  # pylint: disable=too-many-instance-attributes
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""
    vehicle: Vehicle
    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    initial_vehicle_pos_set = False

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed and cache first waypoint"""
        if waypoints and not self.initial_vehicle_pos_set:
            self.initial_vehicle_pos_set = True
        self.route_waypoints = waypoints

    def update_vehicle_state(self, position: Tuple[float, float],
                             velocity_and_time: Tuple[Tuple[float, float], float]):
        """Update the vehicle's positional and velocity values"""
        self.vehicle.actual_velocity_mps, self.vehicle.time = velocity_and_time
        self.vehicle.pos = position

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.vehicle.orientation_rad = orientation

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the target velocity according to the curvature ahead"""
        self.target_velocity_mps = target_velocity_mps

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""
        # generate logs for each driving signal tick
        steering_angle = self._compute_steering_angle()
        targetspeed =  self.target_velocity_mps # if self.route_waypoints else 0.0
        signal = DrivingSignal(steering_angle, targetspeed)
        if self._can_steer():
            print(f"{self.vehicle.time},{self.vehicle.pos[0]},{self.vehicle.pos[1]},{self.vehicle.orientation_rad},{self.vehicle.actual_velocity_mps},{targetspeed},{steering_angle}")
        return signal

    def _compute_steering_angle(self) -> float:
        if not self._can_steer():
            return 0.0

        # use direct steering towards next waypoint for driving curves
        self.vehicle.steer_towards(self._get_aim_point())

        # # correct steering by stanley method for rather straight passages
        # if abs(self.vehicle.steering_angle) > deg2rad(20):
        #     return self.vehicle.steering_angle

        # steering_angle_stanley = self._stanley_steering()
        # # print(f'stanley override: {self.vehicle.steering_angle} -> {steering_angle_stanley}')
        # self.vehicle.set_steering_angle(steering_angle_stanley)
        return self.vehicle.steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 1 \
               and self.vehicle.orientation_rad \
               and self.vehicle.pos

    def _get_aim_point(self) -> Tuple[float, float]:
        # route waypoints contain at least one point behind the car
        #   -> 1st / 2nd route waypoint are not reliable aim points
        if len(self.route_waypoints) > 5:
            return next(wp for wp in self.route_waypoints[1:] if dist(wp, self.vehicle.pos) > 5)
        return self.route_waypoints[1]
        # TODO: what happens if the route contains less than 2 waypoints?

    def _stanley_steering(self) -> float:
        # Controller Settings
        k, k_s = 2, 1e-6

        # only one waypoint in list = destination reached = steering straight
        if len(self.route_waypoints) < 2:
            return 0.0

        pos = self.vehicle.pos
        velocity = self.vehicle.actual_velocity_mps
        prev_wp, next_wp = self.route_waypoints[0], self.route_waypoints[1]

        # calc heading error
        vec_traj = points_to_vector(prev_wp, next_wp)
        dir_traj = vector_to_dir(vec_traj)
        heading_error = dir_traj - self.vehicle.orientation_rad
        heading_error = norm_angle(heading_error)

        # calc crosstrack error
        prev_to_next = points_to_vector(prev_wp, next_wp)
        prev_to_vehicle = points_to_vector(prev_wp, pos)
        cross_prod = -np.cross(prev_to_next, prev_to_vehicle)
        traj_len = vector_len(prev_to_next)
        e_t = cross_prod / traj_len

        arg = (k * e_t) / (k_s + velocity)
        cross_track_error = atan(arg)

        steer_angle = heading_error + cross_track_error
        # print(f"steering angle {steer_angle}")

        max_angle = self.vehicle.max_steer_angle_rad
        steer_angle = min(max(steer_angle, -max_angle), max_angle)

        return steer_angle
