"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

import sys
from typing import Tuple, List
from dataclasses import dataclass, field
from math import cos

from local_planner.core import Vehicle, geometry


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
    target_distance_m: float = 0.0
    initial_vehicle_pos_set: bool = False
    steering_angle : float = 0.0

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed and cache first waypoint"""
        if waypoints:
            if not self.initial_vehicle_pos_set:
                self.initial_vehicle_pos_set = True
        self.route_waypoints = waypoints

    def update_target_velocity(self, velocity_mps: float):
        """Update vehicle's velocity"""
        target_velocity_mps = velocity_mps
        #radius = geometry.approx_curvature_radius(self.route_waypoints)

        #print('Actual Velocity : ', current_velocity_mps)
        # OPTION 1 :
        self.target_velocity_mps = abs(target_velocity_mps * cos(self.steering_angle))

        print('target velocity :', self.target_velocity_mps )
        print('route_waypoints: ', self.route_waypoints[1])
        #print('vehicle pos :', self.vehicle.pos)
        # self.target_distance_m = target_distance_m
        return self.target_velocity_mps
       
    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.vehicle.move(vehicle_pos)

    def update_vehicle_state(self, position: Tuple[float, float], velocity: float):
        """Update the vehicle's positional and velocity values"""
        self.vehicle.actual_velocity_mps = velocity
        self.vehicle.pos = position

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.vehicle.orientation_rad = orientation

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""
        self.steering_angle = self._compute_steering_angle()
        signal = DrivingSignal(self.steering_angle, self.target_velocity_mps)
        return signal

    def _compute_steering_angle(self) -> float:
        if not self._can_steer():
            return 0.0

        # aim_point = self._get_aim_point()
        # self.vehicle.steer_towards(aim_point)
        steering_angle = self.stanley_method()
        self.vehicle.set_steering_angle(steering_angle)
        print('steering angle : ', steering_angle)

        return steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 1 \
               and self.vehicle.orientation_rad \
               and self.vehicle.pos

    def _get_aim_point(self):
        return self.route_waypoints[1]

    def stanley_method(self) -> float:
        """Implementation of Stanley Controller"""

        # only one waypoint in list = destination reached = steering straight
        if len(self.route_waypoints) < 2:
            return 0.0

        pos = self.vehicle.pos

        prev_wp = self.route_waypoints[0]
        next_wp = self.route_waypoints[1]

        #calc heading error
        vec_traj = geometry.points_to_vector(prev_wp, next_wp)
        dir_traj = geometry.vector_to_dir(vec_traj)
        heading_error = dir_traj - self.vehicle.orientation_rad

        #calc crosstrack error
        prev_to_next = geometry.points_to_vector(prev_wp, next_wp)
        prev_to_vehicle = geometry.points_to_vector(prev_wp, self.vehicle.pos)

        # Controller Settings
        k = 0.3
        k_s = 1
        
        return heading_error