"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from typing import Tuple, List
from dataclasses import dataclass, field
from math import cos
from math import atan
from numpy import cross
from numpy.linalg import norm

from local_planner.core import Vehicle, geometry

from local_planner.core import Vehicle, geometry
from math import sqrt, floor

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

    # friction coefficients
    coeff_dry : float = 0.8
    coeff_wet : float = 0.4
    coeff_ice : float = 0.2

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed and cache first waypoint"""
        if waypoints:
            if not self.initial_vehicle_pos_set:
                self.initial_vehicle_pos_set = True
        self.route_waypoints = waypoints

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the target velocity"""

        # State machine
        self.target_velocity_mps = target_velocity_mps

        """
        Pavlo
        print("update target velocity")
        if len(self.route_waypoints) < 2 :
            self.target_velocity_mps = target_velocity_mps
        else:
            middle_waypoint = round(len(self.route_waypoints) / 2)
            # print('len waypoints : ', len(self.route_waypoints))
            radius = geometry.approx_curvature_radius(self.route_waypoints[1],
                self.route_waypoints[middle_waypoint],
                self.route_waypoints[-1])
            # print('RADIUS: ', radius)
            if radius > 100:
                self.target_velocity_mps = target_velocity_mps
            else:
                self.target_velocity_mps = floor(sqrt(9.81 * self.coeff_dry * radius))
        print(target_velocity_mps)
        print(self.target_velocity_mps)
        """

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
        steering_angle = self._compute_steering_angle()
        # targetspeed =  0.0 if not self.route_waypoints else self.target_velocity_mps
        # signal = DrivingSignal(steering_angle, targetspeed)
        signal = DrivingSignal(steering_angle, self.target_velocity_mps)
        return signal

    def _compute_steering_angle(self) -> float:
        if not self._can_steer():
            return 0.0

        # Steer towards method
        print("aim_point")
        print(self._get_aim_point())
        self.vehicle.steer_towards(self._get_aim_point())
        print(self.vehicle.steering_angle)
        steering_angle = self.vehicle.steering_angle

        # stanley method
        # steering_angle = self.stanley_method()
        # self.vehicle.set_steering_angle(steering_angle)

        return steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 1 \
               and self.vehicle.orientation_rad \
               and self.vehicle.pos

    def _get_aim_point(self) -> Tuple[float, float]:
        if len(self.route_waypoints) > 2:
            return self.route_waypoints[2]
        return self.route_waypoints[1]

    def stanley_method(self) -> float:
        """Implementation of Stanley Controller"""

        # only one waypoint in list = destination reached = steering straight
        if len(self.route_waypoints) < 2:
            return 0.0

        pos = self.vehicle.pos
        v = self.vehicle.actual_velocity_mps
        prev_wp = self.route_waypoints[0]
        next_wp = self.route_waypoints[1]

        # calc heading error
        vec_traj = geometry.points_to_vector(prev_wp, next_wp)
        dir_traj = geometry.vector_to_dir(vec_traj)
        heading_error = dir_traj - self.vehicle.orientation_rad

        # Controller Settings
        k = 0.3
        k_s = 1

        # calc crosstrack error
        prev_to_next = geometry.points_to_vector(prev_wp, next_wp)
        prev_to_vehicle = geometry.points_to_vector(prev_wp, pos)

        top = cross(prev_to_next, prev_to_vehicle)
        top = norm(top)
        bottom = norm(prev_to_next)
        e = top/bottom
        arg = (k*e)/(k_s+v)
        cross_track_error = atan(arg)

        return heading_error + cross_track_error
