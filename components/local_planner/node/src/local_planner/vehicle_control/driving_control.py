"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from typing import Tuple, List
from dataclasses import dataclass, field
from local_planner.core import Vehicle, geometry
from math import sqrt, floor

@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float

@dataclass
class DrivingController: # pylint: disable=too-many-instance-attributes
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""
    vehicle: Vehicle
    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    target_distance_m: float = 0.0
    # friction coefficients
    coeff_dry : float = 0.6
    coeff_wet : float = 0.4
    coeff_ice : float = 0.2

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed"""
        self.route_waypoints = waypoints

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the target velocity using kurvature radius"""
        print('TARGET VELOCITY FROM SM: ', target_velocity_mps)
        """
        if len(self.route_waypoints) < 2 :
            self.target_velocity_mps = target_velocity_mps
        else: 
            middle_waypoint = round(len(self.route_waypoints) / 2)
            radius = geometry.approx_curvature_radius(self.route_waypoints[1], 
                self.route_waypoints[middle_waypoint], self.route_waypoints[-1])
            print('RADIUS: ', radius)
            if radius > 90:
                self.target_velocity_mps = target_velocity_mps
            else:
            self.target_velocity_mps = floor(sqrt(9.81 * self.coeff_dry * radius))
        """
        if target_velocity_mps == 0:
            self.target_velocity_mps = 0
        else:
            self.target_velocity_mps = target_velocity_mps - abs(self.coeff_dry * self.vehicle.steering_angle)
        print('STEERING ANGLE: ', self.vehicle.steering_angle)
        print('TARGET VELOCITY SET: ', self.target_velocity_mps)
        

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.vehicle.move(vehicle_pos)

    def update_vehicle_state(self, position: Tuple[float, float], velocity: Tuple[float, float]):
        """Update the vehicle's positional and velocity values"""
        self.vehicle.actual_velocity_mps, time = velocity
        self.vehicle.time = time
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

        aim_point = self._get_aim_point()
        self.vehicle.steer_towards(aim_point)

        return self.vehicle.steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 0 \
            and self.vehicle.orientation_rad \
            and self.vehicle.pos

    def _get_aim_point(self):
        return self.route_waypoints[0]
