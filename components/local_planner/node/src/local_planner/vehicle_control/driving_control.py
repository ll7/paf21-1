"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from typing import Tuple, List
from dataclasses import dataclass, field

from local_planner.core import Vehicle

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

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed"""
        self.route_waypoints = waypoints

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the route to be followed"""
        self.target_velocity_mps = target_velocity_mps
        #self.target_distance_m = target_distance_m

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.vehicle.move(vehicle_pos)

    def update_vehicle_state(self, position: float, velocity: float):
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
