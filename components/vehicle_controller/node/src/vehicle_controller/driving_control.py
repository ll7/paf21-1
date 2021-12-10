"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from math import dist as euclid_dist, degrees
from typing import Tuple, List
from dataclasses import dataclass, field

# import numpy as np
import rospy

from vehicle_controller.vehicle import Vehicle

@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float

@dataclass
class DrivingController: # pylint: disable=too-many-instance-attributes
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""

    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    current_wp_id: int = 0
    vehicle: Vehicle = Vehicle()

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed"""
        self.route_waypoints = waypoints
        rospy.loginfo(f'waypoints {waypoints}')

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the route to be followed"""
        self.target_velocity_mps = target_velocity_mps

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float], orientation: float):
        """Update the vehicle's current position and estimate
        the actual velocity by computing the position / time diffs"""
        self.vehicle.move(vehicle_pos, orientation)

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

        if self.target_velocity_mps > 0:
            rospy.loginfo(
                  f'aim_point {aim_point}, vehicle_position {self.vehicle.pos} '
                + f'steer {degrees(self.vehicle.steering_angle)}, '
                + f'orientation {degrees(self.vehicle.orientation_rad)}')

        return self.vehicle.steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 0 \
            and self.vehicle.orientation_rad

    def _get_aim_point(self):
        aim_point = self.route_waypoints[self.current_wp_id]
        while euclid_dist(aim_point, self.vehicle.pos) < 7 \
                and self.current_wp_id < len(self.route_waypoints) - 1:
            self.current_wp_id += 1
            aim_point = self.route_waypoints[self.current_wp_id]
        return aim_point
