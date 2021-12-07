"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from math import dist as euclid_dist
from datetime import datetime
from typing import Tuple, List
from dataclasses import dataclass, field
import numpy as np

@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float

@dataclass
class SimpleDrivingController:
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""

    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    actual_velocity_mps: float = 0.0
    vehicle_pos: Tuple[float, float] = None
    vehicle_pos_timestamp: datetime = None

    # car length between axles and max steering angle of Tesla Model 3
    car_length_in_m: float = 3.1
    max_steering_angle: float = 0.5

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed"""
        self.route_waypoints = waypoints

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the route to be followed"""
        self.target_velocity_mps = target_velocity_mps

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position and estimate
        the actual velocity by computing the position / time diffs"""

        old_pos = self.vehicle_pos
        new_pos = vehicle_pos
        old_timestamp = self.vehicle_pos_timestamp
        new_timestamp = datetime.utcnow()

        if old_pos is not None:
            dist = euclid_dist(old_pos, new_pos)
            time = (new_timestamp - old_timestamp).total_seconds()
            self.actual_velocity_mps = dist / time

        self.vehicle_pos = new_pos
        self.vehicle_pos_timestamp = new_timestamp

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""

        # apply not overshooting max. steering angle here ...
        steering_angle = self._compute_steering_angle()
        return DrivingSignal(steering_angle, self.target_velocity_mps)

    def _compute_steering_angle(self):
        waypoints = self.route_waypoints
        if len(waypoints) < 5:
            return 0.0

        curve_radius = SimpleDrivingController._compute_curvature_radius(
            waypoints[0], waypoints[2], waypoints[4])
        phi = np.arctan(self.car_length_in_m / curve_radius)
        return phi

    # @staticmethod
    # def euclid_dist(p1, p2):
    #     # use standard function instead
    #     return np.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))

    @staticmethod
    def _compute_curvature_radius(p_1, p_2, p_3):
        triangle_area = ((p_2[0] - p_1[0]) * (p_3[1] - p_1[1]) - \
                        (p_3[0] - p_1[0]) * (p_2[1] - p_1[1])) / 2.0
        menger = (4 * triangle_area) / \
                (euclid_dist(p_1, p_2) * euclid_dist(p_2, p_3) * euclid_dist(p_3, p_1))
        radius = abs(1.0 / menger)
        return radius
