"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from math import dist as euclid_dist
from datetime import datetime
from typing import Tuple, List
from dataclasses import dataclass, field

@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float

@dataclass
class SimpleDrivingController:
    """A class for processing waypoints and
    driving metadata into driving signals
    regarding velocity and steering control"""
    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    actual_velocity_mps: float = 0.0
    vehicle_pos: Tuple[float, float] = None
    vehicle_pos_timestamp: datetime = None

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
        """Compute the next AckermannDrive signal to
        follow the suggested ideal route"""

        steering_angle = self._compute_steering_angle()
        return DrivingSignal(steering_angle, self.target_velocity_mps)

    def _compute_steering_angle(self): # pylint: disable=no-self-use
        # source: https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081 # pylint: disable=line-too-long
        # source: http://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html # pylint: disable=line-too-long
        return 0.0

    # @staticmethod
    # def _compute_turn_circle_radius(waypoints: List[Tuple[float, float]]):
    #     near_points = filter(lambda wp: euclid_dist(waypoints[0], wp) <= 20, waypoints)
        # interpolate the curve, e.g. with a taylor polynomial
