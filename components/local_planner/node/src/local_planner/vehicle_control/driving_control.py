"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

# from math import dist as euclid_dist
from math import degrees
from typing import Tuple, List
from dataclasses import dataclass, field

# import numpy as np
# import rospy

from local_planner.vehicle_control.vehicle import Vehicle

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
    # current_wp_id: int = 0
    vehicle: Vehicle = Vehicle()

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed"""
        self.route_waypoints = waypoints
        print(f'waypoints {waypoints}')

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the route to be followed"""
        self.target_velocity_mps = target_velocity_mps

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.vehicle.move(vehicle_pos)

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.vehicle.orientation_rad = orientation

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""

        steering_angle = self._compute_steering_angle()
        velocity = self._compute_velocity()
        signal = DrivingSignal(steering_angle, velocity)
        print(f'vehicle {self.vehicle}')
        return signal

    def _compute_velocity(self) -> float:
        return 0.0 if len(self.route_waypoints) == 0 else self.target_velocity_mps

    def _compute_steering_angle(self) -> float:
        if not self._can_steer():
            return 0.0

        aim_point = self._get_aim_point()
        self.vehicle.steer_towards(aim_point)

        is_car_moving = self.target_velocity_mps > 0
        if is_car_moving:
            print(f'aim_point {aim_point}, vehicle_position {self.vehicle.pos} '
                + f'steer {degrees(self.vehicle.steering_angle)}, '
                + f'orientation {degrees(self.vehicle.orientation_rad)}')

        return self.vehicle.steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 0 \
            and self.vehicle.orientation_rad \
            and self.vehicle.pos

    def _get_aim_point(self):
        return self.route_waypoints[0]
        # aim_point = self.route_waypoints[self.current_wp_id]
        # while euclid_dist(aim_point, self.vehicle.pos) < 7 \
        #         and self.current_wp_id < len(self.route_waypoints) - 1:
        #     self.current_wp_id += 1
        #     aim_point = self.route_waypoints[self.current_wp_id]
        # return aim_point
