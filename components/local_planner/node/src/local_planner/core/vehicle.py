"""This module represents a vehicle to be controlled"""

from dataclasses import dataclass
from typing import Tuple

from local_planner.core.geometry import \
    norm_angle, points_to_vector, vector_to_dir, \
    unit_vector, scale_vector, add_vector


@dataclass
class Vehicle:
    # pylint: disable=too-many-instance-attributes
    """Representing a vehicle"""
    name: str
    actual_velocity_mps: float = 0.0
    # actual_accel_mps2: float = 0.0
    pos: Tuple[float, float] = None
    orientation_rad: float = 0.0
    steering_angle: float = 0.0
    target_velocity: float = 0.0
    time: float = 0.0
    length_between_axles_m: float = 3.1
    max_steer_angle_rad: float = 0.5
    base_accel_mps2: float = 2.0
    base_brake_mps2: float = -2.0
    vehicle_reaction_time_s: float = 0.1

    @property
    def is_ready(self) -> bool:
        """A boolean indicating whether the vehicle is ready for use"""
        return self.pos is not None

    @property
    def pos_front(self) -> Tuple[float, float]:
        """The vehicle's front axle position"""
        # TODO: orientation seems to be buggy, fix this
        front_offset = scale_vector(unit_vector(self.orientation_rad), 1.5)
        return add_vector(self.pos, front_offset)

    def update_speed(self, speed):
        """function to update the current velocity of the car"""
        self.actual_velocity_mps = speed

    def steer_towards(self, aim_point: Tuple[float, float]):
        """Adjust the steering angle for driving towards the given point"""
        aim_vector = points_to_vector(self.pos, aim_point)
        aim_angle_rad = vector_to_dir(aim_vector)
        steer_angle = aim_angle_rad - self.orientation_rad
        self.steering_angle = norm_angle(steer_angle)

    def set_steering_angle(self, steering_angle_rad: float):
        """"Set the Steering Angles within it's limits"""
        self.steering_angle = norm_angle(steering_angle_rad)
