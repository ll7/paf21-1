"""This module represents a vehicle to be controlled"""

from dataclasses import dataclass
from typing import Tuple

from local_planner.core.geometry import \
    norm_angle, points_to_vector, vector_to_dir


@dataclass
class Vehicle:
    # pylint: disable=too-many-instance-attributes
    """Representing a vehicle"""
    name: str
    actual_velocity_mps: float = 0.0
    actual_accel_mps2: float = 0.0
    pos: Tuple[float, float] = None
    pos_update_rostime: float = 0.0
    orientation_rad: float = 0.0
    steering_angle: float = 0.0
    target_velocity: float = 0.0
    time: float = 0.0
    length_between_axles_m: float = 3.1
    max_steering_angle_rad: float = 0.5
    base_accel_mps2: float = 3.0
    base_brake_mps2: float = -4.0
    vehicle_reaction_time_s: float = 0.1

    @property
    def is_ready(self) -> bool:
        """A boolean indicating whether the vehicle is ready for use"""
        return self.pos is not None

    def move(self, new_pos: Tuple[float, float]):
        """Move the car towards the new position"""
        self.pos = new_pos

    def update_speed(self, speed):
        """function to update the current velocity of the car"""
        self.actual_velocity_mps = speed

    def steer_towards(self, aim_point: Tuple[float, float]):
        """Adjust the steering angle for driving towards the given point"""
        aim_vector = points_to_vector(self.pos, aim_point)
        aim_angle_rad = vector_to_dir(aim_vector)
        steer_angle = aim_angle_rad - self.orientation_rad
        self.steering_angle = norm_angle(steer_angle)
