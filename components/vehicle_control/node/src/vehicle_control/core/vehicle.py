"""This module represents a vehicle to be controlled"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from vehicle_control.core.geometry import rotate_vector, add_vector
#     unit_vector, scale_vector, add_vector


@dataclass
class VehicleMetadata:
    """Representing construction-specific vehicle features"""
    length_between_axles_m: float = 3.1
    max_steer_angle_rad: float = np.deg2rad(30)
    base_accel_mps2: float = 2.0
    base_brake_mps2: float = -2.0
    wheelbase: float = 2.875
    vehicle_reaction_time_s: float = 0.1


@dataclass
class Vehicle:
    """Representing a vehicle"""
    name: str
    velocity_mps: float = None
    steer_angle: float = 0
    pos: Tuple[float, float] = None
    orientation_rad: float = None
    time: float = 0.0
    meta: VehicleMetadata = VehicleMetadata()

    @property
    def is_ready(self) -> bool:
        """A boolean indicating whether the vehicle is ready for use"""
        return self.pos is not None and self.velocity_mps is not None and self.orientation_rad is not None

    # @property
    # def pos_front(self) -> Tuple[float, float]:
    #     """The vehicle's front axle position"""
    #     front_offset = scale_vector(unit_vector(self.orientation_rad), 1.5)
    #     return add_vector(self.pos, front_offset)

    def update_speed(self, speed):
        """function to update the current velocity of the car"""
        self.velocity_mps = speed

    def update_vehicle_state(self, position: Tuple[float, float],
                             velocity_and_time: Tuple[Tuple[float, float], float]):
        """Update the vehicle's positional and velocity values"""
        if self.orientation_rad is not None:
            self.velocity_mps, self.time = velocity_and_time
            axle_length = self.meta.wheelbase / 2
            vector_axle = rotate_vector((axle_length, 0), self.orientation_rad)
            position = add_vector(vector_axle, position)
            self.pos = position

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.orientation_rad = orientation
