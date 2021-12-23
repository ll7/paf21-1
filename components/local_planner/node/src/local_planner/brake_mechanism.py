"""This module handles the vehicle velocity and brake timings"""

from typing import Tuple
from datetime import datetime

class VelocityController:
    """This component controls the velocity of a vehicle"""
    # Max Accel: 10.0
    base_accel_mps2: float = 8.0
    base_brake_mps2: float = -8.0
    vehicle_reaction_time_s: float = 0.5

    # TODO: think of fusing this piece of code with the Vehicle class from the vehicle controller
    actual_pos: Tuple[float, float]
    pos_update_timestamp: datetime
    actual_velocity: float
    actual_accel: float

    @staticmethod
    def _time_until_brake(actual_velocity: float, acceleration_mps2: float,
                         distance_m: float, target_velocity: float = 0) -> float:
        """Compute the braking distance and based on that the time until brake.
        In case this function returns a negative value, it means that it's already
        too late to brake, so you need to launch an emergency protocol"""

        if distance_m < 0:
            raise ValueError('Negative distance is not allowed')
        if actual_velocity < 0 or target_velocity < 0:
            raise ValueError('Negative velocity is not allowed')
        if acceleration_mps2 >= 0:
            raise ValueError('Positive acceleration won\'t brake')

        maneuver_time_s = (target_velocity - actual_velocity) / acceleration_mps2
        braking_dist = actual_velocity * maneuver_time_s + acceleration_mps2 * maneuver_time_s**2 / 2
        time_until_brake = (distance_m - braking_dist) / actual_velocity if actual_velocity > 0 else 0
        return time_until_brake
