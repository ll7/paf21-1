"""This module represents a vehicle to be controlled"""

from dataclasses import dataclass
from typing import Tuple
from datetime import datetime
from math import dist as euclid_dist

from local_planner.vehicle_control.geometry import \
    norm_angle, points_to_vector, vector_to_dir

@dataclass
class Vehicle:
    """Representing a vehicle"""
    actual_velocity_mps: float = 0.0
    actual_accel_mps2: float = 0.0
    pos: Tuple[float, float] = None
    pos_update_timestamp: datetime = None
    orientation_rad: float = None
    steering_angle: float = 0.0
    target_velocity: float = 0.0

    length_between_axles_m: float = 3.1
    max_steering_angle_rad: float = 0.5
    base_accel_mps2: float = 8.0
    base_brake_mps2: float = -8.0
    vehicle_reaction_time_s: float = 3.5

    def move(self, new_pos: Tuple[float, float]):
        """Move the car towards the new position"""
        old_pos = self.pos
        old_timestamp = self.pos_update_timestamp
        new_timestamp = datetime.utcnow()

        if old_pos is not None:
            dist = euclid_dist(old_pos, new_pos)
            time = (new_timestamp - old_timestamp).total_seconds()
            self.actual_velocity_mps = dist / time
            # TODO: use the in-game time instead of actual time

        self.pos = new_pos
        self.pos_update_timestamp = new_timestamp

    def steer_towards(self, aim_point: Tuple[float, float]) -> float:
        """Adjust the steering angle for driving towards the given point"""
        aim_vector = points_to_vector(self.pos, aim_point)
        aim_angle_rad = vector_to_dir(aim_vector)
        steer_angle = aim_angle_rad - self.orientation_rad
        self.steering_angle = norm_angle(steer_angle)

    def update_speed(self, distance_m: float, target_velocity_mps: float):
        print(f"update speed: target={target_velocity_mps}, actual={self.actual_velocity_mps}")

        if target_velocity_mps > self.actual_velocity_mps:
            self.target_velocity = target_velocity_mps
        else:
            self.brake_if_required(distance_m, target_velocity_mps)

    def brake_if_required(self, distance_m: float, target_velocity_mps: float):
        wait_time_s = self._time_until_brake(distance_m, target_velocity_mps)
        if wait_time_s <= self.vehicle_reaction_time_s:
            print(f'initiating brake maneuver')
            self.target_velocity = target_velocity_mps

    def _time_until_brake(self, distance_m: float, target_velocity: float = 0) -> float:
        """Compute the braking distance and based on that the time until brake.
        In case this function returns a negative value, it means that it's already
        too late to brake, so you need to launch an emergency protocol"""

        accel_mps2 = self.base_brake_mps2

        if distance_m < 0:
            raise ValueError('Negative distance is not allowed')
        if self.actual_velocity_mps < 0 or target_velocity < 0:
            raise ValueError('Negative velocity is not allowed')
        if accel_mps2 >= 0:
            raise ValueError('Positive acceleration won\'t brake')

        maneuver_time_s = (target_velocity - self.actual_velocity_mps) / accel_mps2
        braking_dist = self.actual_velocity_mps * maneuver_time_s + \
                       accel_mps2 * maneuver_time_s**2 / 2
        time_until_brake = (distance_m - braking_dist) / self.actual_velocity_mps \
                           if self.actual_velocity_mps > 0 else 0
        return time_until_brake
