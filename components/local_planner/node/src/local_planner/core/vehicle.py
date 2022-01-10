"""This module represents a vehicle to be controlled"""

from dataclasses import dataclass
from typing import Tuple
from math import dist as euclid_dist

import rospy

from local_planner.core.geometry import norm_angle, points_to_vector, vector_to_dir


@dataclass
class Vehicle:
    """Representing a vehicle"""
    name: str
    actual_velocity_mps: float = 0.0
    actual_accel_mps2: float = 0.0
    pos: Tuple[float, float] = None
    pos_update_rostime: float = 0.0
    orientation_rad: float = 0.0
    steering_angle: float = 0.0
    target_velocity: float = 0.0

    length_between_axles_m: float = 3.1
    max_steering_angle_rad: float = 0.5
    base_accel_mps2: float = 10.0
    base_brake_mps2: float = -10.0
    vehicle_reaction_time_s: float = 1

    def move(self, new_pos: Tuple[float, float]):
        """Move the car towards the new position"""
        old_pos = self.pos
        old_timestamp = self.pos_update_rostime
        new_timestamp = rospy.get_time()

        if old_pos is not None and new_timestamp - old_timestamp > 0:
            dist = euclid_dist(old_pos, new_pos)
            time = new_timestamp - old_timestamp
            self.actual_velocity_mps = (dist / time) * 2
            print(f'timestamp:{rospy.get_rostime()}')
        print(self.actual_velocity_mps)
        self.pos = new_pos
        self.pos_update_rostime = new_timestamp

    def update_speed(self, speed):
        self.actual_velocity_mps = speed

    def steer_towards(self, aim_point: Tuple[float, float]):
        """Adjust the steering angle for driving towards the given point"""
        aim_vector = points_to_vector(self.pos, aim_point)
        aim_angle_rad = vector_to_dir(aim_vector)
        steer_angle = aim_angle_rad - self.orientation_rad
        self.steering_angle = norm_angle(steer_angle)
