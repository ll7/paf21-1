"""Representing information on a recently detected object"""

from dataclasses import dataclass
from typing import List, Tuple
from math import dist

from local_planner.object_processing.kalman_filter import KalmanFilter


@dataclass
class ObjectMeta:
    """Representing information on a recently detected object"""
    identifier: int
    obj_class: str
    trajectory: List[Tuple[float, float]]
    velocity: float = 900.0
    kalman_filter: KalmanFilter = KalmanFilter()
    max_velocity_change_rate = 2.0

    def update_object_meta(self, position: Tuple[float, float], delta_time: float):
        """Update the object meta with a new position"""
        last_position = self.trajectory[-1]
        self.trajectory.append(position)
        if len(self.trajectory) > 10:
            self.trajectory = self.trajectory[-10:]
        self.velocity = min(dist(position, last_position) / delta_time,
                            self.velocity + self.max_velocity_change_rate)
        self.kalman_filter.correct(position)
