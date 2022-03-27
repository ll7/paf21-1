"""Representing information on a recently detected object"""

from dataclasses import dataclass
from typing import List, Tuple
from math import dist


@dataclass
class ObjectMeta:
    """Representing information on a recently detected object"""
    identifier: int
    obj_class: str
    trajectory: List[Tuple[float, float]]
    velocity: float = 0.0
    max_vel_change_rate = 2.0
    max_trajectory_entries: int = 10

    def update_object_meta(self, point: Tuple[float, float], delta_time: float):
        """Update the object meta with a new position"""
        last_position = self.trajectory[-1]
        self.trajectory.append(point)
        if len(self.trajectory) > 10:
            self.trajectory = self.trajectory[-10:]

        velocity = dist(point, last_position) / delta_time
        self.velocity = min(velocity, self.velocity + self.max_vel_change_rate)

