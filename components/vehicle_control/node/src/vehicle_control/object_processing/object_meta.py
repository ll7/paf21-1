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
    max_trajectory_entries: int = 5

    def update_object_meta(self, point: Tuple[float, float], delta_time: float):
        """Update the object meta with a new position"""
        last_position = self.trajectory[-1]
        self.trajectory.append(point)
        if len(self.trajectory) > self.max_trajectory_entries:
            self.trajectory = self.trajectory[-self.max_trajectory_entries:]

        velocity = dist(point, last_position) / delta_time
        self.velocity = min(velocity, self.velocity + self.max_vel_change_rate)

