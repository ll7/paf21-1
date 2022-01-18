"""A module defining data structures related to object detection"""

from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class Point:
    """Representing a point with x- and y-coordinate"""
    point: Tuple[float, float]


@dataclass
class ObjectStatus:
    """Representing the object status"""
    trajectory: List[Point]
    velocity: List[float]


@dataclass
class ObjectInfo:
    """Representing information on a recently detected object"""
    object_nummer: int
    object_class: str
    distance_to_collision: float
    object_status: ObjectStatus
