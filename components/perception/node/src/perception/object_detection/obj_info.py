"""A module defining data structures related to object detection"""

from typing import Tuple
from dataclasses import dataclass


@dataclass
class ObjectInfo:
    """Representing information on a recently detected object"""
    identifier: int
    obj_class: str
    rel_position: Tuple[float, float]
