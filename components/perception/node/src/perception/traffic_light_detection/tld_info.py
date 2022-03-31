"""A module defining data structures related to traffic light detection"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Tuple


class TrafficLightPhase(IntEnum):
    """Representing traffic light phases"""
    BACKSIDE = 0
    GREEN = 1
    RED = 2
    YELLOW = 3


@dataclass
class TrafficLightInfo:
    """Representing information on a recently detected traffic light"""
    phase: TrafficLightPhase = TrafficLightPhase.GREEN
    distance: float = 999
    accuracy: float = 0.0
    position: Tuple[float, float] = (0, 0)
