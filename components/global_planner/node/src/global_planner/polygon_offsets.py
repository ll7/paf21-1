"""A module providing vector and bounding box functionality"""

from typing import Tuple, List
from math import sin, cos, pi, sqrt


def points_to_vector(p_1: Tuple[float, float], p_2: Tuple[float, float]) -> Tuple[float, float]:
    """Create the vector starting at p1 and ending at p2"""
    return p_2[0] - p_1[0], p_2[1] - p_1[1]


def add_vector(v_1: Tuple[float, float], v_2: Tuple[float, float]) -> Tuple[float, float]:
    """Add the given vectors"""
    return v_1[0] + v_2[0], v_1[1] + v_2[1]


def sub_vector(v_1: Tuple[float, float], v_2: Tuple[float, float]) -> Tuple[float, float]:
    """Subtract the second vector from the first vector"""
    return v_1[0] - v_2[0], v_1[1] - v_2[1]


def rotate_vector(vector: Tuple[float, float], angle_rad: float) -> Tuple[float, float]:
    """Rotate the given vector by an angle"""
    return (cos(angle_rad) * vector[0] - sin(angle_rad) * vector[1],
            sin(angle_rad) * vector[0] + cos(angle_rad) * vector[1])


def scale_vector(vector: Tuple[float, float], new_len: float) -> Tuple[float, float]:
    """Amplify the length of the given vector"""
    old_len = sqrt(vector[0]**2 + vector[1]**2)
    scaled_vector = (vector[0] * new_len / old_len,
                     vector[1] * new_len / old_len)
    return scaled_vector


def orthogonal_offset(start_point: Tuple[float, float], end_point: Tuple[float, float],
                      road_width: float) -> Tuple[float, float]:
    """Calculate the orthogonal offset according the road width in right direction"""
    vector = points_to_vector(start_point, end_point)
    scaled_vector = scale_vector(vector, offset)
    return rotate_vector(scaled_vector, -pi / 2)


def bounding_box(start_point: Tuple[float, float], end_point: Tuple[float, float],
                 road_width: float) -> List[Tuple[float, float]]:
    """Calculate a bounding box around the start and end point with a given offset to the side."""
    offset = orthogonal_offset(start_point, end_point, road_width)

    # using the point symmetry (relative to origin) to build the points in 180 degree offset
    return [add_vector(start_point, offset),
            sub_vector(start_point, offset),
            sub_vector(end_point, offset),
            add_vector(end_point, offset)]
