from typing import Tuple, List
from math import sin, cos, pi, sqrt


def rotate_vector(vector: Tuple[float, float], angle_rad: float) -> Tuple[float, float]:
    """Rotate the given vector by an angle"""
    return (cos(angle_rad) * vector[0] - sin(angle_rad) * vector[1],
            sin(angle_rad) * vector[0] + cos(angle_rad) * vector[1])


def orthogonal_offset(start_point: Tuple[float, float], end_point: Tuple[float, float],
                      road_width: float) -> Tuple[float, float]:
    """Calculate the orthogonal offset according the road width in right direction"""
    vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    vector_length = sqrt(vector[0]**2 + vector[1]**2)
    scaled_vector = (vector[0] * road_width / vector_length, vector[1] * road_width / vector_length)
    return rotate_vector(scaled_vector, -pi / 2)


def bounding_box(start_point: Tuple[float, float], end_point: Tuple[float, float],
                 road_width: float) -> List[Tuple[float, float]]:
    """Calculate a bounding box around the start and end point with a given offset to the side."""
    offset = orthogonal_offset(start_point, end_point, road_width)

    # using the point symmetry (relative to origin) to build the points in 180 degree offset
    return [(start_point[0] + offset[0], start_point[1] + offset[1]),
            (start_point[0] - offset[0], start_point[1] - offset[1]),
            (end_point[0] - offset[0], end_point[1] - offset[1]),
            (end_point[0] + offset[0], end_point[1] + offset[1])]
