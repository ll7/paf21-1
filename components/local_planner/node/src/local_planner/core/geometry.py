"""This module contains a collection of geometric helper functions"""

from math import dist as euclid_dist, atan2, pi, sqrt, sin, cos
from typing import Tuple, List


def points_to_vector(p_1: Tuple[float, float], p_2: Tuple[float, float]) -> Tuple[float, float]:
    """Create the vector starting at p1 and ending at p2"""
    return p_2[0] - p_1[0], p_2[1] - p_1[1]


def vector_len(vec: Tuple[float, float]) -> float:
    """Compute the given vector's length"""
    return sqrt(vec[0]**2 + vec[1]**2)


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
    old_len = vector_len(vector)
    if old_len == 0:
        return (0, 0)
    scaled_vector = (vector[0] * new_len / old_len,
                     vector[1] * new_len / old_len)
    return scaled_vector


def unit_vector(angle_rad: float) -> Tuple[float, float]:
    """Retrieve the unit vector representing the given direction."""
    return (cos(angle_rad), sin(angle_rad))


def vec2dir(start: Tuple[float, float], end: Tuple[float, float]) -> float:
    """Retrieve the given vector's direction in radians."""
    # TODO: think of removing this one
    vec_dir = sub_vector(end, start)
    return atan2(vec_dir[1], vec_dir[0])


def vector_to_dir(vector: Tuple[float, float]):
    """Get the given vector's direction in radians within [-pi, pi)"""
    normed_vector = norm_vector(vector)
    angle = atan2(normed_vector[1], normed_vector[0])
    return norm_angle(angle)


def norm_angle(angle_rad: float) -> float:
    """Normalize the given angle within [-pi, +pi)"""
    while angle_rad > pi:
        angle_rad -= 2.0 * pi
    while angle_rad < -pi:
        angle_rad += 2.0 * pi
    if angle_rad == pi:
        angle_rad = -pi

    if angle_rad < -pi or angle_rad >= pi:
        print('norm angle failed! this should never happen')

    return angle_rad


def norm_vector(vector: Tuple[float, float]):
    """Normalize the given vector to a proportional vector of length 1"""
    return scale_vector(vector, 1.0)


def orth_offset_right(start_point: Tuple[float, float], end_point: Tuple[float, float],
                      offset: float) -> Tuple[float, float]:
    """Calculate the orthogonal offset according the road width in right direction"""
    vector = points_to_vector(start_point, end_point)
    scaled_vector = scale_vector(vector, offset)
    return rotate_vector(scaled_vector, -pi / 2)


def orth_offset_left(start_point: Tuple[float, float], end_point: Tuple[float, float],
                     offset: float) -> Tuple[float, float]:
    """Calculate the orthogonal offset according the road width in left direction"""
    return sub_vector((0, 0), orth_offset_right(start_point, end_point, offset))


def approx_curvature_radius(p_start: Tuple[float, float],
                            p_middle: Tuple[float, float],
                            p_end: Tuple[float, float],
                            epsilon: float = 0.00001) -> float:
    """Approximate the radius of a curvature given its start, middle
    and end point using Menger's triangle formula"""

    tri_len_a = euclid_dist(p_start, p_middle)
    tri_len_b = euclid_dist(p_middle, p_end)
    tri_len_c = euclid_dist(p_end, p_start)

    tri_area = ((p_middle[0] - p_start[0]) * (p_end[1] - p_start[1]) - \
                (p_end[0] - p_start[0]) * (p_middle[1] - p_start[1])) / 2.0

    menger = (4 * tri_area) / (tri_len_a * tri_len_b * tri_len_c + epsilon)
    radius = abs(1.0 / (menger + epsilon))
    return radius


def angle_between_vectors(vec_1: Tuple[float, float],
                          vec_2: Tuple[float, float]) -> float:
    """Find angle between two vectors"""
    rel_dir = vector_to_dir(vec_1) - vector_to_dir(vec_2)
    return norm_angle(rel_dir)


def angle_triangle(p_a: Tuple[float, float],
                   p_b: Tuple[float, float],
                   p_c: Tuple[float, float]) -> float:
    """Find angle between the vectors a -> b, a -> c"""
    vec_ab = points_to_vector(p_a, p_b)
    vec_ac = points_to_vector(p_a, p_c)
    return angle_between_vectors(vec_ab, vec_ac)

def bounding_box(start_point: Tuple[float, float], end_point: Tuple[float, float],
                 road_width: float) -> List[Tuple[float, float]]:
    """Calculate a bounding box around the start and end point with a given offset to the side."""
    offset = orth_offset_right(start_point, end_point, road_width)
    return [add_vector(start_point, offset), sub_vector(start_point, offset),
            sub_vector(end_point, offset), add_vector(end_point, offset)]
