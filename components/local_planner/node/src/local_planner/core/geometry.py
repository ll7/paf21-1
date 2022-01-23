"""This module contains a collection of geometric helper functions"""

from math import dist as euclid_dist, atan2, pi, sqrt, acos
from numpy import dot
from numpy.linalg import norm
from typing import Tuple

def points_to_vector(p_from: Tuple[float, float], p_to: Tuple[float, float]):
    """Transform the given points into a vector pointing from the start to the end"""
    vector = (p_to[0] - p_from[0], p_to[1] - p_from[1])
    return vector

def vector_to_dir(vector: Tuple[float, float]):
    """Get the given vector's direction in radians within [-pi, pi)"""
    normed_vector = norm_vector(vector)
    angle = atan2(normed_vector[1], normed_vector[0])
    return norm_angle(angle)

def norm_vector(vector: Tuple[float, float]):
    """Normalize the given vector to a proportional vector of length 1"""
    vector_len = sqrt(pow(vector[0], 2) + pow(vector[1], 2))
    normed_vector = (vector[0] / vector_len, vector[1] / vector_len)
    return normed_vector

def norm_angle(angle_rad: float) -> float:
    """Normalize the given angle within [-pi, +pi)"""
    while angle_rad > pi:
        angle_rad -= 2.0 * pi
    while angle_rad < -pi:
        angle_rad += 2.0 * pi
    if angle_rad == pi:
        angle_rad = -pi
    return angle_rad

def angle_between(vec1: Tuple[float, float], vec2: Tuple[float, float]):
    return acos(dot(vec1, vec2)/norm(vec1)/norm(vec2))


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

    menger = (4 * tri_area) / (tri_len_a * tri_len_b * tri_len_c)
    radius = abs(1.0 / (menger + epsilon))
    return radius
