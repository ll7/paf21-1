"""A module for interpolating routes"""

from math import dist as euclid_dist, floor, asin, sqrt, pi
from typing import List, Tuple

from local_planner.core.geometry import \
    rotate_vector, points_to_vector, add_vector, scale_vector, unit_vector


def end_of_circular_arc(start_point: Tuple[float, float], angle: float,
                        length: float, radius: float) -> Tuple[float, float]:
    """Compute the end of a circular arc"""

    # determine the length of |start, end|
    alpha = length / radius
    diff_vec = scale_vector(unit_vector(alpha), radius)
    dist_start_end = euclid_dist(diff_vec, (radius, 0))

    # determine the direction of |start, end| and apply it
    dir_start_end = unit_vector(alpha / 2 + angle)

    # apply vector |start --> end| to the start point to retrieve the end point
    diff_vec = scale_vector(dir_start_end, dist_start_end)
    return add_vector(start_point, diff_vec)


def circular_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                           arc_radius: float) -> List[Tuple[float, float]]:
    """Interpolate points between start / end point
    on top of the circular arc given by the arc radius."""

    step_size = 2.0
    sign = -1 if arc_radius < 0 else 1
    arc_radius = abs(arc_radius)

    # determine the circular angle of the arc
    angle = asin((euclid_dist(start, end) / 2) / arc_radius) * 2

    # construct the mid-perpendicular of |start, end| to determine the circle's center
    conn_middle = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
    center_offset = sqrt(pow(arc_radius, 2) - pow(euclid_dist(start, end) / 2, 2))
    mid_perpend = rotate_vector(points_to_vector(start, end), pi/2 * sign)
    circle_center = add_vector(conn_middle, scale_vector(mid_perpend, center_offset))

    # partition the arc into steps (-> interpol. geometries)
    arc_circumference = arc_radius * angle              # (r * 2 pi) * (angle / 2 pi)
    num_steps = int(arc_circumference / step_size) + 1  # each step < step size

    # compute the interpolated points on the circle arc
    vec = points_to_vector(circle_center, start)
    rot_angles = [angle * (i / num_steps) for i in range(num_steps+1)]
    points = [add_vector(circle_center, rotate_vector(vec, rot * sign)) for rot in rot_angles]

    return points


def linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                         interval_m: float) -> List[Tuple[float, float]]:
    """Interpolate linearly between the given start / end point
    by putting points according to the interval specified."""

    distance = euclid_dist(start, end)
    vector = (end[0] - start[0], end[1] - start[1])

    steps = max(1, floor(distance / interval_m))
    exceeds_interval_cap = distance > interval_m
    step_vector = (vector[0] / steps if exceeds_interval_cap else vector[0],
                   vector[1] / steps if exceeds_interval_cap else vector[1])

    lin_points = [(start[0] + step_vector[0] * i,
                   start[1] + step_vector[1] * i)
                   for i in range(steps)]

    return lin_points


def interpolate_route(orig_route: List[Tuple[float, float]], interval_m=0.5):
    """Interpolate the given route with points inbetween,
    holding the specified distance interval threshold."""

    orig_route = _clean_route_duplicates(orig_route, min_dist=0.01)
    route = []
    for index in range(len(orig_route) - 1):
        waypoints = linear_interpolation(orig_route[index], orig_route[index + 1], interval_m)
        route.extend(waypoints)

    route = route + [orig_route[-1]]
    return _clean_route_duplicates(route, min_dist=0.1)


def _clean_route_duplicates(route: List[Tuple[float, float]],
                            min_dist: float) -> List[Tuple[float, float]]:
    cleaned_route = [route[0]]
    for next_p in route[1:]:
        if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
            cleaned_route.append(next_p)
    return cleaned_route


# def _catmull_rom_spline(points: List[Tuple[float, float]], interval_m):
#     """
#     p0, p1, p2, and p3 should be (x,y) point pairs that define the Catmull-Rom spline.
#     numpoints is the number of points to include in this curve segment.
#     """
#     p_0, p_1, p_2, p_3 = points
#     # calc number of Interpolation Points
#     distance = euclid_dist(p_1, p_2)
#     num_points = max(2, ceil(distance / interval_m) + 1)

#     # Convert the points to numpy so that we can do array multiplication
#     p_0, p_1, p_2, p_3 = map(numpy.array, [p_0, p_1, p_2, p_3])

#     # Parametric constant: 0.5 for the centripetal spline,
#     # 0.0 for the uniform spline, 1.0 for the chordal spline.
#     alpha = 0.25

#     circle = lambda t_i, p_i, p_j: ((p_j[0] - p_i[0]) ** 2 +
#                                     (p_j[1] - p_i[1]) ** 2) ** alpha + t_i
#     # Calculate t0 to t3
#     t_0 = 0
#     t_1 = circle(t_0, p_0, p_1)
#     t_2 = circle(t_1, p_1, p_2)
#     t_3 = circle(t_2, p_2, p_3)

#     # Only calculate points between P1 and P2
#     t = numpy.linspace(t_1, t_2, num_points)

#     # Reshape so that we can multiply by the points P0 to P3
#     # and get a point for each value of t.
#     t = t.reshape(len(t), 1)
#     a1 = (t_1 - t) / (t_1 - t_0) * p_0 + (t - t_0) / (t_1 - t_0) * p_1
#     a2 = (t_2 - t) / (t_2 - t_1) * p_1 + (t - t_1) / (t_2 - t_1) * p_2
#     a3 = (t_3 - t) / (t_3 - t_2) * p_2 + (t - t_2) / (t_3 - t_2) * p_3
#     b1 = (t_2 - t) / (t_2 - t_0) * a1 + (t - t_0) / (t_2 - t_0) * a2
#     b2 = (t_3 - t) / (t_3 - t_1) * a2 + (t - t_1) / (t_3 - t_1) * a3

#     c = (t_2 - t) / (t_2 - t_1) * b1 + (t - t_1) / (t_2 - t_1) * b2
#     spline: List[Tuple[float, float]] = []
#     for next_p in c:
#         spline.append((next_p[0], next_p[1]))

#     return spline
