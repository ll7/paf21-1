"""A module for interpolating routes"""

from math import dist as euclid_dist, floor, ceil
from typing import List, Tuple
import numpy


class RouteInterpolation:
    # pylint: disable-all
    """Representing a helper for interpolating route waypoints"""

    @staticmethod
    def interpolate_route(orig_route: List[Tuple[float, float]], interval_m=0.5):
        """Interpolate the given route waypoints with a given interval"""

        "Clean Duplicates"
        orig_route = RouteInterpolation._clean_route_duplicates(orig_route, min_dist=0.01)

        # use linear_interpolation
        route = []
        for index in range(len(orig_route) - 1):
            waypoints = RouteInterpolation.linear_interpolation(
                orig_route[index], orig_route[index + 1], interval_m=interval_m)
            route.extend(waypoints)
        #
        # for index in range(len(orig_route) - 1):
        #     # use linear interpolation for first and last segment
        #     if index == 0 or index == (len(orig_route) - 2):
        #         waypoints = RouteInterpolation.linear_interpolation(
        #             orig_route[index], orig_route[index + 1], interval_m=interval_m)
        #         route.extend(waypoints)
        #     # else use spline interpolation
        #     else:
        #         waypoints = RouteInterpolation._catmull_rom_spline(
        #             orig_route[index-1:index+3], interval_m=interval_m)
        #         route.extend(waypoints)

        route = route + [orig_route[-1]]
        # print("Spline Route")
        # print(route)
        return RouteInterpolation._clean_route_duplicates(route, min_dist=0.1)

    @staticmethod
    def linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                             interval_m: float) -> List[Tuple[float, float]]:

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

    @staticmethod
    def _catmull_rom_spline(points: List[Tuple[float, float]], interval_m):
        """
        p0, p1, p2, and p3 should be (x,y) point pairs that define the Catmull-Rom spline.
        numpoints is the number of points to include in this curve segment.
        """
        p_0, p_1, p_2, p_3 = points
        # calc number of Interpolation Points
        distance = euclid_dist(p_1, p_2)
        num_points = max(2, ceil(distance / interval_m) + 1)

        # Convert the points to numpy so that we can do array multiplication
        p_0, p_1, p_2, p_3 = map(numpy.array, [p_0, p_1, p_2, p_3])

        # Parametric constant: 0.5 for the centripetal spline,
        # 0.0 for the uniform spline, 1.0 for the chordal spline.
        alpha = 0.25

        circle = lambda t_i, p_i, p_j: ((p_j[0] - p_i[0]) ** 2 +
                                        (p_j[1] - p_i[1]) ** 2) ** alpha + t_i
        # Calculate t0 to t3
        t_0 = 0
        t_1 = circle(t_0, p_0, p_1)
        t_2 = circle(t_1, p_1, p_2)
        t_3 = circle(t_2, p_2, p_3)

        # Only calculate points between P1 and P2
        t = numpy.linspace(t_1, t_2, num_points)

        # Reshape so that we can multiply by the points P0 to P3
        # and get a point for each value of t.
        t = t.reshape(len(t), 1)
        a1 = (t_1 - t) / (t_1 - t_0) * p_0 + (t - t_0) / (t_1 - t_0) * p_1
        a2 = (t_2 - t) / (t_2 - t_1) * p_1 + (t - t_1) / (t_2 - t_1) * p_2
        a3 = (t_3 - t) / (t_3 - t_2) * p_2 + (t - t_2) / (t_3 - t_2) * p_3
        b1 = (t_2 - t) / (t_2 - t_0) * a1 + (t - t_0) / (t_2 - t_0) * a2
        b2 = (t_3 - t) / (t_3 - t_1) * a2 + (t - t_1) / (t_3 - t_1) * a3

        c = (t_2 - t) / (t_2 - t_1) * b1 + (t - t_1) / (t_2 - t_1) * b2
        spline: List[Tuple[float, float]] = []
        for next_p in c:
            spline.append((next_p[0], next_p[1]))

        return spline

    @staticmethod
    def _clean_route_duplicates(route: List[Tuple[float, float]],
                                min_dist: float) -> List[Tuple[float, float]]:
        cleaned_route = [route[0]]
        for next_p in route[1:]:
            if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
                cleaned_route.append(next_p)
        return cleaned_route
