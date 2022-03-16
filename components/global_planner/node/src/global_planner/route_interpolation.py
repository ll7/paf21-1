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
        route = []

        "Clean Duplicates"
        orig_route = RouteInterpolation._clean_route_duplicates(orig_route, min_dist=0.01)

        # use linear_interpolation

        for index in range(len(orig_route) - 1):
            waypoints = RouteInterpolation.linear_interpolation(
                orig_route[index], orig_route[index + 1], interval_m=interval_m)
            route.extend(waypoints)
        print("linear")
        print(route)
        route = []

        # use spline_interpolation
        print("orgi_route")
        print(orig_route)
        for index in range(len(orig_route) - 1):
            # use linear interpolation for first and last segment
            if index == 0 or index == (len(orig_route) - 2):
                waypoints = RouteInterpolation.linear_interpolation(
                    orig_route[index], orig_route[index + 1], interval_m=interval_m)
                route.extend(waypoints)
            # else use spline interpolation
            else:
                waypoints = RouteInterpolation._catmull_rom_spline(orig_route[index - 1],
                                                                   orig_route[index], orig_route[index + 1],
                                                                   orig_route[index + 2], interval_m=interval_m)
                route.extend(waypoints)
        route = route + [orig_route[-1]]
        print("Spline Route")
        print(route)
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
    def _catmull_rom_spline(p0, p1, p2, p3, interval_m):
        """
        p0, p1, p2, and p3 should be (x,y) point pairs that define the Catmull-Rom spline.
        numpoints is the number of points to include in this curve segment.
        """

        # calc number of Interpolation Points
        distance = euclid_dist(p1, p2)
        numpoints = max(2, ceil(distance / interval_m)+1)

        # Convert the points to numpy so that we can do array multiplication
        p0, p1, p2, p3 = map(numpy.array, [p0, p1, p2, p3])

        # Parametric constant: 0.5 for the centripetal spline, 0.0 for the uniform spline, 1.0 for the chordal spline.
        alpha = 0.5
        # Premultiplied power constant for the following tj() function.
        alpha = alpha / 2

        def tj(ti, pi, pj):
            xi, yi = pi
            xj, yj = pj
            return ((xj - xi) ** 2 + (yj - yi) ** 2) ** alpha + ti

        # Calculate t0 to t3
        t0 = 0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)

        # Only calculate points between P1 and P2
        t = numpy.linspace(t1, t2, numpoints)

        # Reshape so that we can multiply by the points P0 to P3
        # and get a point for each value of t.
        t = t.reshape(len(t), 1)
        #print(t)
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        #print(a1)
        #print(a2)
        #print(a3)
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3

        c = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2
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
