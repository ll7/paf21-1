"""A module for interpolating routes"""

from math import dist as euclid_dist, floor
from typing import List, Tuple


class RouteInterpolation:
    # pylint: disable=too-few-public-methods
    """Representing a helper for interpolating route waypoints"""

    @staticmethod
    def interpolate_route(orig_route: List[Tuple[float, float]], interval_m=2.0):
        """Interpolate the given route waypoints with a given interval"""
        route = []

        for index in range(len(orig_route) - 1):
            waypoints = RouteInterpolation._linear_interpolation(
                orig_route[index], orig_route[index + 1], interval_m=interval_m)
            route.extend(waypoints)

        route = route + [orig_route[-1]]
        return RouteInterpolation._clean_route_duplicates(route, min_dist=0.1)

    @staticmethod
    def _linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
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
    def _clean_route_duplicates(route: List[Tuple[float, float]],
                                min_dist: float) -> List[Tuple[float, float]]:
        cleaned_route = [route[0]]
        for next_p in route[1:]:
            if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
                cleaned_route.append(next_p)
        return cleaned_route
