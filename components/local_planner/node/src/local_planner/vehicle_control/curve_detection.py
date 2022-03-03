"""A module for detecting curves given the route as waypoints"""

from dataclasses import dataclass
from typing import List, Tuple
from math import dist as euclid_dist, sqrt
from local_planner.core.geometry import approx_curvature_radius


@dataclass
class CurveObservation:
    """Representing the next curve ahead"""
    dist_until_curve: float=1000
    max_speed: float=500


class CurveDetection:
    # pylint: disable=too-few-public-methods
    """Representing a strategy for detecting curves ahead
    by evaluating route waypoints"""

    @staticmethod
    def find_next_curve(route_wps: List[Tuple[float, float]]) -> CurveObservation:
        """Evaluate the given route for the next curve to be handled"""

        curve_bounds = CurveDetection._find_next_curve_bounds(route_wps)
        if not curve_bounds:
            return CurveObservation()

        curve_bounds: Tuple[int, int]
        curve_start_id, curve_end_id = curve_bounds
        wps_until_curve = route_wps[:curve_start_id]
        wps_curve = route_wps[curve_start_id:curve_end_id]

        print(f'num_points until curve: {len(wps_until_curve)}')
        print(f'num_points of curve: {len(wps_curve)}')

        dist_until_curve = CurveDetection._route_dist(wps_until_curve)
        max_curve_speed = CurveDetection._curve_target_speed(wps_curve)
        return CurveObservation(dist_until_curve, max_curve_speed)

    @staticmethod
    def _route_dist(wps: List[Tuple[float, float]]):
        return sum([euclid_dist(wps[i], wps[i+1]) for i in range(len(wps) - 1)])

    @staticmethod
    def _find_next_curve_bounds(wps: List[Tuple[float, float]]) -> Tuple[int, int] or None:
        """Scans the route waypoints for the next curve ahead and return
        the start / end index of the curve (or None if there's no curve)"""

        # this radius threshold is equal to a 90 deg curve that's ~80 meters long
        radius_threshold = 50

        if len(wps) < 7:
            return None

        ids = range(len(wps) - 7)
        radiuses = iter(map(lambda i: (approx_curvature_radius(wps[i], wps[i+3], wps[i+6]), i), ids))
        start_id = next(filter(lambda r: r[0] < radius_threshold, radiuses), None)
        end_id = next(filter(lambda r: r[0] > radius_threshold, radiuses), None)

        if start_id and end_id:
            print(f'curve detected: {wps[start_id[1]:end_id[1]+1]}')

        return (start_id[1], end_id[1]) if start_id and end_id else None

    @staticmethod
    def _curve_target_speed(wps_curve: List[Tuple[float, float]]) -> float:
        """ Determine the max. speed possible to drive the given curvature
        using a formula that approximates the car's friction given the radius."""

        friction_coeff = 0.6
        gravity_accel = 9.81

        p1, p2, p3 = wps_curve[0], wps_curve[len(wps_curve) // 2], wps_curve[-1]
        radius = approx_curvature_radius(p1, p2, p3)
        max_speed = sqrt(friction_coeff * gravity_accel * radius)

        print(f"radius: {radius}, max speed: {max_speed}")
        return max_speed
