"""A module for detecting curves given the route as waypoints"""

from dataclasses import dataclass
from typing import List, Tuple
from math import dist as euclid_dist, sqrt
import numpy as np

from vehicle_control.core.geometry import approx_curvature_radius


@dataclass
class CurveObservation:
    """Representing the next curve ahead"""
    dist_until_curve: float=1000
    dist_end_curve: float=1000
    max_speed: float=500
    end_id: int=-1


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

        dist_until_curve = CurveDetection._route_dist(wps_until_curve)
        dist_end_curve = dist_until_curve + CurveDetection._route_dist(wps_curve)
        max_curve_speed = CurveDetection._curve_target_speed(wps_curve)
        return CurveObservation(dist_until_curve, dist_end_curve, max_curve_speed, curve_end_id)

    @staticmethod
    def _route_dist(wps: List[Tuple[float, float]]):
        return sum([euclid_dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1)])

    @staticmethod
    def _find_next_curve_bounds(wps: List[Tuple[float, float]]) -> Tuple[int, int] or None:
        """Scans the route waypoints for the next curve ahead and return
        the start / end index of the curve (or None if there's no curve)"""

        # this radius threshold is equal to a 90 deg curve that's ~80 meters long
        radius_threshold = 50
        scan_span = 15

        wp_dists = [euclid_dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
        if len(wps) < 3 or sum(wp_dists) < scan_span:
            return None

        i_start = 0
        radiuses = []
        while True:
            # find end point spanning at least the scan span
            i_end = i_start + 2
            while sum(wp_dists[i_start:i_end]) < scan_span and i_end < len(wp_dists):
                i_end += 1

            # get curve span and the start / end point
            curve_span = sum(wp_dists[i_start:i_end])
            start, end = wps[i_start], wps[i_end]

            # find point in the middle
            start_offsets = np.array([euclid_dist(start, wps[i]) for i in range(i_start+1, i_end)])
            i_middle = np.argmin(np.abs(start_offsets - curve_span/2)) + i_start + 1
            middle = wps[i_middle]

            radius = approx_curvature_radius(start, middle, end)
            radiuses.append((radius, i_start, i_end))

            i_start += 1
            if i_start >= len(wps) - 2 or sum(wp_dists[i_start:]) < scan_span:
                break

        first = next(filter(lambda r: r[0] < radius_threshold, radiuses), None)
        if not first or first[1] + 1 >= len(radiuses):
            return None
        last = next(filter(lambda r: r[0] > radius_threshold, radiuses[first[1] + 1:]), None)
        return (first[1], last[2]) if last else None

    @staticmethod
    def _curve_target_speed(wps_curve: List[Tuple[float, float]]) -> float:
        """ Determine the max. speed possible to drive the given curvature
        using a formula that approximates the car's friction given the radius."""

        friction_coeff = 0.7  # 0.6
        gravity_accel = 9.81

        p_1, p_2, p_3 = wps_curve[0], wps_curve[len(wps_curve) // 2], wps_curve[-1]
        radius_1 = approx_curvature_radius(p_1, p_2, p_3)
        if len(wps_curve) >= 5:
            p_1, p_2, p_3 = wps_curve[0], wps_curve[2], wps_curve[4]
        radius_2 = approx_curvature_radius(p_1, p_2, p_3)

        min_radius = min(radius_1, radius_2)
        max_speed = sqrt(friction_coeff * gravity_accel * min_radius)

        return max_speed
