"""A module for detecting curves given the route as waypoints"""

from dataclasses import dataclass
from typing import List, Tuple
from math import dist as euclid_dist


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

        dist_until_curve = CurveDetection._route_dist(wps_until_curve)
        max_curve_speed = CurveDetection._curve_target_speed(wps_curve)
        return CurveObservation(dist_until_curve, max_curve_speed)

    @staticmethod
    def _route_dist(wps: List[Tuple[float, float]]):
        return sum([euclid_dist(wps[i], wps[i+1]) for i in range(len(wps) - 1)])

    @staticmethod
    def _find_next_curve_bounds(wps: List[Tuple[float, float]]) -> Tuple[int, int] or None:
        # scan route waypoints for the next curve ahead
        # -> return the start / end index of the curve (or some default values if there's no curve)
        # TODO: @Pavlo, implement logic here ...
        pass

    @staticmethod
    def _curve_target_speed(wps_curve: List[Tuple[float, float]]) -> float:
        # determine the max. speed possible to take the given curve
        # TODO: @Pavlo, implement logic here ...
        pass
