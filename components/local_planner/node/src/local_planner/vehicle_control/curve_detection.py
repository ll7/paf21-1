"""A module for detecting curves given the route as waypoints"""

from dataclasses import dataclass
from typing import List, Tuple
from math import dist as euclid_dist, sqrt
from local_planner.core import geometry

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
        """Scans the route waypoints for the next curve ahead and return the start / end index of the curve 
            (or None if there's no curve)"""
        
        curvature = geometry.find_curvature(wps)
        #print("curvature : ", curvature)
        if len(curvature) == 3:
            print("Curvature detected. Returning start and end coords : ",  wps.index(curvature[1]),  wps.index(curvature[2]))
            return (wps.index(curvature[1]),  wps.index(curvature[2]))
        else:
            return None

    @staticmethod
    def _curve_target_speed(wps_curve: List[Tuple[float, float]]) -> float:
        """ Determine the max. speed possible to take the given curve"""
        print('Len of Curve : {}, curve speed: {}'.format(len(wps_curve), sqrt(len(wps_curve))))
        if sqrt(len(wps_curve)) >= 5:
            return sqrt(len(wps_curve))
        else:
            return 3
