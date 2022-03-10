"""A module providing navigation services"""

import json
from time import sleep
from math import dist
from random import choice
from typing import Callable, List, Tuple
from dataclasses import dataclass

import rospy

from nav_srvs.srv import NavigationRequest
from local_planner.core import Vehicle, AnnRouteWaypoint

from local_planner.config import town_spawns


def load_spawn_positions() -> List[Tuple[float, float]]:
    """Retrieve a list of possible spawn positions"""
    # TODO: ROS param server not working reliably yet -> fix this
    # full_param_name = rospy.search_param('town')
    # print(rospy.get_param(full_param_name))
    # active_town = rospy.get_param(full_param_name)['carla']['town']
    # print("Town is : ", active_town)
    return town_spawns.spawns['Town04']


@dataclass
class InfiniteDrivingService():
    """Representing a proxy for requesting navigation services."""
    vehicle: Vehicle
    update_route: Callable[[List[AnnRouteWaypoint]], None]
    destinations: List[Tuple[float, float]] = None
    current_dest: Tuple[float, float] = None

    def __post_init__(self):
        if self.destinations is None:
            self.destinations = load_spawn_positions()

    def run_infinite_driving(self):
        """Launch the infinite driving mode. This will always request new routes
        to random destinations whenever the car completes the previous route."""
        list_except = lambda l, e: [x for x in l if x != e]

        while True:
            if not self.vehicle.is_ready:
                sleep(1)
                continue

            if self.current_dest is None:
                self.current_dest = self.vehicle.pos

            if dist(self.vehicle.pos, self.current_dest) > 10.0:
                sleep(0.01)
                continue

            start_pos = self.vehicle.pos
            orientation = self.vehicle.orientation_rad
            self.current_dest = choice(list_except(self.destinations, self.current_dest))

            route = InfiniteDrivingService._request_new_route(
                start_pos, self.current_dest, orientation)
            self.update_route(route)

    @staticmethod
    def _request_new_route(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           orientation_rad: float) -> List[Tuple[float, float]]:

        service_name = 'navigate'
        rospy.wait_for_service(service_name)

        while True:
            try:
                navigate_proxy = rospy.ServiceProxy(service_name, NavigationRequest)
                response = navigate_proxy(start_pos[0], start_pos[1],
                                          end_pos[0], end_pos[1], orientation_rad)

                if not response.success:
                    continue

                json_list = json.loads(response.waypoints_json)
                waypoints = [
                    AnnRouteWaypoint(
                        (wp['x'], wp['y']),
                        wp['actual_lane'],
                        wp['possible_lanes'],
                        wp['legal_speed'], wp['dist_next_tl'],
                        wp['end_lane_m'])
                    for wp in json_list]
                return waypoints

            except rospy.ServiceException as error:
                print(f"Service call failed: {error}")
                sleep(1)
