#!/usr/bin/env python
"""Main script defining the ROS node"""

import os
import json
from dataclasses import dataclass
from time import sleep

import rospy

from nav_srvs.srv import NavigationRequest, NavigationRequestResponse
from global_planner.global_route_planner import GlobalPlanner
from global_planner.xodr_converter import XODRConverter


@dataclass
class GlobalPlannerNode:
    """A class representing a ROS node that's planning a global route."""
    vehicle_name: str
    map_path: str = None

    def __post_init__(self):
        while True:
            try:
                full_param_name = rospy.search_param('town')
                town = rospy.get_param(full_param_name)['carla']['town']
                self.map_path = f"/app/res/xodr/{town}.xodr"
                print("Town is: ", town)
                break
            except KeyError:
                print('waiting for town rosparam')
                sleep(0.1)

    def run_node(self):
        """Launch the ROS node to receive the map, the start and
        end position and convert them into a globally planned route."""
        rospy.init_node(f'global_planner_{self.vehicle_name}', anonymous=True)
        rospy.Service("navigate", NavigationRequest, self._handle_navigation_request)
        rospy.spin()

    def _handle_navigation_request(self, nav_request):
        try:
            start_pos = (nav_request.start_x, nav_request.start_y)
            end_pos = (nav_request.end_x, nav_request.end_y)
            orient_rad = nav_request.orientation_rad
            print(f'creating route from {start_pos} to {end_pos}')
        except:
            print('malformed request!')
            return NavigationRequestResponse(waypoints_json='[]', success=False)

        if not os.path.exists(self.map_path) or not os.path.isfile(self.map_path):
            print('XODR not initialized!')
            return NavigationRequestResponse(waypoints_json='[]', success=False)

        try:
            xodr_map = XODRConverter.read_xodr(self.map_path)
            global_route = GlobalPlanner.generate_waypoints(
                start_pos, end_pos, orient_rad, xodr_map)
            print("Route generated!")

            route_as_json = [{'x': wp.pos[0], 'y': wp.pos[1],
                              'actual_lane': wp.actual_lane,
                              'possible_lanes': wp.possible_lanes,
                              'legal_speed': wp.legal_speed,
                              'dist_next_tl': wp.dist_next_tl,
                              'end_lane_m': wp.end_lane_m} for wp in global_route]
        except:
            print('request failed!')
            return NavigationRequestResponse(waypoints_json='[]', success=False)

        response = NavigationRequestResponse(
            waypoints_json=json.dumps(route_as_json),
            success=True
        )

        return response


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""
    vehicle_name = "ego_vehicle"

    node = GlobalPlannerNode(vehicle_name)
    node.run_node()


if __name__ == '__main__':
    main()
