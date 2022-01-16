#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass
from pathlib import Path

import rospy

from nav_srvs.srv import NavigationRequest, NavigationRequestResponse
from global_planner.global_route_planner import GlobalPlanner
from global_planner.xodr_converter import XODRConverter, XodrMap


@dataclass
class GlobalPlannerNode:
    """A class representing a ROS node that's planning a global route."""
    vehicle_name: str
    path = Path("/app/res/xodr/Town01.xodr")
    xodr_map: XodrMap = None

    def __post_init__(self):
        if not self.xodr_map:
            self.xodr_map = XODRConverter.read_xodr(self.path)

    def run_node(self):
        """Launch the ROS node to receive the map, the start and
        end position and convert them into a globally planned route."""
        rospy.init_node(f'global_planner_{self.vehicle_name}', anonymous=True)
        rospy.Service("navigate", NavigationRequest, self._handle_navigation_request)
        rospy.spin()

    def _handle_navigation_request(self, nav_request):

        # rospy.loginfo(f'start_pos: {(nav_request.start_x, nav_request.start_y)}, \
        #     end_pos: {(nav_request.end_x, nav_request.end_y)}, \
        #     orientation: {nav_request.orientation_rad}')

        xodr_map = XODRConverter.read_xodr(self.path)
        global_route = GlobalPlanner.generate_waypoints(
            (nav_request.start_x, nav_request.start_y),
            (nav_request.end_x, nav_request.end_y),
            nav_request.orientation_rad,
            xodr_map
        )

        route_as_json = [{'x': pos[0], 'y': pos[1]} for pos in global_route]
        response = NavigationRequestResponse(
            waypoints_json = json.dumps(route_as_json),
            success = True
        )

        for _ in range(1000):
            print('foobar')

        # global_route = [(nav_request.start_x, nav_request.start_y),
        #             (nav_request.end_x, nav_request.end_y)]
        # route_as_json = [{'x': pos[0], 'y': pos[1]} for pos in global_route]

        # response = NavigationRequestResponse(
        #     waypoints_json = json.dumps(route_as_json),
        #     success = True
        # )

        return response


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""
    vehicle_name = "ego_vehicle"

    node = GlobalPlannerNode(vehicle_name)
    node.run_node()


if __name__ == '__main__':
    main()
