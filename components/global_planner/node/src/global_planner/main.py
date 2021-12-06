#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass

import rospy
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image as ImageMsg, NavSatFix as GpsMsg

from global_route_planner import GlobalRoutePlanner


@dataclass
class GlobalPlannerNode:
    """A class representing a ROS node that's planning a global route."""

    vehicle_name: str
    publish_rate_in_hz: int
    global_route_publisher: rospy.Publisher = None
    # image_preprocessor: RgbCameraPreprocessor = RgbCameraPreprocessor()
    global_route_planner: GlobalRoutePlanner = GlobalRoutePlanner()

    def run_node(self):
        """Launch the ROS node to receive the map, the start and
         end position and convert them into a global planned route."""
        self.init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            if self.global_route_planner.update:
                global_route = self.global_route_planner.compute_route()
                self.global_route_publisher.publish(global_route)
                self.global_route_planner.update = False

            rate.sleep()

    def init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        rospy.init_node(f'global_planner_{self.vehicle_name}', anonymous=True)
        self.global_route_publisher = self.init_global_route_publisher()
        self.init_hmi_route_subscriber()
        self.init_gps_subscriber()

    def init_hmi_route_subscriber(self):
        """Initialize the ROS subscriber receiving map and end"""
        in_topic = f"/drive/{self.vehicle_name}/hmi"
        rospy.Subscriber(in_topic, StringMsg, self.global_route_planner.set_map_end)

    def init_gps_subscriber(self):
        """Initialize the ROS subscriber receiving GPS data"""
        in_topic = f"/carla/{self.vehicle_name}/gnss/gnss1/fix"
        rospy.Subscriber(in_topic, GpsMsg, self.global_route_planner.set_gps)

    def init_global_route_publisher(self):
        """Initialize the ROS publisher for submitting local routes"""
        out_topic = f"/drive/{self.vehicle_name}/global_route"
        return rospy.Publisher(out_topic, StringMsg, queue_size=1)

    @classmethod
    def parse_hmi_msg(cls, hmi_json: StringMsg):
        """Parse the hmi message from JSON given a ROS message"""
        return json.load(hmi_json.data)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    publish_rate_in_hz = 1

    node = GlobalPlannerNode(vehicle_name, publish_rate_in_hz)
    node.run_node()


if __name__ == '__main__':
    main()
