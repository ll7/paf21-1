#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass

import rospy
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image as ImageMsg
from local_planner.preprocessing import RgbCameraPreprocessor


@dataclass
class LocalPlannerNode:
    """A class representing a ROS node that's capable of controlling a vehicle
    and make it drive according to a given route the node is constantly receiving."""

    vehicle_name: str
    publish_rate_in_hz: int
    local_route_publisher: rospy.Publisher = None
    image_preprocessor: RgbCameraPreprocessor = RgbCameraPreprocessor()

    def run_node(self):
        """Launch the ROS node to receive globally planned routes
        and convert them into locally planned routes using sensors"""

        self.init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            signal = None
            self.local_route_publisher.publish(signal)
            rate.sleep()

    def init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        rospy.init_node(f'local_planner_{self.vehicle_name}', anonymous=True)
        self.local_route_publisher = self.init_local_route_publisher()
        self.init_global_route_subscriber()
        self.init_rgb_camera_subscriber()

    def init_global_route_subscriber(self):
        """Initialize the ROS subscriber receiving global routes"""
        callback = lambda msg: msg
        in_topic = f"/drive/{self.vehicle_name}/global_route"
        rospy.Subscriber(in_topic, StringMsg, callback)

    def init_rgb_camera_subscriber(self):
        """Initialize the ROS subscriber receiving camera images"""
        in_topic = f"/carla/{self.vehicle_name}/camera/rgb/front/image_color"
        print(in_topic)
        print("/carla/ego_vehicle/camera/rgb/front/image")
        rospy.Subscriber(in_topic, ImageMsg, self.image_preprocessor.process_image)

    def init_local_route_publisher(self):
        """Initialize the ROS publisher for submitting local routes"""
        out_topic = f"/drive/{self.vehicle_name}/local_route"
        return rospy.Publisher(out_topic, StringMsg, queue_size=100)

    @classmethod
    def parse_route(cls, route_json: StringMsg):
        """Parse the route from JSON given a ROS message"""
        json_data = route_json.data
        return json.load(json_data)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    publish_rate_hz = 10

    node = LocalPlannerNode(vehicle_name, publish_rate_hz)
    node.run_node()


if __name__ == '__main__':
    main()
