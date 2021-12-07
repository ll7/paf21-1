#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass
import rospy
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image as ImageMsg, NavSatFix as GpsMsg, Imu as ImuMsg
from local_planner.preprocessing import RgbCameraPreprocessor
from local_planner.route_planner import RouteInfo


@dataclass
class LocalPlannerNode:
    """A class representing a ROS node that's processing route and
    local sensor data to handle the interaction with other vehicles, etc."""

    vehicle_name: str
    publish_rate_in_hz: int
    local_route_publisher: rospy.Publisher = None
    image_preprocessor: RgbCameraPreprocessor = RgbCameraPreprocessor()
    route_planner: RouteInfo = RouteInfo()

    def run_node(self):
        """Launch the ROS node to receive globally planned routes
        and convert them into locally planned routes using sensors"""

        self.init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            local_route = self.route_planner.compute_local_route()
            local_route = json.dumps(local_route)
            self.local_route_publisher.publish(local_route)
            rate.sleep()

    def init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        rospy.init_node(f'local_planner_{self.vehicle_name}', anonymous=True)
        self.local_route_publisher = self.init_local_route_publisher()
        self.init_gps_subscriber()
        self.init_vehicle_orientation_subscriber()
        self.init_global_route_subscriber()
        self.init_front_camera_subscriber()

    def init_global_route_subscriber(self):
        """Initialize the ROS subscriber receiving global routes"""
        in_topic = f"/drive/{self.vehicle_name}/global_route"
        rospy.Subscriber(in_topic, StringMsg, self.route_planner.update_global_route)

    def init_gps_subscriber(self):
        """Initialize the ROS subscriber receiving GPS data"""
        in_topic = f"/carla/{self.vehicle_name}/gnss/gnss1/fix"
        rospy.Subscriber(in_topic, GpsMsg, self.route_planner.update_gps)

    def init_vehicle_orientation_subscriber(self):
        """Initialize the ROS subscriber receiving GPS data"""
        in_topic = f"/carla/{self.vehicle_name}/imu/imu1"
        rospy.Subscriber(in_topic, ImuMsg, self.route_planner.update_vehicle_vector)

    def init_front_camera_subscriber(self):
        """Initialize the ROS subscriber receiving camera images"""
        camera_type = "semantic_segmentation/front/image_segmentation"
        in_topic = f"/carla/{self.vehicle_name}/camera/{camera_type}"
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
