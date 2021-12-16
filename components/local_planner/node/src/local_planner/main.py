#!/usr/bin/env python
"""Main script defining the ROS node"""
from typing import Tuple, List
import json
from dataclasses import dataclass
import rospy
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image as ImageMsg, Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg
from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.route_planner import RouteInfo


@dataclass
class LocalPlannerNode:
    """A class representing a ROS node that's processing route and
    local sensor data to handle the interaction with other vehicles, etc."""

    vehicle_name: str
    publish_rate_in_hz: int
    local_route_publisher: rospy.Publisher = None
    image_preprocessor: SensorCameraPreprocessor = SensorCameraPreprocessor()
    route_planner: RouteInfo = RouteInfo()

    def run_node(self):
        """Launch the ROS node to receive globally planned routes
        and convert them into locally planned routes using sensors"""

        self.init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            local_route = self.route_planner.compute_local_route()
            route_as_json = [{ 'x': pos[0], 'y': pos[1] } for pos in local_route]
            msg = StringMsg(data=json.dumps(route_as_json))
            self.local_route_publisher.publish(msg)
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
        callback = lambda msg: self.route_planner.update_global_route(
            self.json_message_to_waypoints(msg))
        rospy.Subscriber(in_topic, StringMsg, callback)

    @staticmethod
    def json_message_to_waypoints(msg: StringMsg) -> List[Tuple[float, float]]:
        """Convert a ROS message into waypoints"""
        json_list = json.loads(msg.data)
        waypoints = [[wp['x'], wp['y']] for wp in json_list]
        return waypoints


    def init_gps_subscriber(self):
        """Initialize the ROS subscriber receiving GPS data"""
        in_topic = f"/carla/{self.vehicle_name}/odometry"
        rospy.Subscriber(in_topic, OdometryMsg, self.route_planner.update_gps)


    def init_vehicle_orientation_subscriber(self):
        """Initialize the ROS subscriber receiving the orientation of the vehicle"""
        in_topic = f"/carla/{self.vehicle_name}/imu/imu1"
        rospy.Subscriber(in_topic, ImuMsg, self.route_planner.update_vehicle_vector)

    def init_front_camera_subscriber(self):
        """Initialize the ROS subscriber receiving camera images with following sensors :

        - semantic
        - depth
        -rgb
        """
        camera_semantic_seg = "semantic_segmentation/front/image_segmentation"
        in_semantic_topic = f"/carla/{self.vehicle_name}/camera/{camera_semantic_seg}"

        camera_rgb = 'rgb/front/image_color'
        in_rgb_topic = f"/carla/{self.vehicle_name}/camera/{camera_rgb}"

        camera_depth = 'depth/front/image_depth'
        in_depth_topic = f"/carla/{self.vehicle_name}/camera/{camera_depth}"

        rospy.Subscriber(in_semantic_topic, ImageMsg,
                         self.image_preprocessor.process_semantic_image)
        rospy.Subscriber(in_depth_topic, ImageMsg,
                         self.image_preprocessor.process_depth_image)
        rospy.Subscriber(in_rgb_topic, ImageMsg,
                         self.image_preprocessor.process_rgb_image)

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
