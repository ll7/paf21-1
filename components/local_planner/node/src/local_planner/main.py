#!/usr/bin/env python
"""Main script defining the ROS node"""
from typing import Callable, Tuple, List
from dataclasses import dataclass, field

import rospy
from sensor_msgs.msg import Image as ImageMsg, Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
# from nav_msgs.msg import Path as WaypointsMsg

from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.route_planner import RouteInfo
from local_planner.vehicle_control import DrivingController
from local_planner.ros_msg_adapter import RosDrivingMessagesAdapter


@dataclass
class RosMsgMultiplexer:
    """A proxy for forwarding ROS messages to multiple consumers."""
    consumers: List[Callable] = field(default_factory=list)

    def forward_msg(self, msg):
        """Send the given message to all registered consumers."""
        for consumer_func in self.consumers:
            consumer_func(msg)


@dataclass
class LocalPlannerNode:
    """A class representing a ROS node that's processing route and
    local sensor data to handle the interaction with other vehicles, etc."""

    vehicle_name: str
    publish_rate_in_hz: int
    driving_signal_publisher: rospy.Publisher = None
    image_preprocessor: SensorCameraPreprocessor = SensorCameraPreprocessor()
    route_planner: RouteInfo = RouteInfo()
    driving_controller: DrivingController() = DrivingController()

    def run_node(self):
        """Launch the ROS node to receive globally planned routes
        and convert them into locally planned routes using sensors"""

        self._init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)
        self.driving_controller.target_velocity_mps = 10.0

        while not rospy.is_shutdown():
            local_route = self.route_planner.compute_local_route()
            self.driving_controller.update_route(local_route)
            driving_signal = self.driving_controller.next_signal()
            msg = RosDrivingMessagesAdapter.signal_to_message(driving_signal)
            self.driving_signal_publisher.publish(msg)
            rate.sleep()

    def _init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""

        rospy.init_node(f'local_planner_{self.vehicle_name}', anonymous=True)
        self.driving_signal_publisher = self._init_driving_signal_publisher()
        self._init_vehicle_position_subscriber()
        self._init_vehicle_orientation_subscriber()
        self._init_global_route_subscriber()
        self._init_front_camera_subscribers()
        self._init_target_velocity_subscriber()

    def _init_global_route_subscriber(self):
        in_topic = f"/drive/{self.vehicle_name}/global_route"
        msg_to_route = RosDrivingMessagesAdapter.json_message_to_waypoints
        process_route = self.route_planner.update_global_route
        callback = lambda msg: process_route(msg_to_route(msg))
        rospy.Subscriber(in_topic, StringMsg, callback)

    def _init_target_velocity_subscriber(self):
        in_topic = f"/drive/{self.vehicle_name}/target_velocity"
        msg_to_velocity = RosDrivingMessagesAdapter.message_to_target_velocity
        process_velocity = self.driving_controller.update_target_velocity
        callback = lambda msg: process_velocity(msg_to_velocity(msg))
        rospy.Subscriber(in_topic, FloatMsg, callback)

    def _init_vehicle_orientation_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/imu/imu1"
        msg_to_orientation = RosDrivingMessagesAdapter.message_to_orientation
        multi_callback = RosMsgMultiplexer(consumers=[
            lambda msg: self.driving_controller.update_vehicle_orientation(msg_to_orientation(msg)),
            lambda msg: self.route_planner.update_vehicle_orientation(msg_to_orientation(msg))
        ])
        rospy.Subscriber(in_topic, ImuMsg, multi_callback.forward_msg)

    def _init_vehicle_position_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/odometry"
        msg_to_position = RosDrivingMessagesAdapter.message_to_vehicle_position
        multi_callback = RosMsgMultiplexer(consumers=[
            lambda msg: self.driving_controller.update_vehicle_position(msg_to_position(msg)),
            lambda msg: self.route_planner.update_vehicle_position(msg_to_position(msg))
        ])
        rospy.Subscriber(in_topic, OdometryMsg, multi_callback.forward_msg)

    def _init_front_camera_subscribers(self):
        camera_semantic_seg = "semantic_segmentation/front/image_segmentation"
        in_semantic_topic = f"/carla/{self.vehicle_name}/camera/{camera_semantic_seg}"
        rospy.Subscriber(in_semantic_topic, ImageMsg, self.image_preprocessor.process_semantic_image)

        camera_depth = 'depth/front/image_depth'
        in_depth_topic = f"/carla/{self.vehicle_name}/camera/{camera_depth}"
        rospy.Subscriber(in_depth_topic, ImageMsg, self.image_preprocessor.process_depth_image)

        camera_rgb = 'rgb/front/image_color'
        in_rgb_topic = f"/carla/{self.vehicle_name}/camera/{camera_rgb}"
        rospy.Subscriber(in_rgb_topic, ImageMsg, self.image_preprocessor.process_rgb_image)

    def _init_driving_signal_publisher(self):
        out_topic = f"/carla/{self.vehicle_name}/ackermann_cmd"
        return rospy.Publisher(out_topic, AckermannDrive, queue_size=100)




    # def _init_global_route_subscriber(self):
    #     """Initialize the ROS subscriber receiving global routes"""
    #     in_topic = f"/drive/{self.vehicle_name}/global_route"
    #     callback = lambda msg: self.route_planner.update_global_route(
    #         self.json_message_to_waypoints(msg))
    #     rospy.Subscriber(in_topic, StringMsg, callback)

    # @staticmethod
    # def json_message_to_waypoints(msg: StringMsg) -> List[Tuple[float, float]]:
    #     """Convert a ROS message into waypoints"""
    #     json_list = json.loads(msg.data)
    #     waypoints = [[wp['x'], wp['y']] for wp in json_list]
    #     return waypoints

    # def init_gps_subscriber(self):
    #     """Initialize the ROS subscriber receiving GPS data"""
    #     in_topic = f"/carla/{self.vehicle_name}/odometry"
    #     rospy.Subscriber(in_topic, OdometryMsg, self.route_planner.update_gps)

    # def init_vehicle_orientation_subscriber(self):
    #     """Initialize the ROS subscriber receiving the orientation of the vehicle"""
    #     in_topic = f"/carla/{self.vehicle_name}/imu/imu1"
    #     rospy.Subscriber(in_topic, ImuMsg, self.route_planner.update_vehicle_vector)

    # def init_local_route_publisher(self):
    #     """Initialize the ROS publisher for submitting local routes"""
    #     out_topic = f"/drive/{self.vehicle_name}/local_route"
    #     return rospy.Publisher(out_topic, StringMsg, queue_size=100)

    # @classmethod
    # def parse_route(cls, route_json: StringMsg):
    #     """Parse the route from JSON given a ROS message"""
    #     json_data = route_json.data
    #     return json.load(json_data)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    publish_rate_hz = 10
    node = LocalPlannerNode(vehicle_name, publish_rate_hz)
    node.run_node()


if __name__ == '__main__':
    main()
