#!/usr/bin/env python
"""Main script defining the ROS node"""
from typing import Callable, List
from dataclasses import dataclass, field

import rospy
from sensor_msgs.msg import Image as ImageMsg, Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg

from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.route_planner import TrajectoryPlanner
from local_planner.ros_msg_adapter import RosDrivingMessagesAdapter
from local_planner.core import Vehicle


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
    vehicle: Vehicle
    publish_rate_in_hz: int
    driving_signal_publisher: rospy.Publisher = None
    image_preprocessor: SensorCameraPreprocessor = SensorCameraPreprocessor()
    route_planner: TrajectoryPlanner = None

    def __post_init__(self):
        """generate trajectory planner for the vehicle"""
        self.route_planner = TrajectoryPlanner(self.vehicle, self.image_preprocessor)

    def run_node(self):
        """Launch the ROS node to receive globally planned routes
        and convert them into locally planned routes using sensors"""

        self._init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)
        while not rospy.is_shutdown():
            local_route, velocity = self.route_planner.calculate_trajectory()
            self.route_planner.driving_control.update_route(local_route)
            self.route_planner.driving_control.update_target_velocity(velocity)
            driving_signal = self.route_planner.driving_control.next_signal()
            msg = RosDrivingMessagesAdapter.signal_to_message(driving_signal)
            self.driving_signal_publisher.publish(msg)
            rate.sleep()

    def _init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""

        rospy.init_node(f'local_planner_{self.vehicle.name}', anonymous=True)
        self.driving_signal_publisher = self._init_driving_signal_publisher()
        self._init_vehicle_position_subscriber()
        self._init_vehicle_orientation_subscriber()
        self._init_global_route_subscriber()
        self._init_front_camera_subscribers()

    def _init_global_route_subscriber(self):
        in_topic = f"/drive/{self.vehicle.name}/global_route"
        msg_to_route = RosDrivingMessagesAdapter.json_message_to_waypoints
        process_route = self.route_planner.update_global_route
        callback = lambda msg: process_route(msg_to_route(msg))
        rospy.Subscriber(in_topic, StringMsg, callback)

    def _init_vehicle_orientation_subscriber(self):
        in_topic = f"/carla/{self.vehicle.name}/imu/imu1"
        msg_to_orientation = RosDrivingMessagesAdapter.message_to_orientation
        callback = lambda msg: self.route_planner.driving_control.update_vehicle_orientation(
            msg_to_orientation(msg))
        rospy.Subscriber(in_topic, ImuMsg, callback)

    def _init_vehicle_position_subscriber(self):
        in_topic = f"/carla/{self.vehicle.name}/odometry"
        msg_to_position = RosDrivingMessagesAdapter.message_to_vehicle_position
        callback = lambda msg: self.route_planner.driving_control.update_vehicle_position(
            msg_to_position(msg))
        rospy.Subscriber(in_topic, OdometryMsg, callback)

    def _init_front_camera_subscribers(self):
        camera_semantic_seg = "semantic_segmentation/front/image_segmentation"
        in_semantic_topic = f"/carla/{self.vehicle.name}/camera/{camera_semantic_seg}"
        rospy.Subscriber(in_semantic_topic, ImageMsg,
                         self.image_preprocessor.process_semantic_image)

        camera_depth = 'depth/front/image_depth'
        in_depth_topic = f"/carla/{self.vehicle.name}/camera/{camera_depth}"
        rospy.Subscriber(in_depth_topic, ImageMsg, self.image_preprocessor.process_depth_image)

        camera_rgb = 'rgb/front/image_color'
        in_rgb_topic = f"/carla/{self.vehicle.name}/camera/{camera_rgb}"
        rospy.Subscriber(in_rgb_topic, ImageMsg, self.image_preprocessor.process_rgb_image)

    def _init_driving_signal_publisher(self):
        out_topic = f"/carla/{self.vehicle.name}/ackermann_cmd"
        return rospy.Publisher(out_topic, AckermannDrive, queue_size=100)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    vehicle = Vehicle(vehicle_name)
    publish_rate_hz = 10
    node = LocalPlannerNode(vehicle, publish_rate_hz)
    node.run_node()


if __name__ == '__main__':
    main()
