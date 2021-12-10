#!/usr/bin/env python
"""Main script defining and running the ROS node"""

from dataclasses import dataclass

import rospy
from ackermann_msgs.msg import AckermannDrive
# from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32 as FloatMsg
from nav_msgs.msg import Path as WaypointsMsg
from nav_msgs.msg import Odometry as OdometryMsg

from vehicle_controller.driving_control import DrivingController
from vehicle_controller.ros_msg_adapter import RosDrivingMessagesAdapter


@dataclass
class VehicleControllerNode:
    """A class representing a ROS node that's capable of controlling a vehicle
    and make it drive according to a given route the node is constantly receiving."""

    vehicle_name: str
    publish_rate_in_hz: float
    driving_controller = DrivingController()
    driving_signal_publisher: rospy.Publisher = None

    def run_node(self):
        """Launch the ROS node to receive planned routes + GPS
        and convert them into AckermannDrive signals"""
        self._init_publishers_and_subscribers()
        self._send_driving_signals_until_node_shutdown()
        self.driving_controller.target_velocity_mps = 10.0
        print(self.driving_controller)

    def _send_driving_signals_until_node_shutdown(self):
        rate = rospy.Rate(self.publish_rate_in_hz)
        while not rospy.is_shutdown():
            signal = self.driving_controller.next_signal()
            rospy.loginfo(signal)
            msg = RosDrivingMessagesAdapter.signal_to_message(signal)
            self.driving_signal_publisher.publish(msg)
            rate.sleep()

    def _init_publishers_and_subscribers(self):
        rospy.init_node(f'test_simple_driving_{self.vehicle_name}', anonymous=True)
        self.driving_signal_publisher = self._init_driving_signal_publisher()
        self._init_route_subscriber()
        self._init_target_velocity_subscriber()
        self._init_gps_subscriber()

    def _init_route_subscriber(self):
        # in_topic = f"/drive/{self.vehicle_name}/local_route"
        # msg_to_route = RosDrivingMessagesAdapter.json_message_to_waypoints
        # process_route = self.signal_converter.update_route
        # callback = lambda msg: process_route(msg_to_route(msg))
        # rospy.Subscriber(in_topic, StringMsg, callback)
        in_topic = f"/carla/{self.vehicle_name}/waypoints"
        msg_to_route = RosDrivingMessagesAdapter.nav_message_to_waypoints
        process_route = self.driving_controller.update_route
        callback = lambda msg: process_route(msg_to_route(msg))
        rospy.Subscriber(in_topic, WaypointsMsg, callback)

    def _init_target_velocity_subscriber(self):
        in_topic = f"/drive/{self.vehicle_name}/target_velocity"
        msg_to_velocity = RosDrivingMessagesAdapter.message_to_target_velocity
        process_velocity = self.driving_controller.update_target_velocity
        callback = lambda msg: process_velocity(msg_to_velocity(msg))
        rospy.Subscriber(in_topic, FloatMsg, callback)

    def _init_gps_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/odometry"
        callback = self._callback_odometry
        rospy.Subscriber(in_topic, OdometryMsg, callback)

    def _callback_odometry(self, msg):
        pos, orient = RosDrivingMessagesAdapter.message_to_vehicle_position(msg)
        self.driving_controller.update_vehicle_position(pos, orient)

    def _init_driving_signal_publisher(self):
        out_topic = f"/carla/{self.vehicle_name}/ackermann_cmd"
        return rospy.Publisher(out_topic, AckermannDrive, queue_size=100)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    publish_rate_hz = 10

    node = VehicleControllerNode(
        vehicle_name,
        publish_rate_hz)

    node.run_node()


if __name__ == '__main__':
    main()
