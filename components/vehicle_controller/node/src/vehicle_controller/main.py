#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass
from typing import Tuple, List

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
from sensor_msgs.msg import NavSatFix as GpsMsg

from vehicle_controller.driving import DrivingSignal, SimpleDrivingSignalConverter


@dataclass
class VehicleControllerNode:
    """A class representing a ROS node that's capable of controlling a vehicle
    and make it drive according to a given route the node is constantly receiving."""

    vehicle_name: str
    publish_rate_in_hz: float
    signal_converter = SimpleDrivingSignalConverter()
    driving_signal_publisher: rospy.Publisher = None

    def run_node(self):
        """Launch the ROS node to receive planned routes + GPS
        and convert them into AckermannDrive signals"""
        self._init_publishers_and_subscribers()
        self._send_driving_signals_until_node_shutdown()

    def _send_driving_signals_until_node_shutdown(self):
        rate = rospy.Rate(self.publish_rate_in_hz)
        while not rospy.is_shutdown():
            signal = self.signal_converter.next_signal()
            msg = RosDrivingAdapter.signal_to_message(signal)
            self.driving_signal_publisher.publish(msg)
            rate.sleep()

    def _init_publishers_and_subscribers(self):
        rospy.init_node(f'test_simple_driving_{self.vehicle_name}', anonymous=True)
        self.driving_signal_publisher = self._init_driving_signal_publisher()
        self._init_route_subscriber()
        self._init_target_velocity_subscriber()
        self._init_gps_subscriber()

    def _init_route_subscriber(self):
        in_topic = f"/drive/{self.vehicle_name}/local_route"
        msg_to_route = RosDrivingAdapter.message_to_waypoints
        process_route = self.signal_converter.update_route
        callback: lambda msg: process_route(msg_to_route(msg))
        rospy.Subscriber(in_topic, StringMsg, callback)

    def _init_target_velocity_subscriber(self):
        in_topic = f"/drive/{self.vehicle_name}/target_velocity"
        msg_to_velocity = RosDrivingAdapter.message_to_target_velocity
        process_velocity = self.signal_converter.update_target_velocity
        callback: lambda msg: process_velocity(msg_to_velocity(msg))
        rospy.Subscriber(in_topic, FloatMsg, callback)

    def _init_gps_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/gnss/gnss1/fix"
        msg_to_position = RosDrivingAdapter.message_to_vehicle_position
        process_position = self.signal_converter.update_vehicle_position
        callback: lambda msg: process_position(msg_to_position(msg))
        rospy.Subscriber(in_topic, GpsMsg, callback)

    def _init_driving_signal_publisher(self):
        out_topic = f"/carla/{self.vehicle_name}/ackermann_cmd"
        return rospy.Publisher(out_topic, AckermannDrive, queue_size=100)


class RosDrivingAdapter:
    """Convert between ROS messages and driving data"""

    @staticmethod
    def message_to_waypoints(msg: StringMsg) -> List[Tuple[float, float]]:
        """Convert a ROS message into waypoints"""
        json_list = json.loads(msg.data)
        waypoints = [(wp.x, wp.y) for wp in json_list]
        return waypoints

    @staticmethod
    def message_to_target_velocity(msg: FloatMsg) -> float:
        """Convert a ROS message into the target velocity"""
        return msg.data

    @staticmethod
    def message_to_vehicle_position(msg: GpsMsg) -> Tuple[float, float]:
        """Convert a ROS message into the vehicle position"""
        return (msg.longitude, msg.latitude)

    @staticmethod
    def signal_to_message(signal: DrivingSignal) -> AckermannDrive:
        """Convert a driving signal into a ROS message"""
        return AckermannDrive(
            steering_angle=signal.steering_angle_rad,
            steering_angle_velocity=0.0,
            speed=signal.target_velocity_mps,
            acceleration=0.0,
            jerk=0.0)


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
