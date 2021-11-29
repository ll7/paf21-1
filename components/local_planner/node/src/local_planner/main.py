#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass

import rospy
from std_msgs.msg import String as StringMsg
# from local_planner.driving import SimpleDrivingSignalConverter


@dataclass
class VehicleControllerNode:
    """A class representing a ROS node that's capable of controlling a vehicle
    and make it drive according to a given route the node is constantly receiving."""

    vehicle_name: str
    publish_rate_in_hz: int
    driving_signal_publisher: rospy.Publisher = None

    def run_node(self):
        """Launch the ROS node to receive planned routes
        and convert them into AckermannDrive signals"""
        self.init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            signal = None
            self.driving_signal_publisher.publish(signal)
            rate.sleep()

    def init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        rospy.init_node(f'test_simple_driving_{self.vehicle_name}', anonymous=True)
        self.driving_signal_publisher = self.init_driving_signal_publisher()
        self.init_route_subscriber()

    def init_route_subscriber(self):
        """Initialize the ROS subscriber receiving route information"""
        callback = lambda msg: msg
        in_topic = f"/drive/{self.vehicle_name}/planned_route"
        rospy.Subscriber(in_topic, StringMsg, callback)

    def init_driving_signal_publisher(self):
        """Initialize the ROS publisher for submitting AckermannDrive signals"""
        out_topic = f"/carla/{self.vehicle_name}/ackermann_cmd"
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
    input_topics = []

    node = VehicleControllerNode(vehicle_name, publish_rate_hz, input_topics)
    node.run_node()


if __name__ == '__main__':
    main()
