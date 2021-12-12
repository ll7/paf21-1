#!/usr/bin/env python
"""Main script defining the ROS node"""

import json
from dataclasses import dataclass

import rospy
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import NavSatFix as GpsMsg
from sensor_msgs.msg import Imu as ImuMsg
import math
from global_planner.global_route_planner import GlobalRoutePlanner
from nav_msgs.msg import Odometry as OdometryMsg
from typing import Tuple, List
from global_planner.xodr_converter import XODRConverter
from pathlib import Path

@dataclass
class GlobalPlannerNode:
    """A class representing a ROS node that's planning a global route."""

    vehicle_name: str
    publish_rate_in_hz: int
    global_route_publisher: rospy.Publisher = None
    # TODO Change the num_nodes
    global_route_planner: GlobalRoutePlanner = GlobalRoutePlanner(10)
    xodr = XODRConverter()

    def init_map(self, path):
        self.xodr.read_xodr(path)
        self.xodr.create_links()

        global_route_planner = GlobalRoutePlanner(self.xodr.num_nodes)
        global_route_planner.set_matrix(self.xodr.matrix)
        global_route_planner.set_mapping(self.xodr.mapping)

    def run_node(self):
        """Launch the ROS node to receive the map, the start and
         end position and convert them into a global planned route."""
        self.init_ros()
        self.init_map(Path("./../../../xodr/Town01.xodr"))
        rate = rospy.Rate(self.publish_rate_in_hz)

        index_calc = 0
        while not rospy.is_shutdown():
            if self.global_route_planner.update:
                global_route = self.global_route_planner.compute_route()
                self.global_route_publisher.publish(global_route)
                self.global_route_planner.update = False
            # ToDo for Testing only
            """ Start of Testing without hmi """
            rospy.loginfo(f'index:  {index_calc}')
            index_calc += 1
            if index_calc == 20:
                global_route = self.global_route_planner.compute_route()
                self.global_route_publisher.publish(global_route)
            """ End of Testing  """
            rate.sleep()


    def init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        rospy.init_node(f'global_planner_{self.vehicle_name}', anonymous=True)
        self.global_route_publisher = self.init_global_route_publisher()
        self.init_hmi_route_subscriber()
        self._init_gps_subscriber()
        self._init_vehicle_orientation_subscriber()

    def init_hmi_route_subscriber(self):
        """Initialize the ROS subscriber receiving map and end"""
        in_topic = f"/drive/{self.vehicle_name}/hmi"
        rospy.Subscriber(in_topic, StringMsg, self.global_route_planner.update_map_end_pos)

    def _init_vehicle_orientation_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/imu/imu1"
        callback = lambda msg: GlobalRoutePlanner.update_vehicle_orientation(GlobalRoutePlanner, self.message_to_orientation(msg))
        rospy.Subscriber(in_topic, ImuMsg, callback)

    @staticmethod
    def message_to_vehicle_position(msg: OdometryMsg) -> Tuple[float, float]:
        """Convert a ROS message into the vehicle position"""
        pos = msg.pose.pose.position
        return (pos.x, pos.y)

    @staticmethod
    def message_to_orientation(msg: ImuMsg) -> float:
        """Convert a ROS message into the vehicle orientation"""
        quaternion = msg.orientation
        q_x = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        q_y = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        orientation = math.atan2(q_y, q_x)
        return orientation


    def _init_gps_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/odometry"
        # msg_to_position = self.message_to_vehicle_position
        # process_position = GlobalRoutePlanner.update_vehicle_position
        callback = lambda msg: GlobalRoutePlanner.update_vehicle_position(GlobalRoutePlanner, self.message_to_vehicle_position(msg))
        rospy.Subscriber(in_topic, OdometryMsg, callback)

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
    publish_rate_in_hz = 0.5

    node = GlobalPlannerNode(vehicle_name, publish_rate_in_hz)
    node.run_node()


if __name__ == '__main__':
    main()
