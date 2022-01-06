#!/usr/bin/env python
"""Main script defining the ROS node"""

import math
import json
from time import sleep

from dataclasses import dataclass
from typing import Tuple
from pathlib import Path
import rospy

from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg

from global_planner.global_route_planner import GlobalPlanner
from global_planner.xodr_converter import XODRConverter, XodrMap


@dataclass
class GlobalPlannerNode:
    """A class representing a ROS node that's planning a global route."""

    vehicle_name: str
    global_route_publisher: rospy.Publisher = None
    path = Path("/app/res/xodr/Town01.xodr")
    xodr_map: XodrMap = None
    global_planner: GlobalPlanner = None

    def __post_init__(self):
        if not self.xodr_map:
            self.xodr_map = XODRConverter.read_xodr(self.path)
        if not self.global_planner:
            self.global_planner = GlobalPlanner(self.xodr_map)

    def run_node(self):
        """Launch the ROS node to receive the map, the start and
        end position and convert them into a global planned route."""
        rospy.init_node(f'global_planner_{self.vehicle_name}', anonymous=True)

        self.init_ros()

        print('waiting for car position ...')
        while self.global_planner.start_pos is None:
            print('waiting for car position ...')
            sleep(1)

        end_pos = (144.99, -55.5)
        global_route = GlobalPlanner.generate_waypoints(
            self.global_planner.start_pos, end_pos, self.xodr_map)

        route_as_json = [{'x': pos[0], 'y': pos[1]} for pos in global_route]
        msg = StringMsg(data=json.dumps(route_as_json))
        self.global_route_publisher.publish(msg)

        rospy.spin()

    def init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        self.global_route_publisher = self.init_global_route_publisher()
        self._init_gps_subscriber()
        self._init_vehicle_orientation_subscriber()

    def _init_vehicle_orientation_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/imu/imu1"
        callback = lambda msg: self.global_planner.update_vehicle_orientation(
            self.message_to_orientation(msg))
        rospy.Subscriber(in_topic, ImuMsg, callback)

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
        callback = lambda msg: self.global_planner.update_vehicle_position(
            self.message_to_vehicle_position(msg))
        rospy.Subscriber(in_topic, OdometryMsg, callback)

    @staticmethod
    def message_to_vehicle_position(msg: OdometryMsg) -> Tuple[float, float]:
        """Convert a ROS message into the vehicle position"""
        pos = msg.pose.pose.position
        return pos.x, pos.y

    def init_global_route_publisher(self):
        """Initialize the ROS publisher for submitting local routes"""
        out_topic = f"/drive/{self.vehicle_name}/global_route"
        return rospy.Publisher(out_topic, StringMsg, queue_size=1)

    # def init_hmi_route_subscriber(self):
    #     """Initialize the ROS subscriber receiving map and end"""
    #     in_topic = f"/drive/{self.vehicle_name}/hmi"
    #     rospy.Subscriber(in_topic, StringMsg, self.global_route_planner.update_map_end_pos)
    # @classmethod
    # def parse_hmi_msg(cls, hmi_json: StringMsg):
    #     """Parse the hmi message from JSON given a ROS message"""
    #     return json.load(hmi_json.data)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""
    vehicle_name = "ego_vehicle"

    node = GlobalPlannerNode(vehicle_name)
    node.run_node()


if __name__ == '__main__':
    main()
