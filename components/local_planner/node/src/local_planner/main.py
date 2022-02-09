#!/usr/bin/env python
"""Main script defining the ROS node"""
from dataclasses import dataclass
from threading import Thread

import rospy
from sensor_msgs.msg import Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg

from local_planner.navigation import InfiniteDrivingService
from local_planner.route_planner import TrajectoryPlanner
from local_planner.ros_msg_adapter import RosMessagesAdapter
from local_planner.core import Vehicle
from local_planner.vehicle_control import DrivingController
from local_planner.state_machine import SpeedStateMachine


@dataclass
class LocalPlannerNode:
    """A class representing a ROS node that's processing route and
    local sensor data to handle the interaction with other vehicles, etc."""
    vehicle: Vehicle
    publish_rate_in_hz: int
    driving_signal_publisher: rospy.Publisher = None
    nav_service: InfiniteDrivingService = None
    route_planner: TrajectoryPlanner = None
    driving_control: DrivingController = None
    speed_state_machine: SpeedStateMachine = None

    def __post_init__(self):
        if self.route_planner is None:
            self.route_planner = TrajectoryPlanner(self.vehicle)
        if self.driving_control is None:
            self.driving_control = DrivingController(self.vehicle)
        if self.speed_state_machine is None:
            self.speed_state_machine = SpeedStateMachine(self.vehicle)
        if self.nav_service is None:
            route_callback = self.route_planner.update_global_route
            self.nav_service = InfiniteDrivingService(self.vehicle, route_callback)

    def run_node(self):
        """Launch the ROS node to plan actionable trajectories from
        a globally planned route and various sensors"""
        self._init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        nav_thread = Thread(target=self.nav_service.run_infinite_driving)
        nav_thread.start()
        # i: int = 0

        while not rospy.is_shutdown():
            local_route = self.route_planner.calculate_trajectory()
            # i = i + 1
            # print("step")
            # print(i)
            self.speed_state_machine.update_state(self.route_planner.latest_speed_observation)
            velocity = self.speed_state_machine.get_target_speed()
            self.driving_control.update_route(local_route)
            self.driving_control.update_target_velocity(velocity)
            driving_signal = self.driving_control.next_signal()
            # print("driving signal")
            # print(driving_signal)
            msg = RosMessagesAdapter.signal_to_message(driving_signal)
            self.driving_signal_publisher.publish(msg)
            rate.sleep()

    def _init_ros(self):
        rospy.init_node(f'local_planner_{self.vehicle.name}', anonymous=True)
        self.driving_signal_publisher = self._init_driving_signal_publisher()
        self._init_vehicle_position_subscriber()
        self._init_vehicle_orientation_subscriber()
        self._init_tld_info_subscriber()
        self._init_object_info_subscriber()

    def _init_tld_info_subscriber(self):
        in_topic = f"/drive/{self.vehicle.name}/tld_info"
        msg_to_tld_info = RosMessagesAdapter.json_message_to_tld_info
        callback = lambda msg: self.route_planner.update_tld_info(msg_to_tld_info(msg))
        rospy.Subscriber(in_topic, StringMsg, callback)

    def _init_object_info_subscriber(self):
        in_topic = f"/drive/{self.vehicle.name}/object_info"
        msg_to_object_info = RosMessagesAdapter.json_message_to_object_info
        callback = lambda msg: self.route_planner.update_objects(msg_to_object_info(msg))
        rospy.Subscriber(in_topic, StringMsg, callback)

    def _init_vehicle_orientation_subscriber(self):
        in_topic = f"/carla/{self.vehicle.name}/imu/imu1"
        msg_to_orientation = RosMessagesAdapter.message_to_orientation
        callback = lambda msg: self.vehicle.update_vehicle_orientation(
            msg_to_orientation(msg))
        rospy.Subscriber(in_topic, ImuMsg, callback)

    def _init_vehicle_position_subscriber(self):
        in_topic = f"/carla/{self.vehicle.name}/odometry"
        msg_to_position = RosMessagesAdapter.message_to_vehicle_position
        msg_to_velocity_and_time = RosMessagesAdapter.message_to_vehicle_velocity_and_timestamp
        callback = lambda msg: self.vehicle.update_vehicle_state(
            msg_to_position(msg), msg_to_velocity_and_time(msg))
        rospy.Subscriber(in_topic, OdometryMsg, callback)

    def _init_driving_signal_publisher(self):
        out_topic = f"/carla/{self.vehicle.name}/ackermann_cmd"
        return rospy.Publisher(out_topic, AckermannDrive, queue_size=10)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    vehicle = Vehicle(vehicle_name)
    publish_rate_hz = 100
    node = LocalPlannerNode(vehicle, publish_rate_hz)
    node.run_node()


if __name__ == '__main__':
    main()
