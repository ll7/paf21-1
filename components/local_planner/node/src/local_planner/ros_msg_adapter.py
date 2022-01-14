"""Adapter for converting between ROS messages and internal data types"""

import json
from typing import Tuple, List
from math import atan2

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
from nav_msgs.msg import Path as WaypointsMsg
from nav_msgs.msg import Odometry as OdometryMsg
from sensor_msgs.msg import Imu as ImuMsg

from local_planner.state_machine import TrafficLightInfo, TrafficLightPhase
from local_planner.vehicle_control import DrivingSignal


class RosDrivingMessagesAdapter:
    """Convert between ROS messages and driving data"""

    @staticmethod
    def json_message_to_tld_info(msg: StringMsg) -> TrafficLightInfo:
        obj = json.loads(msg.data)
        return TrafficLightInfo(TrafficLightPhase(int(obj['phase'])), float(obj['distance']))

    @staticmethod
    def json_message_to_waypoints(msg: StringMsg) -> List[Tuple[float, float]]:
        """Convert a ROS message into waypoints"""
        json_list = json.loads(msg.data)
        waypoints = [(wp['x'], wp['y']) for wp in json_list]
        return waypoints

    @staticmethod
    def nav_message_to_waypoints(msg: WaypointsMsg) -> List[Tuple[float, float]]:
        """Convert a ROS message into waypoints"""
        waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        return waypoints

    @staticmethod
    def vehicle_status_to_velocity(msg: ImuMsg):
        print(msg)
        print(f'{msg.speed} speed ')
        return msg.speed

    @staticmethod
    def message_to_target_velocity(msg: FloatMsg) -> float:
        """Convert a ROS message into the target velocity"""
        return msg.data

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
        orientation = atan2(q_y, q_x)
        return orientation

    @staticmethod
    def signal_to_message(signal: DrivingSignal) -> AckermannDrive:
        """Convert a driving signal into a ROS message"""
        return AckermannDrive(
            steering_angle=signal.steering_angle_rad,
            steering_angle_velocity=0.0,
            speed=signal.target_velocity_mps,
            acceleration=0.0,
            jerk=0.0)
