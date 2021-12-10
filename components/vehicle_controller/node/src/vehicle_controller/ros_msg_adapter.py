"""Adapter for converting between ROS messages and internal data types"""

import json
from typing import Tuple, List
from math import pi, atan2

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
from nav_msgs.msg import Path as WaypointsMsg
from nav_msgs.msg import Odometry as OdometryMsg
from transformations.transformations import euler_from_quaternion
import rospy

from vehicle_controller.driving_control import DrivingSignal


class RosDrivingMessagesAdapter:
    """Convert between ROS messages and driving data"""

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
    def message_to_target_velocity(msg: FloatMsg) -> float:
        """Convert a ROS message into the target velocity"""
        return msg.data

    @staticmethod
    def message_to_vehicle_position(msg: OdometryMsg) -> Tuple[float, float]:
        """Convert a ROS message into the vehicle position"""
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        yaw_1 = atan2(2.0 * (quat.y * quat.z + quat.w * quat.x),
            quat.w * quat.w - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z)
        quat_tuple = (quat.x, quat.y, quat.z, quat.w)
        _, _, yaw = euler_from_quaternion(quat_tuple)
        norm_angle = RosDrivingMessagesAdapter._normalize_angle(yaw)
        rospy.loginfo(f'yaw {yaw}, yaw_alternative { yaw_1 }, norm {norm_angle}')
        return (pos.x, pos.y), norm_angle

    @staticmethod
    def _normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    @staticmethod
    def signal_to_message(signal: DrivingSignal) -> AckermannDrive:
        """Convert a driving signal into a ROS message"""
        return AckermannDrive(
            steering_angle=signal.steering_angle_rad,
            steering_angle_velocity=0.0,
            speed=signal.target_velocity_mps,
            acceleration=0.0,
            jerk=0.0)
