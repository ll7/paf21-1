"""Adapter for converting between ROS messages and internal data types"""

import json
from typing import Tuple, List

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
from sensor_msgs.msg import NavSatFix as GpsMsg

from vehicle_controller.driving_control import DrivingSignal


class RosDrivingMessagesAdapter:
    """Convert between ROS messages and driving data"""

    @staticmethod
    def message_to_waypoints(msg: StringMsg) -> List[Tuple[float, float]]:
        """Convert a ROS message into waypoints"""
        json_list = json.loads(msg.data)
        waypoints = [(wp['x'], wp['y']) for wp in json_list]
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
