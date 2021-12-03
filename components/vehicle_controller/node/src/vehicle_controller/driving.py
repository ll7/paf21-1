"""Module for transitioning a fine-grained, idealistic route
into Ackermann driving signals"""

import json
import math
from datetime import datetime
from typing import Tuple
from dataclasses import dataclass
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
from sensor_msgs.msg import NavSatFix as GpsMsg

@dataclass
class SimpleDrivingSignalConverter:
    """A class converting a route into AckermannDrive signals"""
    route_waypoints: any = None
    target_velocity_mps: float = 0.0
    actual_velocity_mps: float = 0.0
    vehicle_pos: Tuple[float, float] = None
    vehicle_pos_timestamp: datetime = None

    def update_route(self, msg: StringMsg):
        """Update the route to be followed"""
        self.route_waypoints = json.loads(msg.data)

    def update_target_velocity(self, msg: FloatMsg):
        """Update the route to be followed"""
        self.target_velocity_mps = msg.data

    def update_vehicle_position(self, msg: GpsMsg):
        """Update the vehicle's current position and estimate
        the actual velocity by computing the position diffs"""

        old_pos = self.vehicle_pos
        new_pos = (msg.longitude, msg.latitude)
        old_timestamp = self.vehicle_pos_timestamp
        new_timestamp = datetime.utcnow()

        if old_pos is not None:
            dist = math.dist(old_pos, new_pos)
            time = (new_timestamp - old_timestamp).total_seconds()
            self.actual_velocity_mps = dist / time

        self.vehicle_pos = new_pos
        self.vehicle_pos_timestamp = new_timestamp

    def next_signal(self):
        """Compute the next AckermannDrive signal to
        follow the suggested ideal route"""

        steering_angle = self._compute_steering_angle()
        return AckermannDrive(
            steering_angle=steering_angle,
            steering_angle_velocity=0.0,
            speed=self.target_velocity_mps,
            acceleration=0.0,
            jerk=0.0)

    def _compute_steering_angle(self): # pylint: disable=no-self-use
        # source: https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081 # pylint: disable=line-too-long
        # source: http://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html # pylint: disable=line-too-long
        return 0.0
