"""Module for transitioning a fine-grained, idealistic route
into Ackermann driving signals"""

from dataclasses import dataclass
from ackermann_msgs.msg import AckermannDrive

@dataclass
class SimpleDrivingSignalConverter:
    """A class converting a route into AckermannDrive signals"""
    straight_forward: AckermannDrive = AckermannDrive(
        steering_angle=0.0,
        steering_angle_velocity=0.0,
        speed=10,
        acceleration=0.0,
        jerk=0.0)
    route: any = None

    def update_route(self, route):
        """Update the route to be followed"""
        self.route = route

    def next_signal(self):
        """Get the next AckermannDrive signal for driving the given route"""
        return self.straight_forward
