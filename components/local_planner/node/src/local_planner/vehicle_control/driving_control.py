"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from typing import Protocol, Tuple, List
from dataclasses import dataclass, field

from local_planner.core import Vehicle
from local_planner.vehicle_control import NaiveSteeringController


@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float


class SteeringController(Protocol):
    # pylint: disable=too-few-public-methods
    """Representing a generalization of a steering controller."""

    def compute_steering_angle(self, route: List[Tuple[float, float]]) -> float:
        """Compute the steering angle given the route waypoints."""
        ...


@dataclass
class DrivingController:  # pylint: disable=too-many-instance-attributes
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""
    vehicle: Vehicle
    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    initial_vehicle_pos_set = False
    steer_control: SteeringController = None

    def __post_init__(self):
        if not self.steer_control:
            self.steer_control = NaiveSteeringController(self.vehicle, 5.0)

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed and cache first waypoint"""
        if waypoints and not self.initial_vehicle_pos_set:
            self.initial_vehicle_pos_set = True
        self.route_waypoints = waypoints

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the target velocity according to the curvature ahead"""
        self.target_velocity_mps = target_velocity_mps

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""
        # generate logs for each driving signal tick
        steering_angle = self.steer_control.compute_steering_angle(self.route_waypoints)
        target_speed = self.target_velocity_mps     # if self.route_waypoints else 0.0
        signal = DrivingSignal(steering_angle, target_speed)
        if self.vehicle.is_ready:
            print((f"{self.vehicle.time},{self.vehicle.pos[0]},{self.vehicle.pos[1]},"
                   f"{self.vehicle.orientation_rad},{self.vehicle.velocity_mps},"
                   f"{target_speed},{steering_angle}"))
        return signal
