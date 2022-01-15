"""A route planner based on map and sensor data"""

from math import dist
from dataclasses import dataclass, field
from typing import List, Tuple

from local_planner.vehicle_control import DrivingController
from local_planner.state_machine import SpeedStateMachine
from local_planner.core.vehicle import Vehicle


@dataclass
class TrajectoryPlanner:  # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""
    vehicle: Vehicle
    driving_control: DrivingController = None
    global_route: List[Tuple[float, float]] = field(default_factory=list)
    cached_local_route: List[Tuple[float, float]] = field(default_factory=list)

    def update_global_route(self, waypoints: List[Tuple[float, float]]):
        """Update the global route to follow"""
        print(waypoints)
        self.global_route = waypoints
        self.cached_local_route = [] if self.global_route is None else self.global_route

    def calculate_trajectory(self) -> List[Tuple[float, float]]:
        """combines trajectory and respective velocity to one data struct"""
        vehicle_not_ready = self.vehicle.pos is None or len(self.cached_local_route) == 0
        # print("vehicle_not_ready: ", vehicle_not_ready, " pos: ", self.vehicle.pos, " route: ", self.cached_local_route)
        return [] if vehicle_not_ready else self._compute_local_route()

    def _compute_local_route(self) -> List[Tuple[float, float]]:
        point_counts_as_done = 5
        enumerator = 0

        # TODO: add a more sophisticated approach to filter the waypoints ahead
        for point in self.cached_local_route:
            if dist(point, self.vehicle.pos) > point_counts_as_done:
                break
            enumerator += 1

        self.cached_local_route = self.cached_local_route[enumerator:]
        bound = min(50, len(self.cached_local_route))
        short_term_route = self.cached_local_route[:bound]

        # TODO: include maneuvers here ...
        return short_term_route
