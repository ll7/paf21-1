"""A route planner based on map and sensor data"""
import math
from math import dist
from dataclasses import dataclass, field
from typing import List, Tuple

from local_planner.vehicle_control import DrivingController
from local_planner.core.vehicle import Vehicle

from local_planner.core.geometry import points_to_vector, angle_between


@dataclass
class TrajectoryPlanner:  # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""
    vehicle: Vehicle
    driving_control: DrivingController = None
    global_route: List[Tuple[float, float]] = field(default_factory=list)
    next_wp: int = -1
    prev_wp: int = -1

    def update_global_route(self, waypoints: List[Tuple[float, float]]):
        """Update the global route to follow"""
        print(waypoints)

        self.global_route = waypoints
        if len(self.global_route) < 2:
            self.next_wp = -1
            self.prev_wp = -1
        else:
            self.next_wp = 1
            self.prev_wp = 0

    def calculate_trajectory(self) -> List[Tuple[float, float]]:
        """combines trajectory and respective velocity to one data struct"""
        vehicle_not_ready = not self.vehicle.is_ready
        # print("vehicle_not_ready: ", vehicle_not_ready,
        #       " pos: ", self.vehicle.pos, " route: ", self.cached_local_route)
        # return [] if vehicle_not_ready or len(self.cached_local_route) == 0 else self._compute_local_route()
        return [] if vehicle_not_ready else self._compute_local_route()

    def _compute_local_route(self) -> List[Tuple[float, float]]:

        # routes with length lower 2 are invalid
        if len(self.global_route) < 2:
            return self.global_route

        # not at finish?
        if self.next_wp < len(self.global_route):
            next_to_prev = points_to_vector(self.global_route[self.next_wp], self.global_route[self.prev_wp])
            next_to_pos = points_to_vector(self.global_route[self.next_wp], self.global_route[self.vehicle.pos])
            angle = angle_between(next_to_prev, next_to_pos)
        else:
            bound = min(50, len(self.global_route) - self.prev_wp)
            return self.global_route[self.prev_wp:bound]

        # next Route section and
        while angle > 0.5*math.pi:
            self.next_wp += 1
            self.prev_wp += 1

            #finish?
            if self.next_wp == len(self.global_route):
                break

            next_to_prev = points_to_vector(self.global_route[self.next_wp], self.global_route[self.prev_wp])
            next_to_pos = points_to_vector(self.global_route[self.next_wp], self.global_route[self.vehicle.pos])
            angle = angle_between(next_to_prev, next_to_pos)

        bound = min(50, len(self.global_route) - self.prev_wp)
        return self.global_route[self.prev_wp:bound]








