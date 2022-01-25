"""A route planner based on map and sensor data"""

from typing import List, Tuple, Dict
from dataclasses import dataclass, field
from math import dist
import numpy as np

from local_planner.vehicle_control import DrivingController
from local_planner.core.vehicle import Vehicle
from local_planner.state_machine import SpeedObservation


@dataclass
class ObjectInfo:
    """Representing information on a recently detected object"""
    identifier: int
    obj_class: str
    trajectory: List[Tuple[float, float]]
    velocity: float = 0.0


@dataclass
class TrajectoryPlanner:  # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""
    vehicle: Vehicle
    driving_control: DrivingController = None
    global_route: List[Tuple[float, float]] = field(default_factory=list)
    cached_local_route: List[Tuple[float, float]] = field(default_factory=list)
    objects: Dict[int, ObjectInfo] = field(default_factory=dict)

    def update_global_route(self, waypoints: List[Tuple[float, float]]):
        """Update the global route to follow"""
        print(waypoints)
        self.global_route = waypoints
        self.cached_local_route = [] if self.global_route is None else self.global_route

    def calculate_trajectory(self) -> List[Tuple[float, float]]:
        """Combines trajectory and respective velocity to one data struct"""
        vehicle_not_ready = not self.vehicle.is_ready
        # print("vehicle_not_ready: ", vehicle_not_ready,
        #       " pos: ", self.vehicle.pos, " route: ", self.cached_local_route)
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

    # TODO think of other location for following functions
    def refresh_detected_objects(self, object_list: List[Dict]):
        """Refresh the objects that were detected by the perception"""
        keys = []
        for obj in object_list:
            new_position = self.convert_relative_to_world(obj['rel_position'])
            keys.append(obj['identifier'])
            if obj['identifier'] in self.objects:
                last_position = self.objects[obj['identifier']].trajectory[-1]
                self.objects[obj['identifier']].trajectory.append(new_position)
                self.objects[obj['identifier']].velocity = dist(new_position, last_position) / 0.1
            else:
                self.objects[obj['identifier']] = ObjectInfo(identifier=obj['identifier'],
                                                             obj_class=obj['obj_class'],
                                                             trajectory=[new_position])
        self.objects = {k: self.objects[k] for k in keys}

    def convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        local_coord = coordinate
        translation = self.vehicle.pos
        theta = self.vehicle.orientation_rad
        t_vector = np.array([translation[0], translation[1]])
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        coordinate = np.matmul(rotation_matrix, coordinate) + t_vector
        print('coordinates:', coordinate, translation, local_coord)
        return coordinate[0], coordinate[1]

    def detect_vehicle_in_lane(self, object_list: List[Dict]) -> SpeedObservation:
        """Detect a vehicle in the same direction."""
        self.refresh_detected_objects(object_list)
        spd_obs = SpeedObservation()
        # print(f'Detected Vehicle {self.objects}')
        for _, obj in self.objects.items():
            distance = dist(self.vehicle.pos, obj.trajectory[-1])
            if obj.velocity >= 0 and distance < spd_obs.dist_next_obstacle_m:
                spd_obs.is_trajectory_free = False
                spd_obs.dist_next_obstacle_m = distance
                spd_obs.object_speed_ms = obj.velocity
                print(f'Detected Vehicle {obj.identifier}: Dist {distance} , Velo: {obj.velocity}')

        return spd_obs
