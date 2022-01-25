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
    old_timestamp: float = 0.1
    tld_info: TrafficLightInfo = TrafficLightInfo()

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

    def get_speed_observation(self) -> SpeedObservation:
        if self.cached_local_route:
            speed_obs = self.detect_vehicle_in_lane()
            speed_obs.tl_phase = self.tld_info.phase
            if speed_obs.tl_phase == TrafficLightPhase.RED:
                speed_obs.dist_next_obstacle_m = min(speed_obs.dist_next_obstacle_m,
                                                     self.tld_info.distance)
            print(f'Speed_obs {speed_obs}')
            return speed_obs
        return SpeedObservation()

    def update_tld_info(self, tld_info: TrafficLightInfo):
        self.tld_info = tld_info

    def update_objects(self, object_list: List[Dict]):
        """Refresh the objects that were detected by the perception"""
        keys = []
        # time_difference = self.vehicle.time - self.old_timestamp
        time_difference = 0.1
        if time_difference > 0:
            for obj in object_list:
                new_pos = self.convert_relative_to_world(obj['rel_position'])
                keys.append(obj['identifier'])
                if obj['identifier'] in self.objects:
                    last_pos = self.objects[obj['identifier']].trajectory[-1]
                    self.objects[obj['identifier']].trajectory.append(new_pos)
                    self.objects[obj['identifier']].velocity = dist(new_pos, last_pos) \
                                                               / time_difference
                    #   rel_dist = dist(self.vehicle.pos, new_pos)
                    #   rel_dist = rel_dist - dist(self.vehicle.pos, last_pos)
                    #   rel_velocity = rel_dist / time_difference
                    # abs_velocity = self.vehicle.actual_velocity_mps + rel_velocity
                    print(f'global vel {self.objects[obj["identifier"]].velocity},'
                          f' {time_difference}, {obj["identifier"]}')
                else:
                    self.objects[obj['identifier']] = ObjectInfo(identifier=obj['identifier'],
                                                                 obj_class=obj['obj_class'],
                                                                 trajectory=[new_pos])
            self.objects = {k: self.objects[k] for k in keys}
            self.old_timestamp = self.vehicle.time

    def convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        theta = self.vehicle.orientation_rad - np.pi / 2
        t_vector = np.array(self.vehicle.pos)
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        coordinate = np.matmul(rotation_matrix, coordinate) + t_vector
        return coordinate[0], coordinate[1]

    def detect_vehicle_in_lane(self, object_list: List[Dict]) -> SpeedObservation:
        """Detect a vehicle in the same direction."""
        self.refresh_detected_objects(object_list)
        spd_obs = SpeedObservation()
        cached_local_route = np.concatenate([[self.vehicle.pos], self.cached_local_route])
        for _, obj in self.objects.items():
            last_obj_pos = obj.trajectory[-1]
            for point in cached_local_route:
                # ToDo: Do this for every predicted position of the object
                distance = dist(last_obj_pos, point)
                if distance > 2.0:
                    continue

                distance = dist(self.vehicle.pos, last_obj_pos)
                if distance < spd_obs.dist_next_obstacle_m:
                    spd_obs.is_trajectory_free = False
                    spd_obs.dist_next_obstacle_m = distance
                    spd_obs.object_speed_ms = obj.velocity

        print(spd_obs)
        return spd_obs
