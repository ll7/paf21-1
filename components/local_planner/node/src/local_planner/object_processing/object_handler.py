"""Represents a handler for the detected objects."""

from math import pi, sqrt
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

import numpy as np
from local_planner.object_processing.object_meta import ObjectMeta
from local_planner.core.geometry import norm_angle, rotate_vector
from local_planner.state_machine import SpeedObservation, ManeuverObservation


@dataclass
class ObjectHandler:
    """Represents a handler for the detected objects."""
    objects: Dict[int, ObjectMeta] = field(default_factory=dict)
    delta_time: float = 0.1
    vehicle_pos: Tuple[float, float] = None
    vehicle_rad: float = 0.0
    num_predict: int = int(3.0 / delta_time)

    def get_speed_observation(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Retrieve the speed observation."""
        return self._detect_vehicle_in_lane(local_route)

    def update_objects(self, object_list: List[Dict],  vehicle_pos: Tuple[float, float],
                       vehicle_rad: float):
        """Update the object list, the vehicle position and orientation"""
        self.vehicle_pos = vehicle_pos
        self.vehicle_rad = vehicle_rad

        keys = []
        for obj in object_list:
            obj_id = obj['identifier']
            keys.append(obj_id)

            new_abs_pos = self._convert_relative_to_world(obj['rel_position'])
            if obj_id in self.objects:
                self.objects[obj_id].update_object_meta(new_abs_pos, self.delta_time)
            else:
                self.objects[obj_id] = ObjectMeta(identifier=obj_id,
                                                  obj_class=obj['obj_class'],
                                                  trajectory=[new_abs_pos])
        self.objects = {k: self.objects[k] for k in keys}

    def _detect_vehicle_in_lane(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Detect a vehicle in the same direction."""
        # cache the objects to avoid concurrency bugs
        objects = self.objects.copy()
        print(f'Objects: {objects}')
        spd_obs = SpeedObservation()
        for obj_id, obj in objects.items():
            obj_positions = [obj.trajectory[-1]]
            if len(obj.trajectory) > 4:
                obj_positions += objects[obj_id].kalman_filter.predict_points(self.num_predict)

            for point in local_route:
                distance = ObjectHandler._closest_point(point, obj_positions, threshold=2)
                if distance is None:
                    continue
                distance = ObjectHandler._cumulated_dist(self.vehicle_pos, point)
                if distance < spd_obs.dist_next_obstacle_m:
                    spd_obs.is_trajectory_free = False
                    spd_obs.dist_next_obstacle_m = distance
                    spd_obs.obj_speed_ms = obj.velocity
        return spd_obs

    @staticmethod
    def _closest_point(point, points, threshold):
        threshold = threshold ** 2
        # calculate square distance
        distances = np.sum((np.asarray(points) - np.array(point))**2, axis=1)
        for distance in distances:
            if distance <= threshold:
                return sqrt(distance)
        return None

    @staticmethod
    def _cumulated_dist(veh_pos: Tuple[float, float], route_pos: Tuple[float, float]):
        return abs(veh_pos[0] - route_pos[0]) + abs(veh_pos[1] - route_pos[1])

    def _convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        theta = self.vehicle_rad - pi / 2
        coordinate = rotate_vector(coordinate, theta)
        return coordinate[0] + self.vehicle_pos[0], coordinate[1] + self.vehicle_pos[1]
