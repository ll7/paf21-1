"""Represents a handler for the detected objects."""

from math import dist, pi
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

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
        # cache the route to avoid concurrency bugs
        objects = self.objects.copy()

        spd_obs = SpeedObservation()
        for _, obj in objects.items():
            last_obj_pos = obj.trajectory[-1]
            distances = []
            for point in local_route:
                # ToDo: Do this for every predicted position of the object
                distance = dist(last_obj_pos, point)
                distances.append(distance)
                if distance > 2.0:
                    continue

                # only apply the most relevant object
                distance = dist(self.vehicle_pos, last_obj_pos)
                if distance < spd_obs.dist_next_obstacle_m:
                    spd_obs.is_trajectory_free = False
                    spd_obs.dist_next_obstacle_m = distance
                    spd_obs.obj_speed_ms = obj.velocity

        return spd_obs

    def _predict_obj_movement(self, obj_id: int, predict_sec: float) -> List[Tuple[float, float]]:
        """Predict the object movement for a given time horizon."""
        iterations = int(predict_sec / self.delta_time)
        predicted = [self.objects[obj_id].kalman_filter.last_prediction]
        for i in range(iterations):
            predicted.append(self.objects[obj_id].kalman_filter.predict(predicted[i]))
        return predicted

    def _convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        theta = norm_angle(self.vehicle_rad - pi / 2)
        coordinate = rotate_vector(coordinate, theta)
        return coordinate[0] + self.vehicle_pos[0], coordinate[1] + self.vehicle_pos[1]
