"""Represents a handler for the detected objects."""
import math
from math import pi, sqrt, dist
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

import numpy as np
from local_planner.core import Vehicle
from local_planner.core.geometry import rotate_vector
from local_planner.object_processing.object_meta import ObjectMeta
from local_planner.state_machine import SpeedObservation


@dataclass
class ObjectHandler:
    """Represents a handler for the detected objects."""
    vehicle: Vehicle
    objects: Dict[int, ObjectMeta] = field(default_factory=dict)
    delta_time: float = 0.1
    num_predict: int = int(3.0 / delta_time)
    max_velocity_change_rate: float = 2.0
    street_width: float = 4.0
    cache: Dict[int, int] = field(default_factory=dict)
    dist_safe: int = 10

    def get_speed_observation(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Retrieve the speed observation."""
        if not self.vehicle.is_ready:
            return SpeedObservation()
        return self._detect_vehicle_in_lane(local_route)

    def update_objects(self, object_list: List[Dict]):
        """Update the object list, the vehicle position and orientation"""

        # TODO: figure out if the vehicle data needs to be cached to avoid weird edge cases

        if not self.vehicle.is_ready:
            return SpeedObservation()

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

    def update_objects_with_caching(self, object_list: List[Dict]):
        """Update the object list, the vehicle position and orientation"""
        # TODO: figure out if the vehicle data needs to be cached to avoid weird edge cases

        if not self.vehicle.is_ready:
            return SpeedObservation()
        keys = []
        cache_limit = 20

        for obj in self.objects.items():
            obj_id = obj[1].identifier
            if obj_id in self.cache:
                self.cache[obj_id] += 1
                if self.cache[obj_id] >= cache_limit:
                    del(self.cache[obj_id])
            else:
                self.cache[obj_id] = 1
        cached_keys = list(self.cache.keys())
        for obj in object_list:
            obj_id = obj['identifier']
            keys.append(obj_id)
            self.cache[obj_id] = 0
            new_abs_pos = self._convert_relative_to_world(obj['rel_position'])
            if obj_id in self.objects:
                self.objects[obj_id].update_object_meta(new_abs_pos, self.delta_time)
            else:
                self.objects[obj_id] = ObjectMeta(identifier=obj_id,
                                                  obj_class=obj['obj_class'],
                                                  trajectory=[new_abs_pos])
        concat_keys = set(keys + cached_keys)
        self.objects = {k: self.objects[k] for k in concat_keys}
        for k in concat_keys:
            if k not in keys:
                if len(self.objects[k].trajectory) > 3:

                    new_abs_pos = self.predict_car_movement(self.objects[k])[0]
                    self.objects[k].update_object_meta(new_abs_pos, self.delta_time)

    def _detect_vehicle_in_lane(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Detect a vehicle in the same direction."""
        # cache the objects to avoid concurrency bugs
        route = local_route.copy()
        spd_obs = SpeedObservation()
        blocked_ids, obj = self.get_blocked_ids(route)
        if len(blocked_ids) > 0:
            #distance = self.calculate_min_distance(route, min(blocked_ids), obj.trajectory[-1])
            distance = dist(self.vehicle.pos, obj.trajectory[-1])
            spd_obs.is_trajectory_free = False
            spd_obs.dist_next_obstacle_m = distance
            spd_obs.obj_speed_ms = obj.velocity
            print(spd_obs)
        return spd_obs

    def calculate_min_distance(self, route, until_id, obj_pos):
        distance = 0
        route.insert(0, self.vehicle.pos)
        for point in range(0, len(route)):
            counter = point
            if until_id <= point:
                distance += math.dist(route[counter], obj_pos)
                break
            distance += math.dist(route[point], route[point + 1])
        return distance

    def plan_route_around_objects(self, local_route: List[Tuple[float, float]]):
        """calculates a route at the left side of the obstacle"""
        temp_route = local_route.copy()
        blocked_ids, closest_object = self.get_blocked_ids(temp_route)
        widths = []
        for i in range(0, len(temp_route)):
            if i < (len(temp_route) - 1):
                moving_vector = ObjectHandler.ortho_vector_from_points(temp_route[i],
                                                                       temp_route[i + 1])
            else:
                moving_vector = [0, 0]
            if closest_object is not None:
                width = self.sigmoid_smooth(closest_object.velocity,
                                            [temp_route[u] for u in blocked_ids],
                                            temp_route[i], temp_route[0])
            else:
                width = 0
            widths.append(width)

            temp_route[i] = (temp_route[i][0] + width * moving_vector[0],
                             temp_route[i][1] + width * moving_vector[1])
        new_check, _ = self.get_blocked_ids(temp_route)
        print('Second Block', new_check)
        print('Blocked:', blocked_ids)
        if closest_object is not None:
            print('Distance to object', dist(self.vehicle.pos, closest_object.trajectory[-1]))
            print(widths)
        return local_route if new_check else temp_route

    def get_blocked_ids(self, route):
        """gets the ids of route points that aren't accessible"""
        objects = self.objects.copy()
        blocked_ids = []
        min_id = 100
        min_obj = None
        for obj_id, obj in objects.items():
            obj_positions = [obj.trajectory[-1]]
            if len(obj.trajectory) > 3:
                # obj_positions += objects[obj_id].kalman_filter.predict_points(self.num_predict)
                obj_positions += ObjectHandler.predict_car_movement(obj)
                blocked = self.find_blocked_points(route, obj_positions, 1.8, obj)
                if not blocked:
                    continue
                if min(blocked) < min_id:
                    min_id = min(blocked)
                    min_obj = obj
                blocked_ids += blocked

        blocked_ids = [num for num in blocked_ids if num >= 0]
        blocked_ids = set(blocked_ids)
        return list(blocked_ids), min_obj

    @staticmethod
    def ortho_vector_from_points(point_a, point_b):
        """calculates orthogonal vector"""
        point_a = np.array(point_a)
        point_b = np.array(point_b)
        vector = point_b - point_a
        unit_vector = vector / np.linalg.norm(vector)
        orthogonal_vector = rotate_vector(unit_vector, pi / 2)

        return orthogonal_vector


    def find_blocked_points(self, route, points, threshold, obj):
        """finds blocked points and returns their ids"""
        threshold = threshold ** 2
        # calculate square distance
        closest_object = None
        ids = []
        id = 0
        for point in route:
            for object_coordinates in points:
                distance = np.sum((np.array(point) - np.array(object_coordinates)) ** 2)
                if distance <= threshold:

                    time_obj = (dist(obj.trajectory[-1], point)+threshold) / \
                               obj.velocity if obj.velocity != 0 else 999
                    time_self = (dist(self.vehicle.pos, point)-threshold) / \
                                self.vehicle.velocity_mps if self.vehicle.velocity_mps != 0 else 999
                    zone_clearence_time = time_self - time_obj
                    if zone_clearence_time < 2:
                        ids += range(id-1, id+1)
                        break
            id += 1
        return ids

    @staticmethod
    def _closest_point(point, points, threshold):
        threshold = threshold ** 2
        # calculate square distance
        distances = np.sum((np.asarray(points) - np.array(point)) ** 2, axis=1)
        for distance in distances:
            if distance <= threshold:
                return sqrt(distance)
        return None

    @staticmethod
    def _cumulated_dist(veh_pos: Tuple[float, float], route_pos: Tuple[float, float]):
        return abs(veh_pos[0] - route_pos[0]) + abs(veh_pos[1] - route_pos[1])

    def _convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        coordinate[1] = coordinate[1] + 0.5
        theta = self.vehicle.orientation_rad - pi / 2
        coordinate = rotate_vector(coordinate, theta)
        return coordinate[0] + self.vehicle.pos[0], coordinate[1] + self.vehicle.pos[1]

    @staticmethod
    def predict_car_movement(obj):
        """predict car movement using  simple interpolation of its movement vectors"""
        trajectory = obj.trajectory[-3:]
        time_step = 1 / 10
        distance_per_time = obj.velocity * time_step
        vector_1 = np.array([trajectory[-1][0] - trajectory[-2][0],
                             trajectory[-1][1] - trajectory[-2][1]])
        vector_1 = vector_1 / np.linalg.norm(vector_1)
        vector_2 = np.array([trajectory[-2][0] - trajectory[-3][0],
                             trajectory[-2][1] - trajectory[-3][1]])
        vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(vector_1, vector_2)
        angle = np.arccos(dot_product)
        point = np.array(obj.trajectory[-1])
        predicted_trajectory = []
        for i in range(0, 50):
            vector_1 = rotate_vector(vector_1, angle)
            prediction_vector = [vector_1[0] * distance_per_time, vector_1[1] * distance_per_time]
            point = point + np.array(prediction_vector)
            predicted_trajectory.append(point)
        return predicted_trajectory

    def sigmoid_smooth(self, object_speed, object_coordinates, point, first_coord):
        """tries to smooth out the overtaking maneuver so it can be driven at higher speeds"""
        w = self.street_width  # parameter to stop in the  middle of other lane
        mu = 1  # slope of overtaking
        relative_velocity = self.vehicle.velocity_mps - object_speed
        relative_distance_to_object = dist(point, object_coordinates[0])
        if dist(point, first_coord) > \
                min([dist(first_coord, obj) for obj in object_coordinates]):
            relative_distance_to_object = -relative_distance_to_object
        time_to_collision = 2
        self.dist_safe = max([relative_velocity * time_to_collision, 0])
        # self.dist_safe = 6
        dist_c = max(dist(object_coordinates[0], object_coordinates[-1]), 0)
        x_1 = (1 / mu) * (relative_distance_to_object + self.dist_safe)
        x_2 = (1 / mu) * (relative_distance_to_object - self.dist_safe - dist_c)
        deviation = (w / (1 + math.exp(-x_1))) + ((-w) / (1 + math.exp(-x_2)))
        return deviation
