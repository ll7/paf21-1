"""Represents a handler for the detected objects."""
import math
from math import pi, dist
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

import numpy as np

from local_planner.core import Vehicle#, visualize_route_rviz
from local_planner.core.geometry import rotate_vector, orth_offset_left, add_vector, sub_vector
from local_planner.route_planning.xodr_converter import XodrMap
from local_planner.map_provider import load_xodr_map
from local_planner.object_processing.object_meta import ObjectMeta
from local_planner.state_machine import SpeedObservation


@dataclass
class ObjectHandler:
    """Represents a handler for the detected objects."""
    vehicle: Vehicle
    objects: Dict[int, ObjectMeta] = field(default_factory=dict)
    map: XodrMap = None
    delta_time: float = 0.1
    max_velocity_change_rate: float = 2.0
    street_width: float = 4
    dist_safe: int = 10
    side: int = 0

    def __post_init__(self):
        if not self.map:
            self.map = load_xodr_map()

    def get_speed_observation(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Retrieve the speed observation."""
        if not self.vehicle.is_ready:
            return SpeedObservation()
        return self._detect_vehicle_in_lane(local_route)

    def update_objects(self, object_list: List[Dict]):
        """Update the object list, the vehicle position and orientation"""
        if not self.vehicle.is_ready:
            return

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
        route = local_route.copy()
        spd_obs = SpeedObservation()
        blocked_ids, obj = self.get_blocked_ids(route)
        if len(blocked_ids) > 0:
            # distance = self.calculate_min_distance(route, min(blocked_ids), obj.trajectory[-1])
            distance = self._cumulated_dist(self.vehicle.pos, obj.trajectory[-1])
            spd_obs.is_trajectory_free = False
            spd_obs.dist_next_obstacle_m = distance
            spd_obs.obj_speed_ms = obj.velocity

        return spd_obs

    def get_blocked_ids(self, route: List[Tuple[float, float]]) -> Tuple[List[int], ObjectMeta]:
        """gets the ids of route points that aren't accessible"""
        objects: Dict[int, ObjectMeta] = self.objects.copy()
        min_obj: ObjectMeta = None
        min_id = len(route)
        blocked_ids = []

        for obj_id, obj in objects.items():
            blocked = self.find_blocked_points(route, obj, threshold=1)

            if not blocked:
                continue
            if min(blocked) < min_id:
                min_id = min(blocked)
                min_obj = obj
                blocked_ids += blocked

        blocked_ids = [num for num in blocked_ids if num >= 0]
        if len(blocked_ids) > 0:
            print(f'All blocked_ids: {blocked_ids}, Min_obj: {min_obj}')
        blocked_ids.sort()
        blocked_ids_temp = [blocked_ids[i] for i in range(len(blocked_ids) - 1)
                            if blocked_ids[i - 1] >= blocked_ids[i] - 2]
        if blocked_ids:
            blocked_ids_temp.append(blocked_ids[0])
        blocked_ids.sort()
        return blocked_ids_temp, min_obj

    def find_blocked_points(self, route: List[Tuple[float, float]],
                            obj: ObjectMeta, threshold: float):
        """finds blocked points and returns their ids"""
        threshold = threshold ** 2
        indices = []
        obj_positions = [obj.trajectory[-1]]
        if len(obj.trajectory) > 2:
            obj_positions = ObjectHandler._predict_movement(obj.trajectory, num_points=100)

        veh_pos = self.vehicle.pos
        veh_vel = self.vehicle.velocity_mps

        for index, route_point in enumerate(route):
            # calculate square distance
            distance = ObjectHandler._closest_point(route_point, obj_positions)
            if distance > threshold:
                continue
            zone_clearance_time = ObjectHandler._calculate_zone_clearance(
                route_point, obj_positions[-1], obj.velocity, veh_pos, veh_vel, threshold)
            # print(f'Clearance_zone {zone_clearance_time} for obj_id {obj.identifier}')
            if zone_clearance_time < 0.0:
                indices += [index - 1, index]
                break
        return indices

    @staticmethod
    def _predict_movement(trajectory: List[Tuple[float, float]],
                          num_points: int) -> List[Tuple[float, float]]:
        """This function estimates the position of the object. """
        predictions = trajectory[-2:]
        vec_average = (0.0, 0.0)

        for p_1, p_2 in zip(trajectory[:-1], trajectory[1:]):
            vec_1 = sub_vector(p_2, p_1)
            vec_average = add_vector(vec_1, vec_average)

        vec_average = (vec_average[0] / len(trajectory), vec_average[1] / len(trajectory))
        last_point = trajectory[-1]
        for _ in range(num_points):
            last_point = add_vector(last_point, vec_average)
            predictions.append(last_point)
        return predictions

    @staticmethod
    def _calculate_zone_clearance(route_point, obj_pos, obj_vel, veh_pos, veh_vel, threshold):
        time_obj_leave = (dist(obj_pos, route_point) + threshold) / \
                         obj_vel if obj_vel != 0.0 else 999
        time_obj_enter = (dist(obj_pos, route_point) - threshold) / \
                         obj_vel if obj_vel != 0.0 else 999
        time_self_enter = (dist(veh_pos, route_point) - threshold) / \
                          veh_vel if veh_vel != 0.0 else 999
        time_self_leave = (dist(veh_pos, route_point) - threshold) / \
                          veh_vel if veh_vel != 0.0 else 999
        zone_clearance_time = min(time_obj_leave - time_self_enter,
                                  time_self_leave - time_obj_enter)
        return zone_clearance_time

    @staticmethod
    def _closest_point(point, points) -> float:
        # calculate square distance
        distances = np.sum((np.array(points) - np.array(point)) ** 2, axis=1)
        return np.min(distances)

    def calculate_min_distance(self, route, until_id, obj_pos):
        distance = 0
        route.insert(0, self.vehicle.pos)
        for index, point in enumerate(route):
            if until_id <= index:
                distance += dist(point, obj_pos)
                break
            distance += dist(point, route[index + 1])
        return distance

    @staticmethod
    def _cumulated_dist(veh_pos: Tuple[float, float], route_pos: Tuple[float, float]):
        return abs(veh_pos[0] - route_pos[0]) + abs(veh_pos[1] - route_pos[1])

    def _convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        # coordinate = (coordinate[0], coordinate[1] + 0.5)
        theta = self.vehicle.orientation_rad - pi / 2
        coordinate = rotate_vector(coordinate, theta)
        return coordinate[0] + self.vehicle.pos[0], coordinate[1] + self.vehicle.pos[1]

    def sigmoid_smooth(self, object_speed, object_coordinates, point, first_coord, factor):
        """tries to smooth out the overtaking maneuver so it can be driven at higher speeds"""
        street_width = factor * self.street_width  # parameter to stop in the  middle of other lane
        slope = 3  # slope of overtaking
        relative_velocity = self.vehicle.velocity_mps - object_speed
        # if relative_velocity <= 10/3.6:
        #     return 0
        relative_distance_to_object = -dist(point, object_coordinates[0])
        if dist(point, first_coord) > dist(first_coord, object_coordinates[0]):
            relative_distance_to_object = -relative_distance_to_object
        time_to_collision = 1
        self.dist_safe = max([relative_velocity * time_to_collision, 0])
        # self.dist_safe = 6
        dist_c = dist(object_coordinates[0], object_coordinates[-1]) + 30
        print(relative_distance_to_object, 'dist')
        if relative_velocity < 0:
            dist_c = 0
        x_1 = (1 / slope) * (relative_distance_to_object + self.dist_safe)
        x_2 = (1 / slope) * (relative_distance_to_object - self.dist_safe - dist_c)
        print(x_1, x_2)
        deviation = (street_width / (1 + math.exp(-x_1))) + ((-street_width) / (1 + math.exp(-x_2)))
        return deviation

    def plan_lane_change(self):
        # 1) check if overtake even required
        # 2) decide whether overtake on left / right side
        # 3) determine the bounds where to overtake
        # 4) re-plan the overtaking trajectory
        #    - shift each trajectory point if possible
        #    - update lane assignments for shifted points
        # 5) check whether the new trajectory is drivable
        #    - if yes: apply the trajectory
        #    - if no: keep the old trajectory (car will brake)
        pass

    def plan_route_around_objects(self, local_route: List[Tuple[float, float]], annotations):
        """calculates a route on the left side of the obstacle"""
        ann = annotations
        temp_route = local_route.copy()
        enum_route = local_route.copy()
        blocked_ids, closest_object = self.get_blocked_ids(enum_route)

        if not blocked_ids:
            return temp_route, ann[:0]

        lane, possible_lanes = ann[min(blocked_ids)]
        possible_lanes = np.abs(possible_lanes)
        if lane - 1 in possible_lanes:
            side = 1
        elif lane + 1 in possible_lanes:
            side = -1
        else:
            side = 0

        self.side = side
        widths = []
        lanes = []
        for index, tmp_point in enumerate(enum_route):
            lane, possible_lanes = ann[min(blocked_ids)]
            if lane - side not in possible_lanes:
                side = 0
            if index != 0:
                moving_vector = orth_offset_left(enum_route[index - 1], tmp_point, 1.0)
            else:
                moving_vector = orth_offset_left(tmp_point, enum_route[index + 1], 1.0)

            width = 0.0
            if closest_object is not None:
                blocked_route = [enum_route[blocked_id] for blocked_id in blocked_ids]
                print(index, 'index')
                width = self.sigmoid_smooth(closest_object.velocity, blocked_route,
                                            tmp_point, enum_route[0], side)

            if abs(width) > 0.5:
                lane = lane - side
            side = self.side
            widths.append(width)
            lanes.append(lane)
            temp_route[index] = (tmp_point[0] + width * moving_vector[0],
                                 tmp_point[1] + width * moving_vector[1])

        new_check, _ = self.get_blocked_ids(temp_route)
        print(widths)
        if np.sum(widths) > 50:
            print('here')
        print('Second Block', new_check)
        print('Blocked:', blocked_ids)
        if closest_object is not None:
            print('Distance to object', dist(self.vehicle.pos, closest_object.trajectory[-1]))
            if len(new_check) == 0:
                print(temp_route)
        return (local_route, None) if new_check else (temp_route, lanes)
