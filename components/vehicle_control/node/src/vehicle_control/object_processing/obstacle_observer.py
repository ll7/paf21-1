"""Represents a handler for the detected objects."""
import math
from math import pi, dist
from typing import List, Tuple, Dict, Callable
from dataclasses import dataclass, field

import numpy as np

from vehicle_control.core import Vehicle, visualize_route_rviz
from vehicle_control.core.geometry import rotate_vector, orth_offset_left, add_vector,\
    scale_vector, sub_vector, vector_to_dir, points_to_vector
from vehicle_control.route_planning.xodr_converter import XodrMap
from vehicle_control.route_planning.route_annotation import AnnRouteWaypoint
from vehicle_control.map_provider import load_xodr_map
from vehicle_control.object_processing.object_meta import ObjectMeta
from vehicle_control.state_machine import SpeedObservation


@dataclass
class ObstacleObserver:
    """Represents a handler for the detected objects."""
    vehicle: Vehicle
    objects: Dict[int, ObjectMeta] = field(default_factory=dict)
    map: XodrMap = None
    delta_time: float = 0.1
    max_velocity_change_rate: float = 2.0
    street_width: float = 4
    dist_safe: int = 10

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
        blocked_ids, obj = self.get_blocked_ids(route, prediction_wanted=False)
        if len(blocked_ids) > 0:
            # distance = self.calculate_min_distance(route, min(blocked_ids), obj.trajectory[-1])
            distance = self._cumulated_dist(self.vehicle.pos, local_route[min(blocked_ids)])
            spd_obs.is_trajectory_free = False
            spd_obs.dist_next_obstacle_m = distance
            spd_obs.obj_speed_ms = obj.velocity

        return spd_obs

    def get_blocked_ids(self, route: List[Tuple[float, float]],
                        prediction_wanted: bool = True) -> Tuple[List[int], ObjectMeta]:
        """gets the ids of route points that aren't accessible"""
        objects: Dict[int, ObjectMeta] = self.objects.copy()
        min_obj: ObjectMeta = None
        min_id = len(route)
        blocked_ids = []
        for obj_id, obj in objects.items():
            blocked = self.find_blocked_points(route, obj, threshold=1, prediction_wanted=prediction_wanted)

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
        blocked_ids_temp.sort()
        return blocked_ids_temp, min_obj

    def find_blocked_points(self, route: List[Tuple[float, float]],
                            obj: ObjectMeta, threshold: float, prediction_wanted: bool = True):
        """finds blocked points and returns their ids"""
        indices = []
        obj_positions = [obj.trajectory[-1]]
        if len(obj.trajectory) > 8 and prediction_wanted:
            obj_positions = self._predict_movement(obj.trajectory, obj.velocity, num_points=50)
        #visualize_route_rviz(obj_positions)
        veh_pos = self.vehicle.pos
        veh_vel = self.vehicle.velocity_mps

        for index, route_point in enumerate(route):
            # calculate square distance
            distance = ObstacleObserver._closest_point(route_point, obj_positions)
            distance = math.sqrt(distance)
            if distance > threshold:
                continue
            for pos in obj_positions:
                zone_clearance_time = ObstacleObserver._calculate_zone_clearance(
                    route_point, pos, obj.velocity, veh_pos, veh_vel, threshold)
                #print(f'Clearance_zone {zone_clearance_time} for obj_id {obj.identifier}')
                if zone_clearance_time < 3.0:
                    indices += [index]


        indices = list(set(indices))
        return indices

    def _predict_movement(self, trajectory: List[Tuple[float, float]], velocity: float,
                          num_points: int) -> List[Tuple[float, float]]:
        """This function estimates the position of the object. """
        pos = trajectory[-1]
        predictions = [pos]
        if velocity < 1:
            return predictions
        old_vector = points_to_vector(trajectory[-7], trajectory[-1])
        new_vector = points_to_vector(trajectory[-4], trajectory[-1])

        theta_old = vector_to_dir(old_vector)
        theta = vector_to_dir(new_vector)
        theta_dot = theta - theta_old

        steering_angle = np.arctan((theta_dot/velocity) * self.vehicle.meta.wheelbase)
        steering_angle = np.clip(steering_angle, -np.deg2rad(5), np.deg2rad(5))
        timestep = 1/10
        for _ in range(num_points):
            x_n = pos[0] + velocity * np.cos(theta + steering_angle) * timestep
            y_n = pos[1] + velocity * np.sin(theta + steering_angle) * timestep
            theta = theta + ((velocity * np.tan(steering_angle)) /
                             self.vehicle.meta.wheelbase) * timestep
            pos = (x_n, y_n)
            predictions.append(pos)
        return predictions

    @staticmethod
    def _calculate_zone_clearance(route_point, obj_pos, obj_vel, veh_pos, veh_vel, threshold):
        obj_vel = 0.0001 if obj_vel == 0 else obj_vel
        veh_vel = 0.0001 if veh_vel == 0 else veh_vel
        zone_clearence_times = [10]
        if dist(obj_pos, route_point) < threshold:
            time_obj_enter = 0
            time_obj_leave = (2 * threshold - dist(obj_pos, route_point)) / obj_vel
        else:
            time_obj_leave = (dist(obj_pos, route_point) + threshold) / obj_vel
            time_obj_enter = (dist(obj_pos, route_point) - threshold) / obj_vel
        if dist(veh_pos, route_point) < threshold:
            time_self_enter = 0
            time_self_leave = (2 * threshold - dist(veh_pos, route_point)) / veh_vel
        else:
            time_self_enter = (dist(veh_pos, route_point) - threshold) / veh_vel
            time_self_leave = (dist(veh_pos, route_point) + threshold) / veh_vel
        if time_obj_leave > time_self_enter > time_obj_enter or time_obj_enter > time_self_leave:
            zone_clearence_times.append(time_self_enter - time_obj_leave)
        if time_self_leave > time_obj_enter > time_self_enter or time_self_enter > time_obj_leave:
            zone_clearence_times.append(time_self_enter - time_obj_leave)

        zone_clearance_time = min(zone_clearence_times)
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

    def sigmoid_smooth(self, object_speed: float, object_coordinates: List[Tuple[float, float]],
                       point: Tuple[float, float], first_coord: Tuple[float, float], side: int) -> float:
        """Calculate the orthogonal offset for smooth lane changes."""
        # TODO: use the exact road withds from the XODR map
        # point = ann_point.pos
        street_width = side * self.street_width  # parameter to stop in the  middle of other lane
        slope = 5  # slope of overtaking
        relative_velocity = self.vehicle.velocity_mps - object_speed
        # if relative_velocity <= 10/3.6:
        #     return 0
        relative_distance_to_object = -dist(point, object_coordinates[0])
        if dist(point, first_coord) > dist(first_coord, object_coordinates[0]):
            relative_distance_to_object = -relative_distance_to_object
        time_to_collision = 1
        self.dist_safe = max([relative_velocity * time_to_collision, 0])
        # self.dist_safe = 6
        dist_c = dist(object_coordinates[0], object_coordinates[-1]) + 50
        if relative_velocity < 0:
            dist_c = 0
        x_1 = (1 / slope) * (relative_distance_to_object + self.dist_safe)
        x_2 = (1 / slope) * (relative_distance_to_object - self.dist_safe - dist_c)
        deviation = (street_width / (1 + math.exp(-x_1))) + ((-street_width) / (1 + math.exp(-x_2)))
        return deviation

    def plan_overtaking_maneuver(self, local_route: List[AnnRouteWaypoint],
                                 orig_route: List[AnnRouteWaypoint]) -> List[AnnRouteWaypoint]:
        """Plan the new trajectory of an overtaking maneuver"""

        lanes = [(point.actual_lane, point.possible_lanes) for point in local_route]
        temp_route = [point.pos for point in local_route]
        enum_route = temp_route.copy()
        original_route = [point.pos for point in orig_route]
        # abort with previous trajectory if no overtake required
        blocked_ids, closest_object = self.get_blocked_ids(enum_route, prediction_wanted=False)
        if not blocked_ids:
            return None

        # 2) decide whether overtake on left / right side
        overtake_left, overtake_right = ObstacleObserver._can_overtake(lanes, blocked_ids)
        if not overtake_left and not overtake_right:
            print('no overtake')
            return None
        side = 1 if overtake_left else -1

        # 3) re-plan the overtaking trajectory
        blocked_route = [enum_route[i] for i in blocked_ids]
        sigmoid = lambda wp: self.sigmoid_smooth(closest_object.velocity, blocked_route, wp, enum_route[0], side)
        point_can_be_moved = [1 if wp.actual_lane - side in wp.possible_lanes else 0 for wp in
                              local_route]
        shifted_wps = ObstacleObserver._shift_waypoints(enum_route, sigmoid, point_can_be_moved, original_route)

        new_is_blocked, _ = self.get_blocked_ids(shifted_wps, prediction_wanted=False)
        if new_is_blocked:
            print('new is blocked')
            return None

        # update waypoint annotations
        local_route = self._write_route_into_annotated(shifted_wps, local_route)

        print('overtaking')
        return local_route

    def _write_route_into_annotated(self, shifted_wps: List[Tuple[float, float]],
                                    local_route: List[AnnRouteWaypoint]) -> List[AnnRouteWaypoint]:
        new_route = []
        for i in range(len(local_route)):
            wp = local_route[i]
            pos = shifted_wps[i]
            road_id = wp.road_id
            actual_lane = 0
            for road in list(self.map.roads_by_id.keys()):
                actual_lane = self.map.roads_by_id[road].detect_lane(pos, wp.road_id)
                if actual_lane != 0:
                    road_id = road
                    break
            if actual_lane == 0:
                print('WARNING: new annotated actual lane is 0, this should never happen!')
                continue
            
            new_wp = AnnRouteWaypoint(pos, road_id, actual_lane, wp.possible_lanes,
                                      wp.legal_speed, wp.dist_next_tl, wp.end_lane_m,
                                      stop_sign_m=wp.stop_sign_m)
            new_route.append(new_wp)

        return new_route
    
    @staticmethod
    def _shift_waypoints(enum_route, wp_shift: Callable[[Tuple[float, float]], float],
                         factors: List[int], original_route: List[Tuple[float, float]]):
        offset_vectors = [orth_offset_left(enum_route[i], enum_route[i+1], 1.0)
                          for i in range(len(enum_route) - 1)]

        offset_vectors.insert(0, offset_vectors[0])
        offset_scales = [wp_shift(wp) for wp in enum_route]
        sign = -1 if np.sum(offset_scales) < 0 else 1
        signed_distance_weight = [sign * 4 - (sign * dist(point, orig_point))
                                  for point, orig_point in zip(enum_route, original_route)]
        offset_scales = [wp * factor for wp, factor in zip(offset_scales, factors)]
        #offsets = [scale_vector(vec, vec_len) for vec, vec_len in zip(offset_vectors, offset_scales)]
        weights = [min(scale, distance) if scale != 0
                   else distance for scale, distance in zip(offset_scales, signed_distance_weight)]
        print(weights)
        offsets = [scale_vector(vec, vec_len) for vec, vec_len in zip(offset_vectors, weights)]
        shifted_wps = [add_vector(wp, vec) for wp, vec in zip(enum_route, offsets)]
        return shifted_wps

    @staticmethod
    def _can_overtake(ann, blocked_ids):
        lane, possible_lanes = ann[min(blocked_ids)]
        possible_lanes = np.abs(possible_lanes)
        left_lane, right_lane = (lane - 1, lane + 1) if lane > 0 else (lane + 1, lane - 1)
        overtake_left, overtake_right = left_lane in possible_lanes, right_lane in possible_lanes
        # TODO: consider also the possible lanes of the end point and one point in the middle
        #return overtake_left, overtake_right
        return overtake_left, False # TODO: enforce "drive on the right" rule

    def sigmoid_smooth_lanechange(self, object_speed: float, object_coordinates: List[Tuple[float, float]],
                       point: Tuple[float, float], first_coord: Tuple[float, float], side: int) -> float:
        """Calculate the orthogonal offset for smooth lane changes."""
        # TODO: use the exact road withds from the XODR map
        # point = ann_point.pos
        street_width = side * self.street_width  # parameter to stop in the  middle of other lane
        slope = 3  # slope of overtaking
        relative_velocity = self.vehicle.velocity_mps - object_speed
        # if relative_velocity <= 10/3.6:
        #     return 0
        relative_distance_to_object = -dist(point, object_coordinates[0])
        if dist(point, first_coord) > dist(first_coord, object_coordinates[0]):
            relative_distance_to_object = -relative_distance_to_object
        time_to_collision = 0.5
        self.dist_safe = max([relative_velocity * time_to_collision, 0])
        # self.dist_safe = 6
        x_1 = (1 / slope) * (relative_distance_to_object + self.dist_safe)
        deviation = (street_width / (1 + math.exp(-x_1)))
        return deviation