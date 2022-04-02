"""Represents a handler for the detected objects."""
from math import pi, dist, exp, sqrt
from typing import List, Tuple, Dict, Callable
from dataclasses import dataclass, field
from copy import deepcopy

import numpy as np

from vehicle_control.core import Vehicle
from vehicle_control.core.geometry import \
    rotate_vector, orth_offset_left, add_vector, scale_vector, sub_vector
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
    max_velocity_change_rate: float = 0.1
    street_width: float = 3.5
    dist_safe: int = 10
    tracker: int = 0

    def __post_init__(self):
        if not self.map:
            self.map = load_xodr_map()

    def get_speed_observation(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Retrieve the speed observation containing the information about blocking vehicles"""
        if not self.vehicle.is_ready:
            return SpeedObservation()
        return self._detect_vehicle_in_lane(local_route)

    def update_objects(self, object_list: List[Dict]):
        """Update the object list, their position and orientation"""
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
                obj = ObjectMeta(obj_id, obj['obj_class'], [new_abs_pos])
                self.objects[obj_id] = obj
        self.objects = {k: self.objects[k] for k in keys}

    def _detect_vehicle_in_lane(self, local_route: List[Tuple[float, float]]) -> SpeedObservation:
        """Detects if a vehicle is in our lane and returns the closest blocking one in a Speed Observation"""
        route = deepcopy(local_route)
        spd_obs = SpeedObservation()
        blocked_ids, obj = self.get_blocked_ids(route, prediction_wanted=True)
        if len(blocked_ids) > 0:
            distance = ObstacleObserver._cumulated_dist(self.vehicle.pos,
                                                        local_route[min(blocked_ids)])
            spd_obs.is_trajectory_free = False
            spd_obs.dist_next_obstacle_m = distance
            spd_obs.obj_speed_ms = 0
            spd_obs.object_type = obj.obj_class
        return spd_obs

    def get_blocked_ids(self, route: List[Tuple[float, float]],
                        prediction_wanted: bool = True) -> Tuple[List[int], ObjectMeta]:
        """gets the ids of route points that aren't accessible
        Output: List of all the blocked waypoints by id, closest object
        """
        objects: Dict[int, ObjectMeta] = deepcopy(self.objects)
        min_obj: ObjectMeta = None
        min_id = len(route)
        blocked_ids = []
        for obj_id, obj in objects.items():
            blocked = self.find_blocked_points(route, obj, threshold=1.3,
                                               prediction_wanted=prediction_wanted)

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
        """finds blocked points and returns their ids of an Object
        Return: Lsit of Ids that are blocked by obj
        """
        indices = []
        obj_positions = [obj.trajectory[-1]]
        if len(obj.trajectory) > 4 and prediction_wanted:
            obj_positions = ObstacleObserver._predict_movement(obj.trajectory,
                                                               obj.velocity, num_points=50)
        #visualize_route_rviz(obj_positions)
        veh_pos = self.vehicle.pos
        veh_vel = self.vehicle.velocity_mps

        for index, route_point in enumerate(route):
            # calculate square distance
            distance = ObstacleObserver._closest_point(route_point, obj_positions)
            distance = sqrt(distance)
            if distance > threshold:
                continue
            for pos in obj_positions:
                zone_clearance_time = ObstacleObserver._calculate_zone_clearance(
                    route_point, pos, obj.velocity, veh_pos, veh_vel, threshold)
                if zone_clearance_time < 2.0:
                    indices += [index]
        indices = list(set(indices))
        return indices

    @staticmethod
    def _predict_movement(trajectory: List[Tuple[float, float]], velocity: float,
                          num_points: int) -> List[Tuple[float, float]]:
        """This function estimates the trajectory of the object.
         By averaging the last known trajectory
         and Linear interpolation"""
        predictions = trajectory[-1:]
        vec_average = (0.0, 0.0)
        if velocity < 1:
            return predictions
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
        """Calculates the the time difference of us leaving and
         the other obj entering a specific route point or vice versa
        Inputs: route_point: the blocked waypoint
                obj_pos: the current position of the object
                obj_vel: the velocity of the object
                veh_pos: our position
                veh_vel: our velocity
                threshold: the radius to look around a waypoint
        """
        obj_vel = 0.0001 if obj_vel == 0 else obj_vel
        veh_vel = 0.0001 if veh_vel == 0 else veh_vel
        zone_clearence_times = [1000]
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
        """calculate square distance"""
        distances = np.sum((np.array(points) - np.array(point)) ** 2, axis=1)
        return np.min(distances)

    @staticmethod
    def _cumulated_dist(veh_pos: Tuple[float, float], route_pos: Tuple[float, float]):
        """calculates distance between two points"""
        return abs(veh_pos[0] - route_pos[0]) + abs(veh_pos[1] - route_pos[1])

    def _convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        # coordinate = (coordinate[0], coordinate[1] + 0.5)
        theta = self.vehicle.orientation_rad - pi / 2
        coordinate = rotate_vector(coordinate, theta)
        return coordinate[0] + self.vehicle.pos[0], coordinate[1] + self.vehicle.pos[1]

    def sigmoid_smooth(self, object_speed: float, object_coordinates: List[Tuple[float, float]],
                       point: Tuple[float, float], first_coord: Tuple[float, float], side: int) -> float:
        """Calculate the orthogonal offset for smooth lane changes. using 2 sigmoid functions"""
        # TODO: use the exact road withds from the XODR map
        street_width = side * self.street_width  # parameter to stop in the  middle of other lane
        slope = 5  # slope of overtaking
        relative_velocity = (self.vehicle.velocity_mps + 10/3.6) - object_speed
        relative_distance_to_object = -dist(point, object_coordinates[0])
        if dist(point, first_coord) > dist(first_coord, object_coordinates[0]):
            relative_distance_to_object = -relative_distance_to_object
        time_to_collision = 1
        self.dist_safe = max([relative_velocity * time_to_collision, 0])
        dist_c = dist(object_coordinates[0], object_coordinates[-1]) + max(self.dist_safe, 10)
        if relative_velocity < 0:
            dist_c = 0
        x_1 = (1 / slope) * (relative_distance_to_object + self.dist_safe)
        x_2 = (1 / slope) * (relative_distance_to_object - self.dist_safe - dist_c)
        deviation = (street_width / (1 + exp(-x_1))) + ((-street_width) / (1 + exp(-x_2)))
        return deviation

    def plan_overtaking_maneuver(self, local_route: List[AnnRouteWaypoint],
                                 orig_route: List[AnnRouteWaypoint]) -> List[AnnRouteWaypoint]:
        """Plan the new trajectory of an overtaking maneuver"""

        self.tracker += 1
        lanes = [(point.actual_lane, point.possible_lanes) for point in local_route]
        temp_route = [point.pos for point in local_route]
        enum_route = temp_route.copy()
        original_route = [point.pos for point in orig_route]
        # abort with previous trajectory if no overtake required
        blocked_ids, closest_object = self.get_blocked_ids(enum_route, prediction_wanted=True)
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
        shifted_wps = self._shift_waypoints(enum_route, sigmoid, original_route)

        new_is_blocked, _ = self.get_blocked_ids(shifted_wps, prediction_wanted=True)
        if new_is_blocked:
            print('new is blocked')
            return None

        # update waypoint annotations
        local_route = self._write_route_into_annotated(shifted_wps, local_route, orig_route)

        print('overtaking')
        return local_route

    def _write_route_into_annotated(self, shifted_wps: List[Tuple[float, float]],
                                    local_route: List[AnnRouteWaypoint],
                                    orig_route: List[AnnRouteWaypoint]
                                    ) -> List[AnnRouteWaypoint]:
        new_route = []
        first_road = self.map.roads_by_id[orig_route[0].road_id]
        self.street_width = first_road.lane_widths[orig_route[0].actual_lane]
        for i in range(len(local_route)):
            wp = local_route[i]
            pos = shifted_wps[i]
            orig_wp = orig_route[i].pos
            orig_lane = orig_route[i].actual_lane
            print(self.street_width, 'Street width')
            sign = orig_lane / abs(orig_lane)
            road_id = wp.road_id
            dist_to_org = dist(pos, orig_wp)
            actual_lane = orig_lane - sign if dist_to_org > self.street_width/2 else orig_lane

            if actual_lane == 0:
                print('invalid overtake')
                return None
            if actual_lane not in wp.possible_lanes:
                print(actual_lane, wp.possible_lanes, wp.actual_lane)
                print('invalid possible')
                return None
            new_wp = AnnRouteWaypoint(pos, road_id, actual_lane, wp.possible_lanes,
                                      wp.legal_speed, wp.dist_next_tl, wp.end_lane_m,
                                      stop_sign_m=wp.stop_sign_m)
            new_route.append(new_wp)

        return new_route
    

    def _shift_waypoints(self, enum_route, wp_shift: Callable[[Tuple[float, float]], float],
                         original_route: List[Tuple[float, float]]):
        """shifts waypoints using orthogonal vectors and ensures
         points dont differ too far from original route"""
        offset_vectors = [orth_offset_left(enum_route[i], enum_route[i+1], 1.0)
                          for i in range(len(enum_route) - 1)]

        offset_vectors.insert(0, offset_vectors[0])
        offset_scales = [wp_shift(wp) for wp in enum_route]

        signed_distance_weight = [self.street_width - dist(point, orig_point)
                                  for point, orig_point in zip(enum_route, original_route)]
        weights = [min(scale, distance) for scale, distance
                   in zip(offset_scales, signed_distance_weight)]
        offsets = [scale_vector(vec, vec_len) for vec, vec_len in zip(offset_vectors, weights)]
        shifted_wps = [add_vector(wp, vec) for wp, vec in zip(enum_route, offsets)]
        return shifted_wps

    @staticmethod
    def _can_overtake(ann, blocked_ids):
        """checks """
        lane, possible_lanes = ann[min(blocked_ids)]
        left_lane, right_lane = (lane - 1, lane + 1) if lane > 0 else (lane + 1, lane - 1)
        overtake_left, overtake_right = left_lane in possible_lanes, right_lane in possible_lanes
        # alter following lines to enable overtaking in opposing lanes
        # if left_lane not in possible_lanes:
        #     left_lane = -lane
        #     overtake_left, overtake_right = left_lane in possible_lanes, right_lane in possible_lanes
        # TODO: consider also the possible lanes of the end point and one point in the middle
        # return overtake_left, overtake_right
        return overtake_left, False # TODO: enforce "drive on the right" rule
