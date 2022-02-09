"""A route planner based on map and sensor data"""

from math import dist, pi
from typing import List, Tuple, Dict
from dataclasses import dataclass, field
import numpy as np

from local_planner.vehicle_control import DrivingController
from local_planner.core.vehicle import Vehicle
from local_planner.core.geometry import angle_between_vectors, points_to_vector, \
    norm_angle, rotate_vector
from local_planner.state_machine import SpeedObservation, TrafficLightInfo
from local_planner.vehicle_control import CurveDetection


@dataclass
class ObjectInfo:
    """Representing information on a recently detected object"""
    identifier: int
    obj_class: str
    trajectory: List[Tuple[float, float]]
    velocity: float = 0.0


@dataclass
class TrajectoryPlanner:
    # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""
    vehicle: Vehicle
    driving_control: DrivingController = None
    global_route: List[Tuple[float, float]] = field(default_factory=list)
    next_wp_id: int = -1
    prev_wp_id: int = -1
    cached_local_route: List[Tuple[float, float]] = field(default_factory=list)
    length_route: int = 50
    delta_time: float = 0.1
    objects: Dict[int, ObjectInfo] = field(default_factory=dict)
    tld_info: TrafficLightInfo = TrafficLightInfo()
    curve_detection: CurveDetection = CurveDetection()

    def update_global_route(self, waypoints: List[Tuple[float, float]]):
        """Update the global route to follow"""
        print(f"update global route ({len(waypoints)} points): {waypoints}")
        print("time,veh_x,vehicle_y,vehicle_orient,vehicle_vel,signal_vel,signal_steer")

        self.global_route = waypoints
        if len(self.global_route) < 2:
            self.next_wp_id = -1
            self.prev_wp_id = -1
            self.cached_local_route = []
        else:
            self.next_wp_id = 1
            self.prev_wp_id = 0
            bound = min(self.prev_wp_id + self.length_route, len(self.global_route))
            self.cached_local_route = self.global_route[self.prev_wp_id:bound]

    def calculate_trajectory(self) -> List[Tuple[float, float]]:
        """Combines trajectory and respective velocity to one data struct"""
        vehicle_not_ready = not self.vehicle.is_ready
        # print("vehicle_not_ready: ", vehicle_not_ready,
        #       " pos: ", self.vehicle.pos, " route: ", self.cached_local_route)
        if vehicle_not_ready:
            self.cached_local_route = []
            return []

        self.cached_local_route = self._compute_local_route()
        return self.cached_local_route

    def _compute_local_route(self) -> List[Tuple[float, float]]:
        # cache the route to avoid concurrency bugs because
        # the route might be overwritten by the navigation task
        route = self.global_route

        if len(route) < 2:
            return route

        is_last_wp = self.next_wp_id == len(route)
        if is_last_wp:
            return route[self.prev_wp_id:self.next_wp_id]

        # delete route waypoints behind car
        while True:
            prev_wp = route[self.prev_wp_id]
            next_wp = route[self.next_wp_id]
            vec_route = points_to_vector(prev_wp, next_wp)
            vec_car = points_to_vector(self.vehicle.pos, next_wp)
            angle = angle_between_vectors(vec_route, vec_car)

            is_wp_in_front_of_car = abs(angle) < 0.5 * pi
            is_last_wp_of_route = self.next_wp_id == len(route) - 1
            if is_wp_in_front_of_car or is_last_wp_of_route:
                break

            self.next_wp_id += 1
            self.prev_wp_id += 1
            # print(f'reached route wp {next_wp}, angle={abs(angle)}')

        bound = min(self.prev_wp_id + self.length_route, len(route))
        return route[self.prev_wp_id:bound]

    @property
    def latest_speed_observation(self) -> SpeedObservation:
        """Retrieve the lastest speed observation"""
        if not self.cached_local_route:
            return SpeedObservation()

        # fuse object detection with traffic light detection
        speed_obs = self.detect_vehicle_in_lane()
        speed_obs.tl_phase = self.tld_info.phase
        speed_obs.dist_next_traffic_light_m = self.tld_info.distance

        # TODO: handle yellow traffic lights here ...

        curve_obs = self.curve_detection.find_next_curve(self.cached_local_route)
        speed_obs.dist_next_curve = curve_obs.dist_until_curve
        speed_obs.curve_target_speed = curve_obs.max_speed

        return speed_obs

    def update_tld_info(self, tld_info: TrafficLightInfo):
        """Update latest information on the traffic lights ahead"""
        self.tld_info = tld_info

    def update_objects(self, object_list: List[Dict]):
        """Refresh the objects that were detected by the perception"""
        keys = []
        for obj in object_list:
            obj_id = obj['identifier']
            keys.append(obj_id)

            new_abs_pos = self.convert_relative_to_world(obj['rel_position'])
            if obj_id in self.objects:
                last_abs_pos = self.objects[obj_id].trajectory[-1]
                self.objects[obj_id].trajectory.append(new_abs_pos)
                self.objects[obj_id].velocity = dist(new_abs_pos, last_abs_pos) / self.delta_time
            else:
                self.objects[obj['identifier']] = ObjectInfo(identifier=obj['identifier'],
                                                             obj_class=obj['obj_class'],
                                                             trajectory=[new_abs_pos])
        self.objects = {k: self.objects[k] for k in keys}

    def convert_relative_to_world(self, coordinate: Tuple[float, float]) -> Tuple[float, float]:
        """Converts relative coordinates to world coordinates"""
        theta = norm_angle(self.vehicle.orientation_rad - np.pi / 2)
        offset = self.vehicle.pos
        coordinate = rotate_vector(coordinate, theta)
        return coordinate[0]+offset[0], coordinate[1]+offset[1]

    def detect_vehicle_in_lane(self) -> SpeedObservation:
        """Detect a vehicle in the same direction."""
        # cache the route, objects and position to avoid concurrency bugs
        vehicle_pos = self.vehicle.pos
        route = self.cached_local_route
        objects = self.objects.copy()

        spd_obs = SpeedObservation()
        for _, obj in objects.items():
            last_obj_pos = obj.trajectory[-1]
            distances = []
            for point in route:
                # ToDo: Do this for every predicted position of the object
                distance = dist(last_obj_pos, point)
                distances.append(distance)
                if distance > 2.0:
                    continue

                # only apply the most relevant object
                distance = dist(vehicle_pos, last_obj_pos)
                if distance < spd_obs.dist_next_obstacle_m:
                    spd_obs.is_trajectory_free = False
                    spd_obs.dist_next_obstacle_m = distance
                    spd_obs.obj_speed_ms = obj.velocity

        return spd_obs
