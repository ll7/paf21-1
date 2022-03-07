"""A route planner based on map and sensor data"""

from math import pi
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

from local_planner.core import Vehicle, AnnRouteWaypoint
from local_planner.vehicle_control import DrivingController
from local_planner.core.geometry import angle_between_vectors, points_to_vector
from local_planner.state_machine import SpeedObservation, TrafficLightInfo
from local_planner.vehicle_control import CurveDetection
from local_planner.object_processing import ObjectHandler


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
    obj_handler: ObjectHandler = None
    tld_info: TrafficLightInfo = TrafficLightInfo()
    curve_detection: CurveDetection = CurveDetection()

    def __post_init__(self):
        if not self.obj_handler:
            self.obj_handler = ObjectHandler(self.vehicle)

    def update_global_route(self, ann_waypoints: List[AnnRouteWaypoint]):
        """Update the global route to follow"""
        # print(f"update global route ({len(ann_waypoints)} points): {ann_waypoints}")
        # print("time,veh_x,vehicle_y,vehicle_orient,vehicle_vel,signal_vel,signal_steer")

        self.global_route = [wp.pos for wp in ann_waypoints]
        print(f"update global route ({len(self.global_route)} points): {self.global_route}")

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
        """Retrieve the latest speed observation"""
        if not self.cached_local_route:
            return SpeedObservation()

        # fuse object detection with traffic light detection
        speed_obs = self.obj_handler.get_speed_observation(self.cached_local_route)
        speed_obs.tl_phase = self.tld_info.phase
        speed_obs.dist_next_traffic_light_m = self.tld_info.distance
        curve_obs = self.curve_detection.find_next_curve(self.cached_local_route)
        speed_obs.dist_next_curve = curve_obs.dist_until_curve
        speed_obs.curve_target_speed = curve_obs.max_speed

        return speed_obs

    def update_tld_info(self, tld_info: TrafficLightInfo):
        """Update the latest information on the traffic lights ahead"""
        self.tld_info = tld_info

    def update_objects(self, object_list: List[Dict]):
        """Refresh the objects that were detected by the perception"""
        self.obj_handler.update_objects(object_list)
