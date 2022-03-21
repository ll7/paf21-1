"""A route planner based on map and sensor data"""

from math import pi
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

from local_planner.core import Vehicle, AnnRouteWaypoint
from local_planner.vehicle_control import DrivingController
from local_planner.core.geometry import angle_between_vectors, points_to_vector
from local_planner.state_machine import SpeedObservation, TrafficLightInfo, ManeuverObservation
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
    global_route_ann: List[AnnRouteWaypoint] = field(default_factory=list)
    next_wp_id: int = -1
    prev_wp_id: int = -1
    length_route: int = 100
    obj_handler: ObjectHandler = None
    tld_info: TrafficLightInfo = TrafficLightInfo()
    current_route: List[Tuple[float, float]] = field(default_factory=list)
    curve_detection: CurveDetection = CurveDetection()

    def __post_init__(self):
        if not self.obj_handler:
            self.obj_handler = ObjectHandler(self.vehicle)

    @property
    def global_route(self) -> List[Tuple[float, float]]:
        """Retrieve the x/y coordinates of the global route"""
        return [wp.pos for wp in self.global_route_ann]

    @property
    def cached_local_route(self) -> List[Tuple[float, float]]:
        """Retrieve the x/y coordinates of the global route"""
        if self.prev_wp_id < 0 or not self.vehicle.is_ready:
            return []

        if not self.is_navigation_ready:
            return self.global_route

        if self.is_last_wp:
            return self.global_route[-1:]

        bound = min(self.prev_wp_id + self.length_route, len(self.global_route))
        return self.global_route[self.prev_wp_id:bound]

    @property
    def cached_local_ann_route(self) -> List[AnnRouteWaypoint]:
        """Retrieve the cached local route with metadata annotations"""
        if self.prev_wp_id < 0 or not self.vehicle.is_ready:
            return []

        if not self.is_navigation_ready:
            return self.global_route_ann

        if self.is_last_wp:
            return []

        bound = min(self.prev_wp_id + self.length_route, len(self.global_route))
        return self.global_route_ann[self.prev_wp_id:bound]

    @property
    def is_navigation_ready(self) -> bool:
        """Indicates whether the navigation already provided a route."""
        return self.global_route_ann and len(self.global_route_ann) >= 2

    @property
    def is_last_wp(self) -> bool:
        """Indicates whether the current point is the last point."""
        return self.next_wp_id == len(self.global_route_ann) - 1

    def update_global_route(self, ann_waypoints: List[AnnRouteWaypoint]):
        """Update the global route to follow"""

        self.global_route_ann = ann_waypoints
        print(f"update global route ({len(self.global_route)} points): {self.global_route}")

        if len(self.global_route) < 2:
            self.next_wp_id = -1
            self.prev_wp_id = -1
        else:
            self.next_wp_id = 1
            self.prev_wp_id = 0

    def calculate_trajectory(self) -> List[Tuple[float, float]]:
        """Combines trajectory and respective velocity to one data struct"""
        if not self.vehicle.is_ready:
            return []

        # cache the route to avoid concurrency bugs because
        # the route might be overwritten by the navigation task
        route = self.global_route

        if not self.is_navigation_ready:
            return route

        if self.is_last_wp:
            return [route[-1]]
        # delete route waypoints behind car
        # if len(self.current_route) > 0:
        #    route = route[:self.prev_wp_id] + self.current_route \
        #            + route[self.prev_wp_id + self.length_route:]
        self.check_passed_waypoints(route)

        bound = min(self.prev_wp_id + self.length_route, len(route))
        temp_route = route[self.prev_wp_id:bound]
        # temp_route = self.check_overtake(temp_route)
        self.current_route = temp_route
        return temp_route

    def check_passed_waypoints(self, route):
        """Set the next waypoint index for the not yet passed waypoints"""
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

    def check_overtake(self, route):
        """Checking the route for an overtaking maneuver."""
        curve_obs = self.curve_detection.find_next_curve(self.current_route)
        dist_next_curve = curve_obs.dist_until_curve
        if dist_next_curve > 50 and self.latest_speed_observation.dist_next_traffic_light_m > 50:
            route = self.obj_handler.plan_route_around_objects(route)
        return route

    @property
    def latest_speed_observation(self) -> SpeedObservation:
        """Retrieve the latest speed observation"""

        if not self.current_route:
            return SpeedObservation()
        # fuse object detection with traffic light detection
        speed_obs = self.obj_handler.get_speed_observation(self.current_route)
        speed_obs.tl_phase = self.tld_info.phase
        speed_obs.dist_next_traffic_light_m = self.tld_info.distance
        #speed_obs.dist_next_traffic_light_m = 10000
        curve_obs = self.curve_detection.find_next_curve(self.current_route)
        speed_obs.dist_next_curve = curve_obs.dist_until_curve
        speed_obs.curve_target_speed = curve_obs.max_speed
        cache_local_route = self.cached_local_ann_route
        if len(cache_local_route) > 0:
            speed_obs.detected_speed_limit = cache_local_route[0].legal_speed
        else:
            speed_obs.detected_speed_limit = 0.0
        return speed_obs

    @property
    def latest_maneuver_observation(self) -> ManeuverObservation:
        """Retrieve the latest speed observation"""
        if not self.current_route:
            return ManeuverObservation()
        man_obs = ManeuverObservation()
        curve_obs = self.curve_detection.find_next_curve(self.current_route)
        man_obs.dist_next_curve = curve_obs.dist_until_curve
        return man_obs

    def update_tld_info(self, tld_info: TrafficLightInfo):
        """Update the latest information on the traffic lights ahead"""
        self.tld_info = tld_info

    def update_objects(self, object_list: List[Dict]):
        """Refresh the objects that were detected by the perception"""
        self.obj_handler.update_objects(object_list)
