"""A route planner based on map and sensor data"""

from math import pi, dist
from typing import List, Tuple, Dict
from dataclasses import dataclass, field

from local_planner.core import Vehicle
from local_planner.route_planning.route_annotation import AnnRouteWaypoint
from local_planner.vehicle_control import DrivingController
from local_planner.core.geometry import angle_between_vectors, points_to_vector, vector_len
from local_planner.state_machine import SpeedObservation, TrafficLightInfo, \
                                        TrafficLightPhase, ManeuverObservation
from local_planner.vehicle_control import CurveDetection, CurveObservation
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
    end_curve_id = None
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
            if vector_len(vec_car) == 0 or vector_len(vec_route) == 0:
                break
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

        speed_obs = self.obj_handler.get_speed_observation(self.current_route)

        curve_obs = self.curve_detection.find_next_curve(self.current_route)
        speed_obs.dist_next_curve = curve_obs.dist_until_curve
        speed_obs.curve_target_speed = curve_obs.max_speed

        if self.tld_info:
            speed_obs.tl_phase = self.tld_info.phase
            speed_obs.dist_next_traffic_light_m = self.tld_info.distance

        speed_obs.detected_speed_limit = self.legal_speed_ahead() \
            if len(self.cached_local_ann_route) > 0 else 0.0

        self._handle_american_traffic_lights(speed_obs, curve_obs)

        #print('Speed Obs ', speed_obs)
        
        return speed_obs

    def _handle_american_traffic_lights(self, speed_obs: SpeedObservation, curve_obs: CurveObservation):
        print('tl in m {}, dist till curve in m {}'.format(speed_obs.dist_next_traffic_light_m, curve_obs.dist_until_curve))
        print('Cashed ann route end lane m ', self.cached_local_ann_route[0].end_lane_m)
        print('curve obs end id: ', curve_obs.end_id)
        # ignore the traffic lights of orthogonal lanes until the curve is over
        if self.end_curve_id:
            speed_obs.tl_phase = TrafficLightPhase.GREEN
            speed_obs.dist_next_traffic_light_m = 9999
            if self.end_curve_id < self.prev_wp_id:
                self.end_curve_id = None

        # stop before the traffic light if it is just a regular one and not american
        elif abs(speed_obs.dist_next_traffic_light_m - curve_obs.dist_until_curve) < 5: # and curve_obs.end_id < 20:
            print('Stop at the TL')

        # stop at the end of lane
        elif len(self.cached_local_ann_route) > 0:
            dist_end_lane = self.cached_local_ann_route[0].end_lane_m
            turn_at_crossroad = curve_obs.end_id != -1 and \
                abs(curve_obs.dist_until_curve - dist_end_lane) < 5

            speed_obs.dist_next_traffic_light_m = self.cached_local_ann_route[0].end_lane_m
            if turn_at_crossroad:
                self.end_curve_id = curve_obs.end_id

    def legal_speed_ahead(self) -> float:
        """Calculate at witch point the car need to reduce speed to reach legal speed at the sign"""
        legal_speed = self.cached_local_ann_route[0].legal_speed
        index2 = 0
        for index, ann_point in enumerate(self.cached_local_ann_route):
            if ann_point.legal_speed < legal_speed:
                index2 = index
                break

        if index2 == 0:
            return legal_speed

        cached_point = self.cached_local_ann_route[index2]
        target_velocity = cached_point.legal_speed/3.6
        accel_mps2 = -2.0
        maneuver_time_s = (target_velocity - self.vehicle.velocity_mps) / accel_mps2
        braking_dist = self.vehicle.velocity_mps * maneuver_time_s + \
                       accel_mps2 * maneuver_time_s ** 2 / 2 + 1
        # print(dist(cached_point.pos, self.cached_local_ann_route[0].pos), braking_dist, "ma:",
        #       maneuver_time_s, "ta:", target_velocity, self.vehicle.velocity_mps)
        if dist(cached_point.pos, self.cached_local_ann_route[0].pos) < braking_dist:
            return target_velocity*3.6
        return legal_speed

    def update_tld_info(self, tld_info: TrafficLightInfo):
        """Update the latest information on the traffic lights ahead"""
        self.tld_info = tld_info

    def update_objects(self, object_list: List[Dict]):
        """Refresh the objects that were detected by the perception"""
        self.obj_handler.update_objects(object_list)