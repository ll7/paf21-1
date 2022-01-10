"""A route planner based on map and sensor data"""

from dataclasses import dataclass, field
from math import dist, sin, cos
from typing import List, Tuple, Dict

from cv2 import cv2
import numpy as np

from local_planner.vehicle_control import DrivingController
from local_planner.traffic_light_detection import TrafficLightDetector
from local_planner.lane_detection import LaneDetection
from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.state_machine import SpeedObservation, SpeedStateMachine, TrafficLightPhase
from local_planner.core.vehicle import Vehicle

lane_detect_config: str = "/app/src/local_planner/config/config_lane_detection.yml"
traffic_light_config: str = "/app/src/local_planner/config/config_traffic_light_detection.yml"


@dataclass
class TrajectoryPlanner:  # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""

    # TODO: return an actionable plan for the next few seconds
    #       don't execute the play right away -> add another component that executes the plan
    #       add a data structure representing a short-term plan (waypoints + respective velocity)

    vehicle: Vehicle
    image_sensor_collection: SensorCameraPreprocessor
    driving_control: DrivingController = None
    lane_detection: LaneDetection = LaneDetection(lane_detect_config)
    traffic_light_detection: TrafficLightDetector = TrafficLightDetector(traffic_light_config)
    speed_state_machine: SpeedStateMachine = None
    global_route: List[Tuple[float, float]] = field(default_factory=list)
    cached_local_route: List[Tuple[float, float]] = field(default_factory=list)
    step_semantic: int = 0

    def __post_init__(self):
        self.speed_state_machine = SpeedStateMachine(vehicle=self.vehicle)
        self.driving_control = DrivingController(self.vehicle)

    def update_global_route(self, waypoints: List[Tuple[float, float]]):
        """Update the global route to follow"""
        print(waypoints)
        self.global_route = waypoints
        self.cached_local_route = [] if self.global_route is None else self.global_route

    def compute_local_route(self) -> List[Tuple[float, float]]:
        """Modifies the global route based on sensor data
        to fit the current diving circumstances"""
        print(self.step_semantic)

        # interpolate the route such that the route points are closely aligned
        # filter the parts of the global route of the near future

        self.step_semantic += 1
        point_counts_as_done = 5
        enumerator = 0
        for point in self.cached_local_route:
            print('distance', dist(point, self.vehicle.pos))
            if dist(point, self.vehicle.pos) > point_counts_as_done:
                break
            enumerator += 1

        self.cached_local_route = self.cached_local_route[enumerator:]
        short_term_route = self.cached_local_route[:min(50, len(self.cached_local_route))]

        activate_lane_keeping = False
        if activate_lane_keeping:
            short_term_route = self._keep_lane(self.image_sensor_collection, short_term_route)

        # target_velocity = self._update_vehicle_speed(short_term_route)

        return short_term_route

    def _calculate_vehicle_speed(self):
        turned_on_traffic_light_detection = True
        tl_state, dist_m = TrafficLightPhase.Green, 300
        if turned_on_traffic_light_detection:
            tl_state, dist_m = self._detect_traffic_lights()
        speed_obs = SpeedObservation(tl_state, dist_next_obstacle_m=dist_m)
        self.speed_state_machine.update_state(speed_obs)
        return self.speed_state_machine.get_target_speed()

    def calculate_trajectory(self):
        """combines trajectory and respective velocity to one data struct"""

        vehicle_not_ready = self.vehicle.pos is None \
                            or len(self.cached_local_route) == 0
        cameras_ready = all(image is not None for image in
                            self.image_sensor_collection.get_image_lists().values())
        if vehicle_not_ready or not cameras_ready:
            return [], 0
        trajectory = self.compute_local_route()
        velocity = self._calculate_vehicle_speed()
        print(f'{self.vehicle.actual_velocity_mps*3.6}kmh')
        return trajectory, velocity

    def _detect_traffic_lights(self) -> Tuple[TrafficLightPhase, float]:
        images_list = self.image_sensor_collection.get_image_lists()
        rgb_image = images_list['rgb'][:, :, :3]
        depth_image = cv2.resize(images_list['depth'], (900, 600))
        semantic_image = cv2.resize(images_list['semantic'][:, :, :3], (900, 600))

        # TODO: return the traffic light's position in case something was found
        meters, tl_color, highlighted_img = self.traffic_light_detection.detect_traffic_light(
            semantic_image, rgb_image, depth_image)

        stop_vehicle = tl_color in ['Green', 'Backside', 'Yellow']
        tl_phase = TrafficLightPhase.Green if stop_vehicle else TrafficLightPhase.Red

        if self.step_semantic % 10 == 0 and self.step_semantic < 0:
            img_log_path = f"/app/logs/img_{self.step_semantic}_traffic_light.png"
            cv2.imwrite(img_log_path, highlighted_img)

        return tl_phase, meters

    def _keep_lane(self, images: SensorCameraPreprocessor,
                   short_term_route: List[Tuple[float, float]]) \
            -> List[Tuple[float, float]]:
        route = short_term_route.copy()
        image = images.semantic_image[:, :, :3]  # cut off alpha channel
        _, keep_lane, angle = self.lane_detection.detect_lanes(image)

        if keep_lane:
            yaw_diff = self.vehicle.orientation_rad - np.deg2rad(angle)
            x_coord, y_coord = cos(yaw_diff), sin(yaw_diff)
            predicted_position = (self.vehicle.pos[0] + x_coord, self.vehicle.pos[1] + y_coord)

            predicted_vector = np.subtract(predicted_position, self.vehicle.pos)
            route_vector = np.subtract(route[0], self.vehicle.pos)
            cos_angle = np.dot(route_vector, predicted_vector) / np.linalg.norm(
                route_vector) / np.linalg.norm(predicted_vector)
            off_set_angle = np.rad2deg(np.arccos(cos_angle))

            if abs(off_set_angle) < 30:
                route.insert(0, predicted_position)

        # if self.step_semantic % 10 == 0 and self.step_semantic < 0:
        #     cv2.imwrite(f"/app/logs/img_{self.step_semantic}_highlighted.png", highlighted_img)

        return route
