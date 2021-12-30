"""A route planner based on map and sensor data"""

from math import dist, sin, cos
from typing import List, Tuple

from cv2 import cv2
import numpy as np

from local_planner.traffic_light_detection import TrafficLightDetector
from local_planner.lane_detection import LaneDetection
from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.preprocessing import SingletonMeta
from local_planner.speed_state_machine import SpeedStateMachine


class RouteInfo(metaclass=SingletonMeta):  # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""

    vehicle_vector: Tuple[float, float] = None
    vehicle_position: Tuple[float, float] = None
    global_route: List[Tuple[float, float]] = []
    cached_local_route: List[Tuple[float, float]] = []
    orientation: float = 0
    step_semantic: int = 0
    lane_detect_config: str = "/app/src/local_planner/config/config_lane_detection.yml"
    traffic_light_config: str = "/app/src/local_planner" \
                                "/config/config_traffic_light_detection.yml"
    lane_detection: LaneDetection = LaneDetection(lane_detect_config)
    traffic_light_detection: TrafficLightDetector = TrafficLightDetector(traffic_light_config)
    speed_state_machine: SpeedStateMachine = SpeedStateMachine()

    def update_vehicle_orientation(self, orientation_rad: float):
        """Calculates the vector the car is currently driving"""

        self.orientation = orientation_rad
        self.vehicle_vector = cos(orientation_rad), sin(orientation_rad)

        # # this is probably doing nothing because x_coord / y_coord are always a point
        # # that's sitting on the unit circle, thus the length is always 1 and moreover,
        # # dividing by 1 will give you the same result for both cases
        # length = np.linalg.norm([x_coord, y_coord])
        # if length > 0:
        #     self.vehicle_vector = [x_coord / length, y_coord / length]
        # else:
        #     self.vehicle_vector = [x_coord, y_coord]

    def update_vehicle_position(self, new_pos: Tuple[float, float]):
        """Update the vehicle position"""
        self.vehicle_position = new_pos

    def update_global_route(self, waypoints: List[Tuple[float, float]]):
        """Update the global route to follow"""
        self.global_route = waypoints
        print(self.global_route)
        # self.global_route = None

        if self.global_route is None:
            self.cached_local_route = []
        else:
            self.cached_local_route = self.global_route

    def compute_local_route(self):
        """Modifies the global route based on sensor data
        to fit the current diving circumstances"""
        if self.vehicle_position is None or len(self.cached_local_route) == 0:
            return []
        self.step_semantic += 1
        # interpolate the route such that the route points are closely aligned
        # filter the parts of the global route of the near future
        images = SensorCameraPreprocessor()
        point_counts_as_done = 5.0
        enumerator = 0
        for point in self.cached_local_route:
            if dist(point, self.vehicle_position) > point_counts_as_done:
                break
            enumerator += 1
        self.cached_local_route = self.cached_local_route[enumerator:]
        images_list = {'semantic': images.semantic_image,
                       'rgb': images.rgb_image,
                       'depth': images.depth_image}
        # neighbour_ids = argsort([dist(p, self.vehicle_position) for p in self.cached_local_route])
        # self.cached_local_route = self.global_route[max(neighbour_ids[0], neighbour_ids[1]):]
        short_term_route = self.cached_local_route[:min(50, len(self.cached_local_route))]
        turned_on_traffic_light_detection = True
        turned_on = True
        short_term_route = self.lane_keeping_assistant(images, short_term_route, turned_on)
        tl_state = self.traffic_light_detec(images_list, turned_on_traffic_light_detection)
        self.speed_state_machine.tl_state = tl_state
        print(f'{tl_state}')
        return short_term_route

    def traffic_light_detec(self, images_list, turned_on_traffic_light_detection):
        """set traffic lane color"""
        tl_color = 'Green'
        if all(image is not None for image in images_list.values()) \
                and turned_on_traffic_light_detection:
            rgb_image = images_list['rgb'][:, :, :3]
            depth_image = images_list['depth']
            semantic_image = images_list['semantic'][:, :, :3]
            meters, tl_color, highlighted_img = self.traffic_light_detection. \
                detect_traffic_light(semantic_image, rgb_image, depth_image)
            print(meters)
            if self.step_semantic % 10 == 0 and self.step_semantic < 0:
                cv2.imwrite(f"/app/logs/img_{self.step_semantic}_traffic_light.png",
                            highlighted_img)
        return tl_color

    def lane_keeping_assistant(self, images, short_term_route, turned_on):
        """starts lane detection for this cycle"""
        if images.semantic_image is not None and turned_on:
            image = images.semantic_image[:, :, :3]  # cut off alpha channel
            highlighted_img, keep_lane, angle = self.lane_detection.detect_lanes(image)
            if keep_lane:
                new_yaw = self.orientation - np.deg2rad(angle)
                x_coord, y_coord = cos(new_yaw), sin(new_yaw)
                length = np.linalg.norm([x_coord, y_coord])
                new_driving_vector = [x_coord / length, y_coord / length]
                predicted_position = (self.vehicle_position[0] + new_driving_vector[0],
                                      self.vehicle_position[1] + new_driving_vector[1])
                if len(short_term_route) != 0:
                    predicted_vector = np.subtract(predicted_position, self.vehicle_position)
                    route_vector = np.subtract(short_term_route[0], self.vehicle_position)
                    cos_angle = np.dot(route_vector, predicted_vector) / np.linalg.norm(
                        route_vector) / np.linalg.norm(predicted_vector)
                    off_set_angle = np.rad2deg(np.arccos(cos_angle))
                    print(f'off_set_angle:{off_set_angle}')
                    if abs(off_set_angle) < 30:
                        short_term_route.insert(0, predicted_position)
                    print(f'step: {self.step_semantic}')
                    print(f'next: {short_term_route[0]}')
                else:
                    short_term_route.insert(0, predicted_position)
            if self.step_semantic % 10 == 0 and self.step_semantic < 10000:
                cv2.imwrite(f"/app/logs/img_{self.step_semantic}_highlighted.png", highlighted_img)
        return short_term_route
