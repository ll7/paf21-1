"""A route planner based on map and sensor data"""

from dataclasses import dataclass, field
from math import dist, sin, cos
from typing import List, Tuple, Dict

from cv2 import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg

from local_planner.vehicle_control import DrivingController
from local_planner.traffic_light_detection import TrafficLightDetector
from local_planner.lane_detection import LaneDetection
from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.stateMaschine.speed_state_machine import SpeedObservation, SpeedStateMachine, TrafficLightPhase
from local_planner.vehicle_control.vehicle import Vehicle


@dataclass
class RouteInfo(): # pylint: disable=too-many-locals
    # pylint: disable=too-many-instance-attributes
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""

    driving_control: DrivingController
    vehicle_vector: Tuple[float, float] = None
    last_vehicle_position: Tuple[float, float] = None
    vehicle_position: Tuple[float, float] = None
    global_route: List[Tuple[float, float]] = field(default_factory=list)
    cached_local_route: List[Tuple[float, float]] = field(default_factory=list)
    orientation: float = 0
    step_semantic: int = 0
    # velocity: float = 0
    vehicle: Vehicle = Vehicle()
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

    def compute_local_route(self) -> List[Tuple[float, float]]:
        """Modifies the global route based on sensor data
        to fit the current diving circumstances"""
        print(self.step_semantic)
        if self.vehicle_position is None or len(self.cached_local_route) == 0:
            return []
        self.step_semantic += 1

        # TODO: fix this!!! let the car either compute the velocity by itself or use a sensor
        #       don't assume that this function gets called 10 times per second
        if self.last_vehicle_position is not None:
            distance = dist(self.last_vehicle_position, self.vehicle_position)
            velocity = (distance/(1/10)) * 3.6
            self.vehicle.actual_velocity_mps = velocity

        self.last_vehicle_position = self.vehicle_position
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
        turned_on = False
        short_term_route = self.lane_keeping_assistant(images, short_term_route, turned_on)
        tl_state, dist_m = self.traffic_light_detec(images_list, turned_on_traffic_light_detection)
        print(tl_state == "Backside")
        if tl_state == "Backside":
            tl_state == "Green"
        # if tl_state == 'Red' and dist_m <= 50:
        #     print(f'traffic lights are red. initiate brake in {dist_m} meters')
        #     self.driving_control.update_target_velocity(dist_m, 0)
        # else:
        #     self.driving_control.update_target_velocity(dist_m, 10)
        speed_obs = SpeedObservation(tl_state, dist_next_obstacle_m = dist_m)
        self.speed_state_machine.update_state(speed_obs)
        return short_term_route

    def traffic_light_detec(self, images_list: Dict[str, np.ndarray],
                            turned_on_traffic_light_detection: bool) -> Tuple[TrafficLightPhase, float]:
        """set traffic lane color"""

        tl_color = 'Green'
        meters = 300
        if all(image is not None for image in images_list.values()) \
                and turned_on_traffic_light_detection:
            rgb_image = images_list['rgb'][:, :, :3]
            depth_image = images_list['depth']
            semantic_image = images_list['semantic'][:, :, :3]
            meters, tl_color, highlighted_img = self.traffic_light_detection. \
                detect_traffic_light(semantic_image, rgb_image, depth_image)
            if self.step_semantic % 10 == 0 and self.step_semantic < 10000:
                img_log_path = f"/app/logs/img_{self.step_semantic}_traffic_light.png"
                cv2.imwrite(img_log_path, highlighted_img)

        # TODO: return the traffic light's position in case something was found
        tl_phase = TrafficLightPhase.Green if tl_color == 'Green' else TrafficLightPhase.Red
        return tl_phase, meters

    def lane_keeping_assistant(self, images: SensorCameraPreprocessor,
                               short_term_route: List[Tuple[float, float]], turned_on: bool) \
                                   -> List[Tuple[float, float]]:
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
                    if abs(off_set_angle) < 30:
                        short_term_route.insert(0, predicted_position)
                else:
                    short_term_route.insert(0, predicted_position)
            if self.step_semantic % 10 == 0 and self.step_semantic < 0:
                cv2.imwrite(f"/app/logs/img_{self.step_semantic}_highlighted.png", highlighted_img)
        return short_term_route
