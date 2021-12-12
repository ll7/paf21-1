"""A route planner based on map and sensor data"""

from math import dist
import math
from typing import List, Tuple
from cv2 import cv2

import rospy
import numpy as np
# from numpy.core.fromnumeric import argsort
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Imu as ImuMsg
from nav_msgs.msg import Odometry as OdometryMsg

from local_planner.lane_detection import LaneDetection
from local_planner.preprocessing import SensorCameraPreprocessor
from local_planner.preprocessing import SingletonMeta
# from dataclasses import field

class RouteInfo(metaclass=SingletonMeta): # pylint: disable=too-many-locals
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""
    vehicle_vector: Tuple[float, float] = None
    vehicle_position: Tuple[float, float] = None
    global_route: List[Tuple[float, float]] = []
    cached_local_route: List[Tuple[float, float]] = []
    vehicle_yaw: float = 0
    step_semantic: int = 0
    lane_detect_config: str = "/app/src/local_planner/config/config_lane_detection.yml"
    lane_detection = LaneDetection = LaneDetection(lane_detect_config)

    def update_vehicle_vector(self, msg: ImuMsg):
        """Calculates the vektor the car is currently driving"""

        quaternion = msg.orientation
        self.vehicle_yaw = math.atan2(2.0 *
                                      (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                                      1.0 - 2.0 *
                                      (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

        x_coord, y_coord = math.cos(self.vehicle_yaw), math.sin(self.vehicle_yaw)
        length = np.linalg.norm([x_coord, y_coord])
        if length > 0:
            self.vehicle_vector = [x_coord / length, y_coord / length]
        else:
            self.vehicle_vector = [x_coord, y_coord]

    def update_gps(self, msg: OdometryMsg):
        """Convert a ROS message into the vehicle position"""
        pos = msg.pose.pose.position
        self.vehicle_position = [pos.x, pos.y]

    def update_global_route(self, msg):
        """Update the global route to follow"""
        self.global_route = msg
        if self.global_route is None:
            self.cached_local_route = []

    def compute_local_route(self):
        """Modifies the global route based on sensor data
        to fit the current diving circumstances"""
        self.step_semantic += 1
        short_term_route = []

        # interpolate the route such that the route points are closely aligned
        # filter the parts of the global route of the near future
        images = SensorCameraPreprocessor()
        point_counts_as_done = 0.2
        enumerator = 0
        for point in self.cached_local_route:
            if dist(point, self.vehicle_position) > point_counts_as_done:
                break
            enumerator += 1
        self.cached_local_route = self.cached_local_route[enumerator:]

        # neighbour_ids = argsort([dist(p, self.vehicle_position) for p in self.cached_local_route])
        # self.cached_local_route = self.global_route[max(neighbour_ids[0], neighbour_ids[1]):]
        short_term_route = self.cached_local_route[:min(50, len(self.cached_local_route))]
        if images.semantic_image is not None:
            image = images.semantic_image[:, :, :3] # cut off alpha channel
            highlighted_img, keep_lane, angle = self.lane_detection.detect_lanes(image)
            if keep_lane:
                new_yaw = self.vehicle_yaw - np.deg2rad(angle)
                x_coord, y_coord = math.cos(new_yaw), math.sin(new_yaw)
                length = np.linalg.norm([x_coord, y_coord])
                new_driving_vector = [x_coord / length, y_coord / length]
                predicted_position = (self.vehicle_position[0] + new_driving_vector[0],
                                      self.vehicle_position[1] + new_driving_vector[1])
                short_term_route.insert(0, predicted_position)
            rospy.loginfo(f'step: {self.step_semantic}')
            if self.step_semantic % 10 == 0:
                cv2.imwrite(f"/app/logs/img_{self.step_semantic}_highlighted.png", highlighted_img)
        return short_term_route
