"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

from math import dist as euclid_dist, atan2, pi, degrees
from datetime import datetime
from typing import Tuple, List
from dataclasses import dataclass, field
# import numpy as np
import rospy

@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float

@dataclass
class DrivingController: # pylint: disable=too-many-instance-attributes
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""

    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    actual_velocity_mps: float = 0.0
    vehicle_pos: Tuple[float, float] = None
    vehicle_pos_timestamp: datetime = None
    vehicle_orientation_rad: float = None

    length_between_axles_m: float = 3.1
    last_steering_angle: float = 0.0
    current_wp_id: int = 0
    # max_steering_angle_rad: float = 0.5

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed"""
        self.route_waypoints = waypoints
        # add filter to exclude waypoints behind the vehicle
        rospy.loginfo(waypoints)

    def update_target_velocity(self, target_velocity_mps: float):
        """Update the route to be followed"""
        self.target_velocity_mps = target_velocity_mps

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float], orientation: float):
        """Update the vehicle's current position and estimate
        the actual velocity by computing the position / time diffs"""

        # rospy.loginfo(f'new_pos={vehicle_pos}')

        old_pos = self.vehicle_pos
        new_pos = vehicle_pos
        old_timestamp = self.vehicle_pos_timestamp
        new_timestamp = datetime.utcnow()

        if old_pos is not None:
            dist = euclid_dist(old_pos, new_pos)
            time = (new_timestamp - old_timestamp).total_seconds()
            self.actual_velocity_mps = dist / time

            # self.vehicle_orientation_rad = \
            #     DrivingController._vector_to_dir(old_pos, new_pos)
            # replace this with odometry and speedometer

        self.vehicle_pos = new_pos
        self.vehicle_pos_timestamp = new_timestamp
        self.vehicle_orientation_rad = orientation

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""

        # apply not overshooting max. steering angle here ...
        # steering_angle = self._compute_steering_angle()
        steering_angle = self._compute_steering_angle()
        self.last_steering_angle = steering_angle
        signal = DrivingSignal(steering_angle, self.target_velocity_mps)
        # rospy.loginfo(signal)
        return signal

    def _compute_steering_angle(self) -> float:
        # min_waypoints = 10
        # waypoints = self._pick_next_waypoints_ahead(min_waypoints)
        # rospy.loginfo(f'next waypoints: {waypoints}')
        # if len(waypoints) < min_waypoints:
        #     rospy.loginfo('insufficient waypoints')
        #     return 0.0

        if len(self.route_waypoints) == 0 or self.vehicle_orientation_rad is None:
            return 0.0

        aim_point = self.route_waypoints[self.current_wp_id]
        while euclid_dist(aim_point, self.vehicle_pos) < 7 \
                and self.current_wp_id < len(self.route_waypoints) - 1:
            self.current_wp_id += 1
            aim_point = self.route_waypoints[self.current_wp_id]

        diff = (aim_point[0] - self.vehicle_pos[0], aim_point[1] - self.vehicle_pos[1])
        angle = atan2(diff[1], diff[0] * -1)
        steer_angle = angle - self.vehicle_orientation_rad
        steer_angle = DrivingController._norm_angle(steer_angle)

        if self.target_velocity_mps > 0:
            rospy.loginfo(
                f'aim_point {aim_point}, vehicle_position {self.vehicle_pos} '
                + f'steer {degrees(steer_angle)}, '
                + f'orientation {degrees(self.vehicle_orientation_rad)} '
                + f'exp_orientation {degrees(angle)}')

        return steer_angle

    # def _compute_steering_angle(self) -> float:
    #     min_waypoints = 2
    #     waypoints = self._pick_next_waypoints_ahead(min_waypoints)
    #     if len(waypoints) < min_waypoints:
    #         rospy.loginfo('insufficient waypoints')
    #         return 0.0

    #     start = waypoints[0]
    #     curve_radius = DrivingController._compute_curvature_radius(
    #         start, waypoints[2], waypoints[4])
    #     phi = np.arctan(self.length_between_axles_m / curve_radius) * -1.0
    #     return phi

    # def _pick_next_waypoints_ahead(self, count: int):
    #     waypoints = self.route_waypoints
    #     if len(waypoints) == 0 or self.vehicle_orientation_rad is None:
    #         return []
    #     next_wp_id = DrivingController._get_next_waypoint(
    #         self.vehicle_pos, waypoints, self.vehicle_orientation_rad)
    #     if next_wp_id == -1:
    #         return []
    #     return waypoints[next_wp_id:next_wp_id+count] \
    #         if len(waypoints) > count else waypoints

    # @staticmethod
    # def _get_next_waypoint(vehicle_pos: Tuple[float, float],
    #                        waypoints: List[Tuple[float, float]],
    #                        vehicle_orientation: float) -> Tuple[float, float]:
    #     ids = range(len(waypoints))
    #     dirs = [DrivingController._vector_to_dir(vehicle_pos, waypoints[id]) for id in ids]
    #     same_dir = [id for id in ids if abs(dirs[id] - vehicle_orientation) < 0.5]
    #     same_dir_dists = [euclid_dist(vehicle_pos, waypoints[id]) for id in same_dir]
    #     next_id = np.argmin(same_dir_dists) if len(same_dir_dists) > 0 else -1
    #     if next_id != -1:
    #         rospy.loginfo(f'{vehicle_pos}, {waypoints[next_id]}')
    #     return next_id

    @staticmethod
    def _vector_to_dir(p_from: Tuple[float, float], p_to: Tuple[float, float]):
        angle = atan2(p_to[1] - p_from[1], p_to[0] - p_from[0])
        return DrivingController._norm_angle(angle)

    @staticmethod
    def _norm_angle(angle_rad: float) -> float:
        while angle_rad > pi:
            angle_rad -= 2.0 * pi
        while angle_rad < -pi:
            angle_rad += 2.0 * pi
        return angle_rad

    # @staticmethod
    # def _compute_curvature_radius(p_1: Tuple[float, float],
    #                               p_2: Tuple[float, float],
    #                               p_3: Tuple[float, float]) -> float:
    #     triangle_area = ((p_2[0] - p_1[0]) * (p_3[1] - p_1[1]) - \
    #                     (p_3[0] - p_1[0]) * (p_2[1] - p_1[1])) / 2.0
    #     menger = (4 * triangle_area) / \
    #             (euclid_dist(p_1, p_2) * euclid_dist(p_2, p_3) * euclid_dist(p_3, p_1))
    #     radius = abs(1.0 / (menger + 0.00001))
    #     return radius
