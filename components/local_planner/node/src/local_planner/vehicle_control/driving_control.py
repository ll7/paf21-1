"""Module for transitioning a fine-grained, idealistic route
and other driving metadata into actionable driving signals"""

import sys
from typing import Tuple, List
from dataclasses import dataclass, field
from math import dist, atan, cos

from local_planner.core import Vehicle, geometry


@dataclass
class DrivingSignal:
    """Data class representing a driving signal"""
    steering_angle_rad: float
    target_velocity_mps: float


@dataclass
class DrivingController:  # pylint: disable=too-many-instance-attributes
    """A class for processing waypoints and driving metadata into
    actionable driving signals regarding velocity and steering control"""
    vehicle: Vehicle
    cached_wp: List[Tuple[float, float]] = field(default_factory=list)
    route_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    target_velocity_mps: float = 0.0
    target_distance_m: float = 0.0
    initial_vehicle_pos_set: bool = False
    steering_angle : float = 0.0

    def update_route(self, waypoints: List[Tuple[float, float]]):
        """Update the route to be followed and cache first waypoint"""
        #print("Route Update")
        if waypoints:
            if not self.initial_vehicle_pos_set:
                self.initial_vehicle_pos_set = True
                self.cached_wp.insert(0, self.vehicle.pos)
            #print("IF")
            #print(waypoints[0])
            #print(self.cached_wp[0])
            #print(waypoints[0] != self.cached_wp[0])
            if waypoints[0] != self.cached_wp[0]:
                #print("cached_wp")
                #print(self.cached_wp)
                #print("waypoints")
                #print(waypoints)
                self.cached_wp.insert(0, waypoints[0])

        self.route_waypoints = waypoints

    def update_target_velocity(self, velocity_mps: float):
        """Update vehicle's velocity"""
        target_velocity_mps = velocity_mps
        radius = geometry.approx_curvature_radius()

        current_velocity_mps = self.vehicle.actual_velocity_mps
        print('Actual Velocity : ', current_velocity_mps)
        # OPTION 1 :
        self.target_velocity_mps = abs(target_velocity_mps * cos(self.steering_angle))

        print('target velocity :', self.target_velocity_mps )
        # print('route_waypoints: ', self.route_waypoints)
        #print('vehicle pos :', self.vehicle.pos)
        # self.target_distance_m = target_distance_m
        return self.target_velocity_mps
       

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.vehicle.move(vehicle_pos)

    def update_vehicle_state(self, position: Tuple[float, float], velocity: float):
        """Update the vehicle's positional and velocity values"""
        self.vehicle.actual_velocity_mps = velocity
        self.vehicle.pos = position

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.vehicle.orientation_rad = orientation

    def next_signal(self) -> DrivingSignal:
        """Compute the next driving signal to make the
        vehicle follow the suggested ideal route"""
        self.steering_angle = self._compute_steering_angle()
        signal = DrivingSignal(self.steering_angle, self.target_velocity_mps)
        return signal

    def _compute_steering_angle(self) -> float:
        if not self._can_steer():
            return 0.0

        # aim_point = self._get_aim_point()
        # self.vehicle.steer_towards(aim_point)
        steering_angle = self.stanley_method()
        self.vehicle.set_steering_angle(steering_angle)
        print('steering angle : ', steering_angle)

        return steering_angle

    def _can_steer(self):
        return len(self.route_waypoints) > 0 \
               and self.vehicle.orientation_rad \
               and self.vehicle.pos

    def _get_aim_point(self):
        return self.route_waypoints[0]

    def stanley_method(self) -> float:
        """Implementation of Stanley Controller"""

        # Vehicle Position
        pos = self.vehicle.pos

        # calc nearest and Second-Nearest Waypoint
        first_wp_idx = -1
        first_wp_dist = sys.maxsize
        second_wp_idx = -1
        second_wp_dist = sys.maxsize
        enumerator = 0
        for point in self.cached_wp:

            if point is None:
                break
            if dist(pos, point) < first_wp_dist:
                second_wp_idx = first_wp_idx
                second_wp_dist = first_wp_dist
                first_wp_idx = enumerator
                first_wp_dist = dist(pos, point)
            else:
                if dist(pos, point) < second_wp_dist:
                    second_wp_idx = enumerator
                    second_wp_dist = dist(pos, point)
            enumerator += 1

        # Trajectory Direction
        orientation: int
        traj_direct: Tuple
        if first_wp_idx < second_wp_idx:
            traj_direct = \
                geometry.points_to_vector(self.cached_wp[second_wp_idx],
                                          self.cached_wp[first_wp_idx])
            orientation = (traj_direct[0] * (pos[0] - self.cached_wp[second_wp_idx][0])) - \
                          (traj_direct[1] * (pos[1] - self.cached_wp[second_wp_idx][1]))
        else:
            traj_direct = \
                geometry.points_to_vector(self.cached_wp[first_wp_idx],
                                          self.cached_wp[second_wp_idx])
            orientation = (traj_direct[0] * (pos[0] - self.cached_wp[first_wp_idx][0])) - \
                          (traj_direct[1] * (pos[1] - self.cached_wp[first_wp_idx][1]))

        traj_orientation = geometry.vector_to_dir(traj_direct)

        # Controller Settings
        k = 0.3
        k_s = 1

        # Calculate lateral error

        #print("Vehicle Orientation")
        #print(self.vehicle.orientation_rad)
        #print("Traj_orientation")
        #print(traj_orientation)
        #print("Velocity")
        #print(self.vehicle.actual_velocity_mps)
        #print("first_wp_dist")
        #print(first_wp_dist)

        heading_error = traj_orientation - self.vehicle.orientation_rad
        if orientation > 0:
            cross_track_error = atan(k * first_wp_dist / (k_s + self.vehicle.actual_velocity_mps))
        else:
            cross_track_error = atan(k * (-first_wp_dist) / (k_s + self.vehicle.actual_velocity_mps))

        #print("heading_error")
        #print(heading_error)
        #print(orientation)
        #print("cross_track_error")
        #print(cross_track_error)
        #print(self.vehicle.steering_angle)
        #print("cached_wp")
        #print(self.cached_wp)

        return heading_error + cross_track_error
