"""This module contains a state machine for handling speed states."""

from dataclasses import dataclass
from enum import IntEnum
import numpy as np
from vehicle_control.core import Vehicle


class SpeedState(IntEnum):
    """Representing all possible states of the speed state machine."""
    KEEP = 0
    ACCEL = 1
    BRAKE = 2


class TrafficLightPhase(IntEnum):
    """Representing the traffic light phases"""
    BACKSIDE = 0
    GREEN = 1
    RED = 2
    YELLOW = 3


@dataclass
class TrafficLightInfo:
    """Representing information of a recently detected traffic light"""
    phase: TrafficLightPhase = TrafficLightPhase.GREEN
    distance: float = 1000
    accuracy: float = 0.0


@dataclass
class SpeedObservation:
    """Representing context information used for speed state transitioning."""
    tl_phase: TrafficLightPhase = TrafficLightPhase.GREEN
    is_trajectory_free: bool = True
    dist_next_traffic_light_m: float = 999
    dist_next_obstacle_m: float = 999
    detected_speed_limit: int = None
    obj_speed_ms: float = 500
    dist_next_curve: float = 999
    curve_target_speed: float = 50
    dist_next_stop_m: float = 999
    object_type: str = ''


@dataclass
class SpeedStateMachine:
    """Representing a state machine for speed control decision-making."""
    vehicle: Vehicle
    racing_mode: bool = False
    current_state: SpeedState = SpeedState.ACCEL
    target_speed_mps: float = 0
    legal_speed_limit_mps: float = 50 / 3.6
    speed_offset_up_mps: float = 0 / 3.6
    speed_offset_down_mps: float = 3.0 / 3.6
    count: int = 0

    def update_state(self, obs: SpeedObservation):
        """Update the speed state machine given a new observation
        """
        # WARNING: only uncomment when intending to ignore traffic lights
        # obs.tl_phase = TrafficLightPhase.GREEN

        # print('TL phase : {}, TL in {}, Curve Speed : {}, Next Curve in {}'.
        # format(obs.tl_phase, obs.dist_next_traffic_light_m,  obs.curve_target_speed, obs.dist_next_curve))
        if obs.detected_speed_limit is not None:
            self.legal_speed_limit_mps = obs.detected_speed_limit / 3.6

        if not self.vehicle.is_ready:
            return

        if self.racing_mode:
            obs.dist_next_traffic_light_m = 999
            self.legal_speed_limit_mps = 120/3.6
            if obs.object_type == 'pedestrian':
                obs.dist_next_obstacle_m = 50

        if self.current_state == SpeedState.ACCEL:
            self._handle_accel(obs)
        elif self.current_state == SpeedState.KEEP:
            self._handle_keep(obs)
        elif self.current_state == SpeedState.BRAKE:
            self._handle_brake(obs)
        else:
            raise ValueError(f'Unsupported speed state {self.current_state}!')

    def _is_in_speed_tolerance(self, target_speed: float):
        """checks if the vehicles velocity is in a certain zone around the targeted speed"""
        speed_diff = self.vehicle.velocity_mps - target_speed
        return -self.speed_offset_down_mps <= speed_diff <= self.speed_offset_up_mps

    def _handle_keep(self, obs: SpeedObservation):
        """transition function if previous state is KEEP"""
        needs_brake, target_speed = self._is_brake_required(obs)
        reached_target_speed = self._is_in_speed_tolerance(target_speed)

        if reached_target_speed:
            self.target_speed_mps = target_speed
            return

        if needs_brake:
            self.current_state = SpeedState.BRAKE
            self.target_speed_mps = 0
            return

        self.current_state = SpeedState.ACCEL
        self.target_speed_mps = target_speed

    def _handle_accel(self, obs: SpeedObservation):
        """transition function if previous state is ACCEL"""
        needs_brake, target_speed = self._is_brake_required(obs)
        reached_target_speed = self._is_in_speed_tolerance(target_speed)
        keep_accelerating = not reached_target_speed and not needs_brake

        if keep_accelerating:
            self.target_speed_mps = target_speed
            return

        if needs_brake:
            self.current_state = SpeedState.BRAKE
            self.target_speed_mps = 0
            return

        if reached_target_speed:
            self.current_state = SpeedState.KEEP
            self.target_speed_mps = target_speed

    def _handle_brake(self, obs: SpeedObservation):
        """transition function if previous state is BRAKE"""
        needs_brake, target_speed = self._is_brake_required(obs)
        reached_target_speed = self._is_in_speed_tolerance(target_speed)

        if reached_target_speed:
            self.current_state = SpeedState.KEEP
            self.target_speed_mps = target_speed
            return

        if needs_brake:
            self.target_speed_mps = 0
            return

        self.target_speed_mps = target_speed
        self.current_state = SpeedState.ACCEL

    def _is_brake_required(self, obs: SpeedObservation):
        """Calculates the most critical object and if we need to brake
        Output: is_brake_required: bool, targeted speed: float
        """
        safety_time_s = 2.0
        curve_offset_m = 2.0
        phases_brake = [TrafficLightPhase.RED, TrafficLightPhase.YELLOW]
        tl_wait_time_s = self._time_until_brake(obs.dist_next_traffic_light_m, 0) \
            if obs.tl_phase in phases_brake else 999
        stop_sign_time = self._time_until_brake(obs.dist_next_stop_m, 0) \
            if self.vehicle.velocity_mps > 1 else 999
        obj_wait_time_s = self._time_until_brake(obs.dist_next_obstacle_m, obs.obj_speed_ms)
        distance_m = obs.dist_next_obstacle_m
        accel_mps2 = self.vehicle.meta.base_brake_mps2
        maneuver_time_standing = (0 - self.vehicle.velocity_mps) / accel_mps2
        braking_dist_standing = self.vehicle.velocity_mps * maneuver_time_standing + \
                                accel_mps2 * maneuver_time_standing ** 2 / 2
        if braking_dist_standing > distance_m:
            obj_wait_time_s = -1

        curve_wait_time_s = self._time_until_brake(obs.dist_next_curve, obs.curve_target_speed,
                                                   curve_offset_m)
        speed_tl = 0 if tl_wait_time_s <= safety_time_s else self.legal_speed_limit_mps
        speed_stop = 0 if stop_sign_time <= safety_time_s else self.legal_speed_limit_mps

        obs.obj_speed_ms = obs.obj_speed_ms if obj_wait_time_s <= safety_time_s \
            else self.legal_speed_limit_mps
        obs.curve_target_speed = obs.curve_target_speed if curve_wait_time_s <= safety_time_s \
            else self.legal_speed_limit_mps

        wait_times = [curve_wait_time_s, tl_wait_time_s, obj_wait_time_s, stop_sign_time]

        target_speeds = [obs.curve_target_speed, speed_tl, obs.obj_speed_ms, speed_stop]

        crit_id = np.argmin(wait_times)

        crit_wait_time_s, target_speed = wait_times[crit_id], target_speeds[crit_id]

        needs_brake = crit_wait_time_s <= self.vehicle.meta.vehicle_reaction_time_s
        return needs_brake, min(target_speed, self.legal_speed_limit_mps)

    def _time_until_brake(self, distance_m: float, target_velocity: float = 0,
                          object_offset: float = 10.0) -> float:
        """Compute the braking distance and based on that the time until brake.
        In case this function returns a negative value, it means that it's already
        too late to brake, so you need to launch an emergency protocol"""

        accel_mps2 = self.vehicle.meta.base_brake_mps2

        if distance_m < 0:
            raise ValueError('Negative distance is not allowed')
        if self.vehicle.velocity_mps < 0 or target_velocity < 0:
            raise ValueError('Negative velocity is not allowed')
        if accel_mps2 >= 0:
            raise ValueError('Positive acceleration won\'t brake')

        maneuver_time_s = (target_velocity - self.vehicle.velocity_mps) / accel_mps2
        braking_dist = self.vehicle.velocity_mps * maneuver_time_s + \
                       accel_mps2 * maneuver_time_s ** 2 / 2

        # object_offset = max(target_velocity * 1.8, 4)
        # object_offset = 7
        linear_dist = distance_m - object_offset - braking_dist
        time_until_brake = linear_dist / self.vehicle.velocity_mps \
            if self.vehicle.velocity_mps > 0 else 999
        #print('time till brake', time_until_brake)
        return time_until_brake

    def get_target_speed(self) -> float:
        """Retrieve the currently suggested target speed."""
        return self.target_speed_mps
