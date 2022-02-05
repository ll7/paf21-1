"""This module contains a state machine for handling speed states."""

from dataclasses import dataclass
from enum import IntEnum
from local_planner.core import Vehicle


class SpeedState(IntEnum):
    """Representing all possible states of the speed state machine."""
    KEEP = 0
    ACCEL = 1
    BRAKE = 2


class TrafficLightPhase(IntEnum):
    """Representing traffic light phases"""
    BACKSIDE = 0
    GREEN = 1
    RED = 2
    YELLOW = 3


@dataclass
class TrafficLightInfo:
    """Representing information on a recently detected traffic light"""
    phase: TrafficLightPhase = TrafficLightPhase.GREEN
    distance: float = 1000


@dataclass
class SpeedObservation:
    """Representing context information used for speed state transitioning."""
    tl_phase: TrafficLightPhase = TrafficLightPhase.GREEN
    is_trajectory_free: bool = True
    dist_next_traffic_light_m: float = 999
    dist_next_obstacle_m: float = 999
    detected_speed_limit: int = None
    obj_speed_ms: float = float('inf')


@dataclass
class SpeedStateMachine:
    """Representing a state machine for speed control decision-making."""
    vehicle: Vehicle
    current_state: SpeedState = SpeedState.ACCEL
    target_speed_mps: float = 0
    legal_speed_limit_mps: float = 50 / 3.6
    speed_offset_up_mps: float = 5.0 / 3.6
    speed_offset_down_mps: float = 3.0 / 3.6
    count: int = 0

    def update_state(self, obs: SpeedObservation):
        """Update the speed state machine with a new observation"""
        # WARNING: only uncomment when intending to ignore traffic lights
        # obs.tl_phase = TrafficLightPhase.GREEN
        if obs.detected_speed_limit is not None:
            self.legal_speed_limit_mps = obs.detected_speed_limit / 3.6

        # TODO: fix issue with traffic lights being considered when car spawns in fron of red lights
        #       even though the traffic lights are still far away such that the car should approach

        # """Update the machine's state given a speed observation."""
        if self.current_state == SpeedState.ACCEL:
            self._handle_accel(obs)
        elif self.current_state == SpeedState.KEEP:
            self._handle_keep(obs)
        elif self.current_state == SpeedState.BRAKE:
            self._handle_brake(obs)
        else:
            raise ValueError(f'Unsupported speed state {self.current_state}!')

        # if self.count % 10 == 0:
        #     print(f'speed state: {self.current_state}, target speed: {self.target_speed_mps:.2f},',
        #         f'legal speed: {self.legal_speed_limit_mps:.2f}, obs speed {obs.obj_speed_ms:.2f},'
        #         f'actual speed: {self.vehicle.actual_velocity_mps:.2f}')
        # self.count += 1

    def _is_in_speed_tolerance(self, speed_limit: float):
        speed_diff = self.vehicle.actual_velocity_mps - speed_limit
        return -self.speed_offset_down_mps <= speed_diff <= self.speed_offset_up_mps

    def _is_moving_object(self, obj_speed_mps: float):
        return obj_speed_mps > 10 / 3.6

    def _handle_keep(self, obs: SpeedObservation):
        needs_brake, target_speed = self._is_brake_required(obs)
        reached_target_speed = self._is_in_speed_tolerance(target_speed)

        if reached_target_speed:
            return

        if needs_brake:
            self.current_state = SpeedState.BRAKE
            self.target_speed_mps = target_speed
            return

        self.current_state = SpeedState.ACCEL
        self.target_speed_mps = target_speed

    def _handle_accel(self, obs: SpeedObservation):
        needs_brake, target_speed = self._is_brake_required(obs)
        reached_target_speed = self._is_in_speed_tolerance(target_speed)

        keep_accelerating = not reached_target_speed and not needs_brake
        if keep_accelerating:
            return

        if needs_brake:
            self.current_state = SpeedState.BRAKE
            self.target_speed_mps = target_speed
            return

        if reached_target_speed:
            self.current_state = SpeedState.KEEP
            self.target_speed_mps = target_speed

    def _handle_brake(self, obs: SpeedObservation):
        needs_brake, target_speed = self._is_brake_required(obs)
        reached_target_speed = self._is_in_speed_tolerance(target_speed)

        if needs_brake:
            return

        if reached_target_speed:
            self.current_state = SpeedState.KEEP
            self.target_speed_mps = target_speed
            return

        self.target_speed_mps = target_speed
        self.current_state = SpeedState.ACCEL

    def _is_brake_required(self, obs: SpeedObservation):
        tl_wait_time_s = self._time_until_brake(obs.dist_next_traffic_light_m, 0) \
                         if obs.tl_phase == TrafficLightPhase.RED else float('inf')

        obj_wait_time_s = self._time_until_brake(obs.dist_next_obstacle_m, obs.obj_speed_ms)

        crit_wait_time_s = min(tl_wait_time_s, obj_wait_time_s)
        target_speed = 0 if tl_wait_time_s < obj_wait_time_s else obs.obj_speed_ms

        needs_brake = crit_wait_time_s <= self.vehicle.vehicle_reaction_time_s
        return needs_brake, target_speed

    def _time_until_brake(self, distance_m: float, target_velocity: float = 0) -> float:
        """Compute the braking distance and based on that the time until brake.
        In case this function returns a negative value, it means that it's already
        too late to brake, so you need to launch an emergency protocol"""

        accel_mps2 = self.vehicle.base_brake_mps2

        if distance_m < 0:
            raise ValueError('Negative distance is not allowed')
        if self.vehicle.actual_velocity_mps < 0 or target_velocity < 0:
            raise ValueError('Negative velocity is not allowed')
        if accel_mps2 >= 0:
            raise ValueError('Positive acceleration won\'t brake')

        maneuver_time_s = (target_velocity - self.vehicle.actual_velocity_mps) / accel_mps2
        braking_dist = self.vehicle.actual_velocity_mps * maneuver_time_s + \
                       accel_mps2 * maneuver_time_s ** 2 / 2

        object_offset = 7
        linear_dist = distance_m - object_offset - braking_dist
        time_until_brake = linear_dist / self.vehicle.actual_velocity_mps \
                           if self.vehicle.actual_velocity_mps > 0 else 999
        return time_until_brake

    def get_target_speed(self) -> float:
        """Retrieve the currently suggested target speed."""
        action = self.current_state
        if action == SpeedState.ACCEL:
            self.target_speed_mps = self.legal_speed_limit_mps
        elif action == SpeedState.BRAKE:
            pass
            # self.target_speed_mps = 0
        elif action == SpeedState.KEEP:
            pass
            # self.target_speed_mps = self.vehicle.actual_velocity_mps
        else:
            raise ValueError(f'Unknown speed state {self.current_state} encountered!')
        return self.target_speed_mps
