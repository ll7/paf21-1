"""This module contains a state machine for handling speed states."""

from dataclasses import dataclass
from enum import IntEnum
from local_planner.core import Vehicle


class SpeedState(IntEnum):
    """Representing all possible states of the speed state machine."""
    Keep = 0
    Stop = 1
    Accel = 2
    Brake = 3


class TrafficLightPhase(IntEnum):
    """Representing the phases of a traffic light."""
    Red = 0
    Green = 1


@dataclass
class SpeedObservation:
    """Representing context information used for speed state transitioning."""
    tl_phase: TrafficLightPhase = TrafficLightPhase.Green
    is_junction_free: bool = True
    dist_next_obstacle_m: float = 1000
    detected_speed_limit: int = None


@dataclass
class SpeedStateMachine:
    """Representing a state machine for speed control decision-making."""
    vehicle: Vehicle
    current_state: SpeedState = SpeedState.Stop
    target_speed_mps: float = 0
    legal_speed_limit_mps: float = 30 / 3.6

    def update_state(self, obs: SpeedObservation):
        """Update the machine's state given a speed observation."""
        if self.current_state == SpeedState.Stop:
            self._handle_stop(obs)
        elif self.current_state == SpeedState.Accel:
            self._handle_accel(obs)
        elif self.current_state == SpeedState.Keep:
            self._handle_keep(obs)
        elif self.current_state == SpeedState.Brake:
            self._handle_brake(obs)
        else:
            raise ValueError(f'Unsupported speed state {self.current_state}!')

        print(self.target_speed_mps, obs.is_junction_free, obs.tl_phase)
        print(self.vehicle.actual_velocity_mps, self.legal_speed_limit_mps)
        print(self.current_state)

    def _handle_stop(self, obs: SpeedObservation):
        if obs.tl_phase == TrafficLightPhase.Green and obs.is_junction_free:
            self.current_state = SpeedState.Accel

    def _handle_keep(self, obs: SpeedObservation):
        if self.legal_speed_limit_mps < self.vehicle.actual_velocity_mps or not obs.is_junction_free \
                or (obs.tl_phase == TrafficLightPhase.Red and self._is_brake_required(obs)):
            self.current_state = SpeedState.Brake
        elif self.legal_speed_limit_mps > self.vehicle.actual_velocity_mps \
                and obs.is_junction_free and obs.tl_phase == TrafficLightPhase.Green:
            self.current_state = SpeedState.Accel

    def _handle_accel(self, obs: SpeedObservation):
        if self.legal_speed_limit_mps < self.vehicle.actual_velocity_mps and self._is_brake_required(obs, self.legal_speed_limit_mps) or not obs.is_junction_free \
                or (obs.tl_phase == TrafficLightPhase.Red and self._is_brake_required(obs, 0)):
            self.current_state = SpeedState.Brake
        elif self.legal_speed_limit_mps == self.vehicle.actual_velocity_mps:
            self.current_state = SpeedState.Keep

    def _handle_brake(self, obs: SpeedObservation):
        if self.legal_speed_limit_mps > self.vehicle.actual_velocity_mps \
                and obs.is_junction_free and obs.tl_phase == TrafficLightPhase.Green:
            self.current_state = SpeedState.Accel
        elif self.legal_speed_limit_mps == self.vehicle.actual_velocity_mps:
            self.current_state = SpeedState.Keep
        elif self.vehicle.actual_velocity_mps == 0:
            self.current_state = SpeedState.Stop

    def _is_brake_required_distance(self, obs: SpeedObservation):
        velocity_kmh = self.vehicle.actual_velocity_mps * 3.6
        stopping_distance = (velocity_kmh / 10)**2
        reaction_distance = self.vehicle.vehicle_reaction_time_s * self.vehicle.actual_velocity_mps
        halting_distance = stopping_distance + reaction_distance
        print(halting_distance, obs.dist_next_obstacle_m)
        return halting_distance >= obs.dist_next_obstacle_m

    def _is_brake_required(self, obs: SpeedObservation, speed):
        wait_time_s = self._time_until_brake(obs.dist_next_obstacle_m, speed)
        return wait_time_s <= self.vehicle.vehicle_reaction_time_s

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
        print('braking distance', braking_dist)
        object_offset = 10
        time_until_brake = (distance_m - object_offset - braking_dist) / self.vehicle.actual_velocity_mps \
            if self.vehicle.actual_velocity_mps > 0 else 0
        print('time until brake', time_until_brake)
        return time_until_brake

    def get_target_speed(self) -> float:
        action = self.current_state
        if action == SpeedState.Accel:
            self.target_speed_mps = self.legal_speed_limit_mps
        elif action == SpeedState.Brake:
            self.target_speed_mps = 0
        else:
            self.target_speed_mps = self.vehicle.actual_velocity_mps
        return self.target_speed_mps
