"""This module contains a state machine for handling maneuver states."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List
from local_planner.core import Vehicle


class ManeuverState(IntEnum):
    """Represents the maneuver state."""
    KEEP_LANE = 0
    LEFT_CHANGE = 1
    RIGHT_CHANGE = 2
    CALC_NEW_ROUTE = 3


@dataclass
class ManeuverObservation():
    """Represents context information used for maneuver state transitions."""
    actual_track: int = 1
    left_lane_free: bool = False
    targeted_track: int = 1
    right_lane_free: bool = False
    distance_next_turn_m: float = 999
    lanes_at_turn: List[int] = field(default_factory=list)
    speed_limit_ms: int = 50
    possible_lanes: List[int] = field(default_factory=list)
    blocking_object: bool = False
    new_route_is_ready: bool = False


@dataclass
class ManeuverStateMachine:
    """Represents a state machine used for maneuver state transitioning."""
    vehicle: Vehicle
    states: List[ManeuverState] = field(default_factory=lambda: [e for e in ManeuverState]) 
    current_state: ManeuverState = ManeuverState.KEEP_LANE


    # hyper-params controlling the state machine's behavior (potentially trainable)
    distance_no_more_overtake_m: float = 100
    distance_no_more_lane_swap_m: float = 100
    distance_failed_lane_swap_m: float = 10

    def update_state(self, observation: ManeuverObservation):
        """Update the machine state given a maneuver observation"""
        # Popssible lane to turn?
        possible_turnlane = observation.actual_track in observation.lanes_at_turn
        need_to_swap_lane_count = 0
        if not possible_turnlane:
            need_to_swap_lane_count = observation.lanes_at_turn[0] - observation.actual_track

        track_index = observation.possible_lanes.index(observation.actual_track)

        # Distance possible for takeover
        possible_takeover = observation.distance_next_turn_m >= self.distance_no_more_overtake_m

        # Distance possible for lane swap
        possible_laneswap = observation.distance_next_turn_m >= self.distance_no_more_lane_swap_m
        maneuver_to_late = observation.distance_next_turn_m <= self.distance_failed_lane_swap_m

        if self.current_state == ManeuverState.KEEP_LANE:
            # Frei Fahrt und auf richtigen Spur oder beide spuren sind besetzt
            if possible_turnlane and (not observation.blocking_object or not possible_laneswap) \
                    or not (observation.left_lane_free or observation.right_lane_free):
                observation.targeted_track = observation.actual_track
            # ToDo Rechtsfahrgebot??
            # Left Turn -> Ueberholen, Spurwechsel (turn)
            elif (observation.left_lane_free and not maneuver_to_late and \
                  (possible_takeover or (need_to_swap_lane_count < 0))):
                self.current_state = ManeuverState.LEFT_CHANGE
                observation.targeted_track = observation.possible_lanes[track_index - 1]
                # todo update actual lane
            elif (observation.right_lane_free and not maneuver_to_late and \
                    (possible_takeover or (need_to_swap_lane_count > 0))):
                self.current_state = ManeuverState.RIGHT_CHANGE
                observation.targeted_track = observation.possible_lanes[track_index + 1]
            else:
                self.current_state = ManeuverState.CALC_NEW_ROUTE

        elif self.current_state in [ManeuverState.LEFT_CHANGE, ManeuverState.RIGHT_CHANGE]:
            if observation.actual_track == observation.targeted_track:
                self.current_state = ManeuverState.KEEP_LANE

        elif self.current_state == ManeuverState.CALC_NEW_ROUTE:
            if observation.new_route_is_ready:
                self.current_state = ManeuverState.KEEP_LANE
        else:
            self.current_state = ManeuverState.CALC_NEW_ROUTE
