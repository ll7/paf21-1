"""Builds our Statemachine"""
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List
from local_planner.preprocessing import SingletonMeta
from local_planner.vehicle_control.vehicle import Vehicle

class ManeuverState(IntEnum):
    Keep_Lane = 0
    Left_Change = 1
    Right_Change = 2
    Calc_New_Route = 3

@dataclass
class ManeuverObservation():
        
    actual_track: int = 1
    left_lane_free: bool = False
    targeted_track: int = 1
    right_lane_free: bool = False
    distance_next_turn_m: float = 999
    lanes_at_turn: List[int] = field(default_factory=list)
    speed_limit_ms: int = 50
        
    # All possible drivable lanes including opposite track e.G. [-1,1,2]
    
    possible_lanes: List[int] = field(default_factory=list)
    blocking_object: bool = False
    new_route_is_ready: bool = False

@dataclass
class ManeuverStateMachine(metaclass=SingletonMeta):
    """Doc"""

    states: List[ManeuverState] = field(default_factory=lambda: [e.value for e in ManeuverState]) 
    current_state: ManeuverState = ManeuverState.Keep_Lane

    # Variable to detect if it is reasonable to do maneuver

    vehicle: Vehicle = Vehicle()

    # Statemachine Ruleset Params

    distance_no_more_overtake_m: float = 100
    distance_no_more_lane_swap_m: float = 100
    distance_failed_lane_swap_m: float = 10

    def update_state(self, observation: ManeuverObservation):
        """Doc"""
        # Popssible lane to turn?
        possible_turnlane = observation.actual_track in observation.lanes_at_turn
        need_to_swap_lane_count = 0
        if not possible_turnlane:
            need_to_swap_lane_count = observation.lanes_at_turn[0]- observation.actual_track


        track_index = observation.possible_lanes.index(observation.actual_track)

        # Distance possible for takeover
        possible_takeover = observation.distance_next_turn_m >= self.distance_no_more_overtake_m

        # Distance possible for lane swap
        possible_laneswap = observation.distance_next_turn_m >= self.distance_no_more_lane_swap_m

        maneuver_to_late = observation.distance_next_turn_m <= self.distance_failed_lane_swap_m


        if self.current_state == ManeuverState.Keep_Lane:
            # Frei Fahrt und auf richtigen Spur oder beide spuren sind besetzt
            if possible_turnlane and (not observation.blocking_object or not possible_laneswap) \
                    or not (observation.left_lane_free or observation.right_lane_free):
                observation.targeted_track = observation.actual_track
                pass
            # ToDo Rechtsfahrgebot??
            # Left Turn -> Ueberholen, Spurwechsel (turn)
            elif (observation.left_lane_free and not maneuver_to_late and
                  (possible_takeover or
                   (need_to_swap_lane_count < 0))):
                self.current_state = ManeuverState.Left_Change
                observation.targeted_track = observation.possible_lanes[track_index - 1]
                # todo update actual lane
            elif (observation.right_lane_free and not maneuver_to_late and
                      (possible_takeover or
                       (need_to_swap_lane_count > 0))):
                self.current_state = ManeuverState.Right_Change
                observation.targeted_track = observation.possible_lanes[track_index + 1]
            else:
                self.current_state = ManeuverState.Calc_New_Route

        elif self.current_state == ManeuverState.Left_Change or self.current_state == ManeuverState.Right_Change:
            if observation.actual_track == observation.targeted_track:
                self.current_state = ManeuverState.Keep_Lane

        elif self.current_state == ManeuverState.Calc_New_Route:
            if observation.new_route_is_ready:
                self.current_state = ManeuverState.Keep_Lane
        else: 
            self.current_state = ManeuverState.Calc_New_Route
