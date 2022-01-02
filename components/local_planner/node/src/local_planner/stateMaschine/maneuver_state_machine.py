"""Builds our Statemachine"""
from local_planner.preprocessing import SingletonMeta
from local_planner.vehicle_control.vehicle import Vehicle


class ManeuverStateMachine(metaclass=SingletonMeta):
    """Doc"""
    def __init__(self):
        """Doc"""
        self.states = ['Keep_lane', 'Left_Change', 'Right_Change', 'Calc_New_Route']
        self.current_state = "Keep_lane"
        # All possible drivable lanes including opposite track e.G. [-1,1,2]
        self.possible_lanes = []
        # Variable to detect if it is reasonable to do maneuver
        self.actual_track = 1
        self.targeted_track = 1
        self.left_lane_free = False
        self.right_lane_free = False
        self.distance_next_turn_m = 999
        self.vehicle = Vehicle()
        self.lanes_at_turn = []
        self.speed_limit_ms = 50
        self.blocking_object = False
        self.new_route_is_ready = False
        # Values
        self.distance_no_more_overtake_m = 100
        self.distance_no_more_lane_swap_m = 100
        self.distance_failed_lane_swap_m = 10

    def update_state(self):
        """Doc"""
        # Popssible lane to turn?
        possible_turnlane = self.actual_track in self.lanes_at_turn
        need_to_swap_lane_count = 0
        if not possible_turnlane:
            need_to_swap_lane_count = self.lanes_at_turn[0]- self.actual_track




        # Distance possible for takeover
        possible_takeover = self.distance_next_turn_m >= self.distance_no_more_overtake_m

        # Distance possible for lane swap
        possible_laneswap = self.distance_next_turn_m >= self.distance_no_more_lane_swap_m

        maneuver_to_late = self.distance_next_turn_m <= self.distance_failed_lane_swap_m


        if self.current_state == "Keep_lane":
            # Frei Fahrt und auf richtigen Spur oder beide spuren sind besetzt
            if possible_turnlane and (not self.blocking_object or not possible_laneswap) \
                    or not (self.left_lane_free or self.right_lane_free):
                pass
            # ToDo Rechtsfahrgebot??
            # Left Turn -> Ueberholen, Spurwechsel (turn)
            elif (self.left_lane_free and not maneuver_to_late and
                  (possible_takeover or
                   (need_to_swap_lane_count < 0))):
                self.current_state = "Left_Change"
                self.targeted_track += 1
                # todo update actual lane
            elif (self.right_lane_free and not maneuver_to_late and
                      (possible_takeover or
                       (need_to_swap_lane_count > 0))):
                self.current_state = "Right_Change"
                self.targeted_track -= 1
            else:
                self.current_state = 'Calc_New_Route'

        elif self.current_state == "Left_Change" or self.current_state == "Right_Change":
            if self.actual_track == self.targeted_track:
                self.current_state = "Keep_lane"

        elif self.current_state == "Calc_New_Route":
            if self.new_route_is_ready:
                self.current_state = "Keep_lane"
