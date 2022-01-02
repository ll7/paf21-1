import numpy as np
from cv2 import cv2
from local_planner.stateMaschine.maneuver_state_machine import ManeuverStateMachine, ManeuverObservation, ManeuverState
from local_planner.stateMaschine.speed_state_machine import SpeedStateMachine

def test_should_stay_in_lane():
    st_machine = ManeuverStateMachine()

    st_machine_observation = ManeuverObservation()
    st_machine_observation.blocking_object = False
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = True
    st_machine_observation.right_lane_free = False
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200

    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.Keep_Lane)
    assert(st_machine_observation.targeted_track == 1)
 
def test_should_over_take_left_with_right_option():
    st_machine = ManeuverStateMachine()
    st_machine.current_state = ManeuverState.Keep_Lane

    st_machine_observation = ManeuverObservation()
    st_machine_observation.blocking_object = True
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = True
    st_machine_observation.right_lane_free = True
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200

    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.Left_Change)
    assert(st_machine_observation.targeted_track == -1)

def test_should_over_take_left():
    st_machine = ManeuverStateMachine()
    st_machine.current_state = ManeuverState.Keep_Lane

    st_machine_observation = ManeuverObservation()
    st_machine_observation.blocking_object = True
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = True
    st_machine_observation.right_lane_free = False
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200

    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.Left_Change)
    assert(st_machine_observation.targeted_track == -1)

def test_should_over_take_right():
    st_machine = ManeuverStateMachine()

    st_machine_observation = ManeuverObservation()
    st_machine.current_state = ManeuverState.Keep_Lane

    st_machine_observation.blocking_object = True
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = False
    st_machine_observation.right_lane_free = True
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200
    
    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.Right_Change)
    assert(st_machine_observation.targeted_track == 2)

def load_image(file_name: str) -> np.ndarray:
    return cv2.imread('foobar', f'./test_resources/{file_name}')
