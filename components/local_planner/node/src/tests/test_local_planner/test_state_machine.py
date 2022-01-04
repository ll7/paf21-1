from local_planner.state_machine import ManeuverStateMachine, ManeuverObservation, ManeuverState
# from local_planner.vehicle_control import DrivingController

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
