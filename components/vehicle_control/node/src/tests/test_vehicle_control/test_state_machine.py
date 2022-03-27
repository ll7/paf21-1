from vehicle_control.state_machine import ManeuverStateMachine, ManeuverObservation, ManeuverState
# from local_planner.vehicle_control import DrivingController
from vehicle_control.core.vehicle import Vehicle

def test_should_stay_in_lane():
    vehicle = Vehicle('test_vehicle')
    st_machine = ManeuverStateMachine(vehicle)
    st_machine_observation = ManeuverObservation()
    st_machine_observation.blocking_object = False
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = True
    st_machine_observation.right_lane_free = False
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200

    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.KEEP_LANE)
    assert(st_machine_observation.targeted_track == 1)
 
def test_should_over_take_left_with_right_option():
    vehicle = Vehicle('test_vehicle')
    st_machine = ManeuverStateMachine(vehicle)
    st_machine.current_state = ManeuverState.KEEP_LANE

    st_machine_observation = ManeuverObservation()
    st_machine_observation.blocking_object = True
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = True
    st_machine_observation.right_lane_free = True
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200

    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.LEFT_CHANGE)
    assert(st_machine_observation.targeted_track == -1)

def test_should_over_take_left():
    vehicle = Vehicle('test_vehicle')
    st_machine = ManeuverStateMachine(vehicle)

    st_machine_observation = ManeuverObservation()
    st_machine_observation.blocking_object = True
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = True
    st_machine_observation.right_lane_free = False
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200

    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.LEFT_CHANGE)
    assert(st_machine_observation.targeted_track == -1)

def test_should_over_take_right():
    vehicle = Vehicle('test_vehicle')
    st_machine = ManeuverStateMachine(vehicle)

    st_machine_observation = ManeuverObservation()
    st_machine.current_state = ManeuverState.KEEP_LANE

    st_machine_observation.blocking_object = True
    st_machine_observation.actual_track = 1
    st_machine_observation.left_lane_free = False
    st_machine_observation.right_lane_free = True
    st_machine_observation.possible_lanes = [-1, 1, 2]
    st_machine_observation.lanes_at_turn = [1]
    st_machine_observation.distance_next_turn_m = 200
    
    st_machine.update_state(st_machine_observation)
    assert(st_machine.current_state == ManeuverState.RIGHT_CHANGE)
    assert(st_machine_observation.targeted_track == 2)
