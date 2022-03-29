from vehicle_control.state_machine import SpeedObservation, SpeedStateMachine, TrafficLightPhase, SpeedState
from vehicle_control.core.vehicle import Vehicle

""" Keep States """

def test_drive_curve():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 8
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP
    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True 
    speed_o.dist_next_traffic_light_m=1.4731885194778442 
    speed_o.dist_next_obstacle_m=999 
    speed_o.obj_speed_ms=500 
    speed_o.dist_next_curve=0
    speed_o.curve_target_speed=10
    speed_o.dist_next_stop_m=999
    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()
    print(target_speed)
    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 10)

def test_keep_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 0/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light_m = 5.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)


def test_accel_from_stop():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)

def test_accel_from_stop_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 0 / 3.6

    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)

def test_keep_red_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light_m = 1.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_keep_red_out_of_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.KEEP)
    assert (target_speed == vehicle.velocity_mps)


def test_keep_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 1.0
    speed_o.detected_speed_limit = 50
    speed_o.obj_speed_ms = 0
    speed_o.max_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_keep_static_obstacle_out_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.KEEP)
    assert (target_speed == vehicle.velocity_mps)


"""Accel Cases"""

def test_accel_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light_m = 5.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)


def test_accel_static_start():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 0.1 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)


def test_accel_red_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light_m = 1.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_accel_red_out_of_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)


def test_accel_moving_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 16
    speed_o.detected_speed_limit = 50
    speed_o.obj_speed_ms = 30 / 3.6

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()
    assert(speed_s.current_state == SpeedState.KEEP)
    assert (target_speed == vehicle.velocity_mps)


def test_accel_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 1.0
    speed_o.obj_speed_ms = 0
    speed_o.detected_speed_limit = 50
    speed_o.max_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_accel_static_obstacle_out_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    # print("ts", target_speed)
    assert (target_speed >= 50/3.6)


"""Brake Cases"""


def test_brake_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light_m = 5.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)


def test_brake_red_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_traffic_light_m = 1.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_brake_red_out_of_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.KEEP)
    assert (target_speed >= 50 / 3.6)


def test_brake_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = False
    speed_o.obj_speed_ms = 0
    speed_o.dist_next_obstacle_m = 1.0
    speed_o.detected_speed_limit = 50
    speed_o.max_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_accel_static_obstacle_not_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 30/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50
    speed_o.max_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed >= 50/3.6)


def test_brake_static_obstacle_out_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.pos = (0, 0)
    vehicle.orientation_rad = 1
    vehicle.velocity_mps = 50 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m = 100.0
    speed_o.detected_speed_limit = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()
    assert (speed_s.current_state == SpeedState.KEEP)
    assert (target_speed >= 50 / 3.6)
