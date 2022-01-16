from local_planner.state_machine import SpeedObservation, SpeedStateMachine, TrafficLightPhase, SpeedState
from local_planner.core.vehicle import Vehicle

""" Keep States """

def test_keep_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 0/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 5.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)


def test_accel_from_stop():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 0
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 100.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)

def test_accel_from_stop_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 0
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 100.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)

def test_keep_red_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 50.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 1.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


# TODO: fix
# def test_keep_red_out_of_range():
#     vehicle = Vehicle('test_vehicle')
#     vehicle.actual_velocity_mps = 50.0/3.6
#     speed_s = SpeedStateMachine(vehicle)
#     speed_s.current_state = SpeedState.KEEP

#     speed_o = SpeedObservation()
#     speed_o.tl_phase = TrafficLightPhase.RED
#     speed_o.is_trajectory_free = True
#     speed_o.dist_next_obstacle_m: float = 100.0
#     speed_o.detected_speed_limit: int = 50

#     speed_s.update_state(speed_o)
#     target_speed = speed_s.get_target_speed()

    # assert(speed_s.current_state == SpeedState.KEEP)
    # assert (target_speed == vehicle.actual_velocity_mps)


def test_keep_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 50.0/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.KEEP

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m: float = 1.0
    speed_o.detected_speed_limit: int = 50
    speed_o.object_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


# TODO: fix
# def test_keep_static_obstacle_out_range():
#     vehicle = Vehicle('test_vehicle')
#     vehicle.actual_velocity_mps = 50.0/3.6
#     speed_s = SpeedStateMachine(vehicle)
#     speed_s.current_state = SpeedState.KEEP

#     speed_o = SpeedObservation()
#     speed_o.tl_phase = TrafficLightPhase.RED
#     speed_o.is_trajectory_free = False
#     speed_o.dist_next_obstacle_m: float = 100.0
#     speed_o.detected_speed_limit: int = 50

#     speed_s.update_state(speed_o)
#     target_speed = speed_s.get_target_speed()

#     assert(speed_s.current_state == SpeedState.KEEP)
#     assert (target_speed == vehicle.actual_velocity_mps)


"""Accel Cases"""

def test_accel_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 5.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)



def test_accel_static_start():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 0.1 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 100.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)


def test_accel_red_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 1.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_accel_red_out_of_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30.0/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 100.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)


# TODO: fix
# def test_accel_moving_obstacle_in_range():
#     vehicle = Vehicle('test_vehicle')
#     vehicle.actual_velocity_mps = 30.0 / 3.6
#     speed_s = SpeedStateMachine(vehicle)
#     speed_s.current_state = SpeedState.ACCEL

#     speed_o = SpeedObservation()
#     speed_o.tl_phase = TrafficLightPhase.GREEN
#     speed_o.is_trajectory_free = False
#     speed_o.dist_next_obstacle_m: float = 10
#     speed_o.detected_speed_limit: int = 50
#     speed_o.object_speed_ms = 30 / 3.6

#     speed_s.update_state(speed_o)
#     target_speed = speed_s.get_target_speed()
#     assert(speed_s.current_state == SpeedState.KEEP)
#     assert (target_speed == vehicle.actual_velocity_mps)


def test_accel_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m: float = 1.0
    speed_o.detected_speed_limit: int = 50
    speed_o.object_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


def test_accel_static_obstacle_out_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.ACCEL

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m: float = 100.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    print("ts", target_speed)
    assert (target_speed == 50/3.6)

"""Brake Cases"""

def test_brake_green():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30/3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 5.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert(speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)


def test_brake_red_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 50.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = True
    speed_o.dist_next_obstacle_m: float = 1.0
    speed_o.detected_speed_limit: int = 50

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)


# TODO: fix
# def test_brake_red_out_of_range():
#     vehicle = Vehicle('test_vehicle')
#     vehicle.actual_velocity_mps = 50.0 / 3.6
#     speed_s = SpeedStateMachine(vehicle)
#     speed_s.current_state = SpeedState.BRAKE

#     speed_o = SpeedObservation()
#     speed_o.tl_phase = TrafficLightPhase.RED
#     speed_o.is_trajectory_free = True
#     speed_o.dist_next_obstacle_m: float = 100.0
#     speed_o.detected_speed_limit: int = 50

#     speed_s.update_state(speed_o)
#     target_speed = speed_s.get_target_speed()

#     assert (speed_s.current_state == SpeedState.KEEP)
#     assert (target_speed == 50 / 3.6)


def test_brake_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 50.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.GREEN
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m: float = 1.0
    speed_o.detected_speed_limit: int = 50
    speed_o.object_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.BRAKE)
    assert (target_speed == 0)

def test_brake_static_obstacle_in_range():
    vehicle = Vehicle('test_vehicle')
    vehicle.actual_velocity_mps = 30.0 / 3.6
    speed_s = SpeedStateMachine(vehicle)
    speed_s.current_state = SpeedState.BRAKE

    speed_o = SpeedObservation()
    speed_o.tl_phase = TrafficLightPhase.RED
    speed_o.is_trajectory_free = False
    speed_o.dist_next_obstacle_m: float = 100.0
    speed_o.detected_speed_limit: int = 50
    speed_o.object_speed_ms = 0

    speed_s.update_state(speed_o)
    target_speed = speed_s.get_target_speed()

    assert (speed_s.current_state == SpeedState.ACCEL)
    assert (target_speed == 50/3.6)


# TODO: fix
# def test_brake_static_obstacle_out_range():
#     vehicle = Vehicle('test_vehicle')
#     vehicle.actual_velocity_mps = 50.0 / 3.6
#     speed_s = SpeedStateMachine(vehicle)
#     speed_s.current_state = SpeedState.BRAKE

#     speed_o = SpeedObservation()
#     speed_o.tl_phase = TrafficLightPhase.RED
#     speed_o.is_trajectory_free = False
#     speed_o.dist_next_obstacle_m: float = 100.0
#     speed_o.detected_speed_limit: int = 50

#     speed_s.update_state(speed_o)
#     target_speed = speed_s.get_target_speed()
#     print("ts:", target_speed)
#     assert (speed_s.current_state == SpeedState.KEEP)
#     assert (target_speed == 50 / 3.6)