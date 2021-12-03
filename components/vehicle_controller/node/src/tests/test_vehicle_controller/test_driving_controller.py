from vehicle_controller.driving_control import SimpleDrivingController

def test_should_initially_stand_still():
    controller = SimpleDrivingController()
    signal = controller.next_signal()
    assert(signal.steering_angle_rad == 0.0 \
        and signal.target_velocity_mps == 0.0)

def test_should_have_zero_velocity_when_standing_still():
    controller = SimpleDrivingController()
    controller.update_vehicle_position((0, 0))
    controller.update_vehicle_position((0, 0))
    assert(controller.actual_velocity_mps == 0)

def test_should_have_positive_velocity_when_moving():
    controller = SimpleDrivingController()
    controller.update_vehicle_position((0, 0))
    controller.update_vehicle_position((0, 1))
    assert(controller.actual_velocity_mps > 0)

# def test_should_
