from vehicle_controller.driving import SimpleDrivingSignalConverter

def test_should_drive_straight_forward():
    agent = SimpleDrivingSignalConverter()
    signal = agent.next_signal()
    assert(signal.steering_angle == 0.0 and signal.speed > 0)
