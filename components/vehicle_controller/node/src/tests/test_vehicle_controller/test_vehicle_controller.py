from vehicle_controller.driving import SimpleDrivingSignalConverter

def test_should_stand_still():
    agent = SimpleDrivingSignalConverter()
    signal = agent.next_signal()
    assert(signal.steering_angle_rad == 0.0 \
        and signal.target_velocity_mps == 0.0)
