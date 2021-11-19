from vehicle_controller.driving import SimpleDrivingAgent

def test_should_drive_straight_forward():
    agent = SimpleDrivingAgent()
    signal = agent.next_signal(None)
    assert(signal.steering_angle == 0.0 and signal.speed > 0)
