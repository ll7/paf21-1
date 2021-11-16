from ackermann_msgs.msg import AckermannDrive

class SimpleDrivingAgent:
    def next_signal(self, perception):
        return self.straight_forward()

    def straight_forward(self):
        return AckermannDrive(
            steering_angle=0.0,
            steering_angle_velocity=0.0,
            speed=10,
            acceleration=0.0,
            jerk=0.0)
