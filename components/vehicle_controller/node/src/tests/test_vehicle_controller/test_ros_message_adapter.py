from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg, Float32 as FloatMsg
from sensor_msgs.msg import NavSatFix as GpsMsg

from vehicle_controller.driving_control import DrivingSignal
from vehicle_controller.ros_msg_adapter import RosDrivingMessagesAdapter

def test_should_parse_route_waypoints():
    waypoints_json = '[ { "x": 0.0, "y": 0.0 }, { "x": 0.0, "y": 1.0 }, { "x": 0.2, "y": 1.7 } ]'
    exp_waypoints = [(0.0, 0.0), (0.0, 1.0), (0.2, 1.7)]
    msg = StringMsg(data=waypoints_json)
    waypoints = RosDrivingMessagesAdapter.message_to_waypoints(msg)
    assert(waypoints == exp_waypoints)

def test_should_parse_route_waypoints_and_ignore_metadata():
    waypoints_json = '[ { "x": 0.0, "y": 0.0, "foo": "bar" }, { "some": "thing", "x": 0.0, "y": 1.0 }, { "x": 0.2, "y": 1.7 } ]'
    exp_waypoints = [(0.0, 0.0), (0.0, 1.0), (0.2, 1.7)]
    msg = StringMsg(data=waypoints_json)
    waypoints = RosDrivingMessagesAdapter.message_to_waypoints(msg)
    assert(waypoints == exp_waypoints)

def test_should_parse_target_velocity():
    msg = FloatMsg(data=1.7)
    target_velocity = RosDrivingMessagesAdapter.message_to_target_velocity(msg)
    exp_target_velocity = 1.7
    assert(target_velocity == exp_target_velocity)

def test_should_parse_vehicle_position():
    msg = GpsMsg(longitude=1.7, latitude=0.3)
    vehicle_pos = RosDrivingMessagesAdapter.message_to_vehicle_position(msg)
    exp_vehicle_pos = (1.7, 0.3)
    assert(vehicle_pos == exp_vehicle_pos)

def test_should_generate_ackermann_message_from_signal():
    signal = DrivingSignal(0.7, 30.7)
    msg = RosDrivingMessagesAdapter.signal_to_message(signal)
    msg_exp = AckermannDrive(
            steering_angle=0.7,
            steering_angle_velocity=0.0,
            speed=30.7,
            acceleration=0.0,
            jerk=0.0)
    assert(msg == msg_exp)
