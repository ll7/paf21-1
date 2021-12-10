import time
from vehicle_controller.driving_control import DrivingController

# def test_should_initially_stand_still():
#     controller = DrivingController()
#     signal = controller.next_signal()
#     assert(signal.steering_angle_rad == 0.0 \
#         and signal.target_velocity_mps == 0.0)

# def test_should_have_zero_velocity_when_standing_still():
#     controller = DrivingController()
#     controller.update_vehicle_position((0, 0))
#     controller.update_vehicle_position((0, 0))
#     assert(controller.actual_velocity_mps == 0)

# def test_should_have_positive_velocity_when_moving():
#     controller = DrivingController()
#     controller.update_vehicle_position((0, 0))
#     time.sleep(0.1)
#     controller.update_vehicle_position((0, 1))
#     assert(controller.actual_velocity_mps > 0)

# def test_should_drive_straight_forward():
#     controller = DrivingController()
#     controller.update_route([(0.0, 0.0), (0.0, 10.0)])
#     controller.update_target_velocity(0.5)
#     controller.update_vehicle_position((0, 0))
#     time.sleep(0.1)
#     controller.update_vehicle_position((0, 1))
#     signal = controller.next_signal()
#     assert(controller.actual_velocity_mps > 0 \
#         and signal.steering_angle_rad == 0)

# def test_should_drive_right_curve():
#     controller = SimpleDrivingController()
#     waypoints_right_curve = [
#         {'x': 325.6300046575124, 'y': 0.01132171390924333},
#         {'x': 325.6303408028037, 'y': 0.011321535336526327},
#         {'x': 328.02516734033094, 'y': 0.010049315506384104},
#         {'x': 334.2096359709986, 'y': -2.2290054206649588},
#         {'x': 336.89511162507006, 'y': -7.9630831629196415},
#         {'x': 336.8934041373098, 'y': -10.789998012538765},
#         {'x': 336.8718832281081, 'y': -22.419991513106055},
#         {'x': 336.8718832281081, 'y': -36.419991513106055},
#         {'x': 336.8718832281081, 'y': -46.419991513106055},
#         {'x': 336.8715812226464, 'y': -46.91999142189874},
#         {'x': 336.87067520626124, 'y': -48.41999114827682},
#     ]
#     controller.update_route(waypoints_right_curve)
#     controller.update_target_velocity(0.5)
#     controller.update_vehicle_position((0, 0))
#     time.sleep(0.1)
#     controller.update_vehicle_position((0, 1))
#     signal = controller.next_signal()
#     assert(controller.actual_velocity_mps > 0 \
#         and signal.steering_angle_rad == 0)
