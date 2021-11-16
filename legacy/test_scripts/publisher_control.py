#!/usr/bin/env python

#Publisher from
# node : roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
# topic : carla/ego_vehicle/vehicle_control_cmd

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleControl

def talker():
    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',CarlaEgoVehicleControl , queue_size=10)
    rospy.init_node('control_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        string = "publisher node sends control singal %s" % rospy.get_time()
        rospy.loginfo(string)
        pub.publish(CarlaEgoVehicleControl(throttle = 0.8, steer = 0.0, brake = 0, hand_brake = 0, reverse = 0, gear = 2, manual_gear_shift = False))
        #'header', 'throttle', 'steer', 'brake', 'hand_brake', 'reverse', 'gear', 'manual_gear_shift'
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass