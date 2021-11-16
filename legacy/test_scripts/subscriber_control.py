#Subscriber to
# node : roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
# topic : carla/ego_vehicle/vehicle_control_cmd

#!/usr/bin/env python
import rospy
from carla_msgs.msg import CarlaEgoVehicleControl

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Received %s", data)
    
def listener():

    rospy.init_node('control_sub', anonymous=True)

    rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()