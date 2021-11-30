# Subcribes to the following the /carla/ego_vehicle/vehicle_status topic from following node
# roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch


#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleStatus

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Received %s", data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pavlo_listener', anonymous=True)

    rospy.Subscriber("carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()