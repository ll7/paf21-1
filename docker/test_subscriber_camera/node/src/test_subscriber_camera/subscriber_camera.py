#!/usr/bin/env python

# Subcribes to the following the /carla/ego_vehicle/camera/depth/front/camera_info topic from following node
# roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

import rospy
from sensor_msgs.msg import CameraInfo
from core import SensorLogger

def main():
    rospy.init_node('pavlo_listener', anonymous=True)
    logger = SensorLogger()
    rospy.Subscriber("/carla/ego_vehicle/camera/depth/front/camera_info", CameraInfo, logger.callback)
    rospy.spin()

if __name__ == '__main__':
    main()