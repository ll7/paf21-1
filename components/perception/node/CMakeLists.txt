cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(ros_environment REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs cv_bridge roslaunch)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/perception/main.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
