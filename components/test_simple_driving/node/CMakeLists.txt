cmake_minimum_required(VERSION 2.8.3)
project(test_simple_driving)

find_package(ros_environment REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs roslaunch)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/test_simple_driving/main.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
