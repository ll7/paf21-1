cmake_minimum_required(VERSION 2.8.3)
project(vehicle_controller)

find_package(ros_environment REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs roslaunch)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/vehicle_controller/main.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
