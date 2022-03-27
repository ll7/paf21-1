cmake_minimum_required(VERSION 2.8.3)
project(vehicle_control)

find_package(ros_environment REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs roslaunch)

add_subdirectory(nav_srvs)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/vehicle_control/main.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
