cmake_minimum_required(VERSION 2.8.3)
project(test_subscriber_camera)

find_package(ros_environment REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs roslaunch)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/test_subscriber_camera/subscriber_camera.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# if (CATKIN_ENABLE_TESTING)
#   roslaunch_add_file_check(launch)
#   roslaunch_add_file_check(test)
# endif()

# install(FILES src/carla_ros_bridge/CARLA_VERSION
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(FILES test/ros_bridge_client.test
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# if (CATKIN_ENABLE_TESTING AND NOT $ENV{CI})
#   find_package(rostest REQUIRED)
#   add_rostest(test/ros_bridge_client.test)
# endif()
