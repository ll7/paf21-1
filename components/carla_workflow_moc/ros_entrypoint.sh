#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /app/catkin_ws/devel/setup.bash
roslaunch $@
