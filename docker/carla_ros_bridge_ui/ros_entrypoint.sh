#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the CARLA ROS bridge
roslaunch $@
