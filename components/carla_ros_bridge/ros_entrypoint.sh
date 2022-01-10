#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"
rostopic echo /carla/ego_vehicle/ackermann_control/control_info

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the given ROS nodes from Docker CMD args
roslaunch $@
