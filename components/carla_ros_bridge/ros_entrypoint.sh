#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the given ROS nodes from Docker CMD args
roslaunch $@ &
sleep 5 && python /opt/carla/PythonAPI/examples/spawn_npc.py --host carla-simulator -n 50 -w 50 &
wait
