#!/bin/bash
set -e
source "/opt/carla-ros-bridge/install/setup.bash"

#============ Variables ================
export CONFIG_FILE="/config/Town01/town01_sg0.yaml"
export TOWN="Town01"
export NUM_CARS=40
export NUM_PEDESTRIANS=50
export SPAWN_POINT="$(python /scripts/parse_spawn_pos.py)"

echo $SPAWN_POINT

# =====================================

# load the configuration for the competition manager
sleep 2
rosparam load $CONFIG_FILE

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the given ROS nodes from Docker CMD args
roslaunch $@ &
sleep 5 && python /opt/carla/PythonAPI/examples/spawn_npc.py \
    --host $CARLA_SIM_HOST -n $NUM_CARS -w $NUM_PEDESTRIANS &
wait
