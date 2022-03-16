#!/bin/bash
set -e
source "/opt/carla-ros-bridge/install/setup.bash"

# locate the full path of the configuration file
updatedb
export CONFIG_FILE_FULL_PATH="$(locate "$CONFIG_FILE")"

echo "config file: $CONFIG_FILE_FULL_PATH"
export SPAWN_POINT="$(python /scripts/parse_spawn_pos.py $CONFIG_FILE_FULL_PATH)"

# TODO: parse following settings from the YAML config (don't hardcode this here ...)
export TOWN="${CONFIG_FILE_FULL_PATH:17:6}"
export NUM_CARS=0
export NUM_PEDESTRIANS=0

echo "spawn pos: $SPAWN_POINT"
echo "town: $TOWN"
echo "cars: $NUM_CARS, pedestrians: $NUM_PEDESTRIANS"

# load the configuration for the competition manager
sleep 2
rosparam load $CONFIG_FILE_FULL_PATH

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the given ROS nodes from Docker CMD args
roslaunch $@ &
sleep 5 && python /opt/carla/PythonAPI/examples/spawn_npc.py \
    --host carla-simulator -n $NUM_CARS -w $NUM_PEDESTRIANS --safe --sync &
wait
