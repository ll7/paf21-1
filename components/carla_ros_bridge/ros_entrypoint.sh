#!/bin/bash
set -e
source "/opt/carla-ros-bridge/install/setup.bash"

# locate the full path of the configuration file
updatedb
export CONFIG_FILE_FULL_PATH="$(locate "$CONFIG_FILE")"

echo "config file: $CONFIG_FILE_FULL_PATH"
export TOWN="$(python /scripts/load_env_config.py $CONFIG_FILE_FULL_PATH --town)"
export SPAWN_POINT="$(python /scripts/load_env_config.py $CONFIG_FILE_FULL_PATH --spawn-pos)"
export NUM_CARS="$(python /scripts/load_env_config.py $CONFIG_FILE_FULL_PATH --num-cars)"
export NUM_PEDESTRIANS="$(python /scripts/load_env_config.py $CONFIG_FILE_FULL_PATH --num-peds)"
export RAND_SPAWN_SEED="$(python /scripts/load_env_config.py $CONFIG_FILE_FULL_PATH --spawn-seed)"

echo "spawn pos: $SPAWN_POINT in town $TOWN"
echo "cars: $NUM_CARS, pedestrians: $NUM_PEDESTRIANS, seed: $RAND_SPAWN_SEED"

# load the configuration for the competition manager
sleep 2
rosparam load $CONFIG_FILE_FULL_PATH

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the given ROS nodes from Docker CMD args
roslaunch $@ &
sleep 5 && python /opt/carla/PythonAPI/examples/spawn_npc.py \
    --host $CARLA_SIM_HOST --safe \
    -n $NUM_CARS -w $NUM_PEDESTRIANS -s $RAND_SPAWN_SEED &
wait
