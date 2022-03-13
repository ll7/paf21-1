#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the given ROS nodes from Docker CMD args
python switch_town.py $CARLA_SIM_HOST $TOWN
python set_car_spawn.py $CONFIG_FILE
roslaunch $@ &
sleep 5 && python /opt/carla/PythonAPI/examples/spawn_npc.py --host carla-simulator -n 0 -w 0 &
wait

#============ Variables ================
export CONFIG_FILE="/config/Town02/town02_sg0.yaml"
export TOWN="Town02"
# TODO: 

# =====================================

sleep 25
python switch_town.py $CARLA_SIM_HOST $TOWN
python spawn_car.py $CONFIG_FILE
python /opt/carla/PythonAPI/examples/spawn_npc.py -n 40 -w 50 --host $CARLA_SIM_HOST
echo 'successfully initialized!'

