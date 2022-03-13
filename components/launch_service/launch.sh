#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash

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
