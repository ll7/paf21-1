#!/bin/bash

# ======================================================
#                        U S A G E
# ======================================================
# This script launches the entire project, from installing
# the CARLA simulator to building the Docker images and
# launching the components accordingly. There's a counter
# part to this script in shutdown.sh for system exit.

# The script has a single optional parameter specifying
# which competition yaml file you want to launch.
# The file needs to be located inside the
# ./scenarios/config folder which is mapped to /config
# inside the ROS bridge container, initiating the system.
#
# For reasons of simplicity, the ROS bridge entrypoint
# searches for the yaml file inside the config folder
# and all its subfolders, so you don't need to worry
# about specifying relative paths.
# ======================================================

# set config file
export CONFIG_FILE=$1
if [ -z $CONFIG_FILE ]; then
    export CONFIG_FILE="town01_sg0.yaml"
fi

# set compose file to launch
COMPOSE_FILE=local-carla-sim-compose.yml

# download the CARLA simulator (does nothing when it's already installed)
./setup_simulator.sh

# build docker images
pushd components
    docker-compose -f carla-sim-build.yml build
    if [ $? -ne 0 ]; then exit 1; fi
    docker-compose -f driving-components-build.yml build
    if [ $? -ne 0 ]; then exit 1; fi
popd

# run docker images (needs to disable X11 security feature)
pushd scenarios
    xhost +
    docker-compose -f $COMPOSE_FILE up -d
popd

# run the CARLA simulator
pushd simulator
    ./CarlaUE4.sh
popd

# once the simulator exits, run the shutdown script
./shutdown.sh
