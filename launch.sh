#!/bin/bash

# Download Carla Simulator
./setup_simulator.sh

# set config file
export CONFIG_FILE=$1
if [ -z $CONFIG_FILE ]; then
    export CONFIG_FILE="town01_sg0.yaml"
fi

# set compose file to launch
COMPOSE_FILE=local-carla-sim-compose.yml

# build docker images for the components
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

sleep 5
pushd simulator
    ./CarlaUE4.sh
popd
