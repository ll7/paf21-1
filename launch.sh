#!/bin/bash

# set config file
export CONFIG_FILE=$1
if [ -z $CONFIG_FILE ]; then
    export CONFIG_FILE="town03_sg1.yaml"
fi

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
    docker-compose up -d
popd
