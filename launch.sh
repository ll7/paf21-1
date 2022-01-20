#!/bin/bash

SCENARIO=default_scenario

pushd components
    docker-compose -f carla-sim-build.yml build
    if [ $? -ne 0 ]; then exit 1; fi
    docker-compose -f driving-components-build.yml build
    if [ $? -ne 0 ]; then exit 1; fi
popd

pushd scenarios/$SCENARIO
    xhost +
    docker-compose up -d
popd
