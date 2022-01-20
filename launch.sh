#!/bin/bash

SCENARIO=default_scenario

pushd components
    docker-compose -f driving-components-build.yml build
    docker-compose -f carla-sim-build.yml build
popd

pushd scenarios/$SCENARIO
    xhost +
    docker-compose up -d
popd
