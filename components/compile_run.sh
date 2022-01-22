#!/bin/sh
xhost +
docker-compose -f driving-components-build.yml build
docker-compose -f carla-sim-build.yml build
(cd ../scenarios/default_scenario/; docker-compose up -d)
