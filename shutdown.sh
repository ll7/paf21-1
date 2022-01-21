#!/bin/bash

SCENARIO=default_scenario
LOG_DIR=$PWD/logs
mkdir -p $LOG_DIR

pushd scenarios/$SCENARIO
    docker-compose logs carla-ros-bridge > $LOG_DIR/ros_bridge_log.txt
    docker-compose logs perception > $LOG_DIR/perception_log.txt
    docker-compose logs local-planner > $LOG_DIR/local-planner_log.txt
    docker-compose logs global-planner > $LOG_DIR/global_log.txt
    docker-compose down
popd
