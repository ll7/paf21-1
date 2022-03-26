#!/bin/bash

LOG_DIR=$PWD/logs
mkdir -p $LOG_DIR
export CONFIG_FILE=

pushd scenarios
    docker-compose logs carla-ros-bridge > $LOG_DIR/ros_bridge_log.txt
    docker-compose logs perception > $LOG_DIR/perception_log.txt
    docker-compose logs local-planner > $LOG_DIR/local-planner_log.txt
    docker-compose logs competition-manager > $LOG_DIR/competition_manager.txt
    COMP_MGR_OUT=$(docker-compose logs competition-manager | grep 'The competition was completed')
    docker-compose down
popd

echo '=========================='
echo 'competition manager output'
echo $COMP_MGR_OUT
