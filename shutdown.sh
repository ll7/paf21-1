#!/bin/bash

LOG_DIR=$PWD/logs
mkdir -p $LOG_DIR
export CONFIG_FILE=

pushd scenarios
    docker-compose logs carla-ros-bridge > $LOG_DIR/ros_bridge_log.txt
    docker-compose logs perception > $LOG_DIR/perception_log.txt
    docker-compose logs local-planner > $LOG_DIR/local-planner_log.txt
    docker-compose logs global-planner > $LOG_DIR/global_log.txt
    COMP_MGR_OUT=$(docker-compose logs competition-manager | tail -n 1)
    docker-compose down
popd

echo '=========================='
echo 'competition manager output'
echo $COMP_MGR_OUT
