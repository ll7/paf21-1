#!/bin/bash

LOG_DIR=$PWD/logs
mkdir -p $LOG_DIR
export CONFIG_FILE=

pushd scenarios
    docker-compose logs carla-ros-bridge > $LOG_DIR/ros_bridge_log.txt
    docker-compose logs perception > $LOG_DIR/perception_log.txt
    docker-compose logs vehicle_control > $LOG_DIR/vehicle_control_log.txt
    docker-compose logs competition_manager > $LOG_DIR/competition_manager.txt
    COMP_MGR_OUT=$(docker-compose logs competition_manager | grep 'The competition was completed')
    docker-compose down
popd

echo '=========================='
echo 'competition manager output'
echo $COMP_MGR_OUT
