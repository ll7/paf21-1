#!/bin/bash

LOG_DIR=$PWD/logs
mkdir -p $LOG_DIR

# define empty variable to avoid warning
export CONFIG_FILE=

# set compose file to launch
COMPOSE_FILE=local-carla-sim-compose.yml

pushd scenarios
    docker-compose -f $COMPOSE_FILE logs carla-ros-bridge > $LOG_DIR/ros_bridge_log.txt
    docker-compose -f $COMPOSE_FILE logs perception > $LOG_DIR/perception_log.txt
    docker-compose -f $COMPOSE_FILE logs vehicle_control > $LOG_DIR/vehicle_control_log.txt
    docker-compose -f $COMPOSE_FILE logs competition_manager > $LOG_DIR/competition_manager.txt
    COMP_MGR_OUT=$(docker-compose -f $COMPOSE_FILE logs competition_manager | grep 'The competition was completed')
    docker-compose -f $COMPOSE_FILE down
popd

echo '=========================='
echo 'competition manager output'
echo $COMP_MGR_OUT
