#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash

# make sure that the roscore service is already up
sleep 10

# launch the competition manager (counting time from start to goal)
python /app/src/competition_manager/src/simple_competition_manager.py
