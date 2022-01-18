#!/bin/bash

# check if script was called in quiet mode
if [[ $# -eq 1 && $1 -eq '-q' ]]; then
    QUIET_FLAG='-q'
else
    QUIET_FLAG=
fi

# download ROS bag files for mocking the CARLA simulator
ROSBAG_ASSETS_DIR=./recordings
mkdir -p $ROSBAG_ASSETS_DIR
echo "start downloading ROS bag assets into $ROSBAG_ASSETS_DIR"
wget $QUIET_FLAG -O "$ROSBAG_ASSETS_DIR/recording_0.bag" https://megastore.rz.uni-augsburg.de/get/2FkMSiLkR1/
wget $QUIET_FLAG -O "$ROSBAG_ASSETS_DIR/recording_1.bag" https://megastore.rz.uni-augsburg.de/get/Vzfjj9atqL/
wget $QUIET_FLAG -O "$ROSBAG_ASSETS_DIR/recording_2.bag" https://megastore.rz.uni-augsburg.de/get/jl73avCFAM/
wget $QUIET_FLAG -O "$ROSBAG_ASSETS_DIR/recording_3.bag" https://megastore.rz.uni-augsburg.de/get/DWQcmFd7Jh/
wget $QUIET_FLAG -O "$ROSBAG_ASSETS_DIR/recording_4.bag" https://megastore.rz.uni-augsburg.de/get/b02fO4jaLY/
wget $QUIET_FLAG -O "$ROSBAG_ASSETS_DIR/recording_5.bag" https://megastore.rz.uni-augsburg.de/get/PvlkhB_lqn/

echo 'successfully downloaded assets!'
