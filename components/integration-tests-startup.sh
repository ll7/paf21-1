#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
apt-get update && apt-get install -y ros-noetic-ackermann-msgs && rm -rf /var/lib/apt/lists/*
# make sure all containers are ready
sleep 10

# list summary of available nodes before the test
nodes_before=$(rosnode list)

# replay the recorded ROS messages and dump the output topics to a file
rostopic echo /carla/ego_vehicle/ackermann_cmd > out.msgs &
rosbag play /app/recordings/recording_0.bag /app/recordings/recording_1.bag /app/recordings/recording_2.bag /app/recordings/recording_3.bag /app/recordings/recording_4.bag /app/recordings/recording_5.bag &

# wait for either of the subtasks to terminate
wait -n
kill $(jobs -p)

# list summary of available nodes after the test
nodes_after=$(rosnode list)

# assert that all nodes are still available after the test
# this is required because roslaunch always exits with a 0 code although
# nodes were crashing, so a crash can detected by comparing the lists
if [ $nodes_before -ne $nodes_after ]; then
    echo 'test faules! some nodes crashed during test!'
    exit 1
fi

# assert that the dumped output actually contains ROS messages
if [[ ! $(cat out.msgs) ]]; then
    echo 'test failed! no messages received on expected output topic!'
    exit 1
fi

echo 'test passed!'
