#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash

# make sure all containers are ready
sleep 10

# replay the recorded ROS messages and dump the output topics to a file
rostopic echo /carla/ego_vehicle/ackermann_cmd > out.msgs &
rosbag play /app/recordings/recording.bag &

# wait for either of the subtasks to terminate
wait -n
kill $(jobs -p)

# assert that the dumped output actually contains ROS messages
if [[ ! $(cat out.msgs) ]]; then
    echo 'test failed! no messages received on expected output topic!'
    exit 1
fi

echo 'test passed!'
