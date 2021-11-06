#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"

# override the CARLA simulator hostname
sed -i "s/name='host' default=''/name='host' default='"$CARLA_SIM_HOST"'/g" \
    /opt/carla-ros-bridge/install/share/carla_ros_bridge/launch/carla_ros_bridge.launch
sed -i "s/name='host' default='localhost'/name='host' default='"$CARLA_SIM_HOST"'/g" \
    /opt/carla-ros-bridge/install/share/carla_ros_bridge/launch/carla_ros_bridge_with_example_ego_vehicle.launch

# wait for the CARLA simulator to launch
sleep $CARLA_SIM_WAIT_SECS

# launch the CARLA ROS bridge
roslaunch carla_ros_bridge $@
