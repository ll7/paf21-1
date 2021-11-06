ARG ROS_BRIDGE_BASE_IMG
FROM $ROS_BRIDGE_BASE_IMG

# alias python3 as python
RUN apt-get update && apt-get install -y python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=/opt/carla-ros-bridge/install/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

# override the entrypoint with a more configurable one
RUN rm /ros_entrypoint.sh
COPY ./ros_entrypoint.sh /ros_entrypoint.sh

# set the CARLA simulator to localhost
ENV CARLA_SIM_HOST=localhost

# set the timer waiting for the CARLA simulator to launch
ENV CARLA_SIM_WAIT_SECS=20

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["carla_ros_bridge.launch", "--wait"]
