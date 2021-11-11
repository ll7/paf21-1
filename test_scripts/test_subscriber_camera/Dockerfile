FROM ros:noetic

# install pip
RUN apt-get update && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*

# alias python3 as python
RUN apt-get update && apt-get install -y python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# install pip dependencies
RUN python -m pip install pip --upgrade
ADD ./requirements.txt /app/requirements.txt
RUN python -m pip install -r /app/requirements.txt

# set up the default shell used by RUN tags
SHELL ["/bin/bash", "-c"]

# copy + build the source code
ADD ./node /app/src/test_subscriber_camera
WORKDIR /app/catkin_ws/src
RUN source /opt/ros/noetic/setup.bash && catkin_init_workspace && ln -s /app/src .
WORKDIR /app/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make

# register a startup procedure to launch the ROS node
ADD ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["test_subscriber_camera", "test_subscriber_camera.launch"]
