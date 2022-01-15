FROM ros:noetic

# set up the default shell used by RUN tags
SHELL ["/bin/bash", "-c"]

# install pip and some ROS dependencies
RUN apt-get update && apt-get install -y \
        python3-pip python-is-python3 \
        ros-noetic-ackermann-msgs && \
    rm -rf /var/lib/apt/lists/*

# install pip dependencies
RUN python -m pip install pip --upgrade
ADD ./requirements.txt /app/requirements.txt
RUN python -m pip install -r /app/requirements.txt

# test + lint the source code
ADD ./node /app/src/local_planner
WORKDIR /app/src/local_planner/src
RUN source /opt/ros/noetic/setup.bash && \
    python -m pytest tests && \
    pylint local_planner --fail-under=9.5 --extension-pkg-whitelist=cv2

# deploy the source code to ROS using catkin
WORKDIR /app/catkin_ws/src
RUN source /opt/ros/noetic/setup.bash && catkin_init_workspace && ln -s /app/src .
WORKDIR /app/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make

# register a startup procedure to launch the ROS node
ADD ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["local_planner", "local_planner.launch", "--wait"]
