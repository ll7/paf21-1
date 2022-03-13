
# ARG CARLA_VERSION=0.9.10.1
# FROM carlasim/carla:$CARLA_VERSION as carla

FROM ros:noetic
# COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

# set up the default shell used by RUN tags
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
        python3-pip git python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# # override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
# ENV PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
# ENV PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla

# copy / download source code
WORKDIR /app/src/competition_manager
ARG CM_GITHUB_URL=https://github.com/ll7/paf_competition_manager
RUN git clone $CM_GITHUB_URL . && mv src src_temp && mkdir -p src/competition_manager && \
    mv src_temp src/competition_manager
ADD ./launch ./launch
ADD ./CMakeLists.txt ./CMakeLists.txt
ADD ./CMakeLists.txt ./CMakeLists.txt
ADD ./CMakeLists.txt ./CMakeLists.txt
RUN ls -la /app/src/competition_manager/src

# build the code inside a catkit workspace
WORKDIR /app/catkin_ws/src
RUN source /opt/ros/noetic/setup.bash && catkin_init_workspace && ln -s /app/src .
WORKDIR /app/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make

# register a startup procedure to launch the ROS node
ADD ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["competition_manager", "competition_manager.launch", "--wait"]
