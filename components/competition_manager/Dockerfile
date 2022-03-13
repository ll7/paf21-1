
ARG CARLA_VERSION=0.9.10.1
FROM carlasim/carla:$CARLA_VERSION as carla

FROM ros:noetic
COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

RUN apt-get update && apt-get install -y python3-pip git python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
ENV PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla

WORKDIR /opt/competition_manager/src
ARG CM_GITHUB_URL=https://github.com/ll7/paf_competition_manager
RUN git clone $CM_GITHUB_URL . && \
    source /opt/ros/noetic/setup.bash && \
    catkin_init_workspace && cd .. && catkin_make install
WORKDIR /

# register a startup procedure to launch the ROS node
ADD 
ADD ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
