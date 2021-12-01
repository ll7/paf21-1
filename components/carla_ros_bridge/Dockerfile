
# # base on the original CARLA ROS bridge image from GitHub
# ARG ROS_BRIDGE_BASE_IMG
# FROM $ROS_BRIDGE_BASE_IMG as base

ARG CARLA_VERSION
FROM carlasim/carla:$CARLA_VERSION as carla

# supply the base image with an environment supporting ROS UI via x11
FROM osrf/ros:noetic-desktop-full-focal
COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

# install rendering dependencies for rviz / rqt
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    freeglut3-dev \
  && rm -rf /var/lib/apt/lists/*# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
ENV PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla

# install pip, wget, git and some ROS dependencies for building the CARLA ROS bridge
RUN apt-get update && apt-get install -y python3-pip wget git python-is-python3 \
        ros-noetic-ackermann-msgs \ros-noetic-derived-object-msgs \
        ros-noetic-carla-msgs ros-noetic-pcl-conversions \
        ros-noetic-rviz ros-noetic-rqt ros-noetic-pcl-ros && \
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
WORKDIR /opt/carla-ros-bridge/src
ARG ROS_BRIDGE_GITHUB_URL=https://github.com/carla-simulator/ros-bridge
RUN git clone --recurse-submodules $ROS_BRIDGE_GITHUB_URL . && \
    git checkout 0.9.10.1 && source /opt/ros/noetic/setup.bash && \
    catkin_init_workspace && cd .. && catkin_make install
WORKDIR /
ENV PYTHONPATH=$PYTHONPATH:/opt/carla-ros-bridge/install/lib/python3/dist-packages

# install simple_pid
RUN python -m pip install pip --upgrade \
    && python -m pip install simple_pid pygame

# install the scenario runner from GitHub release source
ENV CARLA_ROOT=/opt/carla
ENV SCENARIO_RUNNER_ROOT=/opt/scenario_runner
ARG SCENARIO_RUNNER_URL=https://github.com/carla-simulator/scenario_runner/archive/refs/tags/v0.9.10.tar.gz
ARG SCENARIO_RUNNER_TAR=scenario_runner.tar.gz
RUN cd /opt && wget $SCENARIO_RUNNER_URL -O $SCENARIO_RUNNER_TAR && \
    tar -xf $SCENARIO_RUNNER_TAR && mv scenario_runner-0.9.10 scenario_runner && \
    rm -rf $SCENARIO_RUNNER_TAR
RUN echo 'pexpect' >> $SCENARIO_RUNNER_ROOT/requirements.txt && \
    python -m pip install -r $SCENARIO_RUNNER_ROOT/requirements.txt

ENV CARLA_SIM_HOST=localhost
ENV CARLA_SIM_WAIT_SECS=20
ENV SCENARIO_RUNNER_PATH=/opt/scenario_runner

COPY ./rosbridge_with_rviz_scenario.launch /rosbridge_with_rviz_scenario.launch
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["/rosbridge_with_rviz_scenario.launch", "--wait"]
