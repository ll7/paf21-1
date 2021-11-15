
# base on the original CARLA ROS bridge image from GitHub
ARG ROS_BRIDGE_BASE_IMG
FROM $ROS_BRIDGE_BASE_IMG as base

# supply the base image with an environment supporting ROS UI via x11
FROM osrf/ros:noetic-desktop-full-focal
COPY --from=base /opt /opt

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

# install pip
RUN apt-get update && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*

# alias python3 as python
RUN apt-get update && apt-get install -y python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install pip --upgrade \
    && python -m pip install simple_pid pygame

# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=/opt/carla-ros-bridge/install/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

# override the entrypoint with a more configurable one
COPY ./ros_entrypoint.sh /ros_entrypoint.sh

# set the CARLA simulator to localhost
ENV CARLA_SIM_HOST=localhost

# set the timer waiting for the CARLA simulator to launch
ENV CARLA_SIM_WAIT_SECS=20

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["carla_ros_bridge", "carla_ros_bridge.launch", "--wait"]
