
# Dockerized Simulation Environment

## Build the ROS Bridge Docker Image

```sh
git clone https://github.com/carla-simulator/ros-bridge
cd ros-bridge

git checkout 0.9.10.1
git submodule update --init --recursive

cd docker
docker build -t carla-ros-bridge -f Dockerfile ./.. "$@" \
    --build-arg ROS_VERSION=noetic --build-arg CARLA_VERSION=0.9.10.1
```

## Launch the Simulation + ROS Bridge

```sh
docker-compose -f carla-rosbridge-compose.yml up
```

## Spawn Some Vehicles and Pedestrians

```sh
docker-compose -f carla-rosbridge-compose.yml exec carla-ros-bridge \
    /bin/sh -c 'cd /opt/carla/PythonAPI/examples && python spawn_npc.py -n 50 -w 100 --host carla-simulator'
```
