
# Dockerized Simulation Environment

## Build the CARLA ROS Docker Images

```sh
docker-compose -f carla-images-build.yml build
```

## Launch the Simulation + ROS Bridge

```sh
xhost +
docker-compose -f carla-rosbridge-compose.yml up
```

## Spawn Some Vehicles and Pedestrians

```sh
docker-compose -f carla-rosbridge-compose.yml exec carla-ros-bridge \
    /bin/sh -c 'cd /opt/carla/PythonAPI/examples && python spawn_npc.py -n 50 -w 100 --host carla-simulator'
```
