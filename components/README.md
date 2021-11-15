
# Dockerized Simulation Environment

## Build the CARLA ROS Docker Images
First, build all Docker images we'll need for launching the simulation.

```sh
docker-compose -f carla-images-build.yml build
```

## Launch the Simulation + ROS Bridge
Now, before you launch the Docker container, you need to allow the x11 server on the host system
to be accessed by the rqt application inside the Docker container like this:

```sh
xhost +
```

Next, launch the Docker containers with the following command:

```sh
docker-compose -f carla-rosbridge-compose.yml up -d
```

## Send Ackermann Control Signals
Now, everything is set up to send some driving signals to the vehicle. As a quick example,
we'll be sending it a command to drive straight forward which gets repeated every 10 seconds.

```sh
docker-compose -f carla-rosbridge-compose.yml exec carla-ros-bridge \
    /bin/bash -c 'source /opt/carla-ros-bridge/install/setup.bash && \
        rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive \
            "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10'
```

To prevemt the vehicle from crashing, send a signal to stop it like this:

```sh
docker-compose -f carla-rosbridge-compose.yml exec carla-ros-bridge \
    /bin/bash -c 'source /opt/carla-ros-bridge/install/setup.bash && \
        rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive \
            "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0, acceleration: 0.0, jerk: 0.0}" -r 10'
```

## Spawn Some Vehicles and Pedestrians
In case you need other actors, you can spawn some vehicles and pedestrians.

```sh
docker-compose -f carla-rosbridge-compose.yml exec carla-ros-bridge \
    /bin/sh -c 'cd /opt/carla/PythonAPI/examples && python spawn_npc.py -n 50 -w 100 --host carla-simulator'
```
