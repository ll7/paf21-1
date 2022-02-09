
# Default Simulation Scenario

## Quickstart 
Run the launch.sh for build, xhost + and docker-compose up -d all at once.
```sh
cd paf21-1/
./launch.sh 
```


## Launch the Simulation + ROS Bridge
Now, before you launch the Docker containers, you need to allow the x11 server on the host system
to be accessed by the rviz application inside the Docker container like this:

```sh
xhost +
```

Next, launch the Docker containers with the following command:

```sh
docker-compose up -d
```

*Note: Make sure to have the Docker images already pre-built prior to this step.*

## Send Ackermann Control Signals

```sh
docker-compose exec carla-ros-bridge \
    /bin/bash -c 'source /opt/carla-ros-bridge/install/setup.bash && \
        rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive \
            "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10'
```

## Spawn Some Vehicles and Pedestrians

```sh
docker-compose exec carla-ros-bridge \
    /bin/sh -c 'cd /opt/carla/PythonAPI/examples && python spawn_npc.py -n 50 -w 100 --host carla-simulator'
```
