
# Default Simulation Scenario

## Quickstart 
For a quickstart, move one directory up and see the launch.sh file usage.
It basically automates all steps described here (continue reading if you want to understand
what the launch script does).

## Manual Launch
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
*See [this tutorial](../components/README.md) for further information on the build procedure.*

## Loading Competition Scenarios
The ROS bridge node fulfills the purpose of loading the system configuration,
launching all ROS nodes related to the ROS bridge, spawning cars / pedestrians, etc.
Go [here](../components/carla_ros_bridge/ros_entrypoint.sh) for further information.

### Registering Competition Configurations
For launching competition comfigurations, just put them inside the config folder
of the related town accordingly. The docker-compose setup will propagate it to the system.
