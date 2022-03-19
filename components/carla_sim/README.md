
# CARLA Simulator

## About
This is a modification of the official CARLA DockerHub image, running the CARLA simulation environment.
The changes made to it allow us to access the simulator's GUI via the host's X11 server through Docker isolation.

Showing the CARLA simulator's GUI directly is not in use anymore because we monitor our car with RVIZ.
So the simulator is basically started in 'server' mode and used as a GPU rendering backend to provide
realistic image sensor data for our perception layer.

## Docker Build

```sh
docker build . -t 'carla-sim'
```
