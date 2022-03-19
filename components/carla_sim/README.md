
# CARLA Simulator

## About
This is a modification of the official CARLA DockerHub image, running the CARLA simulation environment.
The changes made to it allow us to access the simulator's GUI via the host's X11 server through Docker isolation.

*Note: This feature is not in use anymore because we monitor our car with RVIZ. So the simulator
is basically started in 'server' mode.*

## Docker Build

```sh
docker build . -t 'carla-sim'
```
