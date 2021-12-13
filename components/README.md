
# Dockerized Components

## Build all Docker Images
Use following command to build all Docker images you'll need for launching
the CARLA simulation and the driving components.

```sh
docker-compose -f carla-sim-build.yml build
docker-compose -f driving-components-build.yml build
```

*Note: Splitting up the components into CARLA simulation and driving was required
to speed up the server-side GitHub workflows because building the Docker images
from scratch on each commit would imply downloading ~15 GB of pre-built Docker images
which is not feasible at all*

## CARLA Simulation Components
The CARLA simulation involves following components:
- **CARLA simulator**: runs the basic CARLA simulation environment including world rendering, etc.
- **CARLA ROS bridge**: interfaces for information exchange between CARLA and ROS
- **Scenario Runner**: launches and processes scenarios defined as Open Scenario files
- **CARLA RVIZ**: spectates the remote-controlled car and its sensors / actuators

## Driving Components (ROS)
The components for driving are the following:
- **global planner**: serves navigation tasks based on map data
- **local planner**: serves local decision-making tasks and amplifies the route based on sensor data
- **vehicle controller**: transforms the route waypoints etc. into actionable remote-control signals
