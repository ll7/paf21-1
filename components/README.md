
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
which is not feasible*

For running the automated integration tests, execute following command:

```sh
# simulate ROS messages and test whether containers fail to handle the messages
docker-compose -f integration-tests-compose.yml up --abort-on-container-exit

# evaluate the error code of docker-compose
if [ $? -eg 0 ]; then
    echo 'success!'
else
    echo 'failure!'
fi
```

## CARLA Simulation Components
The CARLA simulation involves following components:
- **CARLA simulator**: runs the basic CARLA simulation environment including world rendering, etc.
- **CARLA ROS bridge**: interfaces for information exchange between CARLA and ROS
- **Scenario Runner**: launches and processes scenarios defined as Open Scenario files
- **CARLA RVIZ**: spectates the remote-controlled car and its sensors / actuators

## Driving Components (ROS)
The components for driving are the following:
- **Global Planner**: serves navigation tasks based on map data
- **Local Planner**: serves local decision-making tasks and amplifies the route based on sensor data
- **Vehicle Controller**: transforms the route waypoints etc. into actionable remote-control signals
