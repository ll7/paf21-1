
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

## Run the Integration Tests
Testing the driving components against the CARLA simulator might sometimes be a bit tricky.
Therefore, the integration tests are mocking CARLA with pre-recorded ROS bag files.
This might be especially useful for developing / testing on machines without NVIDIA GPU.

For preparing the integration test assets, run following command:

```sh
./prepare-integration-test-assets.sh
```

Run following command to execute the integration tests:

```sh
docker-compose -f integration-tests-compose.yml up --abort-on-container-exit
```

## CARLA Simulation Components
The CARLA simulation involves following components:
- **CARLA simulator**: runs the basic CARLA simulation environment including world rendering, etc.
- **CARLA ROS bridge**: interfaces for information exchange between CARLA and ROS
  - **Scenario Runner**: launches and monitors well-defined scenarios
  - **CARLA RVIZ**: spectates the remote-controlled car and its sensors / actuators
  
## Driving Components (ROS)
The components for driving are the following:
- **Global Planner**: serves navigation tasks based on map data [Details](https://github.com/ll7/paf21-1/wiki/Global-Planner)
- **Perception**: preprocesses sensor data into high-level driving information [Details](https://github.com/ll7/paf21-1/wiki/Perception)
  - **Traffic Light Detection**: evaluates traffic lights captured by cameras
  - **Object Detection**: tracks stationary / moving objects like other cars / pedestrians and predicts their future movement
- **Local Planner**: serves local decision-making tasks and amplifies the route based on sensor data [Details](https://github.com/ll7/paf21-1/wiki/Local-Planner)
  - **Vehicle Controller**: transforms the planned trajectory into actionable remote-control signals
