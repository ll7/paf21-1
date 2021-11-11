
# Test Subscriber Camera

## About
This is just a ROS node made for testing purposes.
Use it as a blueprint for creating new ROS nodes.

## Build Node

```sh
docker build . -t "test_subscriber_camera"
```

## Run Node

Create a file named docker-compose.yml with following content:

```yml
version: "2"

services:
  roscore:
    image: ros:noetic
    command: roscore
    expose:
      - 11311
    networks:
      - ros

  test-subscriber-camera:
    image: test_subscriber_camera
    environment:
      ROS_MASTER_URI: http://roscore:11311
      ROS_HOSTNAME: test-subscriber-camera
    networks:
      - ros

  test-sender-camera:
    image: ros:noetic
    command: /bin/bash -c 'sleep 10 && rostopic pub "/carla/ego_vehicle/camera/depth/front/camera_info" sensor_msgs/CameraInfo "some message to be sent" -r 10'
    environment:
      ROS_MASTER_URI: http://roscore:11311
      ROS_HOSTNAME: test-subscriber-camera
    networks:
      - ros

networks:
  ros:
```

Run the docker-compose file using following command:

```sh
docker-compose up -d
```
