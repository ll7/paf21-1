
# Traffic Light Detection

## About
This ROS node processes camera data from the CARLA simulator to detect traffic lights.

## Build Node + Run Tests

```sh
docker build . -t "traffic_light_detection"
```

## TensorFlow Training
The traffic light detection uses a small convolutional neural network (~1k weights).
It's implemented in TensorFlow and has an accuracy of ~99 %.

The weights are already pre-trained. But if you want to train them once again,
have a look at [this tutorial](./node/src/traffic_light_detection/README.md).
