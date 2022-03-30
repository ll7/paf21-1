
# Perception

## About
This ROS node processes camera data (semantic, rgb, depth) from the CARLA simulator to detect traffic lights and objects like cars and persons.

Wiki: [Perception](https://github.com/ll7/paf21-1/wiki/Perception)


## TensorFlow Training
The traffic light detection uses a small convolutional neural network (~1k weights).
It's implemented in TensorFlow and has an accuracy of ~99 %.

The weights are already pre-trained. But if you want to train them once again,
have a look at [this tutorial](node/src/perception/traffic_light_detection/README.md).

## Build Node + Run Tests

```sh
docker build . -t "perception"
```
