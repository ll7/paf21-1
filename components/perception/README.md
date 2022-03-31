
# Perception

## About
This ROS node processes camera data (semantic, rgb, depth) from the CARLA simulator to detect
traffic lights and objects like cars and persons.

See the [perception wiki page](https://github.com/ll7/paf21-1/wiki/Perception)
for further information on the technical implementation.

## ROS Data Interface
This component interacts with ROS as follows:

### Incoming Messages
The incoming sensor messages are issued by camera sensors, located at the car's front.
All cameras are looking at the same scenery such that images can be overlayed seemlessly.
For real-world use cases this might be a bit more difficult as cameras cannot be
placed on top of each other.

- /carla/<vehicle_name>/camera/semantic_segmentation/front/image_segmentation of format sensor_msgs/Image
- /carla/<vehicle_name>/camera/depth/front/image_depth of format sensor_msgs/Image
- /carla/<vehicle_name>/camera/rgb/front/image_color of format sensor_msgs/Image

### Outgoing Messages
The outgoing sensor messages outline the locations and features of detected traffic lights / objects.

- /drive/<vehicle_name>/object_info of format std_msgs/String
- /drive/<vehicle_name>/tld_info of format std_msgs/String

A valid object_info message contains data such as the following.
It outlines that there's another car located relatively to our car's position
in 2.5 meters to the side and 10.4 meters to the front that's identified by id=1.

```json
{
    "identifier": 1,
    "obj_class": "vehicle",
    "rel_position": [ 2.5, 10.4 ]
}
```

A valid tl_info message contains data such as the following.
It outlines that there's a traffic light showing green 10 meters ahead.

```json
{
    "phase": 1,
    "distance": 10.65,
    "accuracy": 0.95
}
```

## TensorFlow Training
The traffic light detection uses a small convolutional neural network (~1k weights).
It's implemented in TensorFlow and has an accuracy of ~99 %.

The weights are already pre-trained. But if you want to train them once again,
have a look at [this tutorial](node/src/perception/traffic_light_detection/README.md).

## Build Node + Run Tests

```sh
docker build . -t "perception"
```
