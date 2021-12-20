
# Local Planner

## About
This ROS node processes route data from the global planner and integrates
it into the vehicle's environments using e.g. camera sensors, gps.

## Build Node + Run Tests

```sh
docker build . -t "local_planner"
```

## Modul Overview

### lane_detection.py

Uses the OpenCV-Libary to detetect the Lanes of the Road.

![](/home/daniel/paf21-1/imgs/Lane_detection.png)

Blue: lane delimitation<br />
Red: direction of the car<br />
Green: middle of the lane<br />

Support access to:<br />
projections: Current Image with lane delimitations<br />
keep_lanes: True if lanes are detected<br />
angle: distance from car to middle of lane + angle between car and middle of road<br />

### main.py

Local Planer Ros-Node<br />

Subscriptions:<br />
OdometryMsg: GPS-Information<br />
ImuMsg: vehicle orientation<br />
ImageMsg: semantic-image, depth-image and RGB-image<br />
StingMSG: Global-Route<br />

Publisher:<br />
StringMsg: Local-Route: add Local Waypoints to the Global-Route-Messages

### preprocessing.py

Loads and Preprocess the semantic-image, depth-image and RGB-image from the Ros-Messages

### route_planner.py

inserts new points in the route so that the next point is on the current center of the street

### traffic_light_detection.py

in progress