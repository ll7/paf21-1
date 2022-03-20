
# Competition Manager

## About
This is a wrapper to integrate the official PAF competition manager
from https://github.com/ll7/paf_competition_manager. 

It basically just runs the
[simple_competition_manager.py](https://github.com/ll7/paf_competition_manager/blob/main/src/simple_competition_manager.py)
script inside a ROS Noetic environment printing to CLI how far our ego vehicle is away from start / goal.

## Build Node + Run Tests

```sh
docker build . -t "competition_manager"
```
