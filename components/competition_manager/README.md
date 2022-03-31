
# Competition Manager

## About
This is a wrapper to integrate the official PAF competition manager
from https://github.com/ll7/paf_competition_manager. 

It basically just runs the
[simple_competition_manager.py](https://github.com/ll7/paf_competition_manager/blob/main/src/simple_competition_manager.py)
script inside a ROS Noetic environment printing to CLI how far our ego vehicle is away from start / goal.

*Note: Actually, there were performance issues because the original competition manager did send quite a lot
of network messages, resulting in a queue overflow at the ROS param server that lead to not detecting that the car
had already reached the goal. So, we wrote our own competition manager, now caching the requested data, such that the
amount of network traffic is reduced to a minimum causing no problems.*

## Build Node + Run Tests

```sh
docker build . -t "competition_manager"
```
