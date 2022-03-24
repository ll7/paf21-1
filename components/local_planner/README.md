
# Local Planner

## About
This ROS node processes route data from the global planner and sensor data
from the perception layer to plan trajectories and drive them accurately.

## Components
The local planner consists of following components:

### Sensor Fusion
All sensor information such as positioning, surrounding NPCs, traffic lights / speed signs
are collected and fused together to a cohesive system state that gets processed by
the situational planning layer.

### Situational Planning
For situational planning we use a couple of state machines mainly for regulating
the target speed and whether to execute specific maneuvers like e.g. overtaking.

This is not particularly sophisticated in any way and needs some refinement through
a more statistic / probabilistic approach as the state machines are rather hard-wired
and optimized for the few maps of the competition.

### Vehicle Control
The planned trajectory and suggested speed is executed by the vehicle controller. It uses
the Stanley method (steering) and the PID controller (speed / acceleration) of the
Ackermann Drive ROS node provided by the ROS bridge.

For accurate fast driving (~ 130 km/h) we had to make a couple of enhancements to the
steering controller such as speed-based aim point selection (farther away the faster the
car drives) and integration smoothing of successive heading / cross-track errors.
Moverover, we had to increase the driving signal rate to ~ 100 Hz in oder to react
each 0.36 m the car covers.

## Build Node + Run Tests

```sh
docker build . -t "local_planner"
```

## References
Have a look at this [wiki page](https://github.com/ll7/paf21-1/wiki/Local-Planner)
for further information on the planning tasks.
