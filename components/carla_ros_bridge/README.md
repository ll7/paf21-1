
# Launch Service

## About

The CARLA ROS bridge combines a couple of components for translating between CARLA and ROS.
Moreover, it serves as an entrypoint for launching competition scenarios.

## Components

There are several components glued together by [ros_entrypoint.sh](./ros_entrypoint.sh) and
[rosbridge_with_rviz_scenario.launch](./rosbridge_with_rviz_scenario.launch).

### CARLA ROS Bridge

The ROS bridge translates all CARLA events / sensor signals into ROS messages. Moreover, it transforms our
driving signals into its proper CARLA representation.

### RVIZ + CARLA Spectator Camera

RVIZ is a standard ROS visualization tool to track actors / objects and outline their trajectory in the dashboard.

In case of this CARLA project, there's already a pretty nicely customized RVIZ dashboard available on the
official CARLA ROS bridge repository on GitHub which was slightly adapted to fit our needs.
(it's the file [augmented_rviz.rviz](./augmented_rviz.rviz))

Moreover, there's a ROS node carla_spectator_camera for augmenting the ego vehicle's surrounding with
bounding boxes, drawing the planned trajectory, etc. which we integrated in RVIZ.

### CARLA Ego Vehicle

For spawning ego vehicles, there's a ROS node called carla_ego_vehicle, doing exactly that.
Given the spawn point and sensors configuration, it creates a new car to be remote controlled by our
self-driving software. Our car's sensors are specified in the [sensors.json](./sensors.json) file.

### CARLA NPC Spawn

On the CARLA PythonAPI examples, there's a script [spawn_npc.py](https://github.com/carla-simulator/carla/blob/0.9.10.1/PythonAPI/examples/spawn_npc.py)
for spawning NPCs (other cars, pedestrians). It's called in our [ros_entrypoint.sh](./ros_entrypoint.sh)
to set up the scenery as the competition affords it.

### CARLA Ackermann Control

The Ackermann control facilitates AckermannDrive ROS messages to control the car's steering and velocity.
Moreover, the speed controller (PID) exposes a settings file [pid_settings.yaml](./pid_settings.yaml) which
we adjusted according to our needs (the default settings from CARLA GitHub don't drive as dynamic as we expected).

## Competition Scenario

This ROS node launches a given scenario specified by a yaml file (the files are located
[here](./../../scenarios/config)).

It supports features such as

- loading a CARLA town
- spawning a vehicle at a given position
- spawning a given amount of NPCs (vehicles, pedestrians)
- specifying whether to obey the driving rules

The yaml file format looks like this:

```yaml
competition:
  start:
    position: 
      x: 365.1388854980469
      y: -326.3156433105469
      z: 0.0013327598571777344
    orientation: 
      x: -1.1382753840112753e-05
      y: -2.3948581354485774e-07
      z: -0.9999569207962662
      w: 0.009282048374436797
  goal:
    position: 
      x: 154.44216918945312
      y: -30.514957427978516
      z: 0.0013357733841985464
    orientation: 
      x: -4.853172536190172e-06
      y: -4.497938599336769e-06
      z: -0.7444797552319263
      w: 0.667645035933037
  traffic_rules: true
  ready_for_ego: false
  town_name: Town01
  spawn_npc:
    n: 50
    w: 50
    s: 1
```

## Build Node + Run Tests

```sh
docker build . -t "launch_service"
```
