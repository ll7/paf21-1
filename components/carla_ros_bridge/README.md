
# Launch Service

## About
This ROS node launches a given scenario specified by a yaml file.

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
```

## Build Node + Run Tests

```sh
docker build . -t "launch_service"
```
