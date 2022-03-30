## driving
This component handles the steering and acceleration of the vehicle and is able to detect curves in the trajectory.
## Components
Driving consists of following components:

### driving_control.py
This File Contains:

- DrivingController
  - sets the steering controller
  - returns a DrivingSignal
- Driving Signal Data Class
  - steering_angle_rad: float
  - target_velocity_mps: float

### steering.py
This File Contains the following steering controller to steer the car.

- [StanleySteeringController](https://github.com/ll7/paf21-1/wiki/Vehicle-Controller)
  - [predictive_stanley](https://github.com/ll7/paf21-1/wiki/Vehicle-Controller)
- NaiveSteeringController
  set steering straight towards the next point in the trajektory
- CombinedSteering
  combines Stanley (for straights) and Naive Steering (for curves)

### curve_detection.py

- CurveDetection
  returns a CurveObservation
- CurveObservation
  - dist_until_curve: float
  - dist_end_curve: float
  - max_speed: float
  - end_id: int=-1

## References
Have a look at this [wiki page](https://github.com/ll7/paf21-1/wiki/Vehicle-Controller)
for further information on the Vehicle Controller tasks.
