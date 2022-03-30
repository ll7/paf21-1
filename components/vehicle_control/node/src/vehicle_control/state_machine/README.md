
# Speed State Machine

## About
This component makes sure to transition the acceleration beheaviour based on the the input from the Perception layer. To accomplish this the state machine uses the states [ACCEL, BRAKE, KEEP]. A input in form of a Sensor Fusion called SpeedObservation delivers input information to the Speed State Machine where a transition Function will determine the next State.

## Components
The local planner consists of following components:

### Speed State Machine
####Properties
- current_state: SpeedState
- target_speed_mps: float
- legal_speed_limit_mps: float
- speed_offset_up_mps: float
- speed_offset_down_mps: float
####Interfaces
######update_state
Takes a speed observation and changes the state of the state machine using the transition functions
######get_target_speed
retrieves the target speed based on the current state

### Speed Observation
Dataclass containing import information for state transitioning

- tl_phase: TrafficLightPhase
- is_trajectory_free: bool
- dist_next_traffic_light_m: float
- dist_next_obstacle_m: float
- detected_speed_limit: int
- obj_speed_ms: float
- dist_next_curve: float
- curve_target_speed: float
- dist_next_stop_m: float

## References
Have a look at this [wiki page](https://github.com/ll7/paf21-1/wiki/Speed-State-Machine)
for further information on the decision making tasks.
