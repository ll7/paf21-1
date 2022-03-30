
# Obstacle Observer

## About
This component handles the Obstacle retrieved directly from the Perception layer. The callable methods provide [Speed Observations](https://github.com/ll7/paf21-1/wiki/Speed-State-Machine) and new replanned routes to avoid collisions.
## Components
The obstacle observer consists of following components:

### Obstacle Observer
####Properties
- objects: Dict[int, ObjectMeta]
- max_velocity_change_rate: float (limits the change of velocity for a detected object to prevent unrealistic velocities)
- street_width: float 
- dist_safe: int (parameter to set a fixed safe distance for overtaking)
####Interfaces
######update_objects
Updates the object list, their position and orientation, based on an input list of Objects. This is the interface used to write from Perception layer to the obstacle observer
######get_speed_observation
Retrieve the speed observation containing the information about blocking vehicles. Considers Obstacles blocking the current trajectory
######plan_overtaking_maneuver
Replans a given route around an object, and returns None if there was no valid overtaking route found  

### Object Meta
Dataclass representing an obstacle
####Properties
- identifier: int (Unique Identifier to )
- obj_class: str (Pedestrian or Vehicle)
- trajectory: List[Tuple[float, float]]
- velocity: float = 0.0
- max_vel_change_rate = 2.0 (Limits change of velocity for obstacles between 2 ticks)
- max_trajectory_entries: int = 5 (Limit maximum number of points saved for obstacles trajectory)
####Interfaces
######update_object_meta
Updates the data in the obstacle object
## References
Have a look at this [wiki page](https://github.com/ll7/paf21-1/wiki/Obstacle-Observer)
for further information on the obstacle observation tasks.
