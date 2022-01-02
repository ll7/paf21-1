"""Builds our Statemachine"""
from local_planner.preprocessing import SingletonMeta
from local_planner.vehicle_control.vehicle import Vehicle


class SpeedStateMachine(metaclass=SingletonMeta):
    """Builds our Statemachine"""

    # pylint: disable=too-many-instance-attributes
    # pylint: disable=chained-comparison.
    # pylint: disable=too-few-public-methods

    def __init__(self):

        """Builds our Statemachine"""
        self.states = ['Stop', 'Accel', 'Keep', 'Brake']
        self.current_state = "Stop"

        # ['Green', 'Yellow', 'Red']
        self.vehicle = Vehicle()

        # Desicion of Traffic Light
        self.tl_state = 'Green'

        # Evaluation of Signs
        self.speed_limit_sign_ms = 50 / 3.6

        # Eval if Junction is free
        self.junction_free = True

        # Distance to possible breaking points (tl, stop line, ...)
        self.dist_next_obstacle_m = 1000

        # Evaluated Speed decision
        self.target_limit_ms = 50 / 3.6

    def update_state(self):
        """Builds our Statemachine"""
        if self.current_state == "Stop":
            if self.tl_state == "Green" and self.junction_free:
                self.current_state = 'Accel'

        elif self.current_state == "Accel":
            if self.target_limit_ms < self.vehicle.actual_velocity_mps \
                    or not self.junction_free or self.tl_state == 'Red' and self.brake_if_required():
                self.current_state = "Brake"
            elif self.target_limit_ms == self.vehicle.actual_velocity_mps:
                self.current_state = "Keep"

        elif self.current_state == "Keep":
            if self.target_limit_ms < self.vehicle.actual_velocity_mps \
                    or not self.junction_free or self.tl_state == 'Red' and self.brake_if_required():
                self.current_state = "Brake"
            elif self.target_limit_ms > self.vehicle.actual_velocity_mps \
                    and self.junction_free and self.tl_state == 'Green':
                self.current_state = "Accel"

        elif self.current_state == "Brake":
            if self.target_limit_ms > self.vehicle.actual_velocity_mps \
                    and self.junction_free and self.tl_state == 'Green':
                self.current_state = "Accel"
            elif self.target_limit_ms == self.vehicle.actual_velocity_mps:
                self.current_state = "Keep"
            elif self.vehicle.actual_velocity_mps == 0:
                self.current_state = "Stop"


        print(self.target_limit_ms, self.junction_free, self.tl_state)
        print(self.vehicle.actual_velocity_mps)
        print(self.current_state)

    def brake_if_required(self):
        wait_time_s = self._time_until_brake(self.dist_next_obstacle_m, self.target_limit_ms)
        # print(f'{distance_m}, {self.vehicle_reaction_time_s}')
        return wait_time_s <= self.vehicle.vehicle_reaction_time_s
    # in 20m muss ich 30 fahren, aktuell 50 -> in wie vielen senkunden muss ich anfangen zu bremsen

    def _time_until_brake(self, distance_m: float, target_velocity: float = 0) -> float:
        """Compute the braking distance and based on that the time until brake.
        In case this function returns a negative value, it means that it's already
        too late to brake, so you need to launch an emergency protocol"""

        accel_mps2 = self.vehicle.base_brake_mps2

        if distance_m < 0:
            raise ValueError('Negative distance is not allowed')
        if self.vehicle.actual_velocity_mps < 0 or target_velocity < 0:
            raise ValueError('Negative velocity is not allowed')
        if accel_mps2 >= 0:
            raise ValueError('Positive acceleration won\'t brake')

        maneuver_time_s = (target_velocity - self.vehicle.actual_velocity_mps) / accel_mps2
        braking_dist = self.vehicle.actual_velocity_mps * maneuver_time_s + \
                       accel_mps2 * maneuver_time_s ** 2 / 2
        time_until_brake = (distance_m - braking_dist) / self.vehicle.actual_velocity_mps \
            if self.vehicle.actual_velocity_mps > 0 else 0
        return time_until_brake

    def update_speed(self):
        action = self.current_state
        if action == 'Keep':
            return self.vehicle.actual_velocity_mps
        elif action == 'Stop':
            return 0
        elif action == 'Accel':
            return self.target_limit_ms #self.vehicle.actual_velocity_mps + self.vehicle.base_accel_mps2
        elif action == 'Brake':
            return self.target_limit_ms #self.vehicle.actual_velocity_mps + self.vehicle.base_brake_mps2
