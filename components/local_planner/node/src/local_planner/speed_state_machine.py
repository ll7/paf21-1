"""Builds our Statemachine"""
from local_planner.preprocessing import SingletonMeta

class SpeedStateMachine(metaclass=SingletonMeta):
    """Builds our Statemachine"""

    # pylint: disable=too-many-instance-attributes
    # pylint: disable=chained-comparison.
    # pylint: disable=too-few-public-methods

    def __init__(self):

        """Builds our Statemachine"""
        self.states = ['Stop', 'Accel', 'Keep', 'Brake']
        self.current_state = "Stop"
        #['Green', 'Yellow', 'Red']
        self.tl_state = 'Green'
        # self.speed_limit = 50
        self.target_limit = 50
        self.current_limit = 50
        self.junction_free = True

    def update_state(self):
        """Builds our Statemachine"""
        if self.current_state == "Stop":
            if self.tl_state == "Green" and self.junction_free:
                self.current_state = 'Accel'

        elif self.current_state == "Accel":
            if self.target_limit < self.current_limit\
                    or not self.junction_free or self.tl_state == 'Red':
                self.current_state = "Brake"
            elif self.target_limit == self.current_limit:
                self.current_state = "Keep"

        elif self.current_state == "Keep":
            if self.target_limit < self.current_limit\
                    or not self.junction_free or self.tl_state == 'Red':
                self.current_state = "Brake"
            if self.target_limit > self.current_limit\
                    and self.junction_free and self.tl_state == 'Green':
                self.current_state = "Accel"

        elif self.current_state == "Brake":
            if self.target_limit > self.current_limit\
                    and self.junction_free and self.tl_state == 'Green':
                self.current_state = "Accel"
            elif self.target_limit == self.current_limit:
                self.current_state = "Keep"
            elif self.current_limit == 0:
                self.current_state = "Stop"
