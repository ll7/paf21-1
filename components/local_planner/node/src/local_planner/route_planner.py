"""A route planner based on map and sensor data"""

from math import dist
from typing import List, Tuple

from numpy.core.fromnumeric import argsort
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import NavSatFix as GpsMsg

class RoutePlanner:
    """A route planner based on map and sensor data"""

    vehicle_position: Tuple[float, float]
    global_route: List[Tuple[float, float]]
    cached_local_route: List[Tuple[float, float]]
    # next_target_position: Tuple[float, float]

    def update_gps(self, msg: GpsMsg):
        """Update the GPS position of the vehicle"""
        self.vehicle_position = (msg.longitude, msg.latitude)

    def update_global_route(self, msg: StringMsg):
        """Update the global route to follow"""
        self.global_route = msg
        # call the json parser to extract the data, etc.

    def compute_local_route(self):
        """Modifies the global route based on sensor data
        to fit the current diving circumstances"""

        # interpolate the route such that the route points are closely aligned

        # filter the parts of the global route of the near future
        neihgbour_ids = argsort([dist(p, self.vehicle_position) for p in self.global_route])
        self.cached_local_route = self.global_route[max(neihgbour_ids[0], neihgbour_ids[1]):]
        self.cached_local_route = self.cached_local_route[:min(10, len(self.cached_local_route))]

        # put more sophisticated route computation here ...

        return self.cached_local_route
