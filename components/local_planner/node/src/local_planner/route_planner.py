"""A route planner based on map and sensor data"""

from math import dist
from typing import List, Tuple
import rospy
from numpy.core.fromnumeric import argsort
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import NavSatFix as GpsMsg, Imu as ImuMsg


class RouteInfoMeta(type):
    """
    Makes sure every module uses the same data
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class RouteInfo(metaclass=RouteInfoMeta):
    """A class that keeps all the necessary information needed by all the modules,
    this class should be expanded if needed"""
    vehicle_vector: Tuple[float, float] = None
    vehicle_position: Tuple[float, float] = None
    global_route: List[Tuple[float, float]] = None
    cached_local_route: List[Tuple[float, float]] = None

    def update_vehicle_vector(self, msg: ImuMsg):
        """Calculates the vektor the car is currently driving"""
        rospy.loginfo(msg.orientation)
        length = (msg.orientation.x ** 2 + msg.orientation.y ** 2)**0.5
        if length > 0:
            self.vehicle_vector = [msg.orientation.x / length, msg.orientation.y / length]
        else:
            self.vehicle_vector = [msg.orientation.x, msg.orientation.y]
        rospy.loginfo(self.vehicle_vector)

    def update_gps(self, msg: GpsMsg):
        """Update the GPS position of the vehicle"""
        self.vehicle_position = (msg.longitude, msg.latitude)
        rospy.loginfo(self.vehicle_position)

    def update_global_route(self, msg: StringMsg):
        """Update the global route to follow"""
        self.global_route = msg
        # call the json parser to extract the data, etc.

    def compute_local_route(self):
        """Modifies the global route based on sensor data
        to fit the current diving circumstances"""

        if self.global_route is None:
            return []

        # interpolate the route such that the route points are closely aligned

        # filter the parts of the global route of the near future
        neighbour_ids = argsort([dist(p, self.vehicle_position) for p in self.global_route])
        cached_local_route = self.global_route[max(neighbour_ids[0], neighbour_ids[1]):]
        cached_local_route = cached_local_route[:min(10, len(cached_local_route))]
        # put more sophisticated route computation here ...

        return cached_local_route
