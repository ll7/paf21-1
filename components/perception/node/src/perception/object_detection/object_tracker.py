"""A module that track objects"""

from typing import Tuple, List
from math import dist as euclid_dist

import numpy as np

from perception.object_detection.object_info import ObjectInfo


class ObjectTracker:
    """Represents an object tracker that works on the basis of the euclidean distance."""
    def __init__(self):
        self.object_infos: List[ObjectInfo] = []
        self.id_count = 0
        self.max_distance = 5.0

    def update(self, points: np.ndarray, object_classes: List[str]) -> List[ObjectInfo]:
        """Update the object infos for the list of points."""
        obj_infos = []
        for point, object_class in zip(points, object_classes):
            point = (point[0], point[1])
            obj_infos.append(self.find_nearest_object(point, object_class))

        self.object_infos = obj_infos
        return obj_infos

    def find_nearest_object(self, point: Tuple[float, float], object_class: str) -> ObjectInfo:
        """Return the object info of the nearest saved centroid."""
        same_object_detected = False
        nearest_point_id = -1
        nearest_dist = self.max_distance

        for obj_info in self.object_infos:
            # calculate the euclid distance between the new point and the last relativ position
            distance = euclid_dist(point, obj_info.rel_position)
            if distance < nearest_dist and obj_info.obj_class == object_class:
                # detected the same object
                same_object_detected = True
                nearest_point_id = obj_info.identifier
                nearest_dist = distance

        if same_object_detected is False:
            # set for new object unique identifier
            nearest_point_id = self.id_count
            self.id_count += 1

        return ObjectInfo(identifier=nearest_point_id, obj_class=object_class,
                          rel_position=point)
