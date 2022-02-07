"""A module that track objects"""

from typing import Tuple, List
from math import dist as euclid_dist

# from perception.object_detection.object_info import ObjectInfo
from object_info import ObjectInfo


class ObjectTracker:
    def __init__(self):
        self.object_infos: List[ObjectInfo] = []
        self.id_count = 0
        self.max_distance = 5.0

    def update(self, points: List[Tuple[float, float]],
               object_classes: List[str]) -> List[ObjectInfo]:
        """Update the object infos for the list of points."""
        obj_infos = []
        for point, object_class in zip(points, object_classes):
            obj_infos.append(self._find_nearest_object(point, object_class))

        self.object_infos = obj_infos
        return obj_infos

    def _find_nearest_object(self, point: Tuple[float, float], object_class: str) -> ObjectInfo:
        """Return the object info of the nearest saved centroid."""
        same_object_detected = False
        nearest_point_id = -1
        nearest_dist = self.max_distance

        for obj_info in self.object_infos:
            obj_id, obj_class, obj_pos = obj_info
            distance = euclid_dist(point, obj_pos)
            if distance < nearest_dist and obj_class == object_class:
                same_object_detected = True
                nearest_point_id = obj_id
                nearest_dist = distance

        if same_object_detected is False:
            nearest_point_id = self.id_count
            self.id_count += 1

        return ObjectInfo(identifier=nearest_point_id, obj_class=object_class,
                          rel_position=point)
