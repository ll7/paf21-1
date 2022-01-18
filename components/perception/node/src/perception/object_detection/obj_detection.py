"""A module that detects traffic lights"""

from typing import Tuple
import yaml

import numpy as np
from perception.base_detector import BaseDetector
# from perception.object_detection.obj_info import Point, ObjectStatus, ObjectInfo


class ObjectDetector(BaseDetector):
    """A module that detects traffic lights"""
    # pylint: disable=too-few-public-methods

    def __init__(self, config_path: str):
        super().__init__()
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.lower_mask: Tuple[int, int, int] = config['lower_bound']
            self.upper_mask: Tuple[int, int, int] = config['upper_bound']
            self.box_offset: int = config['box_offset']
        self.counter: int = 1

    def detect_object(self, semantic_image: np.ndarray, depth_image: np.ndarray):
        """main function to get traffic lights and distance"""
        tl_info = None
        patches = self.find_object_patches(semantic_image)

        if len(patches) > 0:
            distances = BaseDetector.object_distances(patches, depth_image)

            # nearest_obj_id = np.argmin(distances)
            # dist, patch = distances[nearest_obj_id], patches[nearest_obj_id]
            # scaling_factor = (rgb_image.shape[0] // semantic_image.shape[0],
            #                   rgb_image.shape[1] // semantic_image.shape[1])
            # rgb_patch = [patch[0] * scaling_factor[0], patch[1] * scaling_factor[1],
            #              patch[2] * scaling_factor[0], patch[3] * scaling_factor[1]]
            #
            # if dist < 100:
            #     cut_rgb_image = ObjectDetector._cut_image_patch(rgb_patch, rgb_image)
            #     tl_phase = self._classify_tl_phase(cut_rgb_image)
            #     tl_info = TrafficLightInfo(phase=tl_phase, distance=dist)
            #
            #     if self.counter % 10 == 0 and self.counter < 10000:
            #         log_image = ObjectDetector._print_text_to_image(
            #             dist, tl_phase, rgb_image, rgb_patch)
            #         cv2.imwrite(f'/app/logs/test_data_tl_{self.counter}.png', log_image)
            #     self.counter += 1

        return tl_info, distances
