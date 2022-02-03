"""A module that detects traffic lights"""

from typing import Tuple, List, Dict
import yaml

from cv2 import cv2
import numpy as np

# pylint: disable=no-name-in-module
from tensorflow.keras import Model
from tensorflow.keras.models import load_model

from perception.traffic_light_detection.preprocessing import resize_image
from perception.traffic_light_detection.tld_info import TrafficLightPhase, TrafficLightInfo
from perception.base_detector import BaseDetector


class TrafficLightDetector(BaseDetector):
    """A module that detects traffic lights"""
    # pylint: disable=too-few-public-methods

    def __init__(self, config_path: str):
        super().__init__()
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.model: Model = load_model(config['weights_path'])
            self.input_shape: Tuple[int, int, int] = config['nn_input_size']
            self.classes_dict: Dict[int, str] = config['classes']
            self.mask: Tuple[int, int, int] = config['tl_mask']
            # self.scaling: float = 1 / config['scaling_factor']
            self.box_offset: int = config['box_offset']
        self.counter: int = 1

    def detect_traffic_light(self, semantic_image: np.ndarray, rgb_image: np.ndarray,
                             depth_image: np.ndarray) -> TrafficLightInfo or None:
        """main function to get traffic lights and distance"""
        tl_info = None
        patches = self.find_object_patches(semantic_image)

        if len(patches) > 0:
            distances = self.object_distances(patches, depth_image)

            nearest_obj_id = np.argmin(distances)
            dist, patch = distances[nearest_obj_id], patches[nearest_obj_id]
            scaling_factor = (rgb_image.shape[0] // semantic_image.shape[0],
                              rgb_image.shape[1] // semantic_image.shape[1])
            rgb_patch = [patch[0] * scaling_factor[0], patch[1] * scaling_factor[1],
                         patch[2] * scaling_factor[0], patch[3] * scaling_factor[1]]

            if dist < 100:
                cut_rgb_image = TrafficLightDetector._cut_image_patch(rgb_patch, rgb_image)
                tl_phase = self._classify_tl_phase(cut_rgb_image)
                tl_info = TrafficLightInfo(phase=tl_phase, distance=dist)

                if self.counter % 10 == 0 and self.counter < 0:
                    message = f'Distance: {dist} m, Color:{tl_phase}'
                    log_image = self.print_text_to_image(message, rgb_image, rgb_patch)
                    cv2.imwrite(f'/app/logs/test_data_tl_{self.counter}.png', log_image)
                self.counter += 1

        return tl_info

    @staticmethod
    def _cut_image_patch(patch: List[int], image: np.ndarray):
        x_1, y_1, x_2, y_2 = BaseDetector.get_corners(patch, image.shape)
        cut_img = image[y_1:y_2, x_1:x_2, :]
        return cut_img

    def _classify_tl_phase(self, image_bgr: np.ndarray) -> TrafficLightPhase:
        image_rgb = image_bgr[:, :, ::-1]
        resize_shape = (self.input_shape[0], self.input_shape[1])
        image_preprocess = resize_image(image_rgb, resize_shape)
        pred = np.squeeze(self.model(np.expand_dims(image_preprocess, axis=0)).numpy())
        class_id = int(np.argmax(pred))
        return TrafficLightPhase(class_id)
