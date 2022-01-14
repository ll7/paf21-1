"""A module that detects traffic lights"""

from typing import Tuple, List, Dict
import yaml

from cv2 import cv2
import numpy as np

# pylint: disable=no-name-in-module
from tensorflow.keras import Model
from tensorflow.keras.models import load_model

from traffic_light_detection.preprocessing import resize_image
from traffic_light_detection.tld_info import TrafficLightPhase, TrafficLightInfo


class TrafficLightDetector:
    """A module that detects traffic lights"""
    # pylint: disable=too-few-public-methods

    def __init__(self, config_path):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.model: Model = load_model(config['weights_path'])
            self.input_shape: Tuple[int, int, int] = config['nn_input_size']
            self.classes_dict: Dict[int, str] = config['classes']
            self.lower_mask: Tuple[int, int, int] = config['lower_bound']
            self.upper_mask: Tuple[int, int, int] = config['upper_bound']
            self.box_offset: int = config['box_offset']
        self.counter: int = 1

    def detect_traffic_light(self, semantic_image: np.ndarray, rgb_image: np.ndarray,
                             depth_image: np.ndarray) -> TrafficLightInfo or None:
        """main function to get traffic lights and distance"""
        tl_info = None
        patches = self._find_traffic_light_patches(semantic_image)

        if len(patches) > 0:
            distances = TrafficLightDetector._object_distances(patches, depth_image)

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

                if self.counter % 10 == 0 and self.counter < 10000:
                    log_image = TrafficLightDetector._print_text_to_image(
                        dist, tl_phase, rgb_image, rgb_patch)
                    cv2.imwrite(f'/app/logs/test_data_tl_{self.counter}.png', log_image)
                self.counter += 1

        return tl_info

    def _find_traffic_light_patches(self, orig_image: np.ndarray):
        lower_mask = np.array(self.lower_mask)
        upper_mask = np.array(self.upper_mask)
        masked_image = cv2.inRange(orig_image, lower_mask, upper_mask)
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        idx = 0
        bounding_boxes = []
        for cnt in contours:
            idx += 1
            x_corner, y_corner, width, height = cv2.boundingRect(cnt)
            width = min(width, orig_image.shape[1] - x_corner)
            height = min(height, orig_image.shape[0] - y_corner)
            bounding_boxes.append([x_corner - self.box_offset, y_corner - self.box_offset,
                                   width + self.box_offset * 2, height + self.box_offset * 2])
        return bounding_boxes

    @staticmethod
    def _cut_image_patch(patch: List[int], image: np.ndarray):
        x_1, y_1, x_2, y_2 = TrafficLightDetector._get_corners(patch, image.shape)
        cut_img = image[y_1:y_2, x_1:x_2, :]
        return cut_img

    @staticmethod
    def _print_text_to_image(meters, tl_color, image, patch):
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_pos = (10, 500)
        font_scale = 1
        font_color = (0, 0, 0)
        thickness = 1
        line_type = 2

        bottom_left = (patch[0], patch[1])
        top_right = (patch[0] + patch[2], patch[1] + patch[3])
        image = cv2.rectangle(np.array(image), bottom_left, top_right,
                              color=(0, 0, 255), thickness=1)

        message = f'Distance: {meters} m, Color:{tl_color}'
        return cv2.putText(np.array(image), message,
                           text_pos, font, font_scale, font_color,
                           thickness, line_type)

    @staticmethod
    def _get_corners(rect: List[int], img_shape: Tuple[int, int]) -> List[int]:
        x1_cord = max(0, rect[0])
        y1_cord = max(0, rect[1])
        x2_cord = min(max(x1_cord + rect[2], x1_cord + 1), img_shape[1])
        y2_cord = min(max(y1_cord + rect[3], y1_cord + 1), img_shape[0])
        return [x1_cord, y1_cord, x2_cord, y2_cord]

    def _classify_tl_phase(self, image_bgr: np.ndarray) -> TrafficLightPhase:
        image_rgb = image_bgr[:, :, ::-1]
        resize_shape = (self.input_shape[0], self.input_shape[1])
        image_preproc = resize_image(image_rgb, resize_shape)
        pred = np.squeeze(self.model(np.expand_dims(image_preproc, axis=0)).numpy())
        class_id = int(np.argmax(pred))
        return TrafficLightPhase(class_id)

    @staticmethod
    def _object_distances(patches, depth_image):
        return [TrafficLightDetector._object_distance(p, depth_image) for p in patches]

    @staticmethod
    def _object_distance(patch, depth_image):
        x_middle = int(patch[0] + patch[2] / 2)
        y_middle = int(patch[1] + patch[3] / 2)
        img_width = depth_image.shape[1] - 1
        img_height = depth_image.shape[0] - 1
        middle_point = [x_middle if x_middle < img_width else img_width,
                        y_middle if y_middle < img_height else img_height]
        depth = depth_image[middle_point[1]][middle_point[0]]
        return depth
