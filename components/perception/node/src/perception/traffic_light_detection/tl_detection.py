"""A module that detects traffic lights"""

from dataclasses import dataclass
from typing import Tuple, List, Dict
import yaml

from cv2 import cv2
import numpy as np

# pylint: disable=no-name-in-module
from tensorflow.keras import Model
from tensorflow.keras.models import load_model

from perception.traffic_light_detection.preprocessing import resize_image
from perception.traffic_light_detection.tld_info import TrafficLightPhase, TrafficLightInfo


@dataclass
class TrafficLightMetadata:
    """Class of Traffic Light Data"""
    distance: float
    patch: List[int]
    rgb_img: np.ndarray
    depth_img: np.ndarray
    sem_img: np.ndarray


class TrafficLightDetector:
    """A module that detects traffic lights"""
    def __init__(self, config_path: str):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.model: Model = load_model(config['weights_path'])
            self.input_shape: Tuple[int, int, int] = config['nn_input_size']
            self.classes_dict: Dict[int, str] = config['classes']
            self.mask: Tuple[int, int, int] = config['tl_mask']
            self.box_offset: int = config['box_offset']
        self.counter: int = 1

    def detect_traffic_light(self, semantic_image: np.ndarray, rgb_image: np.ndarray,
                             depth_image: np.ndarray) -> TrafficLightInfo or None:
        """main function to get traffic lights and distance"""
        tl_info = None
        patches = self._find_object_patches(semantic_image)

        if len(patches) > 0:

            distances = TrafficLightDetector._object_distances(patches, depth_image)
            distances = np.array(distances)

            print('Number of patches detected: ', len(patches), ' ## Patches: ', patches,
                  ' ## distances: ', distances)

            # 1) ignore traffic lights farther away than a given threshold
            # 2) ignore traffic lights that are too small sized (-> filter pedestrian / bicycle TLs)
            max_dist_m = 80
            # area_min, area_max = 0.11, 12
            cond = lambda i: distances[i] <= max_dist_m  # and area_min <= obj_areas[i] <= area_max
            closeby_patches = [(i, patches[i]) for i in range(len(patches)) if cond(i)]
            print('Closeby patches: ', len(closeby_patches), closeby_patches)

            # abort if there are no patches to evaluate
            if not closeby_patches:
                print('all patches rejected!')
                return None

            # classify each of the relevant traffic lights
            inputs = [TrafficLightMetadata(distances[i], patch, rgb_image,
                                           depth_image, semantic_image)
                      for i, patch in closeby_patches]
            results = [self._eval_traffic_light(meta) for meta in inputs]

            # ignore traffic lights of opposing side (for american case)
            results = [r for r in results if r.phase != TrafficLightPhase.BACKSIDE]
            print('results: ', len(results), results)
            # choose the best fit
            tl_info = results[np.argmax([r.accuracy for r in results])] if results else None

        return tl_info

    @staticmethod
    def _hot_zone_score(meta: TrafficLightMetadata) -> float:
        """Check whether the traffic lights are within the expected zones"""
        img_height = meta.sem_img.shape[0]  # 200
        img_width = meta.sem_img.shape[1]  # 300
        coord_width, coord_height, _, _ = meta.patch  # related to the top-left corner

        # info: this score within [0, 1] prefers traffic lights in top and right regions
        top_th, right_th = 0.5, 0.5
        upper_part_score = 0.5 if coord_height / img_height <= top_th \
            else 1 - (coord_height / img_height)
        right_part_score = 0.5 if coord_width / img_width >= right_th \
            else coord_width / img_width

        hot_zone_score = min(upper_part_score + right_part_score, 1.0)

        print('IMG Width and Height from top-left : ', coord_width, coord_height,
              ' ## score: ', hot_zone_score)
        # possible improvement: train a filter using random forests / real adaboost

        return hot_zone_score

    def _eval_traffic_light(self, tl_meta: TrafficLightMetadata) -> TrafficLightInfo:
        """Evaluate a single traffic light."""
        scaling_factor = (tl_meta.rgb_img.shape[0] // tl_meta.sem_img.shape[0],
                          tl_meta.rgb_img.shape[1] // tl_meta.sem_img.shape[1])
        rgb_patch = [tl_meta.patch[0] * scaling_factor[0], tl_meta.patch[1] * scaling_factor[1],
                     tl_meta.patch[2] * scaling_factor[0], tl_meta.patch[3] * scaling_factor[1]]

        cut_rgb_image = TrafficLightDetector._cut_image_patch(rgb_patch, tl_meta.rgb_img)
        tl_phase = self._classify_tl_phase(cut_rgb_image)
        tl_score = TrafficLightDetector._hot_zone_score(tl_meta)
        tl_info = TrafficLightInfo(tl_phase, tl_meta.distance, tl_score, tl_meta.patch)
        return tl_info

    def _find_object_patches(self, semantic_image: np.ndarray) -> List[List[int]]:
        """Find the object patches from the semantic image"""

        # find all regions of interest (according to the segmentation image)
        mask = np.array(self.mask)
        masked_image = cv2.inRange(semantic_image, mask, mask)
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        bounding_boxes = []
        for cnt in contours:
            # bounding box around the region of interest, (x/y) related to the top-left corner
            x_corner, y_corner, width, height = cv2.boundingRect(cnt)
            width = min(width, semantic_image.shape[1] - x_corner)
            height = min(height, semantic_image.shape[0] - y_corner)

            # apply an additional margin (= box offset) to the rectangle
            bounding_boxes.append([x_corner - self.box_offset, y_corner - self.box_offset,
                                   width + self.box_offset * 2, height + self.box_offset * 2])
            # TODO: catch out-of-bounds errors while applying the margin
        return bounding_boxes

    def _classify_tl_phase(self, image_bgr: np.ndarray) -> TrafficLightPhase:
        """Classify the traffic light phase."""
        image_rgb = image_bgr[:, :, ::-1]
        resize_shape = (self.input_shape[0], self.input_shape[1])
        image_preprocess = resize_image(image_rgb, resize_shape)

        pred = np.squeeze(self.model(np.expand_dims(image_preprocess, axis=0)).numpy())
        class_id = int(np.argmax(pred))
        classes = TrafficLightPhase(class_id)
        cv2.imwrite(f'/app/logs/test_{classes}.jpg', (image_preprocess.numpy() + 1) * 127.5)

        return classes

    @staticmethod
    def _cut_image_patch(patch: List[int], image: np.ndarray):
        """Get the path out of the image."""
        x_1, y_1, x_2, y_2 = TrafficLightDetector._get_corners(patch, image.shape)
        cut_img = image[y_1:y_2, x_1:x_2, :]
        return cut_img

    @staticmethod
    def _get_corners(rect: List[int], img_shape: Tuple[int, int]) -> List[int]:
        """Calculate the corners of a rectangle."""
        x1_cord = max(0, rect[0])
        y1_cord = max(0, rect[1])
        x2_cord = min(max(x1_cord + rect[2], x1_cord + 1), img_shape[1])
        y2_cord = min(max(y1_cord + rect[3], y1_cord + 1), img_shape[0])
        return [x1_cord, y1_cord, x2_cord, y2_cord]

    @staticmethod
    def _object_distances(patches: List[List[int]], depth_image: np.ndarray) -> List[float]:
        """Determine the distances of the objects in the depth image."""
        return [TrafficLightDetector._object_distance(p, depth_image) for p in patches]

    @staticmethod
    def _object_distance(patch: List[int], depth_image: np.ndarray) -> float:
        """Determine the distance of the object in the depth image."""
        x_middle = int(patch[0] + patch[2] / 2)
        y_middle = int(patch[1] + patch[3] / 2)
        img_width = depth_image.shape[1] - 1
        img_height = depth_image.shape[0] - 1
        middle_point = [x_middle if x_middle < img_width else img_width,
                        y_middle if y_middle < img_height else img_height]
        depth = depth_image[middle_point[1]][middle_point[0]]
        return depth

    @staticmethod
    def print_text_to_image(message: str, image: np.ndarray, patch: List[int]):
        """Annotate a picture with information."""
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
        return cv2.putText(np.array(image), message, text_pos, font,
                           font_scale, font_color, thickness, line_type)
