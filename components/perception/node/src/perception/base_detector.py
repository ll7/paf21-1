"""A module that is the base detector"""
from typing import List, Tuple
import numpy as np
from cv2 import cv2


class BaseDetector:
    """A module that is the base detector"""
    def __init__(self):
        self.mask: Tuple[int, int, int] = (0, 0, 0)
        self.box_offset: int = 0
        self.counter: int = 0

    def find_object_patches(self, semantic_image: np.ndarray) -> List[List[int]]:
        """Find the object patches from the semantic image"""
        mask = np.array(self.mask)
        masked_image = cv2.inRange(semantic_image, mask, mask)
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        idx = 0
        bounding_boxes = []
        for cnt in contours:
            idx += 1
            x_corner, y_corner, width, height = cv2.boundingRect(cnt)
            width = min(width, semantic_image.shape[1] - x_corner)
            height = min(height, semantic_image.shape[0] - y_corner)
            bounding_boxes.append([x_corner - self.box_offset, y_corner - self.box_offset,
                                   width + self.box_offset * 2, height + self.box_offset * 2])
        return bounding_boxes

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

    @staticmethod
    def get_corners(rect: List[int], img_shape: Tuple[int, int]) -> List[int]:
        """Calculate the corners of a rectangle."""
        x1_cord = max(0, rect[0])
        y1_cord = max(0, rect[1])
        x2_cord = min(max(x1_cord + rect[2], x1_cord + 1), img_shape[1])
        y2_cord = min(max(y1_cord + rect[3], y1_cord + 1), img_shape[0])
        return [x1_cord, y1_cord, x2_cord, y2_cord]

    @staticmethod
    def object_distances(patches: List[List[int]], depth_image: np.ndarray) -> List[float]:
        """Determine the distance of the object in the depth image."""
        return [BaseDetector._object_distance(p, depth_image) for p in patches]

    @staticmethod
    def _object_distance(patch: List[int], depth_image: np.ndarray) -> float:
        x_middle = int(patch[0] + patch[2] / 2)
        y_middle = int(patch[1] + patch[3] / 2)
        img_width = depth_image.shape[1] - 1
        img_height = depth_image.shape[0] - 1
        middle_point = [x_middle if x_middle < img_width else img_width,
                        y_middle if y_middle < img_height else img_height]
        depth = depth_image[middle_point[1]][middle_point[0]]
        return depth

    @staticmethod
    def print_text_to_image_in_patches(message: List[str], image: np.ndarray,
                                       patches: List[List[int]]):
        """Annotate patches with information."""
        assert(len(message) == len(patches))
        font = cv2.FONT_HERSHEY_PLAIN
        image_width = image.shape[1] / 10
        font_color = (0, 0, 0)
        thickness = 1
        image = cv2.resize(image, (900, 600))
        for i, patch in enumerate(patches):
            bottom_left = (patch[0]*3, patch[1]*3)
            top_right = (patch[0]*3 + patch[2]*3, patch[1]*3 + patch[3]*3)
            text_pos = (patch[0]*3, patch[1]*3)
            image = cv2.rectangle(np.array(image), bottom_left, top_right,
                                  color=(0, 255, 0), thickness=1)
            image = cv2.putText(image, message[i], text_pos, font, patch[2]/image_width,
                                font_color, thickness)
        return image
