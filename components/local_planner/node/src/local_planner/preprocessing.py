"""Module for preprocessing CARLA sensor data"""
from typing import Dict

from cv2 import cv2
import numpy as np

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge


class SensorCameraPreprocessor:  # pylint: disable=too-few-public-methods
    """A class for preprocessing image data from sensors"""
    semantic_image: np.ndarray = None
    depth_image: np.ndarray = None
    rgb_image: np.ndarray = None
    write_images: bool = False
    step_semantic: int  = 0
    step_rgb: int = 0
    step_depth: int = 0

    def get_image_lists(self) -> Dict[str, np.ndarray]:
        """Return the different images as dictionary"""
        return {'semantic': self.semantic_image,
                'rgb': self.rgb_image,
                'depth': self.depth_image}

    def process_semantic_image(self, msg: ImageMsg):
        """Preprocess the semantic image"""
        self.step_semantic += 1
        orig_image = SensorCameraPreprocessor.load_image(msg)
        if self.step_semantic % 10 == 0 and self.write_images:
            cv2.imwrite(f"/app/logs/img_{self.step_semantic}_semantic.png", orig_image)

        self.semantic_image = orig_image

    def process_depth_image(self, msg: ImageMsg):
        """Preprocess the depth image"""
        self.step_depth += 1
        orig_image = SensorCameraPreprocessor.load_image(msg)
        if self.step_depth % 10 == 0 and self.write_images:
            cv2.imwrite(f"/app/logs/img_{self.step_depth}_depth.png", orig_image)

        self.depth_image = orig_image

    def process_rgb_image(self, msg: ImageMsg):
        """Preprocess the RGB image"""

        self.step_rgb += 1
        orig_image = SensorCameraPreprocessor.load_image(msg)
        if self.step_rgb % 10 == 0 and self.write_images:
            cv2.imwrite(f"/app/logs/img_{self.step_rgb}_rgb.png", orig_image)

        self.rgb_image = orig_image

    @staticmethod
    def load_image(msg):
        """Loads image from ROS message"""
        bridge = CvBridge()
        orig_image = bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        return orig_image

    @staticmethod
    def msg_from_image(image):
        """Loads image from ROS message"""
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        return img_msg
