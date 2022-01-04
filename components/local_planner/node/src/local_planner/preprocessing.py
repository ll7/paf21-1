"""Module for preprocessing CARLA sensor data"""
from cv2 import cv2
import numpy as np

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

#TODO: move Singleton to own util class
class SingletonMeta(type):
    """
    Makes sure every module uses the same data
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class SensorCameraPreprocessor(metaclass=SingletonMeta):  # pylint: disable=too-few-public-methods
    """A class for preprocessing image data from sensors"""
    semantic_image: np.ndarray = None
    depth_image: np.ndarray = None
    rgb_image: np.ndarray = None
    write_images: bool = False
    step_semantic: int  = 0
    step_rgb: int = 0
    step_depth: int = 0

    def process_semantic_image(self, msg: ImageMsg):
        """Preprocess the semantic image"""
        #config = {}
        #path = "/app/src/local_planner/config/config_lane_detection.yml"

        #lane_detecter = LaneDetection(config)

        self.step_semantic += 1
        orig_image = SensorCameraPreprocessor.load_image(msg)
        # figure out which squeeze causes the callback to crash
        if self.step_semantic % 100 == 0 and self.write_images:
            cv2.imwrite(f"/app/logs/img_{self.step_semantic}_semantic.png", orig_image)

        self.semantic_image = orig_image

    def process_depth_image(self, msg: ImageMsg):
        """Preprocess the depth image"""

        self.step_depth += 1
        orig_image = SensorCameraPreprocessor.load_image(msg)
        # figure out which squeeze causes the callback to crash
        if self.step_depth % 100 == 0 and self.write_images:
            cv2.imwrite(f"/app/logs/img_{self.step_depth}_depth.png", orig_image)

        self.depth_image = orig_image

    def process_rgb_image(self, msg: ImageMsg):
        """Preprocess the RGB image"""

        self.step_rgb += 1
        orig_image = SensorCameraPreprocessor.load_image(msg)
        # figure out which squeeze causes the callback to crash
        if self.step_rgb % 100 == 0 and self.write_images:
            cv2.imwrite(f"/app/logs/img_{self.step_rgb}_rgb.png", orig_image)

        self.rgb_image = orig_image


    @staticmethod
    def load_image(msg):
        """Loads image from ROS message"""
        bridge = CvBridge()
        orig_image = bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        return orig_image
