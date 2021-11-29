"""Module for preprocessing CARLA sensor data"""

from cv2 import cv2

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
from local_planner.line_highlighting import LineHighlighter

# @dataclass
class RgbCameraPreprocessor: # pylint: disable=too-few-public-methods
    """A class for preprocessing RGB image data"""

    step: int = 0

    def process_image(self, msg: ImageMsg):
        """Preprocess the image"""

        self.step += 1

        # load image from the ROS message
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        rgb_image = rgb_image[:, :, :3]

        # highlight the road surface markings
        img = LineHighlighter.highlight_lines(rgb_image)

        if self.step % 30 == 0:
            cv2.imwrite(f"/app/logs/img_{self.step}.png", img)

        return img
