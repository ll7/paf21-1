"""Module for preprocessing CARLA sensor data"""

from cv2 import cv2
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
from local_planner.lane_detection import LaneDetection

# @dataclass
class RgbCameraPreprocessor: # pylint: disable=too-few-public-methods
    """A class for preprocessing RGB image data"""

    step: int = 0

    def process_image(self, msg: ImageMsg):
        """Preprocess the image"""

        self.step += 1

        # load image from the ROS message
        bridge = CvBridge()
        orig_image = bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

        # remove the alpha channel and resize the image
        orig_image = orig_image[:, :, :3]

        # highlight the road surface markings
        projections = LaneDetection.highlight_lines(orig_image)
        highl_image = LaneDetection.augment_image_with_lines(orig_image, projections)

        # figure out which squeeze causes the callback to crash

        if self.step % 30 == 0:
            cv2.imwrite(f"/app/logs/img_{self.step}_orig.png", orig_image)
            cv2.imwrite(f"/app/logs/img_{self.step}_line.png", highl_image)

        return highl_image
