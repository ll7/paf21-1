"""Adapter for converting between ROS messages and internal data types"""

import numpy as np
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String as StringMsg
from cv_bridge import CvBridge

from perception.traffic_light_detection import TrafficLightInfo


class RosMessagesAdapter:
    """Convert between ROS messages and driving data"""
    bridge: CvBridge = CvBridge()

    def image_message_to_numpy(self, msg: ImageMsg) -> np.ndarray:
        """Loads image from ROS message"""
        orig_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # cut alpha channel for colored images (ignore for grayscale)
        orig_image = orig_image[:, :, :3] if len(orig_image.shape) > 2 else orig_image
        return orig_image

    @staticmethod
    def tld_info_to_json_message(tld_info: TrafficLightInfo) -> StringMsg:
        """Convert a driving signal into a ROS message"""
        json_msg = f'{{ "phase": "{tld_info.phase}", "distance": {tld_info.distance} }}'
        return StringMsg(data=json_msg)
