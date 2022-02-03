#!/usr/bin/env python
"""Main script defining the ROS node"""

from dataclasses import dataclass

import numpy as np
import rospy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String as StringMsg

from perception.traffic_light_detection import TrafficLightDetector
from perception.ros_msg_adapter import RosMessagesAdapter
from perception.object_detection import ObjectDetector


class ImagesBuffer:
    # pylint: disable=too-few-public-methods
    """Representing a very straightforward implementation
    of an image buffer for storing the latest image"""
    last_img: np.ndarray = None

    def apply_image(self, image: np.ndarray):
        """Add the next image to the buffer."""
        self.last_img = image


@dataclass
class PerceptionNode:
    """A class representing a ROS node that's processing
    camera data to detect traffic lights."""
    vehicle_name: str
    publish_rate_in_hz: int
    config_path: str = '/app/src/perception/config/detection_config.yml'
    tld_publisher: rospy.Publisher = None
    obj_publisher: rospy.Publisher = None
    tl_detector: TrafficLightDetector = None
    vehicle_detector: ObjectDetector = None
    rbg_buffer: ImagesBuffer = ImagesBuffer()
    depth_buffer: ImagesBuffer = ImagesBuffer()
    semantic_buffer: ImagesBuffer = ImagesBuffer()

    def __post_init__(self):
        self.tl_detector = TrafficLightDetector(self.config_path)
        self.vehicle_detector = ObjectDetector(self.config_path, 'vehicle')

    def run_node(self):
        """Launch the ROS node to receive front camera images
        which get processed to detect traffic lights"""
        self._init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            sem_img, rgb_img, depth_img = (
                self.semantic_buffer.last_img,
                self.rbg_buffer.last_img,
                self.depth_buffer.last_img)

            buffers_contain_img = not (sem_img is None or rgb_img is None or depth_img is None)
            if buffers_contain_img:
                tld_info = self.tl_detector.detect_traffic_light(sem_img, rgb_img, depth_img)
                obj_infos = self.vehicle_detector.detect_object(sem_img, depth_img)
                if tld_info:
                    print(f'Traffic light detected: {tld_info}')
                    msg = RosMessagesAdapter.tld_info_to_json_message(tld_info)
                    self.tld_publisher.publish(msg)

                msg = RosMessagesAdapter.obj_info_to_json_message(obj_infos)
                self.obj_publisher.publish(msg)

            rate.sleep()

    def _init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""
        rospy.init_node(f'local_planner_{self.vehicle_name}', anonymous=True)
        self.tld_publisher = self._init_tld_info_publisher()
        self.obj_publisher = self._init_object_info_publisher()
        self._init_front_camera_subscribers()

    def _init_front_camera_subscribers(self):
        img_preprocessor = RosMessagesAdapter()
        preproc_img = img_preprocessor.image_message_to_numpy

        camera_semantic_seg = "semantic_segmentation/front/image_segmentation"
        in_semantic_topic = f"/carla/{self.vehicle_name}/camera/{camera_semantic_seg}"
        callback = lambda msg: self.semantic_buffer.apply_image(preproc_img(msg))
        rospy.Subscriber(in_semantic_topic, ImageMsg, callback)

        camera_depth = 'depth/front/image_depth'
        in_depth_topic = f"/carla/{self.vehicle_name}/camera/{camera_depth}"
        callback = lambda msg: self.depth_buffer.apply_image(preproc_img(msg))
        rospy.Subscriber(in_depth_topic, ImageMsg, callback)

        camera_rgb = 'rgb/front/image_color'
        in_rgb_topic = f"/carla/{self.vehicle_name}/camera/{camera_rgb}"
        callback = lambda msg: self.rbg_buffer.apply_image(preproc_img(msg))
        rospy.Subscriber(in_rgb_topic, ImageMsg, callback)

    def _init_tld_info_publisher(self):
        out_topic = f"/drive/{self.vehicle_name}/tld_info"
        return rospy.Publisher(out_topic, StringMsg, queue_size=10)

    def _init_object_info_publisher(self):
        out_topic = f"/drive/{self.vehicle_name}/object_info"
        return rospy.Publisher(out_topic, StringMsg, queue_size=10)


def main():
    """The main entrypoint launching the ROS node
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    publish_rate_hz = 10
    node = PerceptionNode(vehicle_name, publish_rate_hz)
    node.run_node()


if __name__ == '__main__':
    main()
