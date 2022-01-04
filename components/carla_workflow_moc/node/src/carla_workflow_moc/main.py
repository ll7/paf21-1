#!/usr/bin/env python
"""Main script defining the ROS node"""
import json

from dataclasses import dataclass
from cv2 import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Imu as ImuMsg, Image as ImageMsg
from nav_msgs.msg import Odometry as OdometryMsg
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String as StringMsg

import rospy



@dataclass
class MockNode:
    """A class representing a ROS node that's processing route and
    local sensor data to handle the interaction with other vehicles, etc."""

    vehicle_name: str
    publish_rate_in_hz: int
    imu_publisher: rospy.Publisher = None
    odo_publisher: rospy.Publisher = None
    sem_publisher: rospy.Publisher = None
    depth_publisher: rospy.Publisher = None
    rgb_publisher: rospy.Publisher = None


    def run_node(self):
        """Launch the ROS node to receive globally planned routes
        and convert them into locally planned routes using sensors"""
        self._init_ros()
        image_path = "/app/src/carla_workflow_moc/test_images/"
        rate = rospy.Rate(self.publish_rate_in_hz)
        image_counter = 100
        rgb_path = image_path + f'img_{image_counter}_rgb.png'
        depth_path = image_path + f'img_{image_counter}_depth.png'
        sem_path = image_path + f'img_{image_counter}_semantic.png'
        frame_id = 0
        while not rospy.is_shutdown():
            print(frame_id)
            quaternion = [3.015298332645633e-06, -9.603338652044666e-05,
                          -0.0032518007504562757, 0.9999947082661869]
            imu = generate_imu_message(quaternion)
            position = [245.84999084472656, -198.74832153320312]
            odo = generate_odo_message(position)
            rgb = generate_basic_image_message(rgb_path)
            sem = generate_basic_image_message(sem_path)
            depth = generate_depth_message(depth_path)
            self.rgb_publisher.publish(rgb)
            self.imu_publisher.publish(imu)
            self.odo_publisher.publish(odo)
            self.sem_publisher.publish(sem)
            self.depth_publisher.publish(depth)
            frame_id += 1
            image_counter += 100
            if image_counter > 500:
                image_counter = 100
            rate.sleep()
            #if frame_id > 500:
            #    rospy.signal_shutdown('tested successfully')

    def _init_ros(self):
        """Initialize the ROS node's publishers and subscribers"""

        rospy.init_node(f'carla_moc_{self.vehicle_name}', anonymous=True)
        self.imu_publisher = self._init_imu_publisher()
        self.odo_publisher = self._init_odo_publisher()
        self.sem_publisher = self._init_semantic_publisher()
        self.depth_publisher = self._init_depth_publisher()
        self.rgb_publisher = self._init_rgb_publisher()
        self._init_global_route_subscriber()
        self._init_ackermann_subscriber()

    def _init_imu_publisher(self):
        """imu publisher"""
        out_topic = f"/carla/{self.vehicle_name}/imu/imu1"
        return rospy.Publisher(out_topic, ImuMsg, queue_size=100)

    def _init_depth_publisher(self):
        """imu publisher"""
        camera_depth = 'depth/front/image_depth'
        out_topic = f"/carla/{self.vehicle_name}/camera/{camera_depth}"
        return rospy.Publisher(out_topic, ImageMsg, queue_size=100)

    def _init_semantic_publisher(self):
        """imu publisher"""
        camera_semantic_seg = "semantic_segmentation/front/image_segmentation"
        out_topic = f"/carla/{self.vehicle_name}/camera/{camera_semantic_seg}"
        return rospy.Publisher(out_topic, ImageMsg, queue_size=100)

    def _init_rgb_publisher(self):
        """imu publisher"""
        camera_rgb = 'rgb/front/image_color'
        out_topic = f"/carla/{self.vehicle_name}/camera/{camera_rgb}"
        return rospy.Publisher(out_topic, ImageMsg, queue_size=100)

    def _init_odo_publisher(self):
        """imu publisher"""
        out_topic = f"/carla/{self.vehicle_name}/odometry"
        return rospy.Publisher(out_topic, OdometryMsg, queue_size=100)

    def _init_global_route_subscriber(self):
        in_topic = f"/drive/{self.vehicle_name}/global_route"
        rospy.Subscriber(in_topic, StringMsg, test_route)

    def _init_ackermann_subscriber(self):
        in_topic = f"/carla/{self.vehicle_name}/ackermann_cmd"
        rospy.Subscriber(in_topic, AckermannDrive, test_control)


def test_control(msg):
    """checks if ackermann control msg gets generated and has right format"""
    checker = [msg_type != 'float32' for msg_type in msg._get_types()]
    if any(checker):
        raise ValueError('Ackermann Control contains non float32 value')

def test_route(msg):
    """checks if global route msg gets generated and has right format"""
    print(msg)


def generate_odo_message(position):
    """mocking odometry message"""
    odo = OdometryMsg()
    odo.header.stamp = rospy.get_rostime()
    odo.pose.pose.position.x = position[0]  # 245.84999084472656
    odo.pose.pose.position.y = position[1]  # -198.74832153320312
    return odo


def generate_imu_message(quaternion):
    """generating imu message"""
    imu = ImuMsg()
    imu.header.stamp = rospy.get_rostime()
    # imu.header.frame_id = "main_frame"
    imu.orientation.x = quaternion[0]
    imu.orientation.y = quaternion[1]
    imu.orientation.z = quaternion[2]
    imu.orientation.w = quaternion[3]
    return imu


def generate_basic_image_message(path):
    """generating a basic png image message"""
    cv_image = cv2.imread(path)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    return image_message


def generate_depth_message(path):
    """generating a depth message"""
    depth_image = cv2.imread(path)
    red = depth_image[:, :, 0]
    green = depth_image[:, :, 1]
    blue = depth_image[:, :, 2]
    ans = (red + green * 256 + blue * 256 * 256)
    normalized = ans / (256 * 256 * 256 - 1)
    in_meters = 1000 * normalized
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(in_meters, encoding="passthrough")
    return image_message


def main():
    """The main entrypoint launching the ROS nodedocker
    with specific configuration parameters"""

    vehicle_name = "ego_vehicle"
    publish_rate_hz = 10
    node = MockNode(vehicle_name, publish_rate_hz)
    node.run_node()


if __name__ == '__main__':
    main()
