#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import CameraInfo
from driving import SimpleDrivingAgent
from perception import AckermannSignalAdapter, PerceptionCache, PerceptionPreprocessor

from typing import List, Dict, NamedTuple
from dataclasses import dataclass, field


@dataclass(eq=True, frozen=True)
class InputTopic:
    sensor_name: str = field(compare=True)
    sensor_type: any = field(compare=False)

@dataclass
class SimpleDrivingNode:
    vehicle_name: str
    publish_rate_in_hz: int
    input_topics: List[InputTopic] = field(default_factory=list)
    signalAdapter = AckermannSignalAdapter()
    driving_agent = SimpleDrivingAgent()
    perception_caches: Dict[str, PerceptionCache] = field(default_factory=dict)
    perception_preprocessor: PerceptionPreprocessor = PerceptionPreprocessor()

    def predict_next_driving_signal(self, perception):
        signal = self.driving_agent.next_signal(perception)
        ackermann_signal = self.signalAdapter.signal_to_ackermann(signal)
        return ackermann_signal

    def get_latest_perception(self):
        latest_perceptions = []

        for key in self.perception_caches:
            print(key)
            perception_cache = self.perception_caches[key]
            perception = (key, perception_cache.last_perception)
            latest_perceptions.append(perception)

        return self.perception_preprocessor.proprocess_perception(latest_perceptions)

    def init_perception_subscriber_callback(self, in_topic: InputTopic):
        cache = PerceptionCache()
        topic_sensor_name = in_topic.sensor_name[1:] \
            if in_topic.sensor_name.startswith('/') else in_topic.sensor_name
        topic_name = f"/carla/{self.vehicle_name}/{topic_sensor_name}"
        rospy.Subscriber(topic_name, in_topic.sensor_type, cache.update)
        return (topic_name, cache)

    def init_driving_signal_publisher(self):
        return rospy.Publisher(
            f"/carla/{self.vehicle_name}/ackermann_cmd",
            AckermannDrive,
            queue_size=100)

    def init_ros(self):
        rospy.init_node(f'test_simple_driving_{self.vehicle_name}', anonymous=True)
        self.driving_signal_publisher = self.init_driving_signal_publisher()
        topic_x_cache_tuples = [self.init_perception_subscriber_callback(topic)
            for topic in self.input_topics]
        self.perception_caches = dict(topic_x_cache_tuples)
        print(self.perception_caches)

    def run_node(self):
        self.init_ros()
        rate = rospy.Rate(self.publish_rate_in_hz)

        while not rospy.is_shutdown():
            perception = self.get_latest_perception()
            signal = self.predict_next_driving_signal(perception)
            self.driving_signal_publisher.publish(signal)
            rate.sleep()


def main():
    vehicle_name = "ego_vehicle"
    publish_rate_hz = 10
    input_topics = [
        InputTopic("/camera/depth/front/camera_info", CameraInfo)
    ]

    node = SimpleDrivingNode(vehicle_name, publish_rate_hz, input_topics)
    node.run_node()


if __name__ == '__main__':
    main()
