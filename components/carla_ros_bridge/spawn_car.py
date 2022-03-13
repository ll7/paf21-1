import sys
import os
import yaml
from typing import Tuple

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion


def spawn_car(pos: Tuple[float, float, float], orient: Tuple[float, float, float, float]):
    header = Header(seq=0, stamp=rospy.Time(secs=0, nsecs=0), frame_id='')
    pose = Pose(
        position=Point(x=pos[0], y=pos[1], z=pos[2]),
        orientation=Quaternion(x=orient[0], y=orient[1], z=orient[2], w=orient[3]))
    cov = [0.0 for _ in range(36)]
    pose_with_cov = PoseWithCovariance(pose=pose, covariance=cov)
    stamped_pose_msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)

    rospy.init_node(f'launch_service_spawn_car', anonymous=True)
    publisher = rospy.Publisher('/carla/ego_vehicle/initialpose',
                                PoseWithCovarianceStamped, queue_size=10)
    publisher.publish(stamped_pose_msg)


def main():
    if len(sys.argv) != 2:
        raise ValueError('Invalid arguments! Expecting a file path to a YAML configuration!')

    config_file = sys.argv[1]
    if not os.path.exists(config_file) or not os.path.isfile(config_file):
        raise ValueError(f'Invalid arguments! The file {config_file} does not exist!')

    with open(config_file, encoding='utf-8') as file:
        config = yaml.safe_load(file)

        pos = config['competition']['start']['position']
        pos = (pos['x'], pos['y'], pos['z'])
        orient = config['competition']['start']['orientation']
        orient = (orient['x'], orient['y'], orient['z'], orient['w'])

        spawn_car(pos, orient)


if __name__ == '__main__':
    main()
