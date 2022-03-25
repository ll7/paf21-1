"""This module provides basic route augmentation in RVIZ."""

from typing import List, Tuple

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


def visualize_route_rviz(route: List[Tuple[float, float]]):
    """Augment the route in RVIZ."""
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.get_rostime()

    for point_x, point_y in route:
        pos = Point(x=point_x, y=point_y, z=0.5)
        orient = Quaternion(x=0, y=0, z=0, w=1)
        pose = PoseStamped(pose=Pose(position=pos, orientation=orient))
        msg.poses.append(pose)

    pub = rospy.Publisher('/carla/ego_vehicle/waypoints', Path, queue_size=1)
    pub.publish(msg)
