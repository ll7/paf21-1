"""A module providing navigation services"""

from time import sleep
import rospy
from vehicle_control.route_planning.xodr_converter import XODRConverter, XodrMap


def load_town_param() -> str:
    while True:
        try:
            town = rospy.get_param('competition/town_name')
            print("Town is: ", town)
            break
        except KeyError:
            print('waiting for town rosparam')
            sleep(0.1)
    return town


def load_xodr_map() -> XodrMap:
    active_town = load_town_param()
    map_path = f"/app/res/xodr/{active_town}.xodr"
    return XODRConverter.read_xodr(map_path)
