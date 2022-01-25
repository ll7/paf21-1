from pathlib import Path
from global_planner.global_route_planner import GlobalPlanner
from global_planner.xodr_converter import XODRConverter

xodr_path = Path("/app/res/xodr/Town01.xodr")
xodr_map = XODRConverter.read_xodr(xodr_path)

def test_route_planner():
    start = (322.09625244140625, -55.15309143066406)
    end = (170.82284545898438, -195.2700958251953)
    route = GlobalPlanner.generate_waypoints(start, end, 0, xodr_map)
    assert len(route) > 2
    assert all(map(lambda wp: wp.x_coord != 0 and wp.y_coord != 0, route))
