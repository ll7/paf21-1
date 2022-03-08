from math import dist
from pathlib import Path

from global_planner.route_annotation import RouteAnnotation
from global_planner.global_route_planner import GlobalPlanner
from global_planner.xodr_converter import XODRConverter


def load_map():
    xodr_path = Path("/app/res/xodr/Town01.xodr")
    xodr_map = XODRConverter.read_xodr(xodr_path)
    return xodr_map

# TODO Test following points 

# Curvature fail???
# Find roads 5 for point (46.14997863769531, -326.9700012207031)
# Find roads 4 for point (259.39117431640625, -133.23997497558594)

# No Route?!
# Find roads 150 for point (338.3089294433594, -324.74908447265625)
# Find roads 158 for point (338.3089294433594, -324.74908447265625)

# Driving into Wall???
# Find roads 163 for point (338.3089294433594, -324.74908447265625)
# Find roads 12 for point (270.94085693359375, -199.05966186523438)


def test_path_finding():
    xodr_map = load_map()
    start = (1.5599901676177979, -149.83001708984375)
    end = (322.09625244140625, -55.15309143066406)
    path = GlobalPlanner.get_shortest_path(start, end, xodr_map)

    assert path == ['-1_0_0', '15_0_1', '13_0_-1', '13_1_-1', '3_1_1', '3_0_1', '117_1_1', '117_0_1',
                    '2_1_1', '2_0_1', '61_1_1', '61_0_1', '1_1_1', '1_0_1', '27_1_1', '27_0_1',
                    '16_0_-1', '16_1_-1', '253_0_-1', '253_1_-1', '10_1_1', '-2_0_0']


def test_route_metadata():
    xodr_map = load_map()
    start = (1.5599901676177979, -149.83001708984375)
    path = ['-1_0_0', '15_0_1', '13_0_-1', '13_1_-1', '3_1_1', '3_0_1', '117_1_1', '117_0_1',
            '2_1_1', '2_0_1', '61_1_1', '61_0_1', '1_1_1', '1_0_1', '27_1_1', '27_0_1',
            '16_0_-1', '16_1_-1', '253_0_-1', '253_1_-1', '10_1_1', '-2_0_0']
    metadata = RouteAnnotation.preprocess_route_metadata(start, path, xodr_map)

    # section ids of path correct
    assert [s.road_id for s in metadata.sections_ahead] == [15, 13, 3, 117, 2, 61, 1, 27, 16, 253, 10]

    # driving directions of sections correct
    assert [s.drive_reversed for s in metadata.sections_ahead] == \
        [True, False, True, True, True, True, True, True, False, False, True]

    # lane id always 1 and possible lanes contain the lane id
    assert all(map(lambda s: s.lane_id == 1, metadata.sections_ahead))
    assert all(map(lambda s: s.lane_id in s.possible_lanes, metadata.sections_ahead))

    # all relevant traffic lights found
    exp_tl_pos = [(77.15649762782473, 0.04155127482227584),
                  (142.7986057071138, 0.045752231493989014),
                  (323.4807371879971, 0.011754717028978167), 
                  (336.87300605856103, -44.56103465562376),
                  (169.19245819220555, -57.49040013751494)]
    assert [tl.pos for tl in metadata.traffic_lights_ahead] == exp_tl_pos
    
    # all relevant traffic signs found
    exp_ss_pos = [(-0.014683089359112911, -127.76235955923758)]
    assert [ss.pos for ss in metadata.speed_signs_ahead] == exp_ss_pos
    assert metadata.initial_speed == 90.0


def test_route_interpolation():
    xodr_map = load_map()
    start = (1.5599901676177979, -149.83001708984375)
    end = (322.09625244140625, -55.15309143066406)
    route = GlobalPlanner.generate_waypoints(start, end, 0, xodr_map)

    # the route contains at least 2 waypoints and the x/y coords have reasonable values
    assert len(route) > 2
    assert all(map(lambda wp: wp.pos[0] != 0 and wp.pos[1] != 0, route))

    # succeeding points are interpolated within max. 4 meters of distance
    neighbors = [(route[i], route[i+1]) for i in range(len(route)-1)]
    assert all(map(lambda n: dist(n[0].pos, n[1].pos) < 4.0, neighbors))


def test_route_annotations():
    xodr_map = load_map()
    start = (1.5599901676177979, -149.83001708984375)
    end = (322.09625244140625, -55.15309143066406)
    route = GlobalPlanner.generate_waypoints(start, end, 0, xodr_map)

    # the speed signs are interpreted correctly
    speed_zones = []
    for wp in route:
        if not wp.legal_speed in speed_zones:
            speed_zones.append(wp.legal_speed)
    assert speed_zones == [90.0, 30.0, 50.0]

    # the distance to traffic lights is constantly decreasing until the next reset
    neighbors = [(route[i], route[i+1]) for i in range(len(route)-1)]
    assert all(map(lambda n: n[0].dist_next_tl > n[1].dist_next_tl - 0.2 or n[0].dist_next_tl < 10, neighbors))
