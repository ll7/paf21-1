from math import dist, pi

from vehicle_control.route_planning.route_annotation import RouteAnnotation
from vehicle_control.route_planning.route_planner import RoutePlanner
from vehicle_control.route_planning.xodr_converter import XODRConverter


def load_town_01():
   xodr_path = "/app/res/xodr/Town01.xodr"
   xodr_map = XODRConverter.read_xodr(xodr_path)
   return xodr_map


def load_town_03():
    xodr_path = "/app/res/xodr/Town03.xodr"
    xodr_map = XODRConverter.read_xodr(xodr_path)
    return xodr_map


def load_town_04():
    xodr_path = "/app/res/xodr/Town04.xodr"
    xodr_map = XODRConverter.read_xodr(xodr_path)
    return xodr_map


# TODO: fix this test (it's prob related to the lane offsets)
def test_path_finding():
    xodr_map = load_town_01()
    start = (1.5599901676177979, -149.83001708984375)
    end = (322.09625244140625, -55.15309143066406)
    path = RoutePlanner.get_shortest_path(start, end, 0.0, xodr_map)

    assert path == ['-1_0_0', '15_0_1', '13_0_-1', '13_1_-1', '3_1_1', '3_0_1', '117_1_1', '117_0_1',
                    '2_1_1', '2_0_1', '61_1_1', '61_0_1', '1_1_1', '1_0_1', '27_1_1', '27_0_1',
                    '16_0_-1', '16_1_-1', '253_0_-1', '253_1_-1', '10_1_1', '-2_0_0']


def test_route_metadata():
    xodr_map = load_town_01()
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
    assert all(map(lambda s: s.lane_id == 1 if s.drive_reversed else s.lane_id == -1, metadata.sections_ahead))
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


# def test_route_metadata_town04():
#     xodr_map = load_town_04()
#     start = (406.0252685546875, 124.70137786865234)
#     end = (182.08697509765625, 395.9513244628906)
#     orient = pi/2
#     route = RoutePlanner.generate_waypoints(start, end, orient, xodr_map)
#     print('fucked up wps', [wp for wp in route if wp.actual_lane == 0])
#     # lane id always 1 and possible lanes contain the lane id
#     assert all(map(lambda s: s.actual_lane != 0, route))


# def test_route_interpolation():
#     xodr_map = load_town_01()
#     start = (1.5599901676177979, -149.83001708984375)
#     end = (322.09625244140625, -55.15309143066406)
#     route = GlobalPlanner.generate_waypoints(start, end, xodr_map)

#     # the route contains at least 2 waypoints and the x/y coords have reasonable values
#     assert len(route) > 2
#     assert all(map(lambda wp: wp.pos[0] != 0 and wp.pos[1] != 0, route))

#     # succeeding points are interpolated within max. 4 meters of distance
#     neighbors = [(route[i], route[i+1]) for i in range(len(route)-1)]
#     assert all(map(lambda n: dist(n[0].pos, n[1].pos) < 4.0, neighbors))


# def test_route_annotations():
#     xodr_map = load_town_01()
#     start = (1.5599901676177979, -149.83001708984375)
#     end = (322.09625244140625, -55.15309143066406)
#     route = GlobalPlanner.generate_waypoints(start, end, xodr_map)

#     # the speed signs are interpreted correctly
#     speed_zones = []
#     for wp in route:
#         if not wp.legal_speed in speed_zones:
#             speed_zones.append(wp.legal_speed)
#     assert speed_zones == [90.0, 30.0, 50.0]

    # the distance to traffic lights is constantly decreasing until the next reset
    # neighbors = [(route[i], route[i+1]) for i in range(len(route)-1)]
    # assert all(map(lambda n: n[0].dist_next_tl > n[1].dist_next_tl - 0.2 or n[0].dist_next_tl < 10, neighbors))
    # TODO: enable this property test back again

# ============================================================

# TODO: enable this regression test
# def test_path_finding_multilane_highway_town_3():
#     xodr_map = load_town_03()
#     start = (-41.71999740600586, 208.92999267578125)
#     end  = (28.129140173585185, -196.96087987528907)
#     path = GlobalPlanner.get_shortest_path(start, end, 0.0, xodr_map)
#     print(path)
#     route = GlobalPlanner._preplan_route(start, end, path, 0.0, xodr_map)

#     print(route)
#     assert False


def test_path_finding_multilane_highway_town_4():
    xodr_map = load_town_04()
    start = (406.0252685546875, 124.70137786865234)
    end = (182.08697509765625, 395.9513244628906)
    path = RoutePlanner.get_shortest_path(start, end, 0.0, xodr_map)
    print(path)
    route = RoutePlanner._preplan_route(start, end, path, 0.0, xodr_map)

    print(route)
    assert True


# def test_path_finding_multilane_highway_town_4():
#     xodr_map = load_town_04()
#     start = (406.0252685546875, 124.70137786865234)
#     end  = (8.901467323303223, -96.35236358642578)

#     wps = GlobalPlanner.generate_waypoints(start, end, xodr_map)
#     # print([wp.pos for wp in wps])
#     assert False


# def test_path_finding_multilane_highway_town_4():
#     xodr_map = load_town_04()
#     start = (262.7838134765625, 118.74906158447266)
#     end  = (16.040634155273438, 170.54249572753906)
#     path = GlobalPlanner.get_shortest_path(start, end, xodr_map)
#     print(path)
#     assert False


# def test_path_finding_multilane_highway_town_4():
#     xodr_map = load_town_04()
#     start = (406.0252685546875, 124.70137786865234)
#     end  = (16.040634155273438, 170.54249572753906)

#     path = GlobalPlanner.get_shortest_path(start, end, xodr_map)
#     assert path == ['-1_0_0', '36_0_3', '862_1_3', '862_0_3', '35_1_3', '35_0_3', '43_1_3', '43_0_3', '266_1_3',
#                     '266_0_3', '42_1_3', '42_0_3', '50_1_3', '50_0_3', '1174_1_3', '1174_0_3', '49_1_3', '49_0_3',
#                     '902_1_3', '902_0_3', '48_1_3', '48_0_3', '775_1_3', '775_0_3', '47_1_3', '47_0_3', '1076_1_2',
#                     '1076_0_2', '44_1_2', '44_0_2', '1194_1_6', '1194_0_6', '39_1_3', '39_0_3', '1101_1_2', '1101_0_2',
#                     '31_1_2', '31_0_2', '1080_1_4', '1080_0_4', '47_0_-1', '47_1_-1', '774_0_-1', '774_1_-1', '48_0_-1',
                    # '48_1_-1', '912_0_-1', '-2_0_0']
    # print(path)
    # assert False

# ============================================================

# def test_path_finding_multilane_town_3():
#     xodr_map = load_town_03()
#     start = (13.72380256652832, -18.817155838012695)
#     end = (-88.3062744140625, -21.53060531616211)
#     path = GlobalPlanner.get_shortest_path(start, end, xodr_map)

#     print(path)
#     assert False
#     # assert path == ['-1_0_0', '15_0_1', '13_0_-1', '13_1_-1', '3_1_1', '3_0_1', '117_1_1', '117_0_1',
#     #                 '2_1_1', '2_0_1', '61_1_1', '61_0_1', '1_1_1', '1_0_1', '27_1_1', '27_0_1',
#     #                 '16_0_-1', '16_1_-1', '253_0_-1', '253_1_-1', '10_1_1', '-2_0_0']


# test for town 05
# (-152.083251953125, -154.6916046142578) to (69.37128448486328, 200.74708557128906)


def test_lane_change_north_right():
    points = [(0.0, 0.0), (0.0, 2.0), (0.0, 3.0), (0.0, 6.0),
              (0.0, 8.0), (0.0, 10.0), (0.0, 12.0), (0.0, 14.0)]
    ref_point = (4.0, 14.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_north_left():
    points = [(0.0, 0.0), (0.0, 2.0), (0.0, 3.0), (0.0, 6.0),
              (0.0, 8.0), (0.0, 10.0), (0.0, 12.0), (0.0, 14.0)]
    ref_point = (-4.0, 14.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_south_right():
    points = [(0.0, 0.0), (0.0, -2.0), (0.0, -3.0), (0.0, -6.0),
              (0.0, -8.0), (0.0, -10.0), (0.0, -12.0), (0.0, -14.0)]
    ref_point = (4.0, -14.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_south_left():
    points = [(0.0, 0.0), (0.0, -2.0), (0.0, -3.0), (0.0, -6.0),
              (0.0, -8.0), (0.0, -10.0), (0.0, -12.0), (0.0, -14.0)]
    ref_point = (-4.0, -14.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_east_right():
    points = [(0.0, 0.0), (2.0, 0.0), (3.0, 0.0), (6.0, 0.0),
              (8.0, 0.0), (10.0, 0.0), (12.0, 0.0), (14.0, 0.0)]
    ref_point = (14.0, 4.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_east_left():
    points = [(0.0, 0.0), (2.0, 0.0), (3.0, 0.0), (6.0, 0.0),
              (8.0, 0.0), (10.0, 0.0), (12.0, 0.0), (14.0, 0.0)]
    ref_point = (14.0, -4.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_west_right():
    points = [(0.0, 0.0), (-2.0, 0.0), (-3.0, 0.0), (-6.0, 0.0),
              (-8.0, 0.0), (-10.0, 0.0), (-12.0, 0.0), (-14.0, 0.0)]
    ref_point = (-14.0, 4.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_lane_change_west_left():
    points = [(0.0, 0.0), (-2.0, 0.0), (-3.0, 0.0), (-6.0, 0.0),
              (-8.0, 0.0), (-10.0, 0.0), (-12.0, 0.0), (-14.0, 0.0)]
    ref_point = (-14.0, -4.0)
    new_points = RoutePlanner._lane_change(points, ref_point, speed_signs=[13.0], slope_m=3.0)
    assert dist(points[0], new_points[0]) < 0.2
    assert dist(ref_point, new_points[-1]) < 0.9
    assert 1.5 < dist(points[5], new_points[5]) < 2.5


def test_filter_path1():
    path = ['0_0_0', '0_0_1', '0_0_2']
    filtered_path = RoutePlanner._filter_path(path)
    assert filtered_path == ['0_0_0', '0_0_2']


def test_filter_path2():
    path = ['0_0_0', '0_0_1', '0_0_2', '0_0_3']
    filtered_path = RoutePlanner._filter_path(path)
    assert filtered_path == ['0_0_0', '0_0_3']


def test_filter_path3():
    path = ['0_0_0', '0_1_1', '0_1_2', '0_1_3']
    filtered_path = RoutePlanner._filter_path(path)
    assert filtered_path == ['0_0_0', '0_1_1', '0_1_3']


def test_filter_path4():
    path = ['-1_0_0', '0_0_1', '1_1_1', '1_0_1']
    filtered_path = RoutePlanner._filter_path(path)
    assert filtered_path == ['-1_0_0', '0_0_1', '1_1_1', '1_0_1']
