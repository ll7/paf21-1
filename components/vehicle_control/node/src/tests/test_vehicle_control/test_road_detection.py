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


def test_find_neighbor_sections_0_right():
    point = (380.0, 2.0)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    _, is_right_road, road = neighbors[0]
    assert is_right_road and road.road_id == 0


def test_find_neighbor_sections_0_left():
    point = (380.0, -2.0)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    _, is_right_road, road = neighbors[0]
    assert not is_right_road and road.road_id == 0


def test_find_neighbor_sections_12_right():
    point = (245.85, -198.75)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    _, is_right_road, road = neighbors[0]
    assert is_right_road and road.road_id == 12


def test_find_neighbor_sections_12_left():
    point = (245.85, -195.75)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    _, is_right_road, road = neighbors[0]
    assert not is_right_road and road.road_id == 12


def test_find_neighbor_sections_curvature_one_sided_right():
    point = (92.00, -207.0)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    road_ids = [n[2].road_id for n in neighbors]
    assert 257 not in road_ids and 271 not in road_ids and \
           256 in road_ids and 272 in road_ids


def test_find_neighbor_sections_curvature_one_sided_left():
    point = (88.0, -207.0)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    road_ids = [n[2].road_id for n in neighbors]
    assert 257 in road_ids and 271 in road_ids and \
           256 not in road_ids and 272 not in road_ids


def test_find_neighbor_sections_5():
    point = (150.99, -57.5)
    xodr_map = load_town_01()
    neighbors = xodr_map.find_sections(point)
    road_ids = [n[2].road_id for n in neighbors]
    assert 334 in road_ids and 355 in road_ids and 333 not in road_ids


def test_find_neighbor_sections_town04_highway_1():
    point = (406.0252685546875, 124.70137786865234)
    xodr_map = load_town_04()
    neighbors = xodr_map.find_sections(point)
    lane_id, _, road = neighbors[0]
    assert road.road_id == 36 and lane_id == 4


def test_find_neighbor_sections_town04_highway_2():
    point = (7.50634155273438, 130.54249572753906)
    xodr_map = load_town_04()
    neighbors = xodr_map.find_sections(point)
    road_ids = [n[2].road_id for n in neighbors]
    print(road_ids)
    assert road_ids == [48]


def test_find_neighbor_sections_town04_highway_3():
    point = (-105.50200653076172, -12.80552864074707)
    xodr_map = load_town_04()
    neighbors = xodr_map.find_sections(point)
    road_ids = [n[2].road_id for n in neighbors]
    print(road_ids)
    assert road_ids == [1184]

 
# def test_find_neighbor_sections_town04_highway_4():
#     point = (-504.7949523925781, -221.73223876953125)
#     neighbors = RoadDetection.find_sections(point, load_town_04())
#     road_ids = [n[2].road_id for n in neighbors]
#     print(road_ids)
#     assert road_ids == [6]


# def test_find_neighbor_sections_town04_highway_5():
#     point = (1.5099804401397705, -249.42999267578125)
#     neighbors = RoadDetection.find_sections(point, load_town_04())
#     road_ids = [n[2].road_id for n in neighbors]
#     print(road_ids)
#     assert road_ids == [46]

# Test target points Town04: 
# (182.08697509765625, 395.9513244628906) Lanechange fuckup
# End Point not found
# [(16.488210678100586, -212.6156005859375), (375.6085205078125, 68.68538665771484),
# (86.74788665771484, -9.935043334960938), (-515.2499389648438, -240.95675659179688),
# (348.0001525878906, 142.8535614013672), (160.5265655517578, 389.1396789550781),
# (136.9810028076172, -209.24661254882812)]
# (-16.895854949951172, 212.5471954345703) Crash into wall

# Town 03
# End Point not found
# (1.1732555627822876, -69.5572509765625), (86.20638275146484, -7.808422565460205)
