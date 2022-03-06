from global_planner.global_route_planner import RoadDetection
from global_planner.xodr_converter import XODRConverter


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
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    _, is_right_road, road = neighbors[0]
    assert is_right_road and road.road_id == 0


def test_find_neighbor_sections_0_left():
    point = (380.0, -2.0)
    xodr_map = load_town_01()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    _, is_right_road, road = neighbors[0]
    assert not is_right_road and road.road_id == 0


def test_find_neighbor_sections_12_right():
    point = (245.85, -198.75)
    xodr_map = load_town_01()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    _, is_right_road, road = neighbors[0]
    assert is_right_road and road.road_id == 12


def test_find_neighbor_sections_12_left():
    point = (245.85, -195.75)
    xodr_map = load_town_01()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    _, is_right_road, road = neighbors[0]
    assert not is_right_road and road.road_id == 12


def test_find_neighbor_sections_curvature_one_sided_right():
    point = (92.00, -207.0)
    xodr_map = load_town_01()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    road_ids = [n[2].road_id for n in neighbors]
    assert 257 not in road_ids and 271 not in road_ids and \
           256 in road_ids and 272 in road_ids


def test_find_neighbor_sections_curvature_one_sided_left():
    point = (88.0, -207.0)
    xodr_map = load_town_01()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    road_ids = [n[2].road_id for n in neighbors]
    assert 257 in road_ids and 271 in road_ids and \
           256 not in road_ids and 272 not in road_ids


def test_find_neighbor_sections_5():
    point = (150.99, -57.5)
    xodr_map = load_town_01()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    road_ids = [n[2].road_id for n in neighbors]
    assert 334 in road_ids and 355 in road_ids and 333 not in road_ids


def test_find_neighbor_sections_town04_highway_1():
    point = (406.0252685546875, 124.70137786865234)
    xodr_map = load_town_04()
    neighbors = RoadDetection.find_neighbor_sections(point, xodr_map)
    road_ids = [n[2].road_id for n in neighbors]
    print(xodr_map.roads_by_id[36].right_ids, xodr_map.roads_by_id[36].left_ids)
    assert road_ids == [36]


def test_find_neighbor_sections_town04_highway_2():
    point = (16.040634155273438, 170.54249572753906)
    neighbors = RoadDetection.find_neighbor_sections(point, load_town_04())
    road_ids = [n[2].road_id for n in neighbors]
    assert road_ids == [912]
