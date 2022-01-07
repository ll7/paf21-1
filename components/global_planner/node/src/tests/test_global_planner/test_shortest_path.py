from pathlib import Path
import numpy as np
from global_planner.global_route_planner import ShortestPath
from global_planner.xodr_converter import XODRConverter


# path = Path("/app/res/xodr/Town01.xodr")
xodr_path = Path("../../../../xodr/Town01.xodr")
xodr_map = XODRConverter.read_xodr(xodr_path)


def test_shortest_path():
    graph = np.zeros(shape=(6, 6))
    graph[2, 4] = 33
    graph[3, 5] = 2
    graph[3, 4] = 20
    graph[4, 5] = 1
    graph[2, 3] = 20
    graph[1, 4] = 10
    graph[1, 3] = 50
    graph[0, 2] = 20
    graph[0, 1] = 10

    path = ShortestPath.shortest_path(0, 0, graph)
    assert(path == [0])

    path = ShortestPath.shortest_path(2, 5, graph)
    assert(path == [2, 3, 5])

    path = ShortestPath.shortest_path(5, 1, graph)
    assert(path == [5])


def test_find_neighbor_sections_1():
    point = (245.85, -198.75)
    neighbors = ShortestPath.find_neighbor_sections(point, xodr_map)
    _, is_right_road, road = neighbors[0]
    assert is_right_road and road.road_id == 12


def test_find_neighbor_sections_2():
    point = (150.99, -57.5)
    neighbors = ShortestPath.find_neighbor_sections(point, xodr_map)
    for n in neighbors:
        print(n)
    assert True
