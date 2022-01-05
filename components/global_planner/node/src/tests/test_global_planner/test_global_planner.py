import numpy as np
from global_planner.global_route_planner import ShortestPath


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
