"""This module computes shortest paths in a navigation graph."""

from typing import List
import numpy as np


def shortest_path(start_pos: int, end_pos: int, adj_matrix: np.ndarray) -> List[int]:
    """Find the shortest path for the given start / end positions and graph"""
    backtrace = _naive_dijkstra(adj_matrix, start_pos)
    return _unroll_shortest_path(start_pos, end_pos, backtrace)


def _unroll_shortest_path(start_pos: int, end_pos: int,
                          backtrace: List[int]) -> List[int]:
    """collect edge trajectories from the end until reaching the start."""

    path = [end_pos]
    pos = end_pos

    while pos != start_pos:
        pos = backtrace[pos]
        if pos == -1:
            return [start_pos]
        path.insert(0, pos)

    return path


def _naive_dijkstra(adj_matrix: np.ndarray, start_node: int) -> List[int]:
    """A very naive and simplified implementation of Dijkstra's algorithm.

    Returns a backtrace adjacency of shortest edges when traversing the graph
    backwards from an arbitrary destination node to the start node."""

    # initialize the distances array with infinite distances
    # and the backtrace array with 'no connection' links
    num_nodes = adj_matrix.shape[0]
    dist = np.full((num_nodes), np.inf, dtype=np.float32)
    backtrace = np.full((num_nodes,), -1, dtype=np.int32)

    dist[start_node] = 0.0
    unvisited_nodes = list(range(num_nodes))

    # visit all nodes reachable from the start node
    while unvisited_nodes:

        # pick the next node to visit
        dist_in_queue = dist[unvisited_nodes]
        node_to_visit = unvisited_nodes[np.argmin(dist_in_queue)]

        if dist[node_to_visit] == np.inf:
            break # only unreachable nodes left -> exit

        # find the visited node's neighbors (only outgoing edges)
        conn_nodes = [i for i in list(range(num_nodes)) if adj_matrix[node_to_visit][i]]

        # allow the shortest paths to use the current node
        # -> update dist / backtrace array if there's a shorter path
        for index in conn_nodes:
            new_dist = dist[node_to_visit] + adj_matrix[node_to_visit][index]
            if new_dist < dist[index]:
                dist[index] = new_dist
                backtrace[index] = node_to_visit

        # mark the node as visited
        unvisited_nodes.remove(node_to_visit)

    return backtrace.tolist()
