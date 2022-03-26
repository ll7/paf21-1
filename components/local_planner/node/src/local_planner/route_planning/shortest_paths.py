"""This module computes shortest paths in a navigation graph."""

from typing import List
import numpy as np


def shortest_path(start_pos: int, end_pos: int, matrix: np.ndarray) -> List[int]:
    """Find the shortest path for the given start / end positions and graph"""

    # compute a shortest edge adjacency given the start position
    backtrace = _naive_dijkstra(matrix, start_pos)

    # unroll the shortest path from behind using the backtrace
    path = [end_pos]
    pos = end_pos
    while pos != start_pos:
        pos = backtrace[pos]
        if pos == -1:
            return [start_pos]
        path.insert(0, pos)

    return path


def _naive_dijkstra(matrix: np.ndarray, start_pos: int) -> np.ndarray:
    """A very naive and simplified implementation of Dijkstra's algorithm.

    Returns a backtrace adjacency of shortest edges when traversing the graph
    backwards from an arbitrary destination node to the start node."""

    # initialize the distances array with infinite distances
    # and the backtrace array with 'no connection' links
    num_nodes = matrix.shape[0]
    dist = np.ones(shape=(num_nodes,)) * np.inf
    backtrace = np.ones(shape=(num_nodes,)).astype('int32') * -1

    dist[start_pos] = 0.0
    queue = list(range(num_nodes))

    # relax edges until there's no further relaxation
    while queue:
        current_node = _next_node(queue, dist)
        if current_node == -1:
            break
        queue.remove(current_node)

        # find all nodes connected to the current node by an outgoing edge
        conn_nodes = [i for i in list(range(num_nodes)) if matrix[current_node][i]]

        # allow the shortest paths to use the current node
        # -> update dist / backtrace array if there's a shorter path
        for index in conn_nodes:
            new_dist = dist[current_node] + matrix[current_node][index]
            if new_dist < dist[index]:
                dist[index] = new_dist
                backtrace[index] = current_node

    return backtrace


def _next_node(queue: List[int], dist: np.ndarray) -> int:
    """Determine the index of the next edge to relax,
    defaulting to -1 when there are no further relaxations possible."""

    minimum = np.inf
    min_index = -1

    # info: this min_key search could be implemented by a prio queue
    for index in queue:
        if dist[index] < minimum:
            minimum = dist[index]
            min_index = index

    return min_index
