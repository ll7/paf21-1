import numpy as np
import networkx as nx
import xml.etree.ElementTree as eTree


class GlobalPlanner:
    def __init__(self, num_nodes):
        self.num_nodes = num_nodes
        # dict with lane-let ids and matrix pos
        self.nodes = {}
        # counter for number of nodes
        self.matrix_pos = 0
        # graph with
        self.graph = np.zeros(shape=(num_nodes, num_nodes))
        # initialize all distances with inf.
        self.dist = np.ones(shape=(num_nodes,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = np.ones(shape=(num_nodes,)) * (-1)
        # path
        self.path = []

    def add_node(self, node_id: str):
        if len(self.nodes) < self.num_nodes:
            # create new entry in dict
            self.nodes[node_id] = self.matrix_pos
            # add the matrix position up
            self.matrix_pos += 1
        else:
            print('Reached maximum of nodes!')

    def get_pos(self, node_id):
        return self.nodes[node_id] if node_id in self.nodes else -1

    def add_edge(self, start_id, target_id, weight):
        # check if start_id and target_id ar in the dict
        if start_id not in self.nodes:
            self.add_node(start_id)
        if target_id not in self.nodes:
            self.add_node(target_id)

        start_pos = self.get_pos(start_id)
        target_pos = self.get_pos(target_id)

        # set the weight in the graph
        self.graph[start_pos, target_pos] = weight
        # self.graph[target_pos, start_pos] = -1.0

    def min_distance(self, queue):
        # initialize min value and min_index as -1
        minimum = np.inf
        min_index = 0

        # from the dist array, pick one which has min value and is till in queue
        for index in range(self.num_nodes):
            if self.dist[index] < minimum and index in queue:
                # set the new minimum
                minimum = self.dist[index]
                # set new index
                min_index = index
        # return the index
        return min_index

    def dijkstra(self, start_id: str):
        # distance of source to itself is 0
        start_pos = self.get_pos(start_id)
        self.dist[start_pos] = 0.0

        # add all nodes_id in queue
        queue = list(range(self.num_nodes))

        # find the shortest path for all nodes
        while queue:
            # pick the minimum dist node from the set of nodes
            index_min = self.min_distance(queue)
            # remove min element
            queue.remove(index_min)

            # update dist value and parent
            for num in range(self.num_nodes):
                # update dist[i] if it is in queue, there is an edge from index_min to i,
                if self.graph[index_min][num] and num in queue:
                    new_dist = self.dist[index_min] + self.graph[index_min][num]
                    if new_dist < self.dist[num]:
                        self.dist[num] = new_dist
                        self.parent[num] = index_min
        # return self.dist, self.parent

    def print_path(self, pos):
        # Base Case : If j is source
        if self.parent[pos] == -1:
            self.path.append(pos)
            return pos
        self.print_path(self.parent[pos])

        self.path.append(pos)
        return pos

    def print_solution(self, target_id):
        for num in range(1, self.num_nodes):
            if num == self.get_pos(target_id):
                self.print_path(num)
        return self.path

    def get_path_ids(self):
        key_list = list(self.nodes.keys())
        return [key_list[p] for p in self.path]

    def show_graph_with_labels(self):
        edges = np.where(self.graph > 0)
        gr = nx.Graph()
        gr.add_edges_from(edges)
        nx.draw(gr, node_size=500, labels=list(self.nodes.keys()), with_labels=True)


class XODRConverter:
    def __init__(self):
        self.filename = ''


if __name__ == "__main__":
    filename = 'Town01.xodr'
    root = eTree.parse(filename).getroot()
    i = 0
    for child in root:
        if child.tag == 'road':
            i = i + 1
