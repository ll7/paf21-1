import math

import numpy as np
import networkx as nx
#from std_msgs.msg import String as StringMsg
#from sensor_msgs.msg import NavSatFix as GpsMsg
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
#import xodr_converter


class GlobalRoutePlanner:
    def __init__(self, num_nodes):
        self.num_nodes = num_nodes
        # dict with lane-let ids and matrix pos
        self.nodes = {}
        # counter for number of nodes
        self.matrix_pos = 0
        # graph with
        self.graph = np.zeros(shape=(num_nodes, num_nodes))
        # initialize all distances with inf.
        self.dist = [float("Inf")] * num_nodes
        # array for the parents to store shortest path tree
        self.parent = [-1] * num_nodes
        # path
        self.path = []
        #mapping
        self.mapping = []

        self.world_radius = 6378137
        self.map_name = ''
        self.end_pos = (0.0, 0.0)
        self.start_pos = (0.0, 0.0)
        # TODO read from data
        self.road_width = 4.0
        self.update = False

    def set_matrix(self, matrix):
        self.graph = matrix

    def set_mapping(self, mapping):
        self.mapping = mapping

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

    # TODO auslagern in XODR Converter / RL
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

    def min_distance2(self, queue, dist):
        # initialize min value and min_index as -1
        minimum = float("Inf")
        min_index = -1
        for i in range(len(dist)):
            if dist[i] < minimum and i in queue:
                minimum = dist[i]
                min_index = i
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

    def dijkstra2(self, src):
        row = len(self.graph)
        col = len(self.graph[0])
        dist = [float("Inf")] * row
        dist[src] = 0.0
        queue = []
        #Fill queue 0- x
        for i in range(row):
            queue.append(i)

        while queue:
            u = self.min_distance2(queue, dist)
            if u == -1:
                return
            queue.remove(u)
            for i in range(col):
                if self.graph[u][i] and i in queue:
                    if dist[u] + self.graph[u][i] < dist[i]:
                        dist[i] = dist[u] + self.graph[u][i]
                        self.parent[i] = u
            self.dist = dist

    def append_path(self, pos):
        # Base Case : If j is source
        if self.parent[pos] == -1:
            self.path.append(pos)
            return pos
        self.append_path(self.parent[pos])

        self.path.append(pos)
        return pos

    def calculate_route(self, target_id):
        for num in range(1, self.num_nodes):
            if num == target_id:
                self.append_path(num)


    def get_path_ids(self):
        key_list = list(self.nodes.keys())
        return [key_list[p] for p in self.path]

    def show_graph_with_labels(self):
        edges = np.where(self.graph > 0)
        gr = nx.Graph()
        gr.add_edges_from(edges)
        nx.draw(gr, node_size=500, labels=list(self.nodes.keys()), with_labels=True)

    def find_nearest_road(self, point):

        ids = []

        for i in range(0, len(self.mapping), 2):
            start_point = self.mapping[i][2]
            end_point = self.mapping[i+1][2]

            # TODO only approx -> calculate right box
            left_lower_corner = start_point[1] - self.road_width
            left_upper_corner = start_point[1] + self.road_width
            right_lower_corner = end_point[1] - self.road_width
            right_upper_corner = end_point[1] + self.road_width

            polygon = Polygon([(start_point[0], left_lower_corner),
                               (start_point[0], left_upper_corner),
                               (end_point[0], right_lower_corner),
                               (end_point[0], right_upper_corner)])

            if polygon.contains(point):
                ids.append([self.mapping[i][0], self.mapping[i][1]])

        return ids

    def compute_route(self):
        # 0. prüfen ob die map daten vorhanden sind
        # nur neu laden falls neue map

        # 1. load and set map
            # Done
        # 2. start and endpoint
            # 2.05 Finde Punkte
                # done
            # 2.1 StartPunkt in Matrix einfuegen --> n-2  /  gefundener punkt

            # 2.2 EndPunkt in Matrix einfuegen --> n-1  /  gefundener punkt

        # from start and end point get id


        # 3. start dijkstra
        # ToDo
        self.dijkstra2(0)
        # TODO
        self.calculate_route(5)
        list_lanes = []
        list_waypoints = []
        print(self.path)

        for path in self.path:
            if int(self.mapping[path][1]) % 2 == 0:
                list_waypoints.append({'x': self.mapping[path][2][0], 'y': self.mapping[path][2][1]})
                list_lanes.append([self.mapping[path][0], self.mapping[path][1]])
        print(list_waypoints)

        # 4. convert to list of dic

        # 5. output thr route
        pass

    def load_map_data(self):
        pass


    #def set_map_end(self, msg: StringMsg):
    def set_map_end(self, msg):
        if self.map_name != msg.map:
            self.map_name = msg.map
            self.load_map_data()

        self.end_pos = msg.end_point
        self.update = True


    def load_map_data(self):
        #self.filepath
        #self.map_name
        pass

    #def set_gps(self, msg: GpsMsg):
    def set_gps(self, msg):
        """Update the GPS position of the vehicle"""
        longitude = msg.longitude
        latitude = msg.latitude

        x = self.world_radius * math.sin(latitude) * math.cos(longitude)
        y = self.world_radius * math.sin(latitude) * math.sin(longitude)

        x2 = self.world_radius * longitude * math.cos(latitude)
        y2 = self.world_radius * latitude

        print(x, '  ', y)
        print(x2, '  ', y2)
        self.start_pos = (x, y)


