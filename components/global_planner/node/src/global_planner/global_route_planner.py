import math
import json
import numpy as np
import networkx as nx
import os.path
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
# for ROS connection
# from std_msgs.msg import String as StringMsg
# from sensor_msgs.msg import NavSatFix as GpsMsg
# import xodr_converter


class GlobalRoutePlanner:
    """A global route planner based on map and hmi data."""
    def __init__(self, num_nodes):
        """Initialize the global route planner."""
        self.num_nodes = num_nodes
        # base filepath to the maps
        self.filepath = r"../../../maps"
        # dict with lane-let ids and matrix pos
        self.nodes = {}
        # counter for number of nodes
        self.matrix_pos = 0
        # graph with
        self.graph = np.zeros(shape=(num_nodes, num_nodes))
        self.graph_start_end = np.zeros(shape=(num_nodes+2, num_nodes+2))
        # initialize all distances with inf.
        self.dist = np.ones(shape=(num_nodes+2,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = np.ones(shape=(num_nodes+2,)).astype('int32') * (-1)
        # path
        self.path = []
        # mapping
        self.mapping = []

        self.world_radius = 6378137
        self.map_name = ''
        self.end_pos = (0.0, 0.0)
        self.start_pos = (0.0, 0.0)
        self.update = False
        # TODO read from data
        self.road_width = 4.0

    def set_matrix(self, matrix: np.ndarray):
        self.graph = matrix

    def set_mapping(self, mapping: list):
        self.mapping = mapping

    def load_map_data(self):
        """Load the data from the file with the map
            name and set the mapping and the matrix."""
        map_path = os.path.join(self.filepath, self.map_name, ".json")
        if not os.path.isfile(map_path):
            return FileNotFoundError

        with open(map_path) as json_file:
            data = json.load(json_file)
            self.set_mapping(data['mapping'])
            self.set_matrix(data['matrix'])

    # def set_map_end(self, msg: StringMsg):
    def set_map_end(self, msg):
        """Update the current map name"""
        if self.map_name != msg.map:
            self.map_name = msg.map
            self.load_map_data()

        self.end_pos = msg.end_point
        self.update = True

    # def set_gps(self, msg: GpsMsg):
    def set_gps(self, msg):
        """Update the GPS position of the vehicle."""
        longitude = msg.longitude
        latitude = msg.latitude

        x = self.world_radius * math.sin(latitude) * math.cos(longitude)
        y = self.world_radius * math.sin(latitude) * math.sin(longitude)

        x2 = self.world_radius * longitude * math.cos(latitude)
        y2 = self.world_radius * latitude

        print(x, '  ', y)
        print(x2, '  ', y2)
        self.start_pos = (x, y)

    def get_pos(self, node_id: str) -> int or KeyError:
        """Get the position for the node id."""
        if node_id not in self.num_nodes:
            return KeyError

        return self.nodes[node_id]

    def _min_distance(self, queue: list) -> int or AttributeError:
        """Calculate the index with the minimum distance."""
        # initialize min value and min_index as -1
        minimum = np.inf
        min_index = -1

        # from the dist array, pick one which has min value and is till in queue
        for index in queue:
            if self.dist[index] < minimum:
                # set the new minimum
                minimum = self.dist[index]
                # set new index
                min_index = index

        if min_index == -1:
            return AttributeError

        # return the index
        return min_index

    def dijkstra(self, start_id: str):
        """Implementation of the Dijkstra algorithm."""
        # distance of source to itself is 0
        start_pos = self.get_pos(start_id)
        self.dist[start_pos] = 0.0

        # add all nodes_id in queue
        queue = list(range(self.num_nodes))

        # find the shortest path for all nodes
        while queue:
            # pick the minimum dist node from the set of nodes
            index_min = self._min_distance(queue)
            # remove min element
            queue.remove(index_min)

            # update dist value and parent
            for num in queue:
                # update dist[i] if it is in queue, there is an edge from index_min to i,
                if self.graph[index_min][num]:
                    new_dist = self.dist[index_min] + self.graph[index_min][num]
                    if new_dist < self.dist[num]:
                        self.dist[num] = new_dist
                        self.parent[num] = index_min

    def _append_pos2path(self, pos: int) -> int:
        """Append the position to the path"""
        self.path.append(pos)
        # check if pos has a parent
        if self.parent[pos] != -1:
            # recursive call
            self._append_pos2path(self.parent[pos])

        return pos

    def _append_id2path(self, target_id: str):

        # get the position
        pos = self.get_pos(target_id)
        # append the position to the path
        self._append_pos2path(pos)

    def get_path_ids(self) -> list:
        key_list = list(self.nodes.keys())
        return [key_list[pos] for pos in self.path]

    def show_graph_with_labels(self):
        # get all edges out of the graph with value greater 0
        edges = np.where(self.graph > 0)
        gr = nx.Graph()
        # add all edges
        gr.add_edges_from(edges)
        # draw the graph
        nx.draw(gr, node_size=500, labels=list(self.nodes.keys()), with_labels=True)

    def find_nearest_road(self) -> (list, list):
        ids_start = []
        ids_end = []
        self.graph_start_end = np.append(np.copy(self.graph), np.zeros((2, self.num_nodes)), axis=0)
        self.graph_start_end = np.append(self.graph_start_end, np.zeros((self.num_nodes+2, 2)), axis=1)
        print(self.graph_start_end.shape)
        print()
        self.mapping.append((-1, 0, self.start_pos))
        self.mapping.append((-2, 0, self.end_pos))

        for i in range(0, len(self.mapping), 2):
            start_point = self.mapping[i][2]
            end_point = self.mapping[i+1][2]
            div = (end_point[0]-start_point[0])
            if div == 0:
                div = 0.000000000001
            alpha = np.arctan((end_point[1]-start_point[1]) / div)
            beta = math.pi + alpha + math.pi/2

            offset_x = math.cos(beta) * self.road_width
            offset_y = math.sin(beta) * self.road_width
            ll_corner = (start_point[0] + offset_x, start_point[1] - offset_y)
            lu_corner = (start_point[0] - offset_x, start_point[1] + offset_y)
            rl_corner = (end_point[0] + offset_x, end_point[1] - offset_y)
            ru_corner = (end_point[0] - offset_x, end_point[1] + offset_y)

            polygon = Polygon([ll_corner, lu_corner, ru_corner, rl_corner])
            if polygon.contains(Point(self.start_pos[0], self.start_pos[1])):
                ids_start.append([self.mapping[i][0], self.mapping[i][1]])
                # ToDo Gewichtung setzten zu start und ende (Abstand)
                self.graph_start_end[i][self.num_nodes] = 10
                self.graph_start_end[self.num_nodes][i] = 10
                print('start road:', self.mapping[i][0], ' i:', i)

            if polygon.contains(Point(self.end_pos[0], self.end_pos[1])):
                ids_end.append([self.mapping[i][0], self.mapping[i][1]])
                self.graph_start_end[i][self.num_nodes+1] = 10
                self.graph_start_end[self.num_nodes+1][i] = 10
                print('end road:', self.mapping[i][0], ' i:', i)
        return ids_start, ids_end

    def calculate_distance(self, point):
        pass

    def compute_route(self):
        # 0. check map data is available
        # reload if new mapp

        # 1. load and set map -> Done

        # 2. start and endpoint
        self.start_pos = (20.0, 0.004)
        self.end_pos = (250.0, -0.004)
        # 2.05 find points
        self.find_nearest_road()
        # 2.1 insert start point to matrix --> n-2  /  found point

        # 2.2 insert end point to matrix --> n-1  /  found point

        # from start and end point get id

        # 3. start dijkstra
        # ToDo
        print('start: ', self.graph_start_end[14][self.num_nodes])
        print('start: ', self.graph_start_end[15][self.num_nodes])
        print('end: ', self.graph_start_end[6][self.num_nodes+1])
        print('end: ', self.graph_start_end[7][self.num_nodes+1])
        self.dijkstra(self.num_nodes)
        print(self.dist)
        # TODO
        self._append_id2path('6')
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
