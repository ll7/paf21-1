"""A global route planner based on map and hmi data."""
import math
import json
import os.path
import numpy as np
import networkx as nx
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
# for ROS connection
# from std_msgs.msg import String as StringMsg
# from sensor_msgs.msg import NavSatFix as GpsMsg
# import xodr_converter
from typing import Tuple, List
import rospy


class GlobalRoutePlanner:
    """A global route planner based on map and hmi data."""
    def __init__(self, num_nodes):
        """Initialize the global route planner."""
        self.num_nodes = num_nodes
        # base filepath to the maps
        self.filepath = r"../../../maps"
        # graph with
        self.graph = np.zeros(shape=(num_nodes, num_nodes))
        self.graph_start_end = np.zeros(shape=(num_nodes+2, num_nodes+2))
        # initialize all distances with inf.
        self.dist = np.ones(shape=(num_nodes+2,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = np.ones(shape=(num_nodes+2,)).astype('int32') * (-1)
        # path
        self.path = []
        # dict with lane-let ids and matrix pos
        self.mapping = {}

        self.map_name = ''
        self.end_pos = np.zeros(shape=(2,))
        self.start_pos = np.zeros(shape=(2,))
        self.orientation = 0.0
        self.update = False
        # TODO read from data
        self.road_width = 4.0

    def set_matrix(self, matrix: np.ndarray):
        """Set the graph with a matrix (numpy array)."""
        self.graph = matrix

    def set_mapping(self, mapping: dict):
        """Set the mapping and the nodes."""
        self.mapping = mapping

    def load_map_data(self) -> None or FileNotFoundError:
        """Load the data from the file with the map
            name and set the mapping and the matrix."""
        map_path = os.path.join(self.filepath, self.map_name, ".json")
        if not os.path.isfile(map_path):
            return FileNotFoundError

        with open(map_path, encoding="utf-8") as json_file:
            data = json.load(json_file)
            self.set_mapping(data['mapping'])
            self.set_matrix(data['matrix'])
            return None

    # def set_map_end_pos(self, msg: StringMsg):
    def update_map_end_pos(self, msg):
        """Update the current map name."""
        if self.map_name != msg.map:
            self.map_name = msg.map
            self.load_map_data()

        self.end_pos = msg.end_point
        self.update = True

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.start_pos = vehicle_pos
        rospy.loginfo(f'start pos:  {self.start_pos}')

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.orientation = orientation
        #rospy.loginfo(f'orientation {self.orientation}')

    # def set_gps(self, msg: GpsMsg):
    def update_gps(self, msg):
        """Update the GPS position of the vehicle."""
        # TODO Switch to odometry message
        # longitude = msg.longitude
        # latitude = msg.latitude
        #
        # x = self.world_radius * math.sin(latitude) * math.cos(longitude)
        # y = self.world_radius * math.sin(latitude) * math.sin(longitude)
        #
        # x2 = self.world_radius * longitude * math.cos(latitude)
        # y2 = self.world_radius * latitude
        #
        # print(x, '  ', y)
        # print(x2, '  ', y2)
        # self.start_pos = (x, y)

    def get_pos(self, node_id: str) -> int or KeyError:
        """Get the position for the node id."""
        if node_id not in self.mapping:
            return KeyError

        return self.mapping[node_id][0]

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

    def dijkstra(self, start_pos: int):
        """Implementation of the Dijkstra algorithm."""
        # distance of source to itself is 0
        # start_pos = self.get_pos(start_id)
        self.dist[start_pos] = 0.0

        # add all nodes_id in queue
        queue = list(range(self.num_nodes+2))

        # find the shortest path for all nodes
        while queue:
            # pick the minimum dist node from the set of nodes
            index_min = self._min_distance(queue)
            #print(index_min)
            if index_min == -1:
                return
            # remove min element
            queue.remove(index_min)

            # update dist value and parent
            for num in queue:
                # update dist[i] if it is in queue, there is an edge from index_min to i,
                if self.graph_start_end[index_min][num]:
                    new_dist = self.dist[index_min] + self.graph_start_end[index_min][num]
                    if new_dist < self.dist[num]:
                        self.dist[num] = new_dist
                        self.parent[num] = index_min

    def _append_pos2path(self, pos_start: int, pos: int):
        """Append the position to the path."""
        # self.path.append(pos)
        self.path.insert(0, pos)
        if self.parent[pos] == pos_start:
            self.path.insert(0, pos_start)
        # check if pos has a parent
        elif self.parent[pos] != -1:
            # recursive call
            self._append_pos2path(pos_start, self.parent[pos])

    def _append_id2path(self, start_id: str, target_id: str):
        """Append the pos of the id to the path."""
        # get the position
        pos_start = self.get_pos(start_id)
        pos_target = self.get_pos(target_id)
        # append the position to the path
        self._append_pos2path(pos_start, pos_target)

    def get_path_ids(self) -> list:
        """Get the ids for the path."""
        key_list = list(self.mapping.keys())
        return [key_list[pos] for pos in self.path]

    def show_graph_with_labels(self):
        """Draw a graph with labeled nodes."""
        # get all edges out of the graph with value greater 0
        edges = np.where(self.graph > 0)
        graph = nx.Graph()
        # add all edges
        graph.add_edges_from(edges)
        # draw the graph
        nx.draw(graph, node_size=500, labels=list(self.mapping.keys()), with_labels=True)

    def find_nearest_road(self) -> (list, list):
        """Find the nearest road to the start and end point."""
        ids_start = []
        ids_end = []
        self.graph_start_end = np.append(np.copy(self.graph), np.zeros((2, self.num_nodes)), axis=0)
        self.graph_start_end = np.append(self.graph_start_end,
                                         np.zeros((self.num_nodes+2, 2)), axis=1)
        #print(self.graph_start_end.shape)
        #print(self.mapping)
        #rospy.loginfo(f'index:  {self.mapping}')
        self.mapping['-1_0'] = self.num_nodes, self.start_pos
        self.mapping['-2_0'] = self.num_nodes+1,  self.end_pos

        key_list = list(self.mapping.keys())

        for i in range(0, self.num_nodes+2, 2):
            start_point = self.mapping[key_list[i]][1]
            end_point = self.mapping[key_list[i+1]][1]
            div = (end_point[0]-start_point[0])
            if div == 0:
                div = 0.000000000001
            alpha = np.arctan((end_point[1]-start_point[1]) / div)
            beta = math.pi + alpha + math.pi/2

            offset_x = math.cos(beta) * self.road_width
            offset_y = math.sin(beta) * self.road_width
            polygon = Polygon([(start_point[0] + offset_x, start_point[1] - offset_y),
                               (start_point[0] - offset_x, start_point[1] + offset_y),
                               (end_point[0] + offset_x, end_point[1] - offset_y),
                               (end_point[0] - offset_x, end_point[1] + offset_y)
                               ])

            if polygon.contains(Point(self.start_pos[0], self.start_pos[1])):
                ids_start.append(key_list[i])
                # TODO set weights for start and end (distance)
                ori_to_point = math.atan2(end_point[1]-start_point[1], div)
                if self.orientation == ori_to_point:
                    self.graph_start_end[i][self.num_nodes] = 10
                    self.graph_start_end[self.num_nodes][i] = 10
                else:
                    self.graph_start_end[i][self.num_nodes] = 10
                    self.graph_start_end[self.num_nodes][i] = 10
                print('start road:', key_list[i], ' i:', i)

            if polygon.contains(Point(self.end_pos[0], self.end_pos[1])):
                ids_end.append(key_list[i])
                self.graph_start_end[i][self.num_nodes+1] = 10
                self.graph_start_end[self.num_nodes+1][i] = 10
                print('end road:', key_list[i], ' i:', i)
        return ids_start, ids_end

    def calculate_distance(self, point):
        """Calculate the distance from the point to the road."""

    def compute_route(self) -> list:
        print("Hallo :D")
        """Compute the route."""
        # 0. check map data is available
        # reload if new mapp

        # 1. load and set map -> Done

        # 2. start and endpoint
            # start from update_vehicle_position()
            #self.start_pos = (20.0, 0.004)
        self.start_pos = np.array([101.62, -328.59])
        self.end_pos = np.array([144.99, -57.5])
        # self.start_pos = np.array([20.0, 0.004])
        # self.end_pos = np.array([255.0, -0.004])
        # 2.05 find points
        ids_end = self.find_nearest_road()
        # 2.1 insert start point to matrix --> n-2  /  found point

        # 2.2 insert end point to matrix --> n-1  /  found point

        # from start and end point get id

        # 3. start dijkstra
        # # TODO
        # print('start: ', self.graph_start_end[14][self.num_nodes])
        # print('start: ', self.graph_start_end[15][self.num_nodes])
        # print('end: ', self.graph_start_end[6][self.num_nodes+1])
        # print('end: ', self.graph_start_end[7][self.num_nodes+1])
        self.dijkstra(self.mapping['-1_0'][0])
        #print(self.dist)

        # TODO
        rospy.loginfo(f'ID END{ids_end[1][0]}')
        rospy.loginfo(f'ID END2{ids_end[1]}')

        for i in range(30):
            rospy.loginfo(" ")
        self._append_id2path('-1_0', '-2_0')
        list_lanes = []
        list_waypoints = []
        #print(self.path)


        key_list = list(self.mapping.keys())

        for path in self.path:
            if int(self.mapping[key_list[path]][0]) % 2 == 0 or index == len(self.path)-1:
                list_waypoints.append({'x': float(self.mapping[key_list[path]][1][0]),
                                       'y': float(self.mapping[key_list[path]][1][1])})
                list_lanes.append(key_list[path])
        print(list_waypoints)
        rospy.loginfo(f'Found Route {list_waypoints}')

        # 4. convert to list of dic

        # 5. output thr route
        return json.dumps(list_waypoints)
