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
    def __init__(self):
        """Initialize the global route planner."""
        self.num_nodes = 0
        # base filepath to the maps
        self.filepath = r"../../../maps"
        # graph with
        self.graph = None
        self.graph_start_end = None
        # initialize all distances with inf.
        self.dist = None   # np.ones(shape=(0+2,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = None     # np.ones(shape=(0+2,)).astype('int32') * (-1)
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
        self.point_dict = {}
        self.road_dict = None

    def set_matrix(self, matrix: np.ndarray):
        """Set the graph with a matrix (numpy array)."""
        print(f'Shape of Graph GRP {matrix.shape}')
        self.graph = np.copy(matrix)
        print(f'Shape of Graph GRP {self.graph.shape}')

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
        #rospy.loginfo(f'start pos:  {self.start_pos}')

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

        return self.mapping[node_id]

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

        self.num_nodes = self.graph.shape[0]+2
        self.dist = np.ones(shape=(self.num_nodes,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = np.ones(shape=(self.num_nodes,)).astype('int32') * (-1)

        # distance of source to itself is 0
        # start_pos = self.get_pos(start_id)
        self.dist[start_pos] = 0.0

        # add all nodes_id in queue
        queue = list(range(self.num_nodes))
        # find the shortest path for all nodes
        while queue:
            # pick the minimum dist node from the set of nodes
            index_min = self._min_distance(queue)
            # print(index_min, queue)
            # remove min element
            queue.remove(index_min)

            # update dist value and parent
            for num in queue:
                # update dist[i] if it is in queue, there is an edge from index_min to i,
                # TODO changed graph start end [index_min][num]
                if self.graph_start_end[num][index_min]:
                # if self.graph_start_end[index_min][num]:
                    new_dist = self.dist[index_min] + self.graph_start_end[num][index_min]
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
        self.num_nodes = self.graph.shape[0] + 2
        print(f'Shape of Graph {len(self.mapping)}')
        start_pos = self.start_pos
        print(f"Start Point: {start_pos}")
        start_pos = (245.8500213623047, -198.7483367919922)
        print(f"Start Point: {start_pos}")
        self.graph_start_end = np.append(np.copy(self.graph), np.zeros((2, self.num_nodes-2)), axis=0)
        self.graph_start_end = np.append(self.graph_start_end,
                                         np.zeros((self.num_nodes, 2)), axis=1)
        print(f'Shape of GraphStartEnd {self.graph_start_end.shape}')

        #print(self.graph_start_end.shape)
        #print(self.mapping)
        #rospy.loginfo(f'index:  {self.mapping}')
        self.mapping['-1_0_0'] = self.num_nodes-2
        self.mapping['-2_0_0'] = self.num_nodes-1

        for road_id, roads in self.point_dict.items():
            for index, geo in enumerate(roads):

                start_point = geo[0]
                end_point = geo[1]

                div = (end_point[0] - start_point[0])
                if div == 0:
                    div = 0.000000000001
                alpha = np.arctan((end_point[1] - start_point[1]) / div)
                beta = math.pi + alpha + math.pi / 2

                offset_x = math.cos(beta) * self.road_width
                offset_y = math.sin(beta) * self.road_width
                polygon = Polygon([(start_point[0] + offset_x, start_point[1] - offset_y),
                                   (start_point[0] - offset_x, start_point[1] + offset_y),
                                   (end_point[0] - offset_x, end_point[1] + offset_y),
                                   (end_point[0] + offset_x, end_point[1] - offset_y),
                                   ])

                if polygon.contains(Point(start_pos[0], start_pos[1])):

                    # print("[{'x': ", start_point[0] + offset_x, ", 'y': ", start_point[1] - offset_y, "},"
                    #       "{'x': ", start_point[0] - offset_x, ", 'y': ", start_point[1] + offset_y, "},"
                    #       "{'x': ", end_point[0] - offset_x, ", 'y': ", end_point[1] + offset_y, "},"
                    #       "{'x': ", end_point[0] + offset_x, ", 'y': ", end_point[1] - offset_y, "}]")
                    polygon2 = Polygon([(start_point[0], start_point[1]),
                                        (start_point[0] - offset_x, start_point[1] + offset_y),
                                       (end_point[0] - offset_x, end_point[1] + offset_y),
                                       (end_point[0], end_point[1]),
                                       ])
                    lanelink = -1
                    reference_point = 1
                    if polygon2.contains(Point(start_pos[0], start_pos[1])):
                        lanelink = 1
                        reference_point = 0
                    ids_start.append([road_id, index, reference_point])
                    distance = self.accumulate_dist(roads, reference_point, index)
                    i = self.find_mapping(road_id, reference_point, lanelink)
                    if reference_point == 0:
                        self.graph_start_end[i][self.num_nodes - 2] = distance
                        # self.graph_start_end[self.num_nodes - 2][i] = distance
                    else:
                        self.graph_start_end[self.num_nodes - 2][i] = distance
                        # self.graph_start_end[i][self.num_nodes - 2] = distance
                    # self.graph_start_end[i][self.num_nodes - 2] = distance
                    # self.graph_start_end[self.num_nodes - 2][i] = distance

                if polygon.contains(Point(self.end_pos[0], self.end_pos[1])):

                    polygon2 = Polygon([(start_point[0], start_point[1]),
                                        (start_point[0] - offset_x, start_point[1] + offset_y),
                                        (end_point[0] - offset_x, end_point[1] + offset_y),
                                        (end_point[0], end_point[1]),
                                        ])
                    lanelink = -1
                    reference_point = 1
                    if polygon2.contains(Point(self.end_pos[0], self.end_pos[1])):
                        lanelink = 1
                        reference_point = 0
                    ids_end.append([road_id,index, reference_point])
                    distance = self.accumulate_dist(roads, reference_point, index)
                    # print(roads, reference_point, index)
                    i = self.find_mapping(road_id, reference_point, lanelink)
                    # if i is None:
                    #     continue
                    # print("i: ", i)
                    # print(self.graph_start_end[249][249])
                    if reference_point == 0:
                        # self.graph_start_end[i][self.num_nodes - 1] = distance
                        self.graph_start_end[self.num_nodes - 1][i] = distance

                    else:
                        # self.graph_start_end[self.num_nodes - 1][i] = distance
                        self.graph_start_end[i][self.num_nodes - 1] = distance
                    # self.graph_start_end[i][self.num_nodes - 1] = distance
                    # self.graph_start_end[self.num_nodes - 1][i] = distance
        print("id_start", ids_start)
        print()
        print("id_end", ids_end)

        return ids_start, ids_end

    def find_mapping(self, road: int, end: int, link: int):
        key = f"{road}_{end}_{link}"
        if key in self.mapping:
            return self.mapping[key]
        # Quick Fix ToDo
        key = f"{road}_{end}_{-link}"
        if key in self.mapping:
            return self.mapping[key]
        print("GRP: ", key)
        return AttributeError

    def accumulate_dist(self, roads, reference_point, geoPos):
        distance = 0.1

        for index2, geo2 in enumerate(roads):
            if reference_point == 0:
                if index2 > geoPos:
                    continue
                elif index2 == geoPos:
                    distance += 1
                    # idst start_pos -> start
                else:
                    distance += geo2[3]
            else:
                if index2 < geoPos:
                    continue
                elif index2 == geoPos:
                    distance += 1
                    # idst start_pos -> start
                else:
                    distance += geo2[3]
        return distance


    def calculate_distance(self, point):
        """Calculate the distance from the point to the road."""

    def linear_interpolation(self, start, end, interval_m):
        listPol = []
        start = [start['x'], start['y']]
        end = [end['x'], end['y']]

        # dist = math.dist(start, end)
        distance = np.linalg.norm(np.array(start) - np.array(end))
        difference_se = (end[0] - start[0], end[1] - start[1])
        #+1 bei Komma
        steps = math.ceil((distance/interval_m))
        adddiff = (0.0, 0.0)
        if distance > interval_m:
            adddiff = (difference_se[0]/ steps, difference_se[1]/ steps,)
        else:
            adddiff = difference_se

        diff = (0.0, 0.0)
        for index in range(steps):
            point = (start[0]+diff[0], start[1] + diff[1])
            diff = (diff[0]+adddiff[0], diff[1]+adddiff[1])
            if index > 0 and (listPol[-1] == point):
                continue
            listPol.append(point)
        return listPol

    @staticmethod
    def calculate_offset(start_point, end_point, road_width) -> List[float]:
        """Calculate the offset according the road_width"""
        div = (end_point[0] - start_point[0])
        if div == 0:
            div = 0.000000000001
        alpha = np.arctan((end_point[1] - start_point[1]) / div)
        beta = math.pi + alpha + math.pi / 2

        return [math.cos(beta) * road_width, math.sin(beta) * road_width]

    def calculate_offset2points(self, start_point, end_point, road_width, direction) -> List[tuple]:
        """Function to calculate an offset to the start and end point."""

        offset = self.calculate_offset(start_point, end_point, road_width)

        if direction < 0:
            return [
                (start_point[0] - offset[0], start_point[1] + offset[1]),
                (end_point[0] - offset[0], end_point[1] + offset[1])
            ]
        else:
            return [
                (start_point[0] + offset[0], start_point[1] - offset[1]),
                (end_point[0] + offset[0], end_point[1] - offset[1])
            ]

    def compute_route(self) -> str:
        """Compute the route."""
        # 0. check map data is available
        # reload if new mapp

        # 1. load and set map -> Done

        # 2. start and endpoint
            # start from update_vehicle_position()
            #self.start_pos = (20.0, 0.004)
        # self.start_pos = np.array([101.62, -328.59])
        # self.end_pos = np.array([144.99, -57.5])
        #self.start_pos = np.array([200.63262939453125, -2.020033836364746])
        self.end_pos = np.array([144.99, -57.5])
        #self.start_pos = np.array([245.850891, -198.75])
        # self.end_pos = np.array([245.850891, -198.75])
        # self.start_pos = np.array([140.99, -55.5])
        # self.end_pos = np.array([255.0, -0.004])
        # 2.05 find points
        ids_start, ids_end = self.find_nearest_road()
        # 2.1 insert start point to matrix --> n-2  /  found point

        # 2.2 insert end point to matrix --> n-1  /  found point

        # from start and end point get id

        # 3. start dijkstra
        # # TODO

        self.dijkstra(self.mapping['-1_0_0'])
        #print(self.dist)

        # TODO
        self._append_id2path('-1_0_0', '-2_0_0')
        list_lanes = []
        list_waypoints = []

        key_list = list(self.mapping.keys())
        print(self.path)
        # print(self.path[:-2])
        # print(self.point_dict)
        minimaping = {}
        lastroad = -5
        for elem in self.path:
            road_key = int((key_list[elem]).split('_')[0])
            if lastroad == road_key:

                listindex = minimaping[road_key]
                listindex.append(elem)
                minimaping[road_key] = listindex
            else:
                listindex = []
                listindex.append(elem)
                minimaping[road_key] = listindex
            lastroad = road_key
        print(self.point_dict)
        print(minimaping)
        for road in minimaping:
            trafficSign = None

            if road >= 0:
                for x in self.road_dict:
                    if x['road_id'] == road:
                        #print(x)
                        if len(x['traffic_signs']):
                            print(x['traffic_signs'])
                            trafficSign = x['traffic_signs']

            listkey = minimaping[road]
            if road < 0:
                continue
            pd = self.point_dict[road]
            #print(listkey[0], " ", listkey[1])
            if len(listkey) == 0:
                pass
            elif len(listkey) == 1:
                pass
            else:
                dif = listkey[0] - listkey[1]
                if dif < 0:
                    for i, list_wp in enumerate(pd):
                        # 2. Element und 2. letztes Element
                        if listkey[0] == 48 or listkey[1]==246:
                            continue
                        else:
                            new_points = self.calculate_offset2points(pd[i][0], pd[i][1], 2.0, -1)
                            list_waypoints.append({'x': new_points[0][0], 'y': new_points[0][1], 'trafficSign': trafficSign})
                else:
                    for i, list_wp in enumerate(pd):
                        if listkey[0] == 48 or listkey[1]==246:
                            continue
                        else:
                            new_points = self.calculate_offset2points(pd[-i-1][0], pd[-i-1][1], 2.0, 1)
                            list_waypoints.append({'x': new_points[0][0], 'y': new_points[0][1], 'trafficSign': trafficSign})

                    pass  # von hinten

        interpolated_list = []
        for i in range(1, len(list_waypoints)):
            interpolated_list.append(list_waypoints[i-1])
            interploated_points = self.linear_interpolation(list_waypoints[i-1], list_waypoints[i], 10)
            for point in interploated_points:
                interpolated_list.append({'x': point[0], 'y': point[1], 'trafficSign': None})





        print(list_waypoints)
        print()
        print(interpolated_list)
        #print(list_lanes)
        rospy.loginfo(f'Found Route {list_waypoints}')
        rospy.loginfo(f"List Lanes: {list_lanes}")
        # 4. convert to list of dic
        #print(self.road_dict)
        # 5. output thr route
        return json.dumps(interpolated_list)
