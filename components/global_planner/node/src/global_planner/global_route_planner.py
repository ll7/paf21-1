"""A global route planner based on map and hmi data."""
import math
import json
import dataclasses
import numpy as np
import networkx as nx
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from typing import Tuple, List
import rospy
# only for debugging
from global_planner.xodr_converter import XodrMap


class EnhancedJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        return super().default(o)


class GlobalRoutePlanner:
    """A global route planner based on map and hmi data."""
    def __init__(self, xodr_map: XodrMap):
    # def __init__(self, xodr_map):
        """Initialize the global route planner."""
        # base filepath to the maps
        self.filepath = r"../../../maps"
        # graph with
        self.xodr_map = xodr_map
        self.num_nodes = len(self.xodr_map.mapping)
        # initialize all distances with inf.
        self.dist = None
        # array for the parents to store shortest path tree
        self.parent = None
        # path
        self.path = []
        # dict with lane-let ids and matrix pos
        self.matrix = np.copy(self.xodr_map.matrix)
        self.mapping = self.xodr_map.mapping
        self.lane_lets = self.xodr_map.lane_lets
        # TODO DELETE
        self.point_dict = {}

        self.map_name = ''
        self.end_pos = None
        self.start_pos = None
        self.orientation = 0.0
        self.update = False
        # TODO read from data
        self.road_width = 4.0

    def set_data(self):
        """Set the graph, the mapping and the lane-lets."""
        self.matrix = np.copy(self.xodr_map.matrix)
        self.mapping = self.xodr_map.mapping
        self.lane_lets = self.xodr_map.lane_lets

    # def load_map_data(self) -> None or FileNotFoundError:
    #     """Load the data from the file with the map
    #         name and set the mapping and the matrix."""
    #     map_path = os.path.join(self.filepath, self.map_name, ".json")
    #     if not os.path.isfile(map_path):
    #         return FileNotFoundError
    #
    #     with open(map_path, encoding="utf-8") as json_file:
    #         data = json.load(json_file)
    #         self.set_data(data['matrix'], data['mapping'], data['lane_lets'])
    #         return None

    # def update_map_end_pos(self, msg):
    #     """Update the current map name."""
    #     if self.map_name != msg.map:
    #         self.map_name = msg.map
    #         self.load_map_data()
    #
    #     self.end_pos = msg.end_point
    #     self.update = True

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        self.start_pos = vehicle_pos

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.orientation = orientation

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
        self.dist = np.ones(shape=(self.num_nodes,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = np.ones(shape=(self.num_nodes,)).astype('int32') * (-1)

        # distance of source to itself is 0
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
                if self.matrix[num][index_min]:
                    new_dist = self.dist[index_min] + self.matrix[num][index_min]
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
        edges = np.where(self.matrix > 0)
        graph = nx.Graph()
        # add all edges
        graph.add_edges_from(edges)
        # draw the graph
        nx.draw(graph, node_size=500, labels=list(self.mapping.keys()), with_labels=True)

    @staticmethod
    def _calculate_offset(start_point, end_point, road_width) -> List[float]:
        """Calculate the offset according the road_width"""
        # TODO check output
        difference = end_point - start_point
        # avoid division by zero
        if difference[0] == 0.0:
            difference[0] = 1e-8
        alpha = np.arctan2(difference[1], difference[0])
        beta = np.pi + alpha + np.pi / 2

        # div = (end_point[0] - start_point[0])
        # if div == 0:
        #     div = 0.000000000001
        # alpha = np.arctan((end_point[1] - start_point[1]) / div)
        # beta = math.pi + alpha + math.pi / 2
        output = [np.cos(beta) * road_width, np.sin(beta) * road_width]

        return output

    def calculate_offset2points(self, start_point, end_point, road_width, direction) -> List[tuple]:
        """Function to calculate an offset to the start and end point."""
        offset = GlobalRoutePlanner._calculate_offset(start_point, end_point, road_width)

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

    def create_polygon(self, start_point, end_point, one_sided=True) -> Polygon:
        """Function to create a polygon."""
        point_2, point_3 = self.calculate_offset2points(start_point, end_point, self.road_width, -1)

        if one_sided:
            return Polygon([(start_point[0], start_point[1]),
                            point_2,
                            point_3,
                            (end_point[0], end_point[1]),
                            ])
        else:
            point_1, point_4 = self.calculate_offset2points(start_point, end_point, self.road_width, 1)
            return Polygon([point_1,
                            point_2,
                            point_3,
                            point_4,
                            ])

    def _append_start_end(self):
        self.num_nodes = self.matrix.shape[0] + 2
        self.matrix = np.append(self.matrix, np.zeros((2, self.num_nodes - 2)), axis=0)
        self.matrix = np.append(self.matrix, np.zeros((self.num_nodes, 2)), axis=1)

        self.mapping['-1_0_0'] = self.num_nodes - 2
        self.mapping['-2_0_0'] = self.num_nodes - 1

    def find_nearest_road(self) -> (list, list):
        """Find the nearest road to the start and end point."""
        ids_start = []
        ids_end = []

        # caching the start pos (in case the car starts driving)
        start_pos = self.start_pos

        # append two rows and columns to graph and append start and end to mapping
        self._append_start_end()

        # TODO DELETE
        for road in self.lane_lets:
            self.point_dict[road.road_id] = [geometry for geometry in road.geometry]

        for road_id, roads in self.point_dict.items():
            for index, geo in enumerate(roads):

                start_point = geo.start_point
                end_point = geo.end_point

                polygon = self.create_polygon(start_point, end_point, False)
                polygon2 = self.create_polygon(start_point, end_point, True)

                if polygon.contains(Point(start_pos[0], start_pos[1])):
                    lane_link = -1
                    reference_point = 1
                    if polygon2.contains(Point(start_pos[0], start_pos[1])):
                        lane_link = 1
                        reference_point = 0

                    ids_start.append([road_id, index, reference_point])
                    distance = self.accumulate_dist(roads, reference_point, index)
                    i = self.find_mapping(road_id, reference_point, lane_link)
                    if reference_point == 0:
                        self.matrix[i][self.num_nodes - 2] = distance
                    else:
                        self.matrix[self.num_nodes - 2][i] = distance

                if polygon.contains(Point(self.end_pos[0], self.end_pos[1])):
                    lane_link = -1
                    reference_point = 1
                    if polygon2.contains(Point(self.end_pos[0], self.end_pos[1])):
                        lane_link = 1
                        reference_point = 0

                    ids_end.append([road_id, index, reference_point])
                    distance = self.accumulate_dist(roads, reference_point, index)
                    i = self.find_mapping(road_id, reference_point, lane_link)

                    if reference_point == 0:
                        self.matrix[self.num_nodes - 1][i] = distance
                    else:
                        self.matrix[i][self.num_nodes - 1] = distance

        print("id_start", ids_start)
        print()
        print("id_end", ids_end)

        return ids_start, ids_end

    def find_mapping(self, road: int, end: int, link: int):
        key = f"{road}_{end}_{link}"
        if key in self.mapping:
            return self.mapping[key]
        # TODO why wrong mapping?
        key = f"{road}_{end}_{-link}"
        if key in self.mapping:
            return self.mapping[key]
        print("GRP: ", key)
        return AttributeError

    @staticmethod
    def accumulate_dist(roads, reference_point, geo_pos):
        distance = 0.1

        for index2, geo2 in enumerate(roads):
            if reference_point == 0:
                if index2 > geo_pos:
                    continue
                elif index2 == geo_pos:
                    distance += 1
                    # idst start_pos -> start
                else:
                    distance += geo2[3]
            else:
                if index2 < geo_pos:
                    continue
                elif index2 == geo_pos:
                    distance += 1
                    # idst start_pos -> start
                else:
                    distance += geo2[3]
        return distance

    @staticmethod
    def _linear_interpolation(start, end, interval_m):
        list_pol = []
        start = [start['x'], start['y']]
        end = [end['x'], end['y']]

        # dist = math.dist(start, end)
        distance = np.linalg.norm(np.array(start) - np.array(end))
        difference_se = (end[0] - start[0], end[1] - start[1])
        # +1 bei Komma
        steps = math.ceil((distance/interval_m))

        if distance > interval_m:
            add_diff = (difference_se[0] / steps, difference_se[1] / steps)
        else:
            add_diff = difference_se

        diff = (0.0, 0.0)
        for index in range(steps):
            point = (start[0] + diff[0], start[1] + diff[1])
            diff = (diff[0] + add_diff[0], diff[1] + add_diff[1])
            if index > 0 and (list_pol[-1] == point):
                continue
            list_pol.append(point)
        return list_pol

    def compute_route(self) -> str:
        """Compute the route."""
        self.end_pos = np.array([144.99, -57.5])

        ids_start, ids_end = self.find_nearest_road()
        self.dijkstra(self.mapping['-1_0_0'])

        self._append_id2path('-1_0_0', '-2_0_0')
        list_lanes = []
        list_waypoints = []

        key_list = list(self.mapping.keys())

        mini_mapping = {}
        last_road = -5
        for elem in self.path:
            road_key = int((key_list[elem]).split('_')[0])
            if last_road == road_key:

                list_index = mini_mapping[road_key]
                list_index.append(elem)
                mini_mapping[road_key] = list_index
            else:
                list_index = [elem]
                mini_mapping[road_key] = list_index
            last_road = road_key

        for index, road in enumerate(mini_mapping):
            traffic_sign = None

            if road >= 0:
                for x in self.lane_lets:
                    if x.road_id == road:
                        if len(x.traffic_signs):
                            traffic_sign = x.traffic_signs

            list_key = mini_mapping[road]
            if road < 0:
                continue
            pd = self.point_dict[road]

            if len(list_key) == 0:
                pass
            elif len(list_key) == 1:
                pass
            else:
                dif = list_key[0] - list_key[1]
                if dif < 0:
                    for i, list_wp in enumerate(pd):
                        # 2. Element und 2. letztes Element
                        if index == 1 and i == 0:
                            continue
                        else:
                            new_points = self.calculate_offset2points(pd[i].start_point,
                                                                      pd[i].end_point,
                                                                      2.0, -1)
                            list_waypoints.append({'x': new_points[0][0],
                                                   'y': new_points[0][1],
                                                   'trafficSign': traffic_sign})
                else:
                    for i, list_wp in enumerate(pd):
                        if index == 1 and i == 0:
                            continue
                        else:
                            new_points = self.calculate_offset2points(pd[-i-1].start_point,
                                                                      pd[-i-1].end_point,
                                                                      2.0, 1)
                            list_waypoints.append({'x': new_points[0][0],
                                                   'y': new_points[0][1],
                                                   'trafficSign': traffic_sign})

        interpolated_list = []
        for i in range(1, len(list_waypoints)):
            interpolated_list.append(list_waypoints[i-1])
            interpolated_points = self._linear_interpolation(list_waypoints[i - 1], list_waypoints[i], 10)
            for point in interpolated_points:
                interpolated_list.append({'x': point[0], 'y': point[1], 'trafficSign': None})

        rospy.loginfo(f'Found Route {list_waypoints}')
        rospy.loginfo(f"List Lanes: {list_lanes}")
        rospy.loginfo(f"List Lanes: {interpolated_list}")

        # output the interpolated route list as json
        return json.dumps(interpolated_list, cls=EnhancedJSONEncoder)
