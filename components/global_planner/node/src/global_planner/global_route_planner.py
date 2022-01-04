"""A global route planner based on map and hmi data."""
from math import atan2, pi, sin, cos, ceil, dist as euclid_dist
import json
import dataclasses
import numpy as np
import networkx as nx
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from typing import Tuple, List, Dict
import rospy
# only for debugging
from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key
# from xodr_converter import XodrMapAdapter, Geometry, Road, create_key


class EnhancedJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        return super().default(o)


class ShortestPath:
    @staticmethod
    def _append_start_end(matrix, mapping):
        num_nodes = matrix.shape[0] + 2
        matrix = np.append(matrix, np.zeros((2, num_nodes - 2)), axis=0)
        matrix = np.append(matrix, np.zeros((num_nodes, 2)), axis=1)

        mapping['-1_0_0'] = num_nodes - 2
        mapping['-2_0_0'] = num_nodes - 1

        return matrix

    @staticmethod
    def _extend_matrix(start_pos: Tuple[float, float], end_pos: Tuple[float, float], xodr_map: XodrMap):
        """Find the nearest road to the start and end point."""
        # append two rows and columns to graph and append start and end to mapping
        xodr_map.matrix = ShortestPath._append_start_end(xodr_map.matrix, xodr_map.mapping)
        num_nodes = xodr_map.matrix.shape[0]

        for road in xodr_map.lane_lets:
            for index, geo in enumerate(road.geometries):
                poly, poly2 = GlobalPlanner.create_polygons(geo, road.road_width)

                is_within_outer = poly.contains(Point(start_pos)) or poly.contains(Point(end_pos))
                if not is_within_outer:
                    continue

                is_start_within_inner = poly2.contains(Point(start_pos))
                is_end_within_inner = poly2.contains(Point(end_pos))

                is_within_inner = is_start_within_inner or is_end_within_inner
                lane_link = 1 if is_within_inner else -1
                ref_id = 0 if is_within_inner else 1

                distance = GlobalPlanner.accumulate_dist(road.geometries, ref_id, index)
                key_index = GlobalPlanner.find_mapping(road.road_id, ref_id, lane_link,
                                                       xodr_map.mapping)

                if is_start_within_inner:
                    if ref_id == 0:
                        xodr_map.matrix[key_index][num_nodes - 2] = distance
                    else:
                        xodr_map.matrix[num_nodes - 2][key_index] = distance

                if is_end_within_inner:
                    if ref_id == 0:
                        xodr_map.matrix[num_nodes - 1][key_index] = distance
                    else:
                        xodr_map.matrix[key_index][num_nodes - 1] = distance

    @staticmethod
    def _edge_relaxation(queue: List[int], dist: np.ndarray) -> int:
        """Calculate the index with the minimum distance."""
        # initialize min value and min_index as -1
        minimum = np.inf
        min_index = -1

        # from the dist array, pick one which has min value and is till in queue
        for index in queue:
            if dist[index] < minimum:
                # set the new minimum
                minimum = dist[index]
                # set new index
                min_index = index

        if min_index == -1:
            raise AttributeError

        # return the index
        return min_index

    @staticmethod
    def _dijkstra(start_pos: int, matrix: np.ndarray):
        """Implementation of the Dijkstra algorithm."""
        num_nodes = matrix.shape[0]

        # init
        dist = np.ones(shape=(num_nodes,)) * np.inf
        # array for the parents to store shortest path tree
        parent = np.ones(shape=(num_nodes,)).astype('int32') * start_pos

        # distance of source to itself is 0
        dist[start_pos] = 0.0

        # add all nodes_id in queue
        queue = list(range(num_nodes))

        # find the shortest path for all nodes
        while queue:
            # pick the minimum dist node from the set of nodes
            relaxed_edge = ShortestPath._edge_relaxation(queue, dist)
            # remove min element
            queue.remove(relaxed_edge)

            # update dist value and parent
            for index in queue:
                # update dist[i] if it is in queue, there is an edge from index_min to i,
                if matrix[index][relaxed_edge]:
                    new_dist = dist[relaxed_edge] + matrix[index][relaxed_edge]
                    if new_dist < dist[index]:
                        dist[index] = new_dist
                        parent[index] = relaxed_edge

        return parent

    @staticmethod
    def shortest_path(start_pos: int, end_pos: int, matrix: np.ndarray) -> List[int]:
        parent = ShortestPath._dijkstra(start_pos, matrix)

        path = [end_pos]
        pos = end_pos
        while pos != start_pos:
            pos = parent[pos]
            path.insert(0, pos)

        return path


class GlobalPlanner:
    """A global route planner based on map and hmi data."""
    def __init__(self, xodr_map: XodrMap):
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

    def update_vehicle_position(self, vehicle_pos: Tuple[float, float]):
        """Update the vehicle's current position"""
        print(f'new vehicle position: {vehicle_pos}')
        self.start_pos = vehicle_pos

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's current orientation"""
        self.orientation = orientation

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
    def _calculate_offset(start_point: Tuple[float, float], end_point: Tuple[float, float],
                          road_width: float) -> Tuple[float, float]:
        """Calculate the offset according the road_width"""
        difference = (end_point[0] - start_point[0], end_point[1] - start_point[1])
        # TODO avoid division by zero
        # if difference[0] == 0.0:
        #     difference[0] = 1e-8
        alpha = atan2(difference[1], difference[0])
        beta = pi + alpha + pi / 2

        return cos(beta) * road_width, sin(beta) * road_width

    @staticmethod
    def _calculate_offset2points(start_point: Tuple[float, float], end_point: Tuple[float, float],
                                 road_width: float) -> List[Tuple[float, float]]:
        """Function to calculate an offset to the start and end point."""
        offset = GlobalPlanner._calculate_offset(start_point, end_point, road_width)

        return [(start_point[0] + offset[0], start_point[1] - offset[1]),
                (start_point[0] - offset[0], start_point[1] + offset[1]),
                (end_point[0] - offset[0], end_point[1] + offset[1]),
                (end_point[0] + offset[0], end_point[1] - offset[1])]

    @staticmethod
    def create_polygons(geometry: Geometry, road_width: float) -> Tuple[Polygon, Polygon]:
        """Function to create a polygon."""
        start_point, end_point = geometry.start_point, geometry.end_point

        points = GlobalPlanner._calculate_offset2points(start_point, end_point, road_width)
        point_1, point_2, point_3, point_4 = points

        return (Polygon([point_1, point_2, point_3, point_4]),
                Polygon([start_point, point_2, point_3, end_point]))

    @staticmethod
    def find_mapping(road: int, end: int, link: int, mapping: dict) -> int:
        key = create_key(road, end, link)
        if key in mapping:
            return mapping[key]
        # TODO should never happen
        key = create_key(road, end, -link)
        if key in mapping:
            return mapping[key]

        raise AttributeError

    @staticmethod
    def accumulate_dist(geometries: List[Geometry], reference_point, geo_index):
        distance = 0.1

        for index, geo in enumerate(geometries):
            do_increment = (reference_point == 0 and index <= geo_index) or \
                           (reference_point != 0 and index >= geo_index)

            if do_increment:
                distance += (1 if index == geo_index else geo.length)

        return distance

    @staticmethod
    def _linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                              interval_m: float) -> List[Tuple[float, float]]:

        distance = euclid_dist(start, end)
        vector = (end[0] - start[0], end[1] - start[1])

        steps = ceil(distance / interval_m)
        exceeds_interval_cap = distance > interval_m
        step_vector = (vector[0] / steps if exceeds_interval_cap else vector[0],
                       vector[1] / steps if exceeds_interval_cap else vector[1])

        lin_points = [(start[0] + step_vector[0] * i,
                       start[1] + step_vector[1] * i)
                      for i in range(steps)]

        # first = lin_points[0]
        # lin_points = lin_points[1:]
        # pol_points = [first] + [p for p in lin_points if not first == p]
        return lin_points

    @staticmethod
    def get_shortest_path(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                          xodr_map: XodrMap) -> List[str]:
        """Compute the route."""
        start_key = '-1_0_0'
        end_key = '-2_0_0'

        ShortestPath._extend_matrix(start_pos, end_pos, xodr_map)
        path_ids = ShortestPath.shortest_path(xodr_map.mapping[start_key],
                                              xodr_map.mapping[end_key],
                                              xodr_map.matrix)

        key_list = list(sorted(xodr_map.mapping.keys()))
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def generate_waypoints(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           xodr_map: XodrMap) -> List[Tuple[float, float]]:

        print(f'generating path from {start_pos} to {end_pos} ...')
        path = GlobalPlanner.get_shortest_path(start_pos, end_pos, xodr_map)
        print(path)

        id2road = dict([(road.road_id, road) for road in xodr_map.lane_lets])
        route_waypoints = []

        for path_id, lane_key in enumerate(path):
            road_id = int(lane_key.split('_')[0])

            road = id2road[road_id]
            is_road_end = int(lane_key.split('_')[1])

            road_geometries = list(reversed(road.geometries)) if is_road_end else road.geometries
            road_waypoints = [geo.start_point for geo in road_geometries] + [road_geometries[-1].end_point]

            is_start_road = path_id == 0
            if is_start_road:
                print(f'generating path from {start_pos} to {end_pos} ...')
                print(f'waypoints first road: {road_waypoints}')

            route_waypoints += road_waypoints

        # TODO: add traffic signs and interpolation
        #       prob. need to create a class representing a route

        print(route_waypoints)
        return route_waypoints
