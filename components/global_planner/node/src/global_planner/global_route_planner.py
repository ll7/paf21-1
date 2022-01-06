"""A global route planner based on map and hmi data."""
from math import atan2, pi, sin, cos, ceil, dist as euclid_dist
import json
import dataclasses
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from typing import Tuple, List
from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key
# only for debugging
# from xodr_converter import XODRConverter, XodrMap, Geometry, Road, create_key


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
    def extend_matrix(start_pos: Tuple[float, float], end_pos: Tuple[float, float], xodr_map: XodrMap):
        """Find the nearest road to the start and end point."""
        # append two rows and columns to graph and append start and end to mapping
        xodr_map.matrix = ShortestPath._append_start_end(xodr_map.matrix, xodr_map.mapping)
        num_nodes = xodr_map.matrix.shape[0]

        for road in xodr_map.lane_lets:
            for index, geo in enumerate(road.geometries):
                poly, poly2 = GlobalPlanner.create_polygons(geo, road.road_width)
                # TODO some roads only one lane !!
                is_start_within_outer = poly.contains(Point(start_pos))
                is_end_within_outer = poly.contains(Point(end_pos))

                if not (is_start_within_outer or is_end_within_outer):
                    continue

                is_start_within_inner = poly2.contains(Point(start_pos))
                is_end_within_inner = poly2.contains(Point(end_pos))

                if is_start_within_inner:
                    print(f'Start is in inner: {road.road_id}')
                elif is_start_within_outer:
                    print(f'Start is in outer: {road.road_id}')

                if is_end_within_inner:
                    print(f'End is in inner: {road.road_id}')
                elif is_end_within_outer:
                    print(f'End is in outer: {road.road_id}')

                is_within_inner = is_start_within_inner or is_end_within_inner
                lane_link = 1 if is_within_inner else -1
                ref_id = 0 if is_within_inner else 1

                distance = GlobalPlanner.accumulate_dist(road.geometries, ref_id, index)
                distance2 = GlobalPlanner.accumulate_dist(road.geometries, 1-ref_id, index)
                key_index = GlobalPlanner.find_mapping(road.road_id, ref_id, lane_link,
                                                       xodr_map.mapping)
                key_index2 = GlobalPlanner.find_mapping(road.road_id, 1-ref_id, lane_link,
                                                        xodr_map.mapping)

                # TODO: handle one-way streets
                if is_start_within_inner:
                    xodr_map.matrix[num_nodes - 2][key_index] = 999
                    xodr_map.matrix[num_nodes - 2][key_index2] = distance
                elif is_start_within_outer:
                    xodr_map.matrix[num_nodes - 2][key_index] = distance2
                    xodr_map.matrix[num_nodes - 2][key_index2] = 999

                if is_end_within_inner:
                    xodr_map.matrix[key_index][num_nodes - 1] = 999
                    xodr_map.matrix[key_index2][num_nodes - 1] = distance
                elif is_end_within_outer:
                    xodr_map.matrix[key_index][num_nodes - 1] = distance2
                    xodr_map.matrix[key_index2][num_nodes - 1] = 999


    @staticmethod
    def _edge_relaxation(queue: List[int], dist: np.ndarray) -> int:
        """Calculate the index with the minimum distance."""
        minimum = np.inf
        min_index = -1

        for index in queue:
            if dist[index] < minimum:
                minimum = dist[index]
                min_index = index

        return min_index

    @staticmethod
    def _dijkstra(start_pos: int, matrix: np.ndarray):
        """Implementation of the Dijkstra algorithm."""
        num_nodes = matrix.shape[0]

        dist = np.ones(shape=(num_nodes,)) * np.inf
        parent = np.ones(shape=(num_nodes,)).astype('int32') * -1

        dist[start_pos] = 0.0

        queue = list(range(num_nodes))
        while queue:
            relaxed_edge = ShortestPath._edge_relaxation(queue, dist)
            if relaxed_edge == -1:
                break
            queue.remove(relaxed_edge)

            for index in list(range(num_nodes)):
                if matrix[relaxed_edge][index]:
                    new_dist = dist[relaxed_edge] + matrix[relaxed_edge][index]
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
            if pos == -1:
                return [start_pos]
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
                Polygon([start_point, point_1, point_4, end_point]))

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
        # TODO check function
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

        ShortestPath.extend_matrix(start_pos, end_pos, xodr_map)
        path_ids = ShortestPath.shortest_path(xodr_map.mapping[start_key],
                                              xodr_map.mapping[end_key],
                                              xodr_map.matrix)

        key_list = list(xodr_map.mapping.keys())
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def generate_waypoints(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           xodr_map: XodrMap) -> List[Tuple[float, float]]:

        print(f'generating path from {start_pos} to {end_pos} ...')
        path = GlobalPlanner.get_shortest_path(start_pos, end_pos, xodr_map)
        print(path)

        id2road = dict([(road.road_id, road) for road in xodr_map.lane_lets])
        route_waypoints = []

        path = [(path[i], path[i+1]) for i in range(len(path)-1)]

        for p_1, p_2 in path:
            road_id1 = int(p_1.split('_')[0])
            road_id2 = int(p_2.split('_')[0])

            drive_road_from_start_to_end = road_id1 == road_id2
            is_initial_section = road_id1 == -1
            is_final_section = road_id2 == -2

            if drive_road_from_start_to_end:
                print(f'compute intermediate road section {road_id1}')
                road = id2road[road_id1]
                moving_towards_end = int(p_1.split('_')[1])
                lane_link = int(p_1.split('_')[2])
                road_geometries = list(reversed(road.geometries)) if moving_towards_end else road.geometries

                road_waypoints = []
                for geo in road_geometries:
                    points = GlobalPlanner._calculate_offset2points(geo.start_point,
                                                                    geo.end_point,
                                                                    road.road_width/2)
                    # TODO multiple lanes
                    if lane_link > 0:
                        road_waypoints.append(points[0])
                    else:
                        road_waypoints.append(points[1])

                route_waypoints += road_waypoints

            elif is_initial_section:
                print('compute initial section')
                route_waypoints.append(start_pos)
                road = id2road[road_id2]
                moving_towards_end = int(p_2.split('_')[1])
                road_end_point = road.geometries[-1] if moving_towards_end else road.geometries[0]
                road_end_point = road_end_point.end_point if moving_towards_end else road_end_point.start_point
                route_waypoints.append(road_end_point)

            elif is_final_section:
                print('compute final section')
                road = id2road[road_id1]
                moving_towards_end = int(p_1.split('_')[1])
                road_end_point = road.geometries[-1] if moving_towards_end else road.geometries[0]
                road_end_point = road_end_point.end_point if moving_towards_end else road_end_point.start_point
                route_waypoints.append(road_end_point)
                route_waypoints.append(end_pos)

        # TODO: add traffic signs and interpolation
        #       prob. need to create a class representing a route
        print(f'Raw route waypoints: {route_waypoints}')

        interpolated_route_waypoints = []
        for index in range(len(route_waypoints) - 1):
            waypoints = GlobalPlanner._linear_interpolation(route_waypoints[index],
                                                            route_waypoints[index+1], 2.0)
            interpolated_route_waypoints.extend(waypoints)

        print(f'Interpolated route waypoints: {interpolated_route_waypoints}')

        return interpolated_route_waypoints


if __name__ == '__main__':
    from pathlib import Path

    file_path = Path("../../../xodr/Town01.xodr")
    xodr_map2 = XODRConverter.read_xodr(file_path)
    global_planner = GlobalPlanner(xodr_map2)

    # start_position = (245.85, -198.75)
    # end_position = (144.99, -57.5)

    # start_position = (3.3689340432558032e+2, -0.9789686312079139e+1)
    # end_position = (320.00, -56.5)

    start_position = (396.6376037597656, -208.82986450195312)
    end_position = (144.99, -55.5)

    global_route = GlobalPlanner.generate_waypoints(start_position, end_position, xodr_map2)

    route_as_json = [{'x': pos[0], 'y': pos[1]} for pos in global_route]
    msg = json.dumps(route_as_json)
