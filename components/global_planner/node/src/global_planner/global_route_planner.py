"""A global route planner based on map and hmi data."""
from math import floor, dist as euclid_dist
import json
import dataclasses
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from typing import Tuple, List
from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key
from global_planner.polygon_offsets import bounding_box
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
    def extend_matrix(start_pos: Tuple[float, float],
                      end_pos: Tuple[float, float], xodr_map: XodrMap):
        """Find the nearest road to the start and end point."""
        # append two rows and columns to graph and append start and end to mapping
        xodr_map.matrix = ShortestPath._append_start_end(xodr_map.matrix, xodr_map.mapping)

        start_neighbors = ShortestPath.find_neighbor_sections(start_pos, xodr_map)
        ShortestPath._insert_matrix_edges(xodr_map, start_pos, start_neighbors, is_start=True)
        end_neighbors = ShortestPath.find_neighbor_sections(end_pos, xodr_map)
        ShortestPath._insert_matrix_edges(xodr_map, end_pos, end_neighbors, is_start=False)

    @staticmethod
    def _insert_matrix_edges(xodr_map: XodrMap, point: Tuple[float, float],
                             neighbor_sections: List[Tuple[int, bool, Road]], is_start: bool):
        num_nodes = xodr_map.matrix.shape[0]
        u_turn_penalty = 100.0
        for _, is_right_road_side, road in neighbor_sections:
            # TODO extend multiple lane per side
            lane_link = -1 if is_right_road_side else 1
            ref_id = 1 if is_right_road_side else 0

            road_start, road_end = road.geometries[0].start_point, road.geometries[-1].end_point
            dist_start, dist_end = euclid_dist(point, road_start), euclid_dist(point, road_end)
            if not is_right_road_side:
                dist_start, dist_end = (dist_end, dist_start)

            key_index = GlobalPlanner.find_mapping(road.road_id, ref_id, lane_link,
                                                   xodr_map.mapping)
            key_index2 = GlobalPlanner.find_mapping(road.road_id, 1 - ref_id, lane_link,
                                                    xodr_map.mapping)
            if is_start:
                xodr_map.matrix[num_nodes - 2][key_index] = dist_end
                xodr_map.matrix[num_nodes - 2][key_index2] = dist_start + u_turn_penalty
            else:
                xodr_map.matrix[key_index][num_nodes - 1] = dist_start
                xodr_map.matrix[key_index2][num_nodes - 1] = dist_end

    @staticmethod
    def find_neighbor_sections(pos: Tuple[float, float],
                               xodr_map: XodrMap) -> List[Tuple[int, bool, Road]]:
        # TODO extend multiple lane per side
        neighbors = []
        for road in xodr_map.lane_lets:
            for index, geo in enumerate(road.geometries):
                max_road_width = max(len(road.right_ids), len(road.left_ids)) * road.road_width
                poly, poly2 = GlobalPlanner.create_polygons(geo, max_road_width)
                is_within_outer = poly.contains(Point(pos))

                if not is_within_outer:
                    continue

                is_right_road_side = poly2.contains(Point(pos))
                is_not_on_onesided_lane_right = is_right_road_side and not road.right_ids
                is_not_on_onesided_lane_left = not is_right_road_side and not road.left_ids

                if is_not_on_onesided_lane_right or is_not_on_onesided_lane_left:
                    continue

                neighbors.append((index, is_right_road_side, road))
                print(f'Find roads {road.road_id} for point {pos}')
        return neighbors

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
        self.xodr_map = xodr_map
        self.num_nodes = len(self.xodr_map.mapping)
        self.start_pos = None

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
    def create_polygons(geometry: Geometry, road_width: float) -> Tuple[Polygon, Polygon]:
        """Function to create a polygon."""
        start_point, end_point = geometry.start_point, geometry.end_point

        points = bounding_box(start_point, end_point, road_width)
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
    def _linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                              interval_m: float) -> List[Tuple[float, float]]:

        distance = euclid_dist(start, end)
        vector = (end[0] - start[0], end[1] - start[1])

        steps = max(1, floor(distance / interval_m))
        exceeds_interval_cap = distance > interval_m
        step_vector = (vector[0] / steps if exceeds_interval_cap else vector[0],
                       vector[1] / steps if exceeds_interval_cap else vector[1])

        lin_points = [(start[0] + step_vector[0] * i,
                       start[1] + step_vector[1] * i)
                      for i in range(steps)]

        return lin_points

    @staticmethod
    def _clean_route_duplicates(route: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        cleaned_route = [route[0]]
        for next_p in route[1:]:
            if euclid_dist(cleaned_route[-1], next_p) >= 0.1:
                cleaned_route.append(next_p)
        return cleaned_route

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
                # road_waypoints = [geo.start_point for geo in road_geometries]
                road_waypoints = []
                for geo in road_geometries:
                    points = bounding_box(geo.start_point, geo.end_point, road.road_width/2)
                    # TODO multiple lanes
                    road_waypoints.append(points[0] if lane_link < 0 else points[1])
                route_waypoints += road_waypoints

            elif is_initial_section:
                print('compute initial section')
                route_waypoints.append(start_pos)
                road = id2road[road_id2]
                moving_towards_end = int(p_2.split('_')[1])

                road_end_point = road.geometries[-1] if moving_towards_end else road.geometries[0]
                points = bounding_box(road_end_point.start_point, road_end_point.end_point,
                                      road.road_width / 2)
                route_waypoints.append(points[3] if moving_towards_end else points[1])

            elif is_final_section:
                print('compute final section')
                road = id2road[road_id1]
                moving_towards_end = int(p_1.split('_')[1])
                road_end_point = road.geometries[-1] if moving_towards_end else road.geometries[0]
                print(f'Road width {road.road_width}')
                points = bounding_box(road_end_point.start_point, road_end_point.end_point,
                                      road.road_width / 2)
                print(f'Bounding Box {points}')
                route_waypoints.append(points[2] if moving_towards_end else points[0])
                route_waypoints.append(end_pos)

        # TODO: refactor function, points displacement, check all edges (especially roads with one lane)

        # TODO: add traffic signs and interpolation
        #       prob. need to create a class representing a route
        print(f'Raw route waypoints: {route_waypoints}')

        interpolated_route = []
        for index in range(len(route_waypoints) - 1):
            waypoints = GlobalPlanner._linear_interpolation(route_waypoints[index],
                                                            route_waypoints[index+1], interval_m=2.0)
            interpolated_route.extend(waypoints)

        interpolated_route = GlobalPlanner._clean_route_duplicates(interpolated_route)

        print(f'Interpolated route waypoints: {interpolated_route}')

        return interpolated_route


if __name__ == '__main__':
    from pathlib import Path

    file_path = Path("../../../xodr/Town01.xodr")
    xodr_map2 = XODRConverter.read_xodr(file_path)
    global_planner = GlobalPlanner(xodr_map2)

    start_position = (245.85, -198.75)
    end_position = (144.99, -57.5)

    # start_position = (3.3689340432558032e+2, -0.9789686312079139e+1)
    # end_position = (320.00, -56.5)

    # start_position = (396.6376037597656, -208.82986450195312)
    # end_position = (144.99, -55.5)

    global_route = GlobalPlanner.generate_waypoints(start_position, end_position, xodr_map2)

    route_as_json = [{'x': pos[0], 'y': pos[1]} for pos in global_route]
    msg = json.dumps(route_as_json)
