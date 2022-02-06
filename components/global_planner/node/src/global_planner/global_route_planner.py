"""A global route planner based on map and hmi data."""

from dataclasses import dataclass
from math import floor, dist as euclid_dist
from typing import Tuple, List

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key
from global_planner.geometry import bounding_box
from global_planner.route_interpolation import RouteInterpolation
from global_planner.route_annotation import AnnRouteWaypoint, RouteAnnotation


class ShortestPath:
    # pylint: disable=too-few-public-methods
    """Representing a class for finding shortest paths based on an adjacency matrix"""

    @staticmethod
    def shortest_path(start_pos: int, end_pos: int, matrix: np.ndarray) -> List[int]:
        """Find the shortest path for the given start / end positions and graph"""
        parent = ShortestPath._dijkstra(start_pos, matrix)

        path = [end_pos]
        pos = end_pos
        while pos != start_pos:
            pos = parent[pos]
            if pos == -1:
                return [start_pos]
            path.insert(0, pos)

        return path

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
    def _edge_relaxation(queue: List[int], dist: np.ndarray) -> int:
        """Calculate the index with the minimum distance."""
        minimum = np.inf
        min_index = -1

        for index in queue:
            if dist[index] < minimum:
                minimum = dist[index]
                min_index = index

        return min_index


class RoadDetection:
    # pylint: disable=too-few-public-methods
    """Representing a helper for detecting roads belonging to positions on a map"""

    @staticmethod
    def find_neighbor_sections(pos: Tuple[float, float],
                               xodr_map: XodrMap) -> List[Tuple[int, bool, Road]]:
        """Find the neighboring road sections related to the given position on the map"""
        neighbors = []
        for road in xodr_map.lane_lets:
            for index, geo in enumerate(road.geometries):
                max_road_width = max(len(road.right_ids), len(road.left_ids)) * road.road_width
                poly, poly2 = RoadDetection._create_polygons(geo, max_road_width)
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
    def _create_polygons(geometry: Geometry, road_width: float) -> Tuple[Polygon, Polygon]:
        """Function to create a polygon."""
        start_point, end_point = geometry.start_point, geometry.end_point

        points = bounding_box(start_point, end_point, road_width)
        point_1, point_2, point_3, point_4 = points

        return (Polygon([point_1, point_2, point_3, point_4]),
                Polygon([start_point, point_1, point_4, end_point]))


class AdjMatrixPrep:
    # pylint: disable=too-few-public-methods
    """Representing a helper for preparing """

    @staticmethod
    def extend_matrix(start_pos: Tuple[float, float],
                      end_pos: Tuple[float, float], xodr_map: XodrMap):
        """Find the nearest road to the start and end point."""
        # append two rows and columns to graph and append start and end to mapping
        xodr_map.matrix = AdjMatrixPrep._append_start_and_end(xodr_map.matrix, xodr_map.mapping)

        start_neighbors = RoadDetection.find_neighbor_sections(start_pos, xodr_map)
        AdjMatrixPrep._insert_matrix_edges(xodr_map, start_pos, start_neighbors, is_start=True)
        end_neighbors = RoadDetection.find_neighbor_sections(end_pos, xodr_map)
        AdjMatrixPrep._insert_matrix_edges(xodr_map, end_pos, end_neighbors, is_start=False)

    @staticmethod
    def _append_start_and_end(matrix, mapping):
        num_nodes = matrix.shape[0] + 2
        matrix = np.append(matrix, np.zeros((2, num_nodes - 2)), axis=0)
        matrix = np.append(matrix, np.zeros((num_nodes, 2)), axis=1)

        mapping['-1_0_0'] = num_nodes - 2
        mapping['-2_0_0'] = num_nodes - 1

        return matrix

    @staticmethod
    def _insert_matrix_edges(xodr_map: XodrMap, point: Tuple[float, float],
                             neighbor_sections: List[Tuple[int, bool, Road]], is_start: bool):
        num_nodes = xodr_map.matrix.shape[0]
        u_turn_penalty = 100.0
        for _, is_right_road_side, road in neighbor_sections:
            lane_link = -1 if is_right_road_side else 1
            ref_id = 1 if is_right_road_side else 0

            road_start, road_end = road.geometries[0].start_point, road.geometries[-1].end_point
            dist_start, dist_end = euclid_dist(point, road_start), euclid_dist(point, road_end)
            if not is_right_road_side:
                dist_start, dist_end = (dist_end, dist_start)

            key_index = AdjMatrixPrep._find_mapping(
                road.road_id, ref_id, lane_link, xodr_map.mapping)
            key_index2 = AdjMatrixPrep._find_mapping(
                road.road_id, 1 - ref_id, lane_link, xodr_map.mapping)

            if is_start:
                start_node_id = num_nodes - 2
                xodr_map.matrix[start_node_id][key_index] = dist_end
                xodr_map.matrix[start_node_id][key_index2] = dist_start + u_turn_penalty
            else:
                end_node_id = num_nodes - 1
                xodr_map.matrix[key_index][end_node_id] = dist_start
                xodr_map.matrix[key_index2][end_node_id] = dist_end

    @staticmethod
    def _find_mapping(road: int, end: int, link: int, mapping: dict) -> int:
        key = create_key(road, end, link)
        if key in mapping:
            return mapping[key]
        raise AttributeError


class GlobalPlanner:
    """A global route planner based on map and hmi data."""

    @staticmethod
    def generate_waypoints(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           orientation_rad: float, xodr_map: XodrMap) -> List[AnnRouteWaypoint]:
        """Generate route waypoints for the given start / end positions using the map"""
        path = GlobalPlanner.get_shortest_path(start_pos, end_pos, xodr_map)
        route_waypoints = GlobalPlanner._preplan_route(start_pos, end_pos, path, xodr_map)
        print("wps:", route_waypoints)
        interpol_route = RouteInterpolation.interpolate_route(route_waypoints, interval_m=2.0)
        route_metadata = RouteAnnotation.preprocess_route_metadata(start_pos, path, xodr_map)
        ann_route = RouteAnnotation.annotate_waypoints(interpol_route, route_metadata)
        return ann_route

    @staticmethod
    def _preplan_route(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                       path: List[str], xodr_map: XodrMap) -> List[Tuple[float, float]]:
        route_waypoints = []
        sections = [(path[i], path[i+1]) for i in range(len(path)-1)]

        for sec_1, sec_2 in sections:
            road_id1 = int(sec_1.split('_')[0])
            road_id2 = int(sec_2.split('_')[0])

            drive_road_from_start_to_end = road_id1 == road_id2
            is_initial_section = road_id1 == -1
            is_final_section = road_id2 == -2

            if drive_road_from_start_to_end:
                road_1 = xodr_map.road_by_id(road_id1)
                interm_wps = GlobalPlanner._get_intermed_section_waypoints(sec_1, road_1)
                route_waypoints += interm_wps

            elif is_initial_section:
                route_waypoints.append(start_pos)
                road = xodr_map.road_by_id(road_id2)
                displaced_points = GlobalPlanner._displace_points(road, sec_2, is_final=False)
                route_waypoints.append(displaced_points)

            elif is_final_section:
                road = xodr_map.road_by_id(road_id1)
                displaced_points = GlobalPlanner._displace_points(road, sec_1, is_final=True)
                route_waypoints.append(displaced_points)
                route_waypoints.append(end_pos)

        return route_waypoints

    @staticmethod
    def _get_intermed_section_waypoints(sec_1: str, road: Road):
        moving_towards_end = int(sec_1.split('_')[1])
        lane_link = int(sec_1.split('_')[2])

        road_geometries = list(reversed(road.geometries)) \
            if moving_towards_end else road.geometries

        road_waypoints = []
        for geo in road_geometries:
            points = bounding_box(geo.start_point, geo.end_point, road.road_width/2)
            road_waypoints.append(points[0] if lane_link < 0 else points[1])
        return road_waypoints

    @staticmethod
    def get_shortest_path(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           xodr_map: XodrMap) -> List[str]:
        AdjMatrixPrep.extend_matrix(start_pos, end_pos, xodr_map)
        start_id, end_id = xodr_map.mapping['-1_0_0'], xodr_map.mapping['-2_0_0']
        path_ids = ShortestPath.shortest_path(start_id, end_id, xodr_map.matrix)
        key_list = list(xodr_map.mapping.keys())
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def _displace_points(road: Road, sec: str, is_final: bool):
        moving_towards_end = int(sec.split('_')[1])
        end_geo = road.geometries[-1] if moving_towards_end else road.geometries[0]
        points = bounding_box(end_geo.start_point, end_geo.end_point, road.road_width / 2)
        if is_final:
            return points[2] if moving_towards_end else points[0]
        return points[3] if moving_towards_end else points[1]
