"""A global route planner based on map and hmi data."""

from math import dist as euclid_dist, pi
from typing import Tuple, List, Dict

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key
from global_planner.geometry import add_vector, bounding_box, \
                                    orth_offset_right, orth_offset_left, \
                                    scale_vector, sub_vector, unit_vector, vec2dir
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

        # TODO: figure out something more intelligent than looping over all roads
        #       e.g. only check nearby roads (euclid distance) or perform some hashing, etc.
        for road in xodr_map.roads_by_id.values():

            if not road.geometries:
                print('road without geometries, this should never happen!')
                continue

            # compute polygons for curved roads
            right_polygons, left_polygons = RoadDetection.compute_polygons(road)

            # determine whether the road contains the point
            # and if so, whether the point is on the right or left side
            lane_id = 0
            for id in right_polygons:
                if right_polygons[id].contains(Point(pos)):
                    lane_id = id
                    break
            for id in left_polygons:
                if left_polygons[id].contains(Point(pos)):
                    lane_id = id
                    break

            if not lane_id:
                continue

            is_right_road_side = lane_id < 0
            neighbors.append((lane_id, is_right_road_side, road))

        return neighbors

    @staticmethod
    def compute_polygons(road: Road) -> Tuple[Dict[int, Polygon], Dict[int, Polygon]]:
        """Compute the polygons representing the road bounds."""

        # compute intermediate offset vectors for curved road sections
        offsets_vectors = []
        if len(road.geometries) > 1:
            geo_pairs = zip(road.geometries[:-1], road.geometries[1:])
            offsets_vectors = [RoadDetection._compute_intermediate_offset_vectors(p[0], p[1], road)
                               for p in geo_pairs]

        # compute offset vectors for first / last geometry
        width = road.road_width
        start_0, end_0 = road.geometries[0].start_point, road.geometries[0].end_point
        start_n, end_n = road.geometries[-1].start_point, road.geometries[-1].end_point
        offsets_start = (orth_offset_left(start_0, end_0, 1),
                         orth_offset_right(start_0, end_0, 1))
        offsets_end = (orth_offset_left(start_n, end_n, 1),
                       orth_offset_right(start_n, end_n, 1))

        # put everything together
        offsets_vectors.insert(0, offsets_start)
        offsets_vectors.append(offsets_end)

        # compute right / left bounds of the lane polygons
        middle = [geo.start_point for geo in road.geometries] + [road.geometries[-1].end_point]
        left_bounds, right_bounds = { 0: middle }, { 0: middle }
        for lane_id in road.left_ids:
            bounds = [add_vector(geo.start_point,
                        scale_vector(offsets_vectors[id][0], abs(lane_id) * road.road_width))
                      for id, geo in enumerate(road.geometries)]
            bounds.append(add_vector(road.geometries[-1].end_point, offsets_vectors[-1][0]))
            left_bounds[lane_id] = bounds
        for lane_id in road.right_ids:
            bounds = [add_vector(geo.start_point,
                        scale_vector(offsets_vectors[id][1], abs(lane_id) * road.road_width))
                      for id, geo in enumerate(road.geometries)]
            bounds.append(add_vector(road.geometries[-1].end_point, offsets_vectors[-1][1]))
            right_bounds[lane_id] = bounds

        # put the bounds together to retrieve polygon boxes
        # note: for a road with only one geometry this defaults to a rectangle
        left_polygons = {}
        if road.left_ids:
            left_ids = [0] + road.left_ids
            for id_0, id_1 in zip(left_ids[:-1], left_ids[1:]):
                left_polygons[id_1] = Polygon(left_bounds[id_0] + list(reversed(left_bounds[id_1])))

        right_polygons = {}
        if road.right_ids:
            right_ids = [0] + road.right_ids
            for id_0, id_1 in zip(right_ids[:-1], right_ids[1:]):
                right_polygons[id_1] = Polygon(right_bounds[id_0] + list(reversed(right_bounds[id_1])))

        return right_polygons, left_polygons

    @staticmethod
    def _compute_intermediate_offset_vectors(geo_0: Geometry, geo_1: Geometry, road: Road) \
                                                 -> Tuple[Tuple[float, float], Tuple[float, float]]:
        # road_width = road.road_width
        # num_lanes_right = len(road.right_ids)
        # num_lanes_left = len(road.left_ids)

        # directions of vectors, geo_0 pointing forward, geo_1 pointing backward
        dir_0 = vec2dir(geo_0.start_point, geo_0.end_point)
        dir_1 = vec2dir(geo_1.end_point, geo_0.end_point)

        # halving line between vectors for right / left side
        diff_angle = (dir_1 - dir_0) % (2 * pi)
        offset_left = diff_angle / 2

        vec_left = unit_vector(dir_0 + offset_left)
        vec_right = sub_vector((0, 0), vec_left)
        # vec_left = scale_vector(vec_left, road_width * num_lanes_left)
        # vec_right = scale_vector(vec_right, road_width * num_lanes_right)

        return vec_left, vec_right


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

        if not start_neighbors or not end_neighbors:
            print('start / end section not found:', start_neighbors, end_neighbors)

    @staticmethod
    def _append_start_and_end(matrix: np.ndarray, mapping: Dict[str, int]):
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
            if (road.road_id == 36 and lane_link==1):
                print(xodr_map.mapping)
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

        if len(path) < 1:
            road_start = RoadDetection.find_neighbor_sections(start_pos, xodr_map)
            road_end = RoadDetection.find_neighbor_sections(start_pos, xodr_map)
            raise ValueError(f'start / end of route not found! \
                Starts with {road_start} and ends with {road_end}.')

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
                road_1 = xodr_map.roads_by_id[road_id1]
                interm_wps = GlobalPlanner._get_intermed_section_waypoints(sec_1, road_1)
                route_waypoints += interm_wps

            elif is_initial_section:
                route_waypoints.append(start_pos)
                road = xodr_map.roads_by_id[road_id2]
                displaced_points = GlobalPlanner._displace_points(road, sec_2, is_final=False)
                route_waypoints.append(displaced_points)

            elif is_final_section:
                road = xodr_map.roads_by_id[road_id1]
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
        # TODO: this logic is not suitable for curved roads
        end_geo = road.geometries[-1] if moving_towards_end else road.geometries[0]
        points = bounding_box(end_geo.start_point, end_geo.end_point, road.road_width / 2)
        if is_final:
            return points[2] if moving_towards_end else points[0]
        return points[3] if moving_towards_end else points[1]
