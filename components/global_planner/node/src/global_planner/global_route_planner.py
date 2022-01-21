"""A global route planner based on map and hmi data."""

from dataclasses import dataclass
from math import floor, dist as euclid_dist
from typing import Tuple, List

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key, TrafficSignType
from global_planner.geometry import bounding_box, points_to_vector, vector_len


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
        # TODO extend multiple lane per side
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
            # TODO extend multiple lane per side
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
        key = create_key(road, end, -link)
        if key in mapping:
            return mapping[key]
        raise AttributeError


@dataclass
class RoadWithWaypoints:
    """Representing a road and a set of related waypoints"""
    road: Road
    waypoints: List[Tuple[float, float]]


class RouteInterpolation:
    # pylint: disable=too-few-public-methods
    """Representing a helper for interpolating route waypoints"""

    @staticmethod
    def interpolate_route(roads_with_wps: List[RoadWithWaypoints], interval_m=2.0):
        """Interpolate the given route waypoints with a given interval"""

        for road_with_wps in roads_with_wps:
            interpol_route = []
            route = road_with_wps.waypoints

            for index in range(len(route) - 1):
                waypoints = RouteInterpolation._linear_interpolation(
                    route[index], route[index + 1], interval_m=interval_m)
                interpol_route.extend(waypoints)

            clean_route = RouteInterpolation._clean_route_duplicates(interpol_route, min_dist=0.1)
            road_with_wps.waypoints = clean_route

        return roads_with_wps

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
    def _clean_route_duplicates(route: List[Tuple[float, float]],
                                min_dist: float) -> List[Tuple[float, float]]:
        if len(route) == 0:
            return []

        cleaned_route = [route[0]]
        for next_p in route[1:]:
            if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
                cleaned_route.append(next_p)
        return cleaned_route


@dataclass
class AnnotatedRouteWaypoint:
    """Representing a route waypoint with some useful driving meta-data annotations"""
    x_coord: float
    y_coord: float
    next_tl_m: float
    legal_speed: float
    possible_lanes: List[int]
    actual_lane: int
    end_of_lane_m: float
    # other_signs: List[Tuple[str, float]]


class RouteAnnotation:
    # pylint: disable=too-few-public-methods
    """Representing a helper for annotation route waypoints with meta-data."""

    @staticmethod
    def annotate_route(path: List[str], roads_with_wps: List[RoadWithWaypoints]) \
                       -> List[AnnotatedRouteWaypoint]:
        """Annotate the given route waypoints with road meta-data."""
        ann_points = []
        for points_of_road in roads_with_wps:

            road = points_of_road.road
            wps = points_of_road.waypoints

            dist_start = euclid_dist(wps[0], road.road_start)
            dist_end = euclid_dist(wps[-1], road.road_end)
            is_start_to_end = dist_start < dist_end

            actual_lane = RouteAnnotation._get_actual_lane(path, road)
            possible_lanes = RouteAnnotation._get_possible_lanes(road, is_start_to_end)
            speed_signs, traffic_lights = RouteAnnotation.\
                _get_speed_signs_and_traffic_lights(road, is_start_to_end)

            legal_speed = 50
            length_done = 0
            last_point = (0,0)
            for i, waypoint in enumerate(wps):
                if i > 0:
                    length_done += vector_len(points_to_vector(waypoint, last_point))
                # TODO consider more than one sign

                if (speed_signs and length_done >= speed_signs[0].dist_from_road_entrance):
                    legal_speed = speed_signs[0].legal_speed
                    speed_signs.pop(0)

                last_point = waypoint
                next_tl_m = 999
                if (traffic_lights and length_done <= traffic_lights[0].dist_from_road_entrance):
                    next_tl_m = traffic_lights[0].dist_from_road_entrance - length_done
                    traffic_lights.pop(0)

                end_of_lane_m = vector_len(points_to_vector(waypoint, wps[-1]))

                ann_wp = AnnotatedRouteWaypoint(
                    x_coord=waypoint[0], y_coord=waypoint[1], next_tl_m=next_tl_m,
                    legal_speed=legal_speed,
                    possible_lanes=possible_lanes,
                    actual_lane=actual_lane,
                    end_of_lane_m=end_of_lane_m)

                ann_points.append(ann_wp)

        return ann_points

    @staticmethod
    def _get_actual_lane(path: List[str], road: Road):
        actual_lane = 0
        edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        for sec_1, sec_2 in edges:
            road_id1 = int(sec_1.split('_')[0])
            road_id2 = int(sec_2.split('_')[0])
            if road_id1 == road_id2 == road.road_id:
                actual_lane = int(sec_1.split('_')[2])
            elif road_id1 == -1 == road.road_id:
                actual_lane = int(sec_1.split('_')[2])
            elif road_id2 == -2 == road.road_id:
                actual_lane = int(sec_2.split('_')[2])
        return actual_lane

    @staticmethod
    def _get_possible_lanes(road: Road, is_start_to_end: bool):
        possible_lanes = []
        # TODO Check it for other Towns
        if road.line_type == "broken":
            if is_start_to_end:
                possible_lanes = road.left_ids + road.right_ids
            else:
                # TODO: invert sign
                right_lanes = [-lane for lane in road.right_ids]
                left_lanes = [-lane for lane in road.left_ids]
                possible_lanes = right_lanes + left_lanes
        else:
            possible_lanes = road.right_ids if is_start_to_end else road.left_ids
        return possible_lanes

    @staticmethod
    def _get_speed_signs_and_traffic_lights(road: Road, is_start_to_end: bool):
        speed_signs = [s for s in road.traffic_signs
                        if s.sign_type == TrafficSignType.SPEED_LIMIT]

        traffic_lights = road.traffic_lights
        if is_start_to_end:
            speed_signs = list(reversed(speed_signs))
            traffic_lights = list(reversed(traffic_lights))

        return speed_signs, traffic_lights


class GlobalPlanner:
    # pylint: disable=too-few-public-methods
    """A global route planner based on map and hmi data."""

    @staticmethod
    def generate_waypoints(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           orient_rad: float, xodr_map: XodrMap) -> List[AnnotatedRouteWaypoint]:
        """Generate route waypoints for the given start / end positions using the map"""

        # print(f'generating path from {start_pos} to {end_pos} ...')
        path = GlobalPlanner._get_shortest_path(start_pos, end_pos, xodr_map)

        route_waypoints = GlobalPlanner._preplan_route(
            start_pos, end_pos, path, xodr_map.lane_lets)
        # print(f'Raw route waypoints: {route_waypoints}')

        preplan_wps = []
        for road in route_waypoints:
            preplan_wps += road.waypoints
        print(f'Pre-planned route waypoints: {preplan_wps}')

        assert all(map(lambda wps: len(wps.waypoints) > 1, route_waypoints))

        interpol_route = RouteInterpolation.interpolate_route(route_waypoints, interval_m=2.0)

        interpol_wps = []
        for road in interpol_route:
            interpol_wps += road.waypoints
        print(f'Interpolated route waypoints: {interpol_wps}')

        ann_route = RouteAnnotation.annotate_route(path, interpol_route)
        # print(f'Annotated route waypoints: {ann_route}')
        return ann_route

    @staticmethod
    def _preplan_route(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                          path: List[str], lane_lets: List[Road]) -> List[RoadWithWaypoints]:
        id2road = {road.road_id: road for road in lane_lets}
        edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        points_per_road: List[RoadWithWaypoints] = []

        for sec_1, sec_2 in edges:
            road_id1 = int(sec_1.split('_')[0])
            road_id2 = int(sec_2.split('_')[0])

            drive_road_from_start_to_end = road_id1 == road_id2
            is_initial_section = road_id1 == -1
            is_final_section = road_id2 == -2

            if drive_road_from_start_to_end:
                interm_wps = GlobalPlanner._get_intermed_section_waypoints(sec_1, id2road[road_id1])
                assert len(interm_wps) > 1
                points_per_road.append(RoadWithWaypoints(id2road[road_id1], interm_wps))

            elif is_initial_section:
                road = id2road[road_id2]
                displaced_point = GlobalPlanner._displace_point(road, sec_2, is_final=False)
                points_per_road.append(RoadWithWaypoints(road, [start_pos] + [displaced_point]))

            elif is_final_section:
                road = id2road[road_id1]
                displaced_point = GlobalPlanner._displace_point(road, sec_1, is_final=True)
                points_per_road.append(RoadWithWaypoints(road, [displaced_point]+ [end_pos]))

        return points_per_road

    @staticmethod
    def _get_intermed_section_waypoints(sec_1: str, road: Road) -> List[Tuple[float, float]]:
        moving_towards_end = int(sec_1.split('_')[1])
        lane_link = int(sec_1.split('_')[2])

        road_geometries = road.geometries
        if moving_towards_end:
            road_geometries = list(reversed(road.geometries))

        # TODO add support for multiple lanes
        road_waypoints = []
        for geo in road_geometries:
            points = bounding_box(geo.start_point, geo.end_point, road.road_width / 2)
            road_waypoints.append(points[0] if lane_link < 0 else points[2])

        if len(road_geometries) == 1:
            geo = road_geometries[-1]
            points = bounding_box(geo.start_point, geo.end_point, road.road_width / 2)
            road_waypoints.append(points[3] if lane_link < 0 else points[1])

        return road_waypoints

    @staticmethod
    def _get_shortest_path(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           xodr_map: XodrMap) -> List[str]:
        AdjMatrixPrep.extend_matrix(start_pos, end_pos, xodr_map)
        start_id, end_id = xodr_map.mapping['-1_0_0'], xodr_map.mapping['-2_0_0']
        path_ids = ShortestPath.shortest_path(start_id, end_id, xodr_map.matrix)
        key_list = list(xodr_map.mapping.keys())
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def _displace_point(road: Road, sec: str, is_final: bool):
        moving_towards_end = int(sec.split('_')[1])
        road_end_point = road.geometries[-1] if moving_towards_end else road.geometries[0]
        points = bounding_box(road_end_point.start_point, road_end_point.end_point,
                              road.road_width / 2)
        if is_final:
            return points[2] if moving_towards_end else points[0]
        return points[3] if moving_towards_end else points[1]
