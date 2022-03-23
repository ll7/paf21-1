"""A global route planner based on map and hmi data."""

from math import atan2, dist as euclid_dist, pi, sqrt, exp
from typing import Tuple, List, Dict

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from global_planner.xodr_converter import XodrMap, Geometry, Road, create_key, split_key
from global_planner.geometry import add_vector, points_to_vector, rotate_vector, orth_offset_right,\
                                    orth_offset_left, norm_angle, scale_vector, sub_vector, \
                                    unit_vector, vec2dir, vector_len
from global_planner.route_interpolation import RouteInterpolation
from global_planner.route_annotation import AnnRouteWaypoint, RouteAnnotation


class ShortestPath:
    # pylint: disable=too-few-public-methods
    """Representing a class for finding shortest paths based on an adjacency matrix"""

    @staticmethod
    def shortest_path(start_pos: int, end_pos: int, matrix: np.ndarray) -> List[int]:
        """Find the shortest path for the given start / end positions and graph"""
        parent = ShortestPath._dijkstra(start_pos, matrix)

        path_short = [end_pos]
        pos = end_pos
        while pos != start_pos:
            pos = parent[pos]
            if pos == -1:
                return [start_pos]
            path_short.insert(0, pos)

        return path_short

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
    def find_sections(pos: Tuple[float, float], xodr_map: XodrMap) -> List[Tuple[int, bool, Road]]:
        """Find the neighboring road sections related to the given position on the map"""
        sections = []

        for road in xodr_map.roads_by_id.values():

            if not road.geometries:
                print('road without geometries, this should never happen!')
                continue

            # compute polygons for curved roads
            polygons = RoadDetection.compute_polygons(road)

            # determine whether the road contains the point
            # and if so, whether the point is on the right or left side
            point = Point(pos)
            poly_hits = map(lambda p_id: (p_id, polygons[p_id].contains(point)), polygons)
            poly_hit = next(filter(lambda x: x[1], poly_hits), None)
            if not poly_hit:
                continue
            lane_id, _ = poly_hit
            is_right_road_side = lane_id < 0
            sections.append((lane_id, is_right_road_side, road))

        return sections

    @staticmethod
    def compute_polygons(road: Road) -> Dict[int, Polygon]:
        """Compute the polygons representing the road bounds."""

        # compute intermediate offset vectors for curved road sections
        offset_vectors = RoadDetection._compute_offset_vectors(road)

        # compute the middle of lane (line of geometries without offset)
        middle = [geo.start_point for geo in road.geometries] + [road.geometries[-1].end_point]

        # shift the middle by the lane offset
        # assumption: all lane offsets of a road are the same
        lane_offset = road.geometries[0].offset
        middle = [add_vector(middle[i], scale_vector(offset_vectors[i][0], lane_offset))
                  for i in range(len(middle))]

        all_polygons: Dict[int, Polygon] = {}

        # compute the inner / outer bounds for each lane and create the lane polygon from it
        for lane_id in road.left_ids + road.right_ids:
            vec_id = 1 if lane_id < 0 else 0
            uniform_lane_offsets = [pair[vec_id] for pair in offset_vectors]


            inner_scale = road.lane_offsets[lane_id] - road.lane_widths[lane_id]
            outer_scale = road.lane_offsets[lane_id]

            inner_offsets = [scale_vector(vec, inner_scale) for vec in uniform_lane_offsets]
            outer_offsets = [scale_vector(vec, outer_scale) for vec in uniform_lane_offsets]

            inner_bound = [add_vector(middle[i], inner_offsets[i]) for i in range(len(middle))]
            outer_bound = [add_vector(middle[i], outer_offsets[i]) for i in range(len(middle))]

            poly_points = inner_bound + list(reversed(outer_bound))
            all_polygons[lane_id] = Polygon(poly_points)

        return all_polygons

    @staticmethod
    def _compute_offset_vectors(road: Road) -> List[Tuple[Tuple[float, float],
                                                          Tuple[float, float]]]:
        offsets_vectors = []
        if len(road.geometries) > 1:
            geo_pairs = zip(road.geometries[:-1], road.geometries[1:])
            offsets_vectors = [RoadDetection._compute_intermediate_offset_vectors(p[0], p[1])
                               for p in geo_pairs]

        # compute offset vectors for first / last geometry
        start_0, end_0 = road.geometries[0].start_point, road.geometries[0].end_point
        start_n, end_n = road.geometries[-1].start_point, road.geometries[-1].end_point
        offsets_start = (orth_offset_left(start_0, end_0, 1),
                         orth_offset_right(start_0, end_0, 1))
        offsets_end = (orth_offset_left(start_n, end_n, 1),
                       orth_offset_right(start_n, end_n, 1))
        offsets_vectors.insert(0, offsets_start)
        offsets_vectors.append(offsets_end)

        return offsets_vectors

    @staticmethod
    def _compute_intermediate_offset_vectors(
            geo_0: Geometry, geo_1: Geometry) -> Tuple[Tuple[float, float], Tuple[float, float]]:

        # directions of vectors, geo_0 pointing forward, geo_1 pointing backward
        dir_0 = vec2dir(geo_0.start_point, geo_0.end_point)
        dir_1 = vec2dir(geo_1.end_point, geo_0.end_point)

        # halving line between vectors for right / left side
        diff_angle = (dir_1 - dir_0) % (2 * pi)
        offset_left = diff_angle / 2

        vec_left = unit_vector(dir_0 + offset_left)
        vec_right = sub_vector((0, 0), vec_left)

        return vec_left, vec_right


class AdjMatrixPrep:
    # pylint: disable=too-few-public-methods
    """Representing a helper for preparing """

    @staticmethod
    def extend_matrix(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                      orient_rad: float, xodr_map: XodrMap):
        """Find the nearest road to the start and end point."""
        # append two rows and columns to graph and append start and end to mapping
        xodr_map.matrix = AdjMatrixPrep._append_start_and_end(xodr_map.matrix, xodr_map.mapping)

        left_offset = rotate_vector(unit_vector(orient_rad), pi/2)
        right_offset = rotate_vector(unit_vector(orient_rad), -pi/2)

        # handle spawns on sections next to a drivable lane
        start_neighbors = []
        for shift in np.arange(0.0, 7.0, 0.5):
            shift_left = add_vector(start_pos, scale_vector(left_offset, shift))
            start_neighbors = RoadDetection.find_sections(shift_left, xodr_map)
            if start_neighbors:
                break

            shift_right = add_vector(start_pos, scale_vector(right_offset, shift))
            start_neighbors = RoadDetection.find_sections(shift_right, xodr_map)
            if start_neighbors:
                break

        AdjMatrixPrep._insert_matrix_edges(xodr_map, start_pos, start_neighbors, is_start=True)
        end_neighbors = RoadDetection.find_sections(end_pos, xodr_map)
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

        print('neighbors:', neighbor_sections)

        for lane_id, is_right_road_side, road in neighbor_sections:
            link_dir = 1 if is_right_road_side else 0

            road_start, road_end = road.geometries[0].start_point, road.geometries[-1].end_point
            dist_start, dist_end = euclid_dist(point, road_start), euclid_dist(point, road_end)
            if not is_right_road_side:
                dist_start, dist_end = (dist_end, dist_start)

            key_index = AdjMatrixPrep._find_mapping(
                road.road_id, link_dir, lane_id, xodr_map.mapping)
            key_index2 = AdjMatrixPrep._find_mapping(
                road.road_id, 1 - link_dir, lane_id, xodr_map.mapping)

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
                           orient_rad: float, xodr_map: XodrMap) -> List[AnnRouteWaypoint]:
        """Generate route waypoints for the given start / end positions using the map"""
        print(f'Start-Pos: {start_pos}  End-Pos: {end_pos}')
        path = GlobalPlanner.get_shortest_path(start_pos, end_pos, orient_rad, xodr_map)
        print(f'planned path: {path}')

        if len(path) < 1:
            road_start = RoadDetection.find_sections(start_pos, xodr_map)
            road_end = RoadDetection.find_sections(start_pos, xodr_map)
            raise ValueError(f'start / end of route not found! \
                Starts with {road_start} and ends with {road_end}.')

        route_metadata = RouteAnnotation.preprocess_route_metadata(start_pos, path, xodr_map)
        route_waypoints = GlobalPlanner._preplan_route(
            start_pos, end_pos, path, orient_rad, xodr_map)
        print("wps:", route_waypoints)

        interpol_route = RouteInterpolation.interpolate_route(route_waypoints, interval_m=2.0)
        print("wps_interpol:", interpol_route)
        interpol_route = GlobalPlanner._filter_waypoints(interpol_route)
        print("wps_filtered:", interpol_route)

        ann_route = RouteAnnotation.annotate_waypoints(interpol_route, route_metadata)
        ann_route = GlobalPlanner.advanced_speed(ann_route)
        print(print("route_annotated", ann_route))
        return ann_route

    @staticmethod
    def get_shortest_path(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                          orient_rad: float, xodr_map: XodrMap) -> List[str]:
        """Calculate the shortest path with a given xodr map and return
        a list of keys (road_id, direction, lane_id)."""
        AdjMatrixPrep.extend_matrix(start_pos, end_pos, orient_rad, xodr_map)
        start_id, end_id = xodr_map.mapping['-1_0_0'], xodr_map.mapping['-2_0_0']
        path_ids = ShortestPath.shortest_path(start_id, end_id, xodr_map.matrix)
        key_list = list(xodr_map.mapping.keys())
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def _filter_waypoints(route_input):
        # discard waypoints that are too steem
        i = 0
        while 0 <= i < len(route_input) - 3:
            dir_1 = vec2dir(route_input[i+1], route_input[i])
            dir_2 = vec2dir(route_input[i+2], route_input[i+1])

            # keep point if angle is not too steem
            print('Filter Wps', abs(norm_angle(dir_2 - dir_1)))
            if abs(norm_angle(dir_2 - dir_1)) <= pi/8:
                i += 1
                continue

            route_input.pop(i+1)
            i -= 1
        return route_input

    @staticmethod
    def _filter_path(path: List[str]) -> List[str]:
        filtered_path = []
        last_road, last_pos = (-1, -1)
        second_last_road, second_last_pos = (-1, -1)
        for sec in path:
            road, pos, _ = split_key(sec)
            if road != last_road or pos != last_pos or \
                    road != second_last_road or pos != second_last_pos:
                filtered_path.append(sec)
            else:
                filtered_path.pop()
                filtered_path.append(sec)

            second_last_road, second_last_pos = last_road, last_pos
            last_road, last_pos = road, pos
        return filtered_path

    @staticmethod
    def _preplan_route(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                       path: List[str], orient_rad: float,
                       xodr_map: XodrMap) -> List[Tuple[float, float]]:
        route_waypoints = []
        path = GlobalPlanner._filter_path(path)
        sections = [(path[i], path[i+1]) for i in range(len(path)-1)]

        for sec_1, sec_2 in sections:
            road_id1, forward_id1, lane_id1 = split_key(sec_1)
            road_id2, forward_id2, _ = split_key(sec_2)

            same_road = road_id1 == road_id2
            drive_road_from_start_to_end = forward_id1 != forward_id2
            same_lane = lane_id1 == lane_id2
            is_initial_section = road_id1 == -1
            is_final_section = road_id2 == -2

            if same_road and drive_road_from_start_to_end and same_lane:
                road_1 = xodr_map.roads_by_id[road_id1]
                interm_wps_1 = GlobalPlanner._get_intermed_section_waypoints(
                    road_1, lane_id1, forward_id1)
                interpolated_wps = []
                for i in range(len(interm_wps_1[:-1])):
                    interpolated_wps.extend(RouteInterpolation.linear_interpolation(
                        interm_wps_1[i], interm_wps_1[i + 1], interval_m=2.0))
                route_waypoints.extend(interpolated_wps)

            elif is_initial_section:
                route_waypoints.append(start_pos)
                road = xodr_map.roads_by_id[road_id2]
                displaced_points = GlobalPlanner._displace_points_start(
                    road, sec_2, start_pos, orient_rad)
                route_waypoints.extend(displaced_points)

            elif is_final_section:
                road = xodr_map.roads_by_id[road_id1]
                displaced_points = GlobalPlanner._displace_points_end(road, sec_1, end_pos)
                route_waypoints.extend(displaced_points)
                route_waypoints.append(end_pos)

            else:
                # lane change
                road_1 = xodr_map.roads_by_id[road_id1]
                interm_wps_1 = GlobalPlanner._get_intermed_section_waypoints(
                    road_1, lane_id1, not forward_id1)
                road_2 = xodr_map.roads_by_id[road_id2]
                interm_wps_2 = GlobalPlanner._get_intermed_section_waypoints(
                    road_2, lane_id2, forward_id2)
                if euclid_dist(interm_wps_1[-1], interm_wps_2[0]) < 0.5:
                    continue
                speed_signs = [sign.legal_speed for sign in road_1.speed_signs]
                displaced_points = GlobalPlanner._lane_change(interm_wps_1, interm_wps_2[0],
                                                              speed_signs, slope_m=8.0)
                route_waypoints[-len(displaced_points):] = displaced_points
        return route_waypoints

    @staticmethod
    def _get_intermed_section_waypoints(road: Road, lane_id: int, reverse: bool):
        polygon = RoadDetection.compute_polygons(road)[lane_id]
        poly_x, poly_y = polygon.exterior.xy
        polygon_points: List[Tuple[float, float]] = list(zip(poly_x, poly_y))
        polygon_points = polygon_points[:-1]

        bound_len = len(polygon_points) // 2
        geo_pairs = list(zip(polygon_points[:bound_len], reversed(polygon_points[bound_len:])))
        road_waypoints = [((p[0][0] + p[1][0]) / 2, (p[0][1] + p[1][1]) / 2) for p in geo_pairs]

        return list(reversed(road_waypoints)) if reverse else road_waypoints

    @staticmethod
    def _displace_points_start(road: Road, sec: str, pos: Tuple[float, float],
                               orient_rad: float) -> List[Tuple[float, float]]:
        """Determine the waypoints from the spawn position onto the planned route"""
        _, forward, lane_id = split_key(sec)
        reverse = not forward
        waypoints_whole_lane = GlobalPlanner._get_intermed_section_waypoints(road, lane_id, reverse)

        waypoints = []
        for wp1, wp2 in zip(waypoints_whole_lane[:-1], waypoints_whole_lane[1:]):
            waypoints.extend(RouteInterpolation.linear_interpolation(wp1, wp2, interval_m=2.0))

        # discard waypoints behind the car
        reachable_index = -1
        threshold = pi/6    # equals 30 degrees offset to the side
        for index, waypoint in enumerate(waypoints):
            vec = points_to_vector(pos, waypoint)
            vec_dir = atan2(vec[1], vec[0])
            norm_diff = norm_angle(vec_dir - orient_rad)
            if abs(norm_diff) <= threshold:
                reachable_index = index
                break

        print("reachable_index", reachable_index)
        return waypoints[reachable_index:] if reachable_index > -1 else []

    @staticmethod
    def _displace_points_end(road: Road, sec: str,
                             pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Determine the waypoints from the spawn position onto the planned route"""
        _, forward, lane_id = split_key(sec)
        waypoints_whole_lane = GlobalPlanner._get_intermed_section_waypoints(road, lane_id, forward)

        waypoints = []
        for wp1, wp2 in zip(waypoints_whole_lane[:-1], waypoints_whole_lane[1:]):
            waypoints.extend(RouteInterpolation.linear_interpolation(wp1, wp2, interval_m=2.0))

        # discard waypoints before the car
        reachable_index = -1
        threshold = pi/2
        for index, waypoint in enumerate(waypoints):
            vec_dir = vec2dir(pos, waypoint)
            norm_diff = norm_angle(vec_dir)
            if abs(norm_diff) <= threshold:
                reachable_index = index - 1
                break

        print("reachable_index", reachable_index)
        return waypoints[:reachable_index] if reachable_index > -1 else []

    @staticmethod
    def _lane_change(points: List[Tuple[float, float]], ref_point: Tuple[float, float],
                     speed_signs: List[float], slope_m=3.0) -> List[Tuple[float, float]]:
        interpolated_wps = []
        for i in range(len(points[:-1])):
            interpolated_wps.extend(RouteInterpolation.linear_interpolation(
                points[i], points[i + 1], interval_m=2.0))
        interpolated_wps.append(points[-1])
        speed = max(speed_signs) if speed_signs else 50

        time_to_collision = 0.3
        dist_safe = max(speed * time_to_collision, 5.0)
        street_width_vec = sub_vector(ref_point, points[-1])
        displaced_points = []
        for point in points:
            displaced_point = GlobalPlanner._sigmoid_displace(ref_point, point, street_width_vec,
                                                              slope_m, dist_safe)
            displaced_points.append(displaced_point)

        return displaced_points

    @staticmethod
    def _sigmoid_displace(ref_point: Tuple[float, float], point: Tuple[float, float],
                          street_width_vec: Tuple[float, float], slope_m: float,
                          dist_safe: float) -> Tuple[float, float]:
        distance_to_ref = euclid_dist(point, ref_point)
        street_width = vector_len(street_width_vec)
        try:
            rel_dist = -sqrt(distance_to_ref**2 - street_width ** 2)
        except ValueError:
            print('Value Error')
            rel_dist = street_width
        x_1 = (1 / slope_m) * (rel_dist + dist_safe)
        deviation = (street_width / (1 + exp(-x_1)))
        return add_vector(point, scale_vector(street_width_vec, deviation))

    @staticmethod
    def advanced_speed(anno_waypoints: List[AnnRouteWaypoint]):
        """Set higher speed if more then 3 lanes and no limit is detected"""
        for index, annotated_waypoint in enumerate(anno_waypoints):
            if len(annotated_waypoint.possible_lanes) > 3 and annotated_waypoint.legal_speed == 50:
                # TODO find out how fast we are allowed to drive on a highway
                anno_waypoints[index].legal_speed = 70

        return anno_waypoints
