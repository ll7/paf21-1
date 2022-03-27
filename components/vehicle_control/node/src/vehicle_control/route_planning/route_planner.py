"""A global route planner based on XODR map data."""

from math import atan2, dist as euclid_dist, pi, sqrt, exp
from typing import Tuple, List, Dict
import numpy as np

from vehicle_control.core.geometry import add_vector, points_to_vector, rotate_vector, norm_angle, \
                                    scale_vector, sub_vector, unit_vector, vec2dir, vector_len
from vehicle_control.route_planning.xodr_converter import XodrMap, Road, create_key, split_key
from vehicle_control.route_planning.route_interpolation import interpolate_route, linear_interpolation
from vehicle_control.route_planning.route_annotation import AnnRouteWaypoint, RouteAnnotation
from vehicle_control.route_planning.shortest_paths import shortest_path


class AdjMatrixPrep:
    # pylint: disable=too-few-public-methods
    """Representing a helper for preparing the navigation graph."""

    @staticmethod
    def extend_nav_graph(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                         orient_rad: float, xodr_map: XodrMap) -> np.ndarray:
        """Extend the navigation graph by inserting the start / end position
        into the graph such that a shortest path can be computed."""

        # make a working copy of the navigation graph (no overwrites)
        nav_graph = np.copy(xodr_map.nav_graph)

        # find roads to connect to the start / end position for bootstrapping
        start_sect, end_sect = AdjMatrixPrep._find_neighbors(
            start_pos, end_pos, orient_rad, xodr_map)

        AdjMatrixPrep._insert_graph_edges(nav_graph, xodr_map.mapping, start_pos, start_sect, True)
        AdjMatrixPrep._insert_graph_edges(nav_graph, xodr_map.mapping, end_pos, end_sect, False)

        return nav_graph

    @staticmethod
    def _find_neighbors(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                         orient_rad: float, xodr_map: XodrMap):

        left_offset = rotate_vector(unit_vector(orient_rad), pi/2)
        right_offset = rotate_vector(unit_vector(orient_rad), -pi/2)

        # handle spawns on sections next to a drivable lane
        # this helps when spawning e.g. on a parking lot / hard shoulder
        start_neighbors = []
        for shift in np.arange(0.0, 7.0, 0.5):
            shift_left = add_vector(start_pos, scale_vector(left_offset, shift))
            start_neighbors = xodr_map.find_sections(shift_left)
            if start_neighbors:
                break

            shift_right = add_vector(start_pos, scale_vector(right_offset, shift))
            start_neighbors = xodr_map.find_sections(shift_right)
            if start_neighbors:
                break

        end_neighbors = xodr_map.find_sections(end_pos)
        return start_neighbors, end_neighbors


    @staticmethod
    def _insert_graph_edges(graph: np.ndarray, mapping: Dict[str, int], point: Tuple[float, float],
                            neighbor_sections: List[Tuple[int, bool, Road]], is_start: bool):
        num_nodes = graph.shape[0]
        u_turn_penalty = 100.0

        for lane_id, is_right_road_side, road in neighbor_sections:
            link_dir = 1 if is_right_road_side else 0

            road_start, road_end = road.geometries[0].start_point, road.geometries[-1].end_point
            dist_start, dist_end = euclid_dist(point, road_start), euclid_dist(point, road_end)
            if not is_right_road_side:
                dist_start, dist_end = (dist_end, dist_start)

            key_index = mapping[create_key(road.road_id, link_dir, lane_id)]
            key_index2 = mapping[create_key(road.road_id, 1-link_dir, lane_id)]

            if is_start:
                start_node_id = num_nodes - 2
                graph[start_node_id][key_index] = dist_end
                graph[start_node_id][key_index2] = dist_start + u_turn_penalty
            else:
                end_node_id = num_nodes - 1
                graph[key_index][end_node_id] = dist_start
                graph[key_index2][end_node_id] = dist_end


class RoutePlanner:
    """Representing a helper for planning routes base on XODR map data."""

    @staticmethod
    def generate_waypoints(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           orient_rad: float, xodr_map: XodrMap) -> List[AnnRouteWaypoint]:
        """Generate route waypoints for the given start / end positions using the map"""

        path = RoutePlanner.get_shortest_path(start_pos, end_pos, orient_rad, xodr_map)
        # print(f'planned path: {path}')

        if len(path) < 1:
            road_start = xodr_map.find_sections(start_pos)
            road_end = xodr_map.find_sections(start_pos)
            raise ValueError(f'start / end of route not found! \
                Starts with {road_start} and ends with {road_end}.')

        route_metadata = RouteAnnotation.preprocess_route_metadata(start_pos, path, xodr_map)
        route_waypoints = RoutePlanner._preplan_route(
            start_pos, end_pos, path, orient_rad, xodr_map)
        # print("wps:", route_waypoints)

        interpol_route = interpolate_route(route_waypoints, interval_m=2.0)
        # print("wps_interpol:", interpol_route)
        interpol_route = RoutePlanner._filter_steem_waypoints(interpol_route, pi/8)
        # print("wps_filtered:", interpol_route)

        ann_route = RouteAnnotation.annotate_waypoints(interpol_route, route_metadata)
        ann_route = RoutePlanner.advanced_speed(ann_route)
        print(print("route_annotated", ann_route))
        return ann_route

    @staticmethod
    def get_shortest_path(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                          orient_rad: float, xodr_map: XodrMap) -> List[str]:
        """Calculate the shortest path with a given xodr map and return
        a list of keys (road_id, direction, lane_id)."""
        nav_graph = AdjMatrixPrep.extend_nav_graph(start_pos, end_pos, orient_rad, xodr_map)
        start_id, end_id = xodr_map.mapping['-1_0_0'], xodr_map.mapping['-2_0_0']
        path_ids = shortest_path(start_id, end_id, nav_graph)
        print(f'path ids: {path_ids}')
        key_list = list(xodr_map.mapping.keys())
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def _filter_steem_waypoints(route_input: List[Tuple[float, float]],
                                angle_rad: float) -> List[Tuple[float, float]]:
        i = 0
        while 0 <= i < len(route_input) - 3:
            dir_1 = vec2dir(route_input[i+1], route_input[i])
            dir_2 = vec2dir(route_input[i+2], route_input[i+1])

            # only keep point if angle is not too steem
            if abs(norm_angle(dir_2 - dir_1)) <= angle_rad:
                i += 1
                continue

            route_input.pop(i+1)
            i -= 1
        return route_input

    @staticmethod
    def _filter_path(path: List[str]) -> List[str]:
        # TODO: rename into a more expressive name
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
        path = RoutePlanner._filter_path(path)
        sections = [(path[i], path[i+1]) for i in range(len(path)-1)]

        for sec_1, sec_2 in sections:
            road_id1, forward_id1, lane_id1 = split_key(sec_1)
            road_id2, forward_id2, lane_id2 = split_key(sec_2)

            same_road = road_id1 == road_id2
            drive_road_from_start_to_end = forward_id1 != forward_id2
            same_lane = lane_id1 == lane_id2
            is_initial_section = road_id1 == -1
            is_final_section = road_id2 == -2

            if same_road and drive_road_from_start_to_end and same_lane:
                road_1 = xodr_map.roads_by_id[road_id1]
                interm_wps_1 = RoutePlanner._get_intermed_section_waypoints(
                    road_1, lane_id1, forward_id1)
                interpolated_wps = []
                for i in range(len(interm_wps_1[:-1])):
                    interpolated_wps.extend(linear_interpolation(
                        interm_wps_1[i], interm_wps_1[i + 1], interval_m=2.0))
                route_waypoints.extend(interpolated_wps)

            elif is_initial_section:
                route_waypoints.append(start_pos)
                road = xodr_map.roads_by_id[road_id2]
                displaced_points = RoutePlanner._displace_points_start(
                    road, sec_2, start_pos, orient_rad)
                route_waypoints.extend(displaced_points)

            elif is_final_section:
                road = xodr_map.roads_by_id[road_id1]
                displaced_points = RoutePlanner._displace_points_end(road, sec_1, end_pos)
                route_waypoints.extend(displaced_points)
                route_waypoints.append(end_pos)

            else:
                # lane change
                road_1 = xodr_map.roads_by_id[road_id1]
                interm_wps_1 = RoutePlanner._get_intermed_section_waypoints(
                    road_1, lane_id1, not forward_id1)
                road_2 = xodr_map.roads_by_id[road_id2]
                interm_wps_2 = RoutePlanner._get_intermed_section_waypoints(
                    road_2, lane_id2, forward_id2)
                if euclid_dist(interm_wps_1[-1], interm_wps_2[0]) < 0.5:
                    continue
                speed_signs = [sign.legal_speed for sign in road_1.speed_signs]
                displaced_points = RoutePlanner._lane_change(
                    interm_wps_1, interm_wps_2[0], speed_signs, slope_m=8.0)
                route_waypoints[-len(displaced_points):] = displaced_points
        return route_waypoints

    @staticmethod
    def _get_intermed_section_waypoints(road: Road, lane_id: int, reverse: bool):
        polygon_points = road.lane_polygons[lane_id]

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
        waypoints_whole_lane = RoutePlanner._get_intermed_section_waypoints(road, lane_id, reverse)

        waypoints = []
        for wp1, wp2 in zip(waypoints_whole_lane[:-1], waypoints_whole_lane[1:]):
            waypoints.extend(linear_interpolation(wp1, wp2, interval_m=2.0))

        # discard waypoints behind the car
        reachable_index = -1
        threshold = pi/6
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
        waypoints_whole_lane = RoutePlanner._get_intermed_section_waypoints(road, lane_id, forward)

        waypoints = []
        for wp1, wp2 in zip(waypoints_whole_lane[:-1], waypoints_whole_lane[1:]):
            waypoints.extend(linear_interpolation(wp1, wp2, interval_m=2.0))

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
            interpolated_wps.extend(linear_interpolation(points[i], points[i + 1], interval_m=2.0))
        interpolated_wps.append(points[-1])
        speed = max(speed_signs) if speed_signs else 50

        time_to_collision = 0.3
        dist_safe = max(speed * time_to_collision, 5.0)
        street_width_vec = sub_vector(ref_point, points[-1])
        displaced_points = []
        for point in points:
            displaced_point = RoutePlanner._sigmoid_displace(ref_point, point, street_width_vec,
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
                anno_waypoints[index].legal_speed = 90

        return anno_waypoints
