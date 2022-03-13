"""A global route planner based on map and hmi data."""

from dis import dis
from math import atan2, dist as euclid_dist, pi, radians
from typing import Tuple, List, Dict

import numpy as np
# from components.global_planner.node.src.global_planner.main import main
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
            neighbors.append((lane_id, is_right_road_side, road))
            # TODO: refactor the result as a tuple of (road_id, lane_id)

        return neighbors

    @staticmethod
    def compute_polygons(road: Road) -> Dict[int, Polygon]:
        """Compute the polygons representing the road bounds."""

        # compute intermediate offset vectors for curved road sections
        offsets_vectors = []
        if len(road.geometries) > 1:
            geo_pairs = zip(road.geometries[:-1], road.geometries[1:])
            offsets_vectors = [RoadDetection._compute_intermediate_offset_vectors(p[0], p[1])
                               for p in geo_pairs]

        # compute offset vectors for first / last geometry
        # width = road.road_width
        start_0, end_0 = road.geometries[0].start_point, road.geometries[0].end_point
        start_n, end_n = road.geometries[-1].start_point, road.geometries[-1].end_point
        offsets_start = (orth_offset_left(start_0, end_0, 1),
                         orth_offset_right(start_0, end_0, 1))
        offsets_end = (orth_offset_left(start_n, end_n, 1),
                       orth_offset_right(start_n, end_n, 1))
        offsets_vectors.insert(0, offsets_start)
        offsets_vectors.append(offsets_end)

        # compute right / left bounds of the lane polygons
        middle = [geo.start_point for geo in road.geometries] + [road.geometries[-1].end_point]
        left_bounds, right_bounds = { 0: middle }, { 0: middle }

        for lane_id in road.left_ids:
            scale = abs(lane_id) * road.lane_widths[lane_id] + road.geometries[0].offset
            scaled_offsets = [scale_vector(offsets_vectors[i][0], scale)
                              for i in range(len(middle))]
            bounds = [add_vector(m, scaled_offsets[i]) for i, m in enumerate(middle)]
            left_bounds[lane_id] = bounds

        for lane_id in road.right_ids:
            scale = abs(lane_id) * road.lane_widths[lane_id] + road.geometries[0].offset
            scaled_offsets = [scale_vector(offsets_vectors[i][1], scale)
                              for i in range(len(middle))]
            bounds = [add_vector(m, scaled_offsets[i]) for i, m in enumerate(middle)]
            right_bounds[lane_id] = bounds

        # put the bounds together to retrieve polygon boxes
        # note: for a road with only one geometry this defaults to a rectangle
        left_polygons = {}
        if road.left_ids:
            left_ids = [0] + list(sorted(road.left_ids))
            for id_0, id_1 in zip(left_ids[:-1], left_ids[1:]):
                left_polygons[id_1] = Polygon(left_bounds[id_0] \
                    + list(reversed(left_bounds[id_1])))

        right_polygons = {}
        if road.right_ids:
            right_ids = [0] + list(reversed(sorted(road.right_ids)))
            for id_0, id_1 in zip(right_ids[:-1], right_ids[1:]):
                right_polygons[id_1] = Polygon(right_bounds[id_0] \
                    + list(reversed(right_bounds[id_1])))

        all_polygons = {**right_polygons, **left_polygons}
        return all_polygons

    @staticmethod
    def _compute_intermediate_offset_vectors(geo_0: Geometry, geo_1: Geometry) \
                                                 -> Tuple[Tuple[float, float], Tuple[float, float]]:

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
                           xodr_map: XodrMap) -> List[AnnRouteWaypoint]:
        """Generate route waypoints for the given start / end positions using the map"""

        print ("Startpos:", start_pos, "endpos:", end_pos)
        path = GlobalPlanner.get_shortest_path(start_pos, end_pos, xodr_map)
        print(f'planned path:', path)

        if len(path) < 1:
            road_start = RoadDetection.find_neighbor_sections(start_pos, xodr_map)
            road_end = RoadDetection.find_neighbor_sections(start_pos, xodr_map)
            raise ValueError(f'start / end of route not found! \
                Starts with {road_start} and ends with {road_end}.')

        route_metadata = RouteAnnotation.preprocess_route_metadata(start_pos, path, xodr_map)
        route_waypoints = GlobalPlanner._preplan_route(start_pos, end_pos, path, xodr_map)
        print("wps:", route_waypoints)

        interpol_route = RouteInterpolation.interpolate_route(route_waypoints, interval_m=2.0)
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
                # road = xodr_map.roads_by_id[road_id2]
                # displaced_points = GlobalPlanner._displace_points(
                #     road, sec_2, start_pos, is_final=False)
                # route_waypoints.extend(displaced_points)

            elif is_final_section:
                # road = xodr_map.roads_by_id[road_id1]
                # displaced_points = GlobalPlanner._displace_points(
                #     road, sec_1, end_pos, is_final=True)
                # route_waypoints.extend(displaced_points)
                route_waypoints.append(end_pos)

        return route_waypoints

    @staticmethod
    def _get_intermed_section_waypoints(sec_1: str, road: Road):
        reverse = int(sec_1.split('_')[1])
        lane_id = int(sec_1.split('_')[2])

        road_waypoints = []
        polygon = RoadDetection.compute_polygons(road)[lane_id]
        poly_x, poly_y = polygon.exterior.xy
        polygon_points: List[Tuple[float, float]] = list(zip(poly_x, poly_y))
        polygon_points = polygon_points[:-1] # TODO: why index -1???
        # print(f'road: {road.road_id}, lane: {lane_id}, points: {polygon_points}')

        bound_len = len(polygon_points) // 2
        geo_pairs = list(zip(polygon_points[:bound_len], reversed(polygon_points[bound_len:])))
        road_waypoints = [((p[0][0] + p[1][0]) / 2, (p[0][1] + p[1][1]) / 2) for p in geo_pairs]

        return list(reversed(road_waypoints)) if reverse else road_waypoints

    @staticmethod
    def get_shortest_path(start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           xodr_map: XodrMap) -> List[str]:
        """Calculate the shortest path with a given xodr map and return
        a list of keys (road_id, direction, lane_id)."""
        AdjMatrixPrep.extend_matrix(start_pos, end_pos, xodr_map)
        start_id, end_id = xodr_map.mapping['-1_0_0'], xodr_map.mapping['-2_0_0']
        path_ids = ShortestPath.shortest_path(start_id, end_id, xodr_map.matrix)
        key_list = list(xodr_map.mapping.keys())
        return [key_list[p_id] for p_id in path_ids]

    @staticmethod
    def _displace_points(road: Road, sec: str, pos: Tuple[float, float],
                         is_final: bool) -> List[Tuple[float, float]]:

        moving_towards_end = int(sec.split('_')[1])
        lane_id = int(sec.split('_')[2])

        # end_geo = road.geometries[-1] if moving_towards_end else road.geometries[0]
        # end_pos = end_geo.end_point if moving_towards_end else end_geo.start_point
        # polygon = Polygon(bounding_box(pos, end_pos, 50))

        polygon = RoadDetection.compute_polygons(road)[lane_id]
        poly_x, poly_y = polygon.exterior.xy
        polygon_points: List[Tuple[float, float]] = list(zip(poly_x, poly_y))
        polygon_points = polygon_points[:-1]

        bound_len = len(polygon_points) // 2
        geo_pairs = list(zip(polygon_points[:bound_len], reversed(polygon_points[bound_len:])))

        lane_sectors = [[geo_pairs[i][0], geo_pairs[i][1], geo_pairs[i+1][1], geo_pairs[i+1][0]]
                        for i in range(len(geo_pairs)-1)]
        lane_sectors = lane_sectors if moving_towards_end else list(reversed(lane_sectors))

        # find the lane sector containing the point
        sector_id = -1
        for i, sector_points in enumerate(lane_sectors):
            contains_sector = Polygon(sector_points).contains(Point(pos))
            if contains_sector:
                sector_id = i
                break

        # get the waypoints in the middle of the lane polygon
        # and ignore the lane sectors behind the spawn position / ahead of end position
        waypoints = GlobalPlanner._get_intermed_section_waypoints(sec, road)
        bound = sector_id + 1 if is_final else len(lane_sectors) - sector_id - 1
        print('sector_id:', sector_id, 'middles:', waypoints, 'road_id', road.road_id)
        return waypoints[bound:]


# def load_town_04():
#     from xodr_converter import XODRConverter
#     from os import path

#     xodr_path = "/home/axel/paf21-1/components/global_planner/xodr/Town04.xodr"
#     print("File exists:", path.exists(xodr_path))
#     xodr_map = XODRConverter.read_xodr(xodr_path)
#     return xodr_map

# def test_cirle():
#     p1 = (0 ,100)
#     p2 = (100, 0)
#     rad = 400
#     points = Road._circular_interpolation(p1, p2, rad)
#     x = [p[0] for p in points]
#     y = [p[1] for p in points]
#     import matplotlib.pyplot as plt
#     plt.plot(x, y)
#     plt.show()

# if __name__ == "__main__":

#     # test_cirle()
#     xodr = load_town_04()
#     start = (406.0249938964844, 124.69999694824219)
#     # start = (182.8696438619712, 388.94453431093973)
#     end = (7.50634155273438, 130.54249572753906)
#     path = GlobalPlanner.get_shortest_path(start,end, xodr)
#     print(path)
#     route_waypoints = GlobalPlanner._preplan_route(start, end, path, xodr)
#     x = [p[0] for p in route_waypoints]
#     y = [p[1] for p in route_waypoints]
#     import matplotlib.pyplot as plt
#     plt.scatter(x, y)
#     plt.show()
#     print(route_waypoints)
