"""A module for annotating routes with valuable metadata"""

from math import dist as euclid_dist
from dataclasses import dataclass
from typing import Tuple, List, Callable

from global_planner.geometry import bounding_box
from global_planner.xodr_converter import XodrMap, Road, TrafficLight, TrafficSign, TrafficSignType


@dataclass
class AnnRouteWaypoint:
    """Representing a route waypoint including
    annotations regarding the driving context"""
    pos: Tuple[float, float]
    actual_lane: int
    possible_lanes: List[int]
    legal_speed: float
    dist_next_tl: float


@dataclass
class PathSection:
    """Representing a path section"""
    road_id: int
    lane_id: int
    drive_reversed: bool
    possible_lanes: List[int]
    end_pos: Tuple[float, float]


@dataclass
class RouteMetadata:
    """Representing a collection of route metadata
    that can be used to annotate an entire route."""
    sections_ahead: List[PathSection]
    traffic_lights_ahead: List[TrafficLight]
    speed_signs_ahead: List[TrafficSign]


class RouteAnnotation:
    """Representing helper functionality to annotate pre-planned routes with metadata"""

    @staticmethod
    def preprocess_route_metadata(path: List[str], xodr_map: XodrMap) -> RouteMetadata:
        """Evaluate the path selected by the navigation algorithm
        regarding route metadata to annotate."""

        path_sections: List[PathSection] = RouteAnnotation._norm_path(path, xodr_map.road_by_id)
        print("path_sections", path_sections)
        traffic_lights: List[TrafficLight] = []
        speed_signs: List[TrafficSign] = []

        for section in path_sections:
            road = xodr_map.road_by_id(section.road_id)

            sec_traffic_lights = RouteAnnotation._filter_items(road.traffic_lights, section)
            sec_speed_signs = RouteAnnotation._filter_items(road.speed_signs, section)

            for traffic_light in sec_traffic_lights:
                traffic_lights.append(traffic_light)
            for speed_sign in sec_speed_signs:
                speed_signs.append(speed_sign)

        return RouteMetadata(path_sections, traffic_lights, speed_signs)

    @staticmethod
    def _filter_items(items: List, section: PathSection) -> List:
        items = [tl for tl in items if not tl.is_reversed ^ section.drive_reversed]
        items = list(sorted(items, key=lambda tl: tl.dist_from_start))
        if section.drive_reversed:
            items = list(reversed(items))
        return items

    @staticmethod
    def _norm_path(path: List[str], road_by_id: Callable) -> List[PathSection]:
        """Remove duplicates from the path and convert
        the path strings into a normalized structure"""

        if path[0] != '-1_0_0' or path[-1] != '-2_0_0':
            raise ValueError('Invalid arguments! Path does not\
                              contain the first and/or the last node!')

        last_road_id = -1
        path_sections = []
        path_without_start_and_end = path[1:-1]

        for edge in path_without_start_and_end:
            split = edge.split('_')
            road_id, is_road_end, lane_id = int(split[0]), bool(split[1]), int(split[2])
            is_entering_new_section = road_id == last_road_id

            if is_entering_new_section:
                road: Road = road_by_id(road_id)
                drive_reverse = is_road_end
                norm_lane_id = abs(lane_id)
                poss_lanes = road.left_ids + road.right_ids
                if road.line_type != "broken":
                    if is_road_end == 0:
                        poss_lanes = road.left_ids
                    else:
                        poss_lanes = road.right_ids
                poss_lanes = list(sorted([id * (1 if drive_reverse else -1) for id in poss_lanes]))
                lane_offset = road.road_width * (norm_lane_id - 0.5)
                road_bounds = bounding_box(road.road_start, road.road_end, lane_offset)
                end_pos = road_bounds[1] if drive_reverse else road_bounds[3]
                section = PathSection(road_id, norm_lane_id, is_road_end, poss_lanes, end_pos)
                path_sections.append(section)

            last_road_id = road_id

        return path_sections

    @staticmethod
    def annotate_waypoints(waypoints: List[Tuple[float, float]], metadata: RouteMetadata) \
                           -> List[AnnRouteWaypoint]:
        """Annotate the waypoints with route metadata"""

        # TODO: refactor this function, remove duplication

        max_dist = 999.0
        radius_handled = 5.0
        default_speed = 50.0
        # TODO consider first and last road
        tl_id, ss_id, sec_id = 0, 0, -1
        tl_dist, ss_dist, sec_dist = float('inf'), float('inf'), float('inf')

        next_tl_pos = lambda: metadata.traffic_lights_ahead[tl_id].pos \
                                if tl_id < len(metadata.traffic_lights_ahead) else None
        next_ss_pos = lambda: metadata.speed_signs_ahead[ss_id].pos \
                                if ss_id < len(metadata.speed_signs_ahead) else None
        next_sec_end = lambda: metadata.sections_ahead[sec_id].end_pos \
                                 if sec_id < len(metadata.sections_ahead) else None

        ann_waypoints: List[AnnRouteWaypoint] = []

        print(f'num traffic lights: {len(metadata.traffic_lights_ahead)}')
        print(f'num speed signs: {len(metadata.speed_signs_ahead)}')
        for wp in waypoints:
            if sec_id < 0:
                sec_id += 1
                continue
            tl_pos, ss_pos, sec_end_pos = next_tl_pos(), next_ss_pos(), next_sec_end()
            tl_dist = euclid_dist(wp, tl_pos) if tl_pos else max_dist
            ss_dist = euclid_dist(wp, ss_pos) if ss_pos else max_dist
            sec_dist = euclid_dist(wp, sec_end_pos) if sec_end_pos else max_dist
            if (len(metadata.sections_ahead)<= sec_id):
                print("this will fail", len(metadata.sections_ahead), sec_id, "last:", metadata.sections_ahead[sec_id-1])
                continue
            actual_lane = metadata.sections_ahead[sec_id].lane_id
            poss_lanes = metadata.sections_ahead[sec_id].possible_lanes
            legal_speed = metadata.speed_signs_ahead[ss_id-1].legal_speed if ss_id > 0 else default_speed

            ann_wp = AnnRouteWaypoint(wp, actual_lane, poss_lanes, legal_speed, tl_dist)
            ann_waypoints.append(ann_wp)

            if tl_dist < radius_handled:
                print(f'traffic light {tl_id} found at {wp}')
                tl_id += 1
            if ss_dist < radius_handled:
                print(f'speed sign {ss_id} found at {wp}')
                ss_id += 1
            if sec_dist < radius_handled:
                print(f'section {sec_id} end found at {wp}')
                sec_id += 1
        print(ann_waypoints)
        return ann_waypoints
