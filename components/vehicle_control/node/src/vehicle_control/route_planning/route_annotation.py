"""A module for annotating routes with valuable metadata"""

from math import dist as euclid_dist
from dataclasses import dataclass, field
from typing import Tuple, List, Callable

from vehicle_control.core.geometry import bounding_box
from vehicle_control.route_planning.xodr_converter import \
    XodrMap, Road, TrafficLight, TrafficSign, TrafficSignal


@dataclass
class AnnRouteWaypoint:
    """Representing a route waypoint including
    annotations regarding the driving context"""
    pos: Tuple[float, float]
    road_id: int
    actual_lane: int
    possible_lanes: List[int]
    legal_speed: float
    dist_next_tl: float
    end_lane_m: float
    stop_sign_m: float # TODO: needs to be reworked to concat multiple road sections
                      #       without a crossroad / highway exit between them
    


@dataclass
class PathSection:
    """Representing a path section"""
    road_id: int
    lane_id: int
    drive_reversed: bool
    possible_lanes: List[int]
    end_pos: Tuple[float, float]
    # road: Road = field(repr=False)
    # length: float


@dataclass
class RouteMetadata:
    """Representing a collection of route metadata
    that can be used to annotate an entire route."""
    sections_ahead: List[PathSection]
    traffic_lights_ahead: List[TrafficLight]
    speed_signs_ahead: List[TrafficSign]
    stop_signs_ahead: List[TrafficSign]
    initial_speed: float


class RouteAnnotation:
    """Representing helper functionality to annotate pre-planned routes with metadata"""

    @staticmethod
    def annotate_waypoints(waypoints: List[Tuple[float, float]], metadata: RouteMetadata,
                           map: XodrMap) -> List[AnnRouteWaypoint]:
        """Annotate the waypoints with route metadata"""
        max_dist = 999.0
        radius_handled = 5.0
        default_speed = 50.0
        stop_sign_m = 999
        tl_id, ss_id, sec_id, stop_id = 0, 0, 0, 0
        tl_dist, ss_dist, sec_dist = float('inf'), float('inf'), float('inf')

        next_tl_pos = lambda: metadata.traffic_lights_ahead[tl_id].pos \
                                if tl_id < len(metadata.traffic_lights_ahead) else None
        next_speedsign_pos = lambda: metadata.speed_signs_ahead[ss_id].pos \
                                if ss_id < len(metadata.speed_signs_ahead) else None
        next_sec_end = lambda: metadata.sections_ahead[sec_id].end_pos \
                                 if sec_id < len(metadata.sections_ahead) else None
        next_stopsign_pos = lambda: metadata.stop_signs_ahead[stop_id].pos \
                                if stop_id < len(metadata.stop_signs_ahead) else None
        ann_waypoints: List[AnnRouteWaypoint] = []

        legal_speed = metadata.initial_speed
        for waypoint in waypoints:
            tl_pos, ss_pos, sec_end_pos = next_tl_pos(), next_speedsign_pos(), next_sec_end()
            stop_sign_pos = next_stopsign_pos()
            tl_dist = euclid_dist(waypoint, tl_pos) if tl_pos else max_dist
            ss_dist = euclid_dist(waypoint, ss_pos) if ss_pos else max_dist
            stopsign_dist = euclid_dist(waypoint, stop_sign_pos) if stop_sign_pos else max_dist
            
            sec_dist = euclid_dist(waypoint, sec_end_pos) if sec_end_pos else max_dist

            current_road = map.roads_by_id[metadata.sections_ahead[sec_id].road_id]
            actual_lane = metadata.sections_ahead[sec_id].lane_id
            poss_lanes = metadata.sections_ahead[sec_id].possible_lanes

            ann_wp = AnnRouteWaypoint(waypoint, current_road.road_id, actual_lane,
                                      poss_lanes, legal_speed, tl_dist, sec_dist, stopsign_dist)
            ann_waypoints.append(ann_wp)

            if tl_dist < radius_handled:
                tl_id += 1
            if ss_dist < radius_handled:
                legal_speed = metadata.speed_signs_ahead[ss_id].legal_speed
                ss_id += 1
            if stopsign_dist < radius_handled:
                stop_id +=1
            if sec_dist < radius_handled:
                legal_speed = default_speed
                sec_id = min(sec_id + 1, len(metadata.sections_ahead) - 1)

        return ann_waypoints

    @staticmethod
    def preprocess_route_metadata(spawn_pos: Tuple[float, float], path: List[str],
                                  xodr_map: XodrMap) -> RouteMetadata:
        """Evaluate the path selected by the navigation algorithm
        regarding route metadata to annotate."""

        road_by_id = lambda id: xodr_map.roads_by_id[id]
        path_sections: List[PathSection] = RouteAnnotation._norm_path(path, road_by_id)
        traffic_lights: List[TrafficLight] = []
        speed_signs: List[TrafficSign] = []
        stop_signs: List[TrafficSign] = []
        inital_speed = 50.0

        for i, section in enumerate(path_sections):
            road = xodr_map.roads_by_id[section.road_id]
            sec_traffic_lights = RouteAnnotation._filter_items(road.traffic_lights, section)
            sec_speed_signs = RouteAnnotation._filter_items(road.speed_signs, section)
            sec_stop_signs = RouteAnnotation._filter_items(road.stop_signs, section)

            is_first_section = i == 0
            if is_first_section:
                road_len = road.approx_road_length
                dist_spawn_end = euclid_dist(section.end_pos, spawn_pos)
                s_value_car = road_len - dist_spawn_end

                for speed_s in sec_speed_signs:
                    if speed_s.dist_from_start < s_value_car:
                        inital_speed = speed_s.legal_speed
                        sec_speed_signs.remove(speed_s)

                for traffic_l in sec_traffic_lights:
                    if traffic_l.dist_from_start < s_value_car:
                        sec_traffic_lights.remove(traffic_l)

                for stop_sign in sec_stop_signs:
                    if stop_sign.dist_from_start < s_value_car:
                        sec_stop_signs.remove(stop_sign)

            traffic_lights.extend(sec_traffic_lights)
            speed_signs.extend(sec_speed_signs)
            stop_signs.extend(sec_stop_signs)

        return RouteMetadata(path_sections, traffic_lights, speed_signs, stop_signs, inital_speed)

    @staticmethod
    def _filter_items(items: List[TrafficSignal], section: PathSection) -> List:
        items = [tl for tl in items if not tl.is_right_side ^ section.drive_reversed]
        items = list(sorted(items, key=lambda tl: tl.dist_from_start))
        if section.drive_reversed:
            items = list(reversed(items))
        return items

    @staticmethod
    def _norm_path(path: List[str], road_by_id: Callable[[int], Road]) -> List[PathSection]:
        """Remove duplicates from the path and convert
        the path strings into a normalized structure"""

        if path[0] != '-1_0_0' or path[-1] != '-2_0_0':
            raise ValueError('Invalid arguments! Path does not\
                              contain the first and/or the last node!')

        path_without_start_and_end = path[1:-1]
        last_road_id = -1
        path_sections = []

        for i, edge in enumerate(path_without_start_and_end):
            split = edge.split('_')
            road_id, is_road_end, lane_id = int(split[0]), bool(int(split[1])), int(split[2])
            is_first_section = i == 0
            is_road_end = not is_road_end if is_first_section else is_road_end

            is_entering_new_section = road_id != last_road_id
            if is_entering_new_section:
                road = road_by_id(road_id)
                drive_reverse = is_road_end
                poss_lanes = RouteAnnotation._get_poss_lanes(road, drive_reverse)
                lane_offset = road.lane_offsets[lane_id] - road.lane_widths[lane_id]/2
                road_bounds = bounding_box(road.road_start, road.road_end, lane_offset)
                end_pos = road_bounds[1] if drive_reverse else road_bounds[3]
                section = PathSection(road_id, lane_id, drive_reverse, poss_lanes, end_pos)
                path_sections.append(section)

            last_road_id = road_id

        return path_sections

    @staticmethod
    def _get_poss_lanes(road: Road, drive_reverse: bool):
        poss_lanes = road.left_ids + road.right_ids

        can_use_oncoming_lanes = road.line_type == "broken"
        if not can_use_oncoming_lanes:
            poss_lanes = road.left_ids if drive_reverse else road.right_ids
        return poss_lanes
