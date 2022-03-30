"""A xodr converter based on xodr files."""
from abc import ABC
from enum import IntEnum
from typing import List, Tuple, Dict
from dataclasses import dataclass, field
from math import dist as euclid_dist, pi

from xml.etree import ElementTree as eTree
from xml.etree.ElementTree import Element

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

import numpy as np

from vehicle_control.core.geometry import \
    unit_vector, points_to_vector, vector_len, \
    scale_vector, add_vector, vec2dir, sub_vector, \
    orth_offset_left, orth_offset_right
from vehicle_control.route_planning.route_interpolation import \
    circular_interpolation, end_of_circular_arc


def create_key(road: int, pos: int, link: int) -> str:
    """Function to create the key of the mapping dictionary."""
    return f"{road}_{pos}_{link}"


def split_key(key: str) -> Tuple[int, bool, int]:
    """Function to split the key of the mapping dictionary."""
    key_split = key.split('_')
    return int(key_split[0]), bool(int(key_split[1])), int(key_split[2])


def global_pos(road_start: Tuple[float, float], road_end: Tuple[float, float],
               dist_from_road_start: float) -> Tuple[float, float]:
    """Determine the global position based on the offset from the road's start point"""
    road_vec = points_to_vector(road_start, road_end)
    road_start_to_sign_vec = scale_vector(road_vec, dist_from_road_start)
    return add_vector(road_start, road_start_to_sign_vec)


@dataclass
class Geometry:
    """Represents the data of a geometry object."""
    start_point: Tuple[float, float]
    end_point: Tuple[float, float]
    length: float
    offset: float

    @property
    def direction(self):
        """The direction the geometry is pointing towards."""
        return vec2dir(self.start_point, self.end_point)


class TrafficSignType(IntEnum):
    """Representing a traffic sign type"""
    SPEED_LIMIT = 0
    STOP = 1
    CROSSWALK = 2


class TrafficSignal(ABC):
    # pylint: disable=too-few-public-methods
    """Represents the data of a traffic sign object."""
    name: str
    dist_from_start: float
    is_right_side: bool
    pos: Tuple[float, float]

    def __init__(self, node_xml: Element, road_start: Tuple[float, float],
                 road_end: Tuple[float, float]):
        self.name = node_xml.get('name')
        self.is_right_side = float(node_xml.get('t')) > 0
        dist_from_start = float(node_xml.get('s'))

        # TODO: use geometries for distance calculations
        dist_from_end = vector_len(points_to_vector(road_start, road_end)) - dist_from_start

        self.pos = global_pos(road_start, road_end, dist_from_start)
        self.dist_from_start = dist_from_end if self.is_right_side else dist_from_start


class TrafficLight(TrafficSignal):
    # pylint: disable=too-few-public-methods
    """Represents the data of a traffic sign object."""
    # remove this class because it doesn't add anything to the model


class TrafficSign(TrafficSignal):
    """Represents the data of a traffic sign object."""

    @property
    def sign_type(self) -> TrafficSignType or None:
        """Retrieve the sign type."""
        if self.name.startswith('Speed_') or self.name.startswith('speed_'):
            return TrafficSignType.SPEED_LIMIT
        if self.name.startswith('Stencil_STOP') or self.name.startswith('Sign_Stop') \
                or self.name.startswith('StopLine'):
            return TrafficSignType.STOP
        if self.name.startswith('SimpleCrosswalk'):
            return TrafficSignType.CROSSWALK
        return None

    @property
    def legal_speed(self) -> float:
        """Retrieve the legal speed limit introduced by this traffic sign."""
        if not self.sign_type == TrafficSignType.SPEED_LIMIT:
            raise RuntimeError('Invalid operation! Traffic sign is no speed sign!')
        return float(self.name[6:])


class LinkType(IntEnum):
    """Represents the type of the link."""
    PRE = 0
    SUC = 1


@dataclass
class RoadLink:
    """Represents a link between two roads"""
    road_id: int
    type: str
    contact_point: str
    link_type: LinkType

    @property
    def contact_link(self) -> int:
        """Representing the contact point, depending on whether the
        road link goes from start-to-end or end-to-start"""
        return 0 if self.contact_point == 'start' else 1

    @property
    def sign(self) -> int:
        """Representing the side of road"""
        is_start = self.contact_point == 'start'
        is_pre_linkage = self.link_type == LinkType.PRE
        return 1 if is_start ^ is_pre_linkage else -1

    @property
    def contact_road(self) -> int:
        """Representing the contact point of the connecting road, depending on
        whether the other road link goes from start-to-end or end-to-start"""
        return 0 if self.link_type == LinkType.PRE else 1

    def __init__(self, road_link_xml: Element, link_type: LinkType):
        self.road_id = int(road_link_xml.get('elementId'))
        self.type = road_link_xml.get('elementType')
        self.contact_point = road_link_xml.get('contactPoint')
        self.link_type = link_type


@dataclass
class Road:
    """Represents the data of a road object."""
    road_id: int
    junction: int
    left_ids: List[int]
    right_ids: List[int]
    line_type: str
    traffic_signs: List[TrafficSign]
    traffic_lights: List[TrafficLight]
    geometries: List[Geometry]
    lane_widths: Dict[int, float]
    lane_offsets: Dict[int, float]
    lane_polygons: Dict[int, List[Tuple[float, float]]]
    suc: RoadLink = None
    pre: RoadLink = None

    def __init__(self, road_xml: Element):
        self.road_id = int(road_xml.get('id'))
        self.junction = int(road_xml.get('junction'))

        link = road_xml.find('link')
        self.suc = None if link.find('successor') is None \
            else RoadLink(link.find('successor'), LinkType.SUC)
        self.pre = None if link.find('predecessor') is None \
            else RoadLink(link.find('predecessor'), LinkType.PRE)

        self.left_ids, self.right_ids = Road._get_lane_ids(road_xml)
        self.line_type = Road._get_line_type(road_xml)
        self.geometries = Road._get_geometry(road_xml)
        self.lane_widths, self.lane_offsets = Road._get_lane_widths_and_offsets(road_xml)
        self.lane_polygons = self._compute_polygons()
        self.traffic_signs = Road._get_traffic_signs(road_xml, self.road_start, self.road_end)
        self.traffic_lights = Road._get_traffic_lights(road_xml, self.road_start, self.road_end)

    @property
    def has_driving_lanes(self) -> bool:
        """Determines whether the road contains lanes to drive on"""
        return len(self.left_ids) + len(self.right_ids) > 0

    @property
    def road_start(self) -> Tuple[float, float]:
        """The start of the first road geometry"""
        return self.geometries[0].start_point

    @property
    def road_end(self) -> Tuple[float, float]:
        """The end of the last road geometry"""
        return self.geometries[-1].end_point

    @property
    def approx_road_length(self) -> float:
        """A linear approximation of the road length based on the start and end point"""
        if not self.geometries:
            raise RuntimeError('Invalid operation! Road has no geometries!')
        if len(self.geometries) == 1:
            return vector_len(points_to_vector(self.road_start, self.road_end))
        return sum([geo.length for geo in self.geometries])

    @property
    def speed_signs(self) -> List[TrafficSign]:
        """All speed signs from the traffic signs list."""
        return [s for s in self.traffic_signs if s.sign_type == TrafficSignType.SPEED_LIMIT]

    @property
    def stop_signs(self) -> List[TrafficSign]:
        """All stop signs from the traffic signs list."""
        return [s for s in self.traffic_signs if s.sign_type == TrafficSignType.STOP]

    # def contains_point(self, pos: Tuple[float, float]) -> bool:
    #     if not self.lane_polygons:
    #         return False
    #     return Polygon(self.lane_polygons[0]).contains(Point(pos))

    def detect_lane(self, pos: Tuple[float, float]) -> int:
        """Assuming the point is on the road, detect the lane."""
        point = Point(pos)
        for lane_id in self.lane_polygons:
            poly = Polygon(self.lane_polygons[lane_id])
            if poly.contains(point):
                return lane_id
        return 0

    @staticmethod
    def _get_line_type(road: Element) -> str:
        """Get the line type out of the road."""
        lane_sec = road.find('lanes').find('laneSection')
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    @staticmethod
    def _get_lane_ids(road: Element) -> Tuple[List[int], List[int]]:
        """Get the lane id."""
        directions = ['left', 'right']
        return_ids = [[], []]

        lane_sec = road.find('lanes').find('laneSection')
        for i, direction in enumerate(directions):
            if lane_sec.find(direction) is None:
                continue
            for lane in lane_sec.find(direction).findall('lane'):
                if lane.get('type') not in ['driving']:
                    continue
                return_ids[i].append(int(lane.get('id')))

        # sort the right / left ids outgoing from road center
        left_ids, right_ids = return_ids
        left_ids, right_ids = list(sorted(left_ids)), list(reversed(sorted(right_ids)))
        return left_ids, right_ids

    @staticmethod
    def _get_lane_widths_and_offsets(road: Element) -> Tuple[Dict[int, float], Dict[int, float]]:
        """Get the lane widths as dictionary by lane id."""
        lane_widths: Dict[int, float] = {}
        lane_sec = road.find('lanes').find('laneSection')

        for direction in ['left', 'right']:
            if lane_sec.find(direction) is None:
                continue
            lanes = lane_sec.find(direction).findall('lane')
            for lane in lanes:
                lane_id = int(lane.get('id'))
                lane_width = float(lane.find('width').get('a'))
                lane_widths[lane_id] = lane_width

        # all lanes are required, also ones that are non-drivable like shoulders, etc.
        all_left_ids = list(sorted([key for key in lane_widths if key > 0]))
        all_right_ids = list(reversed(sorted([key for key in lane_widths if key < 0])))

        lane_offsets = {}
        cumulated_width = 0.0
        for lane_id in all_left_ids:
            cumulated_width += lane_widths[lane_id]
            lane_offsets[lane_id] = cumulated_width
        cumulated_width = 0.0
        for lane_id in all_right_ids:
            cumulated_width += lane_widths[lane_id]
            lane_offsets[lane_id] = cumulated_width

        return lane_widths, lane_offsets

    @staticmethod
    def _get_traffic_signs(road: Element, road_start: Tuple[float, float],
                           road_end: Tuple[float, float]) -> List[TrafficSign]:
        """Get the traffic signs out of the road."""
        traffic_signs = []
        traffic_signs_xml = road.find('objects')

        if traffic_signs_xml is None:
            return traffic_signs

        traffic_signs = [TrafficSign(obj, road_start, road_end)
                         for obj in traffic_signs_xml.findall('object')]
        traffic_signs = [s for s in traffic_signs if s.sign_type is not None]
        return traffic_signs

    @staticmethod
    def _get_traffic_lights(road: Element, road_start: Tuple[float, float],
                            road_end: Tuple[float, float]) -> List[TrafficLight]:
        """Get the traffic lights out of the road."""
        traffic_lights = []
        traffic_lights_xml = road.find('signals')

        if traffic_lights_xml is None:
            return traffic_lights

        return [TrafficLight(obj, road_start, road_end)
                for obj in traffic_lights_xml.findall('signal')]

    @staticmethod
    def _get_geometry(road: Element):
        """Get the geometry out of the road."""
        plan_view = road.find('planView')
        if plan_view is None:
            return []

        objects = []
        geometries = plan_view.findall('geometry')

        offsets = road.find('lanes').findall('laneOffset')
        offsets = [float(o.get('a')) for o in offsets]

        for i, geo_0 in enumerate(geometries):
            start = (float(geo_0.get('x')), float(geo_0.get('y')))
            length = float(geo_0.get('length'))
            is_last_geometry = i == len(geometries)-1

            arc = [float(node.get('curvature')) for node in geo_0 if node.tag == 'arc']
            arc_radius = 1 / arc[0] if arc else 0

            if is_last_geometry:
                geo_orient = float(geo_0.get('hdg'))
                diff_vec = scale_vector(unit_vector(geo_orient), length)
                end = add_vector(start, diff_vec) if arc_radius == 0.0 \
                    else end_of_circular_arc(start, geo_orient, length, arc_radius)
            else:
                geo_1 = geometries[i+1]
                end = (float(geo_1.get('x')), float(geo_1.get('y')))

            offset = offsets[i] if offsets and i < len(offsets) else 0

            is_circular_arc = arc_radius != 0
            if not is_circular_arc:
                objects.append(Geometry(start, end, length, offset))
                continue

            points = circular_interpolation(start, end, arc_radius)
            geo_vecs = zip(points[:-1], points[1:])
            vec_len = euclid_dist(points[0], points[1])
            objects.extend([Geometry(start, end, vec_len, offset) for (start, end) in geo_vecs])

        return objects

    def _compute_polygons(self) -> Dict[int, List[Tuple[float, float]]]:
        """Compute the polygons representing the road bounds."""

        # compute intermediate offset vectors for curved road sections
        offset_vectors = self._compute_offset_vectors()

        # compute the middle of lane (line of geometries without offset)
        middle = [geo.start_point for geo in self.geometries] + [self.geometries[-1].end_point]

        # shift the middle by the lane offset
        # assumption: all lane offsets of a road are the same
        lane_offset = self.geometries[0].offset
        middle = [add_vector(middle[i], scale_vector(offset_vectors[i][0], lane_offset))
                    for i in range(len(middle))]

        all_polygons: Dict[int, List[Tuple[float, float]]] = {}

        # compute the inner / outer bounds for each lane and create the lane polygon from it
        for lane_id in self.left_ids + self.right_ids:
            vec_id = 1 if lane_id < 0 else 0
            uniform_lane_offsets = [pair[vec_id] for pair in offset_vectors]

            inner_scale = self.lane_offsets[lane_id] - self.lane_widths[lane_id]
            outer_scale = self.lane_offsets[lane_id]

            inner_offsets = [scale_vector(vec, inner_scale) for vec in uniform_lane_offsets]
            outer_offsets = [scale_vector(vec, outer_scale) for vec in uniform_lane_offsets]

            inner_bound = [add_vector(middle[i], inner_offsets[i]) for i in range(len(middle))]
            outer_bound = [add_vector(middle[i], outer_offsets[i]) for i in range(len(middle))]

            poly_points = inner_bound + list(reversed(outer_bound))
            all_polygons[lane_id] = poly_points

        # # compute the road polygon spanning all drivable lanes of the entire road
        # if self.left_ids or self.right_ids:
        #     leftmost_poly = all_polygons[min(self.left_ids)] if self.left_ids \
        #                         else all_polygons[max(self.right_ids)]
        #     rightmost_poly = all_polygons[max(self.right_ids)] if self.right_ids \
        #                         else all_polygons[min(self.left_ids)]

        #     num_geos = len(leftmost_poly) // 2
        #     road_poly = leftmost_poly[num_geos:] + rightmost_poly[:num_geos]
        #     all_polygons[0] = road_poly

        return all_polygons

    def _compute_offset_vectors(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        offsets_vectors = []
        if len(self.geometries) > 1:
            geo_pairs = zip(self.geometries[:-1], self.geometries[1:])
            offsets_vectors = [Road._compute_intermediate_offset_vectors(p[0], p[1])
                               for p in geo_pairs]

        # compute offset vectors for first / last geometry
        start_0, end_0 = self.geometries[0].start_point, self.geometries[0].end_point
        start_n, end_n = self.geometries[-1].start_point, self.geometries[-1].end_point
        offsets_start = (orth_offset_left(start_0, end_0, 1),
                         orth_offset_right(start_0, end_0, 1))
        offsets_end = (orth_offset_left(start_n, end_n, 1),
                       orth_offset_right(start_n, end_n, 1))
        offsets_vectors.insert(0, offsets_start)
        offsets_vectors.append(offsets_end)

        return offsets_vectors

    @staticmethod
    def _compute_intermediate_offset_vectors(geo_0: Geometry, geo_1: Geometry) \
            -> Tuple[Tuple[float, float], Tuple[float, float]]:

        # directions of vectors, geo_0 pointing forward, geo_1 pointing backward
        dir_0 = vec2dir(geo_0.start_point, geo_0.end_point)
        dir_1 = vec2dir(geo_1.end_point, geo_0.end_point)

        # halving angle between vectors for right / left side
        diff_angle = (dir_1 - dir_0) % (2 * pi)
        offset_left = diff_angle / 2

        vec_left = unit_vector(dir_0 + offset_left)
        vec_right = sub_vector((0, 0), vec_left)

        return vec_left, vec_right


@dataclass
class Connection:
    """Represents the connection in a junction."""
    connection_id: int
    incoming_road: int
    connecting_road: int
    contact_point: str
    lane_links: List[Tuple[int, int]] = field(default_factory=list)


@dataclass
class Junction:
    """Represents the junction."""
    junction_id: int
    connections: List[Connection]

    def __init__(self, junction_xml: Element, mapping: Dict[str, int], road_ids: List[int]):
        self.junction_id = int(junction_xml.attrib['id'])
        self.connections = []

        for connection_xml in junction_xml.findall('connection'):
            connection = Connection(int(connection_xml.get('id')),
                                    int(connection_xml.get('incomingRoad')),
                                    int(connection_xml.get('connectingRoad')),
                                    connection_xml.get('contactPoint'))

            links = Junction._get_connection_links(connection_xml, connection, mapping)
            if len(links) == 0:
                continue

            # set the links to the junction dict
            connection.lane_links = links

            if connection.incoming_road in road_ids and connection.connecting_road in road_ids:
                self.connections.append(connection)

    @staticmethod
    def _get_connection_links(lane_links: Element, connection: Connection,
                              mapping: Dict[str, int]) -> List[Tuple[int, int]]:
        links = []
        for lane_link in lane_links:
            from_lane, to_lane = int(lane_link.get('from')), int(lane_link.get('to'))

            key_from = create_key(connection.incoming_road, 0, from_lane)
            key_to = create_key(connection.connecting_road, 0, to_lane)

            if key_from in mapping and key_to in mapping:
                links.append((from_lane, to_lane))
        return links


@dataclass
class XodrMap:
    """Represents the data of a xodr map."""
    roads_by_id: Dict[int, Road]
    junctions_by_id: Dict[int, Junction]
    mapping: Dict[str, int]
    nav_graph: np.ndarray = None

    def __post_init__(self):
        if self.roads_by_id is None:
            self.roads_by_id = {}
        if self.nav_graph is None:
            self.nav_graph = self._create_links()

    # def find_sections(self, pos: Tuple[float, float]) -> List[Tuple[int, bool, Road]]:
    #     """Find the neighboring road sections related to the given position on the map"""
    #     sections = []

    #     for road in self.roads_by_id.values():

    #         if not road.geometries:
    #             print('road without geometries, this should never happen!')
    #             continue

    #         if not road.contains_point(pos):
    #             continue

    #         lane_id = road.detect_lane(pos)
    #         is_right_road_side = lane_id < 0
    #         sections.append((lane_id, is_right_road_side, road))

    #     return sections

    def find_sections(self, pos: Tuple[float, float]) -> List[Tuple[int, bool, Road]]:
        """Find the neighboring road sections related to the given position on the map"""
        sections = []

        for road in self.roads_by_id.values():

            if not road.geometries:
                print('road without geometries, this should never happen!')
                continue

            # determine whether the road contains the point
            # and if so, get the lane id and side of road
            point = Point(pos)

            for lane_id in road.lane_polygons:
                poly = Polygon(road.lane_polygons[lane_id])

                if poly.contains(point):
                    is_right_road_side = lane_id < 0
                    sections.append((lane_id, is_right_road_side, road))
                    break # polygons don't intersect -> only 1 hit per road

        return sections

    def _create_links(self):
        """Link geometry, predecessor and successor in the weighted matrix."""
        num_nodes = len(self.mapping)
        matrix = np.zeros(shape=(num_nodes, num_nodes))
        self._link_roads(matrix)
        self._link_pre_suc(matrix)

        return matrix

    def _link_roads(self, matrix):
        """Link the geometries between each other."""
        for road_id in self.roads_by_id:
            road = self.roads_by_id[road_id]
            length = sum([g.length for g in road.geometries])

            # create graph edges for linking each lane's start and end point
            # info: the navigation graph contains a node for each start / end of a lane
            for discount, link in enumerate(road.left_ids):
                index_start = self.mapping[create_key(road.road_id, 1, link)]
                index_end = self.mapping[create_key(road.road_id, 0, link)]
                matrix[index_start][index_end] = max(length - discount * 0.5, 0.1)
            for discount, link in enumerate(road.right_ids):
                index_start = self.mapping[create_key(road.road_id, 0, link)]
                index_end = self.mapping[create_key(road.road_id, 1, link)]
                length = max(length - 1, 0.0)
                matrix[index_start][index_end] = max(length - discount * 0.5, 0.1)

            # connect neighbored lanes of a road at the end (in logical driving
            # direction) to allow planned lane changes
            if len(road.left_ids) > 1:
                for id_1, id_2 in zip(road.left_ids[:-1], road.left_ids[1:]):
                    id_1_end = self.mapping[create_key(road.road_id, 0, id_1)]
                    id_2_end = self.mapping[create_key(road.road_id, 0, id_2)]
                    matrix[id_1_end][id_2_end] = 1e-3
                    matrix[id_2_end][id_1_end] = 1e-3
            if len(road.right_ids) > 1:
                for id_1, id_2 in zip(road.right_ids[:-1], road.right_ids[1:]):
                    id_1_end = self.mapping[create_key(road.road_id, 1, id_1)]
                    id_2_end = self.mapping[create_key(road.road_id, 1, id_2)]
                    matrix[id_1_end][id_2_end] = 1e-3
                    matrix[id_2_end][id_1_end] = 1e-3

    def _apply_junction_connection(self, road: Road, link: RoadLink, matrix: np.ndarray):

        # TODO: map entering road sections to the rightmost lanes
        #       e.g. when entering a highway, lane 1 should not map to the overtaking lane 1

        connections = self.junctions_by_id[link.road_id].connections \
            if link.road_id in self.junctions_by_id else []
        connections = [c for c in connections if c.incoming_road != road.road_id]

        for connection in connections:
            contact_incoming = 0 if link.link_type == LinkType.PRE else 1
            contact_connecting = 0 if connection.contact_point == 'start' else 1

            for lane_links in connection.lane_links:
                key = create_key(connection.incoming_road, contact_incoming, lane_links[0])
                index_incoming = self.mapping[key]
                key = create_key(connection.connecting_road, contact_connecting, lane_links[1])
                index_connecting = self.mapping[key]

                matrix[index_incoming][index_connecting] = 1e-6

    def _apply_road_connection(self, road: Road, link: RoadLink, matrix: np.ndarray):
        # for lane_link in road.right_ids:
        conn_road = self.roads_by_id[link.road_id]
        conn_ids = conn_road.right_ids if link.sign == 1 else conn_road.left_ids
        index_links = self._get_connected_lane_ids(road.road_id, road.right_ids, link, conn_ids)
        for index_link, index_road in index_links:
            if link.link_type == LinkType.PRE:
                matrix[index_link][index_road] = 1e-6
            else:
                matrix[index_road][index_link] = 1e-6

        # for lane_link in road.left_ids:
        conn_ids = conn_road.left_ids if link.sign == 1 else conn_road.right_ids
        index_links = self._get_connected_lane_ids(road.road_id, road.left_ids, link, conn_ids)
        for index_link, index_road in index_links:
            if link.link_type == LinkType.PRE:
                matrix[index_road][index_link] = 1e-6
            else:
                matrix[index_link][index_road] = 1e-6

    def _get_connected_lane_ids(self, road_id: int, lane_link_ids: List[int], link: RoadLink,
                                conn_lane_link_ids: List[int]) -> List[Tuple[int, int]]:
        # TODO: support multi-lane navigation
        index_links = []
        if not conn_lane_link_ids or not lane_link_ids:
            return index_links

        diff = len(conn_lane_link_ids) - len(lane_link_ids)
        if diff > 0:
            fill_ids = [lane_link_ids[-1] for _ in range(abs(diff))]
            lane_link_ids.extend(fill_ids)
        elif diff < 0:
            fill_ids = [conn_lane_link_ids[-1] for _ in range(abs(diff))]
            conn_lane_link_ids.extend(fill_ids)

        for lane_link, conn_link in zip(lane_link_ids, conn_lane_link_ids):
            key_conn = create_key(link.road_id, link.contact_link, conn_link)
            key_road = create_key(road_id, link.contact_road, lane_link)
            index_links.append((self.mapping[key_conn], self.mapping[key_road]))

        return index_links

    def _link_pre_suc(self, matrix: np.ndarray):
        """Link the predecessor and successor in the weighted matrix."""
        for road_id in self.roads_by_id:
            road = self.roads_by_id[road_id]
            links = [road.pre, road.suc]
            for link in links:
                if link is None:
                    continue
                road_type = link.type
                if road_type == 'road':
                    self._apply_road_connection(road, link, matrix)
                elif road_type == 'junction':
                    self._apply_junction_connection(road, link, matrix)


class XODRConverter:
    # pylint: disable=too-few-public-methods
    """A xodr converter based on xodr files."""

    @staticmethod
    def read_xodr(filepath: str) -> XodrMap:
        """Read the xodr file from the file path."""
        root = eTree.parse(filepath).getroot()

        # parse all roads
        lanelets = [Road(road) for road in root.findall('road')]
        lanelets = [road for road in lanelets if road.has_driving_lanes]

        # TODO: check if those lanelets are relevant to driving
        road_ids = set([r.road_id for r in lanelets])
        for road in lanelets:
            if road.suc.road_id not in road_ids:
                road.suc = None
            if road.pre.road_id not in road_ids:
                road.pre = None

        # create the road mapping
        mapping = XODRConverter._create_road_mapping(lanelets)

        road_ids = [road.road_id for road in lanelets]
        junctions = [Junction(j_xml, mapping, road_ids) for j_xml in root.findall('junction')]

        lanelets = {r.road_id: r for r in lanelets}
        junctions = {j.junction_id:j for j in junctions}

        return XodrMap(lanelets, junctions, mapping)

    @staticmethod
    def _create_road_mapping(lane_lets: List[Road]) -> Dict[str, int]:
        """Fill the mapping with the position in the matrix and the point"""
        # init the mapping and the counter
        mapping = {}
        counter = 0

        # iterate over all entries in the lane-lets list
        for road in lane_lets:
            all_links = road.left_ids + road.right_ids
            for link in all_links:
                mapping[create_key(road.road_id, 0, link)] = counter
                mapping[create_key(road.road_id, 1, link)] = counter + 1
                counter += 2

        # insert generic start / end node for navigation
        num_nodes = len(mapping)
        mapping['-1_0_0'] = num_nodes
        mapping['-2_0_0'] = num_nodes + 1

        return mapping
