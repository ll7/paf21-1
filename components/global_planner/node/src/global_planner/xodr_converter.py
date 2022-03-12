"""A xodr converter based on xodr files."""
from abc import ABC
from enum import IntEnum
from typing import List, Tuple, Dict
from dataclasses import dataclass, field
from math import dist as euclid_dist, sqrt, pi, asin

from xml.etree import ElementTree as eTree
from xml.etree.ElementTree import Element
import numpy as np

from global_planner.geometry import rotate_vector, unit_vector, points_to_vector, \
    vector_len, scale_vector, add_vector, vec2dir


def create_key(road: int, pos: int, link: int) -> str:
    """Function to create the key of the mapping dictionary."""
    return f"{road}_{pos}_{link}"


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


class TrafficSign(TrafficSignal):
    """Represents the data of a traffic sign object."""

    # def __init__(self, node_xml: Element, road_start: Tuple[float, float],
    #              road_end: Tuple[float, float]):
    #     super(TrafficSign, self).__init__(node_xml, road_start, road_end)

    @property
    def sign_type(self) -> TrafficSignType or None:
        """Retrieve the sign type."""
        if self.name.startswith('Speed_'):
            return TrafficSignType.SPEED_LIMIT
        if self.name.startswith('Stencil_STOP'):
            return TrafficSignType.STOP
        if self.name.startswith('SimpleCrosswalk'):
            return TrafficSignType.CROSSWALK

        # LadderCrosswalk
        # StopLine
        # SolidSingleWhite
        # print("parsing unknown object: ", self.name)
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
    # road_width: float
    lane_widths: Dict[int, float]
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
        self.lane_widths = Road._get_lane_widths(road_xml)
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
    def _get_lane_widths(road: Element) -> Dict[int, float]:
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

        return lane_widths

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
                angle = float(geo_0.get('hdg'))
                end = Road._calculate_end_point(start, angle, length, arc_radius)
                if (euclid_dist(start, end) > length+0.2):
                    raise Exception("End Point was set incorrectly", start, end, angle, length)
            else:
                geo_1 = geometries[i+1]
                end = (float(geo_1.get('x')), float(geo_1.get('y')))

            offset = offsets[i] if offsets and i < len(offsets) else 0

            circular_interpolation = arc_radius != 0
            if not circular_interpolation:
                objects.append(Geometry(start, end, length, offset))
                continue

            points = Road._circular_interpolation(start, end, arc_radius)
            geo_vecs = zip(points[:-1], points[1:])
            vec_len = euclid_dist(points[0], points[1])
            objects.extend([Geometry(start, end, vec_len, offset) for (start, end) in geo_vecs])

        return objects

    @staticmethod
    def _circular_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                                arc_radius: float) -> List[Tuple[float, float]]:

        step_size = 2.0
        sign = -1 if arc_radius < 0 else 1
        arc_radius = abs(arc_radius)

        # determine the circular angle of the arc
        angle = asin((euclid_dist(start, end) / 2) / arc_radius) * 2
        assert(arc_radius > euclid_dist(start, end) / 2)

        # construct the mid-perpendicular of |start, end| to determine the circle's center
        conn_middle = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        center_offset = sqrt(pow(arc_radius, 2) - pow(euclid_dist(start, end) / 2, 2))
        mid_perpend = rotate_vector(points_to_vector(start, end), pi/2 * sign)
        circle_center = add_vector(conn_middle, scale_vector(mid_perpend, center_offset))

        # partition the arc into steps (-> interpol. geometries)
        arc_circumference = arc_radius * angle # (r * 2 pi) * (angle / 2 pi)
        num_steps = int(arc_circumference / step_size) + 1 # each step < step size

        # compute the interpolated points on the circle arc
        vec_to_p = points_to_vector(circle_center, start)
        rot_angles = [angle * (i / num_steps) for i in range(num_steps+1)]
        points = [add_vector(circle_center, rotate_vector(vec_to_p, rot * sign))
                    for rot in rot_angles]

        return points

    @staticmethod
    def _calculate_end_point(start_point: Tuple[float, float], angle: float,
                             length: float, radius: float) -> Tuple[float, float]:

        # simple case for no arc
        if radius == 0.0:
            diff_vec = scale_vector(unit_vector(angle), length)
            return add_vector(start_point, diff_vec)

        # determine the length of |start, end|
        alpha = length / radius
        diff_vec = scale_vector(unit_vector(alpha), radius)
        dist_start_end = euclid_dist(diff_vec, (radius, 0))

        # determine the direction of |start, end| and apply it
        dir_start_end = unit_vector(alpha / 2 + angle)
        # TODO: figure out why the direction is actually 'alpha / 2 + angle'

        # apply vector |start --> end| to the start point to retrieve the end point
        diff_vec = scale_vector(dir_start_end, dist_start_end)
        return add_vector(start_point, diff_vec)


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

            # TODO: is pos = 0 appropriate for multi-lane navigation? how are roads modeled?
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
    matrix: np.ndarray = None

    def __post_init__(self):
        if self.roads_by_id is None:
            self.roads_by_id = {}
        if self.matrix is None:
            self.matrix = self._create_links()

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

            for link in road.left_ids:
                index_start = self.mapping[create_key(road.road_id, 1, link)]
                index_end = self.mapping[create_key(road.road_id, 0, link)]
                matrix[index_start][index_end] = length

            for link in road.right_ids:
                index_start = self.mapping[create_key(road.road_id, 0, link)]
                index_end = self.mapping[create_key(road.road_id, 1, link)]
                matrix[index_start][index_end] = length

            # connect neighbored lanes of a road at the end (multi-lane support)
            if len(road.left_ids) > 1:
                for id_1, id_2 in zip(road.left_ids[:-1], road.left_ids[1:]):
                    id_1_end = self.mapping[create_key(road.road_id, 0, id_1)]
                    id_2_end = self.mapping[create_key(road.road_id, 0, id_2)]
                    matrix[id_1_end][id_2_end] = 5
                    matrix[id_2_end][id_1_end] = 5
            if len(road.right_ids) > 1:
                for id_1, id_2 in zip(road.right_ids[:-1], road.right_ids[1:]):
                    id_1_end = self.mapping[create_key(road.road_id, 1, id_1)]
                    id_2_end = self.mapping[create_key(road.road_id, 1, id_2)]
                    matrix[id_1_end][id_2_end] = 5
                    matrix[id_2_end][id_1_end] = 5

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
            # if len(lane_link_ids) == 0:
            #     print(f'link {road_id} to {link.road_id} has no driving lanes. should never happen')
            #     print(f'{lane_link_ids}, {conn_lane_link_ids}')
            fill_ids = [lane_link_ids[-1] for _ in range(abs(diff))]
            lane_link_ids.extend(fill_ids)
        elif diff < 0:
            # if len(conn_lane_link_ids) == 0:
            #     print(f'link {road_id} to {link.road_id} has no driving lanes. should never happen')
            #     print(f'{lane_link_ids}, {conn_lane_link_ids}')
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

        return mapping
