"""A xodr converter based on xodr files."""
from math import sin, cos
from enum import IntEnum
from typing import List, Tuple, Dict
from dataclasses import dataclass, field
from pathlib import Path

from xml.etree import ElementTree as eTree
from xml.etree.ElementTree import Element
import numpy as np


def create_key(road: int, pos: int, link: int) -> str:
    """Function to create the key of the mapping dictionary."""
    return f"{road}_{pos}_{link}"


@dataclass
class TrafficSign:
    """Represents the data of a traffic sign object."""
    name: str
    relative_pos: Tuple[float, float]


@dataclass
class TrafficLight:
    """Represents the data of a traffic light object."""
    type: str
    s_value: float


@dataclass
class Geometry:
    """Represents the data of a geometry object."""
    start_point: Tuple[float, float]
    end_point: Tuple[float, float]
    length: float


class LinkType(IntEnum):
    """Represents the type of the link."""
    PRE = 0
    SUC = 1


@dataclass
class RoadLink:
    """Represents a link between two roads"""
    road_link_id: int
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
        self.road_link_id = int(road_link_xml.get('elementId'))
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
    road_width: float
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

        self.left_ids, self.right_ids = Road._get_lane_id(road_xml)
        self.line_type = Road._get_line_type(road_xml)
        self.traffic_signs = Road._get_traffic_signs(road_xml)
        self.traffic_lights = Road._get_traffic_lights(road_xml)
        self.geometries = Road._get_geometry(road_xml)
        self.road_width = Road._get_road_width(road_xml)

    @staticmethod
    def _get_line_type(road: Element) -> str:
        """Get the line type out of the road."""
        lane_sec = road.find('lanes').find('laneSection')
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    @staticmethod
    def _get_lane_id(road: Element) -> List[List[int]]:
        """Get the lane id."""
        directions = ['left', 'right']
        return_ids = [[], []]

        lane_sec = road.find('lanes').find('laneSection')
        for i, direction in enumerate(directions):
            if lane_sec.find(direction) is None:
                continue
            for lane in lane_sec.find(direction).findall('lane'):
                if lane.get('type') != 'driving':
                    continue

                return_ids[i].append(int(lane.get('id')))
        return return_ids

    @staticmethod
    def _get_possible_lanes(road: Element) -> List[int]:
        lane_id = Road._get_lane_id(road)
        lane_type = Road._get_line_type(road)
        if lane_type == "broken":
            return lane_id
        # Just Return the right lanes
        return []

    @staticmethod
    def _get_road_width(road: Element) -> float:
        """Get the lane id."""
        default_road_width = 4.0
        directions = ['left', 'right']

        lane_sec = road.find('lanes').find('laneSection')
        for direction in directions:
            if lane_sec.find(direction) is None:
                continue
            for lane in lane_sec.find(direction).findall('lane'):
                if lane.get('type') != 'driving':
                    continue
                return float(lane.find('width').get('a'))
        return default_road_width

    @staticmethod
    def _get_traffic_signs(road: Element) -> List[TrafficSign]:
        """Get the traffic signs out of the road."""
        traffic_signs_xml = road.find('objects')
        if traffic_signs_xml is None:
            return []

        return [TrafficSign(obj.get('name'), (float(obj.get('s')), float(obj.get('t'))))
                for obj in traffic_signs_xml.findall('object')]

    @staticmethod
    def _get_traffic_lights(road: Element) -> List[TrafficLight]:
        xml_signals = road.find('signal')

        signals = []
        if xml_signals is None:
            return signals

        for obj in xml_signals.findall('signal'):
            signal_type = obj.get('name')
            if signal_type == "Signal_3Light_Post01":
                signals.append(TrafficLight(signal_type, float(obj.get('s'))))

        return signals

    @staticmethod
    def _get_geometry(road: Element):
        """Get the geometry out of the road."""
        plan_view = road.find('planView')
        if plan_view is None:
            return []

        objects = []
        geometries = plan_view.findall('geometry')

        for i, geo_0 in enumerate(geometries):
            start_point = (float(geo_0.get('x')), float(geo_0.get('y')))
            length = float(geo_0.get('length'))
            is_last_geometry = i == len(geometries)-1

            if is_last_geometry:
                angle = float(geo_0.get('hdg'))
                end_point = Road._calculate_end_point(start_point, angle, length)
            else:
                geo_1 = geometries[i+1]
                end_point = (float(geo_1.get('x')), float(geo_1.get('y')))

            objects.append(Geometry(start_point, end_point, length))

        return objects

    @staticmethod
    def _calculate_end_point(start_point: Tuple[float, float], angle: float,
                             length: float) -> Tuple[float, float]:
        return start_point[0] + cos(angle) * length, \
               start_point[1] + sin(angle) * length


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
    def _get_connection_links(lane_links, connection, mapping) -> List[Tuple[int, int]]:
        links = []
        for lane_link in lane_links:
            from_lane = int(lane_link.get('from'))
            key_from = create_key(connection.incoming_road, 0, from_lane)
            to_lane = int(lane_link.get('to'))
            key_to = create_key(connection.connecting_road, 0, to_lane)

            if key_from in mapping and key_to in mapping:
                links.append((from_lane, to_lane))
        return links


@dataclass
class XodrMap:
    """Represents the data of a xodr map."""
    lane_lets: List[Road]
    junctions: List[Junction]
    mapping: Dict[str, int]
    matrix: np.ndarray = None

    def __post_init__(self):
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
        for road in self.lane_lets:
            # TODO Cost function
            length = 0
            for geometry in road.geometries:
                length += geometry.length

            for link in road.left_ids:
                index_start = self.mapping[create_key(road.road_id, 0, link)]
                index_end = self.mapping[create_key(road.road_id, 1, link)]
                matrix[index_end][index_start] = length

            for link in road.right_ids:
                index_start = self.mapping[create_key(road.road_id, 0, link)]
                index_end = self.mapping[create_key(road.road_id, 1, link)]
                matrix[index_start][index_end] = length

    def _junc_ids_entries(self, junction_id: int) -> List[Connection]:
        """Get the list of indices to the corresponding junction id."""
        for junction in self.junctions:
            if junction.junction_id == junction_id:
                return junction.connections
        return []

    def _apply_junction_connection(self, road: Road, link: RoadLink, matrix: np.ndarray):
        for connection in self._junc_ids_entries(link.road_link_id):
            if connection.incoming_road != road.road_id:
                continue

            contact_incoming = 0 if link.link_type == LinkType.PRE else 1
            contact_connecting = 0 if connection.contact_point == 'start' else 1

            for lane_links in connection.lane_links:
                key = create_key(connection.incoming_road, contact_incoming, lane_links[0])
                index_incoming = self.mapping[key]
                key = create_key(connection.connecting_road, contact_connecting, lane_links[1])
                index_connecting = self.mapping[key]

                matrix[index_incoming][index_connecting] = 1e-6

    def _apply_road_connection(self, road: Road, link: RoadLink, matrix: np.ndarray):
        for lane_link in road.right_ids:
            index_link, index_road = self._get_connected_lane_ids(lane_link, link, road.road_id)
            if link.link_type == LinkType.PRE:
                matrix[index_link][index_road] = 1e-6
            else:
                matrix[index_road][index_link] = 1e-6

        for lane_link in road.left_ids:
            index_link, index_road = self._get_connected_lane_ids(lane_link, link, road.road_id)
            if link.link_type == LinkType.PRE:
                matrix[index_road][index_link] = 1e-6
            else:
                matrix[index_link][index_road] = 1e-6

    def _get_connected_lane_ids(self, lane_link: int, link: RoadLink, road_id: int):
        key = create_key(link.road_link_id, link.contact_link, link.sign * lane_link)
        index_link = self.mapping[key]
        key = create_key(road_id, link.contact_road, lane_link)
        index_road = self.mapping[key]
        return index_link, index_road

    def _link_pre_suc(self, matrix: np.ndarray):
        """Link the predecessor and successor in the weighted matrix."""
        for road in self.lane_lets:
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
    def read_xodr(filepath: Path) -> XodrMap:
        """Read the xodr file from the file path."""
        root = eTree.parse(filepath).getroot()

        # parse all roads
        lane_lets = [Road(road) for road in root.findall('road')]
        lane_lets = [road for road in lane_lets if len(road.left_ids + road.right_ids)]

        # create the road mapping
        mapping = XODRConverter._create_road_mapping(lane_lets)

        road_ids = [road.road_id for road in lane_lets]
        junctions = [Junction(j_xml, mapping, road_ids) for j_xml in root.findall('junction')]

        return XodrMap(lane_lets, junctions, mapping)

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
