"""A xodr converter based on xodr files."""

from xml.etree import ElementTree as eTree
from os.path import isfile, exists
from pathlib import Path
from typing import List, Tuple, Dict
from dataclasses import dataclass
import numpy as np


@dataclass
class TrafficSign:
    """Represents the data of a traffic sign object."""
    name: str
    relative_pos: Tuple[float, float]


@dataclass
class Geometry:
    """Represents the data of a geometry object."""
    start_point: np.ndarray
    end_point: np.ndarray
    angle: float
    length: float
    curvature: float


@dataclass
class Road:
    """Represents the data of a road object."""
    road_id: int
    junction: int
    suc_id: int
    suc_type: str
    suc_contact_point: str
    pre_id: int
    pre_type: str
    pre_contact_point: str
    left_ids: List[int]
    right_ids: List[int]
    line_type: str
    traffic_signs: List[TrafficSign]
    geometry: List[Geometry]

@dataclass
class XodrMap:
    """Represents the data of a xodr map."""
    lane_lets: List[Road]
    mapping: Dict[str, int]
    matrix: np.ndarray


class XODRConverter:
    """A xodr converter based on xodr files."""

    @staticmethod
    def read_xodr(filepath: Path):
        """Read the xodr file from the file path."""
        root = XODRConverter._get_root(filepath)

        # parse all roads
        lane_lets = [XODRConverter._create_road_dict(road) for road in root.findall('road')]
        lane_lets = [road for road in lane_lets if len(road['left_ids'] + road['right_ids'])]

        # create the road mapping
        mapping = XODRConverter._create_road_mapping(lane_lets)

        road_ids = [road['road_id'] for road in lane_lets]
        # parse all junctions
        junctions = XODRConverter._get_junction_dic(road_ids, root.findall('junction'), mapping)

        # create links in the matrix
        matrix = XODRConverter._create_links(lane_lets, junctions, mapping)
        return lane_lets, mapping, matrix

    @staticmethod
    def _get_lane_sec(road):
        """Get the lane section out of the road."""
        return road.find('lanes').find('laneSection')

    @staticmethod
    def _get_line_type(road) -> str:
        """Get the line type out of the road."""
        lane_sec = XODRConverter._get_lane_sec(road)
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    @staticmethod
    def _get_lane_id(road) -> list:
        """Get the lane id."""
        directions = ['left', 'right']
        return_ids = [[], []]

        lane_sec = XODRConverter._get_lane_sec(road)
        for i, direction in enumerate(directions):
            if lane_sec.find(direction) is None:
                continue
            for lane in lane_sec.find(direction).findall('lane'):
                if lane.get('type') != 'driving':
                    continue

                return_ids[i].append(int(lane.get('id')))
        return return_ids

    @staticmethod
    def _get_traffic_signs(road) -> list:
        """Get the traffic signs out of the road."""
        traffic_signs = road.find('objects')
        if traffic_signs is None:
            return []

        # return a list with the traffic signs
        return [TrafficSign(obj.get('name'), (float(obj.get('s')), float(obj.get('t'))))
                for obj in traffic_signs.findall('object')]

    @staticmethod
    def _calculate_end_point(start_point: np.ndarray, angle: float,
                             length: float, arc: float) -> np.ndarray:
        """Calculate the end point based on the start point, angle, length and arc."""
        # TODO check implementation and add arc
        # https://www.delftstack.com/howto/numpy/curvature-formula-numpy/
        rotation = np.array([np.cos(angle), np.sin(angle)], dtype=np.float32)
        end_point = start_point + rotation * length
        return end_point

    @staticmethod
    def _get_geometry(road):
        """Get the geometry out of the road."""
        plan_view = road.find('planView')
        if plan_view is None:
            return []

        objects = []
        for geometry in plan_view.findall('geometry'):
            start_point = np.array([geometry.get('x'), geometry.get('y')], dtype=np.float32)
            angle = float(geometry.get('hdg'))
            length = float(geometry.get('length'))

            curvature = 0.0
            if geometry.find('arc') is not None:
                curvature = float(geometry.find('arc').get('curvature'))

            end_point = XODRConverter._calculate_end_point(start_point, angle, length, curvature)

            objects.append(Geometry(start_point, end_point, angle, length, curvature))

        return objects

    @staticmethod
    def _get_junction_dic(road_ids: list, all_junctions: list, mapping: dict) -> List[dict]:
        """Get the junction dictionary."""
        junctions = []

        for junction in all_junctions:
            junc_id = int(junction.attrib['id'])
            # TODO
            for connection in junction.findall('connection'):
                junction_dict = {
                    'junction_id': junc_id,
                    'connection_id': int(connection.get('id')),
                    'incoming_road': int(connection.get('incomingRoad')),
                    'connecting_road': int(connection.get('connectingRoad')),
                    'contact_point': connection.get('contactPoint')
                }
                links = XODRConverter._get_connection_links(connection, junction_dict, mapping)
                if links:
                    continue

                # set the links to the junction dict
                junction_dict['lane_links'] = links
                # add the junction to dict if the roads are driving roads
                if junction_dict['incoming_road'] and junction_dict['connecting_road'] in road_ids:
                    junctions.append(junction_dict)

        return junctions

    @staticmethod
    def _get_connection_links(connection, junction_dict, mapping):
        links = []
        for lane_link in connection:
            from_lane = int(lane_link.get('from'))
            key_from = XODRConverter._create_key(junction_dict['incoming_road'], 0, from_lane)
            to_lane = int(lane_link.get('to'))
            key_to = XODRConverter._create_key(junction_dict['connecting_road'], 0, to_lane)

            if key_from in mapping and key_to in mapping:
                links.append([from_lane, to_lane])
        return links

    @staticmethod
    def _create_key(road: int, pos: int, link: int) -> str:
        return f"{road}_{pos}_{link}"

    @staticmethod
    def _create_road_mapping(lane_lets):
        """Fill the mapping with the position in the matrix and the point"""
        # init the mapping and the counter
        mapping = {}
        counter = 0

        # iterate over all entries in the lane-lets list
        for road in lane_lets:
            # iterate over all links
            for link in road['left_ids'] + road['right_ids']:
                mapping[XODRConverter._create_key(road['road_id'], 0, link)] = counter
                mapping[XODRConverter._create_key(road['road_id'], 1, link)] = counter + 1
                counter += 2

        return mapping

    @staticmethod
    def _get_linked_elements(element) -> tuple:
        element_id = int(element.get('elementId'))
        element_type = element.get('elementType')
        element_contact_point = element.get('contactPoint')

        return element_id, element_type, element_contact_point

    @staticmethod
    def _get_root(filepath: Path) -> eTree.Element:
        # check if file exist
        if not exists(filepath) or not isfile(filepath):
            raise FileNotFoundError

        # get the root of the xml file
        return eTree.parse(filepath).getroot()

    @staticmethod
    def _create_road_dict(road):
        # TODO Dataclass
        road_dict = {}
        road_dict['road_id'] = int(road.get('id'))
        road_dict['junction'] = int(road.get('junction'))
        # parse the link tag
        link = road.find('link')
        # parse the successor
        suc = link.find('successor')
        if suc is not None:
            tmp = XODRConverter._get_linked_elements(suc)
            road_dict['suc_id'], road_dict['suc_type'], road_dict['suc_contact_point'] = tmp
        # parse the predecessor
        pre = link.find('predecessor')
        if pre is not None:
            tmp = XODRConverter._get_linked_elements(pre)
            road_dict['pre_id'], road_dict['pre_type'], road_dict['pre_contact_point'] = tmp
        # get the lane ids and the line type
        road_dict['left_ids'], road_dict['right_ids'] = XODRConverter._get_lane_id(road)
        road_dict['line_type'] = XODRConverter._get_line_type(road)
        # parse traffic signs
        road_dict['traffic_signs'] = XODRConverter._get_traffic_signs(road)
        # parse all geometries
        road_dict['geometry'] = XODRConverter._get_geometry(road)

        return road_dict

    @staticmethod
    def _link_roads(lane_lets, mapping, matrix):
        """Link the geometries between each other."""
        for index, road in enumerate(lane_lets):
            # TODO Cost function
            length = 0
            for geometry in road['geometry']:
                length += geometry[3]

            for link in road['left_ids']:
                identifier = mapping[XODRConverter._create_key(road['road_id'], 0, link)]
                matrix[identifier+1][identifier] = length

            for link in road['right_ids']:
                identifier = mapping[XODRConverter._create_key(road['road_id'], 0, link)]
                matrix[identifier][identifier+1] = length
        return matrix

    @staticmethod
    def _road_id2index(road_id: int, mapping: dict) -> Tuple[int, int]:
        """Get the index to the corresponding road id."""
        founded_keys = [v for k, v in mapping.items() if k.startswith(f"{road_id}_")]

        # return the first index and the last index
        return founded_keys[0], founded_keys[-1]

    @staticmethod
    def _junc_ids_entries(junction_id: int, junctions: list) -> List[int]:
        """Get the list of indices to the corresponding junction id."""
        return [index for index, junction in enumerate(junctions)
                if junction['junction_id'] == junction_id]

    @staticmethod
    def _road_connection(road, link: str, mapping: dict, matrix: np.ndarray):
        # link = {'pre', 'suc'}
        if road[link + '_contact_point'] == 'start':
            contact_link = 0
            sign = -1 if link == "pre" else 1
        else:
            contact_link = 1
            sign = 1 if link == "pre" else -1

        contact_road = 0 if link == "pre" else 1

        for lane_link in road['right_ids']:
            # TODO Dataclass
            index_link, index_road = XODRConverter._get_connected_lane_ids(contact_link, contact_road, lane_link, link,
                                                                          mapping, road, sign)
            matrix[index_link][index_road] = 1e-6

        for lane_link in road['left_ids']:
            index_link, index_road = XODRConverter._get_connected_lane_ids(contact_link, contact_road, lane_link, link,
                                                                          mapping, road, sign)
            matrix[index_road][index_link] = 1e-6

    @staticmethod
    def _get_connected_lane_ids(contact_link, contact_road, lane_link, link, mapping, road, sign):
        # TODO Dataclass
        key = XODRConverter._create_key(road[link + '_id'], contact_link, sign * lane_link)
        index_link = mapping[key]
        key = XODRConverter._create_key(road['road_id'], contact_road, lane_link)
        index_road = mapping[key]
        return index_link, index_road

    @staticmethod
    def _junction_connection(road, link: str, junctions, mapping, matrix):
        for i in XODRConverter._junc_ids_entries(road[link+'_id'], junctions):
            if junctions[i]['incoming_road'] != road['road_id']:
                continue

            connecting_road = int(junctions[i]['connecting_road'])
            contact_point = junctions[i]['contact_point']
            incoming_road = int(junctions[i]['incoming_road'])

            contact_incoming = 0 if link == 'pre' else 1
            contact_connecting = 0 if contact_point == 'start' else 1

            for lane_links in junctions[i]['lane_links']:
                key = XODRConverter._create_key(incoming_road, contact_incoming, lane_links[0])
                index_incoming = mapping[key]
                key = XODRConverter._create_key(connecting_road, contact_connecting, lane_links[1])
                index_connecting = mapping[key]

                matrix[index_incoming][index_connecting] = 1e-6

    @staticmethod
    def _link_pre_suc(lane_lets, junctions, mapping, matrix):
        """Link the predecessor and successor in the weighted matrix."""
        links = ['pre', 'suc']
        for road in lane_lets:
            for link in links:
                if link+'_id' not in road.keys():
                    continue
                road_type = road[link+'_type']
                if road_type == 'road':
                    XODRConverter._road_connection(road, link, mapping, matrix)
                elif road_type == 'junction':
                    XODRConverter._junction_connection(road, link, junctions, mapping, matrix)

    @staticmethod
    def _create_links(lane_lets, junctions, mapping):
        """Link geometry, predecessor and successor in the weighted matrix."""
        num_nodes = len(mapping)
        matrix = np.zeros(shape=(num_nodes, num_nodes))
        XODRConverter._link_roads(lane_lets, mapping, matrix)
        XODRConverter._link_pre_suc(lane_lets, junctions, mapping, matrix)

        return matrix
