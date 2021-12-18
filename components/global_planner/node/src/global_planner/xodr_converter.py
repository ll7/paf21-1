"""A xodr converter based on xodr files."""
import os.path
import xml.etree.ElementTree as eTree
from pathlib import Path
from typing import List
import numpy as np


class XODRConverter:
    """A xodr converter based on xodr files."""
    def __init__(self):
        """Initialize the xodr converter."""
        self.filepath = None
        self.lane_lets = []
        self.junctions = []
        self.num_nodes = 0
        self.road = None
        # matrix with weighted edges
        self.matrix = None
        # road mapping
        self.mapping = {}
        self.point_dict = {}

    def _get_lane_sec(self):
        """Get the lane section out of the road."""
        return self.road.find('lanes').find('laneSection')

    def _get_line_type(self) -> str:
        """Get the line type out of the road."""
        lane_sec = self._get_lane_sec()
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    def _get_lane_id(self) -> list:
        """Get the lane id."""
        directions = ['left', 'right']
        return_ids = [[], []]

        lane_sec = self._get_lane_sec()
        for i, direction in enumerate(directions):
            if lane_sec.find(direction) is not None:
                for lane in lane_sec.find(direction).findall('lane'):
                    if lane.get('type') == 'driving':
                        return_ids[i].append(int(lane.get('id')))
        return return_ids

    def _get_traffic_signs(self) -> list:
        """Get the traffic signs out of the road."""
        objects = []
        traffic_signs = self.road.find('objects')
        if traffic_signs is not None:
            for obj in traffic_signs.findall('object'):
                objects.append([obj.get('name'), obj.get('s'), obj.get('t')])
        return objects

    @staticmethod
    def calculate_end_point(start_point: np.ndarray,angle: float,
                            length: float, arc: float) -> np.ndarray:
        """Calculate the end point based on the start point, angle, length and arc."""
        # ToDo check implementation and add arc
        # https://www.delftstack.com/howto/numpy/curvature-formula-numpy/
        rotation = np.array([np.cos(angle), np.sin(angle)], dtype=np.float32)
        end_point = start_point + rotation * length
        return end_point

    def _get_geometry(self):
        """Get the geometry out of the road."""
        objects = []
        plan_view = self.road.find('planView')
        if plan_view is not None:
            for geometry in plan_view.findall('geometry'):
                start_point = np.array([geometry.get('x'), geometry.get('y')], dtype=np.float32)
                angle = float(geometry.get('hdg'))
                length = float(geometry.get('length'))

                curvature = 0.0
                if geometry.find('arc') is not None:
                    curvature = float(geometry.find('arc').get('curvature'))

                end_point = self.calculate_end_point(start_point, angle, length, curvature)
                objects.append([start_point, end_point, angle, length, curvature])

        return objects

    def _get_junction_dic(self, all_junctions: list):
        """Get the junction dictionary."""
        list_ids = [road['road_id'] for road in self.lane_lets]

        for junction in all_junctions:
            junc_id = int(junction.attrib['id'])
            for connection in junction.findall('connection'):
                junction_dict = {
                    'junction_id': junc_id,
                    'connection_id': int(connection.get('id')),
                    'incoming_road': int(connection.get('incomingRoad')),
                    'connecting_road': int(connection.get('connectingRoad')),
                    'contact_point': connection.get('contactPoint')
                }
                links = []
                for lane_link in connection:
                    from_lane = int(lane_link.get('from'))
                    key_from = f"{junction_dict['incoming_road']}_0_{from_lane}"
                    to_lane = int(lane_link.get('to'))
                    key_to = f"{junction_dict['connecting_road']}_0_{to_lane}"

                    if key_from in self.mapping and key_to in self.mapping:
                        links.append([from_lane, to_lane])
                if len(links) == 0:
                    continue

                # set the links to the junction dict
                junction_dict['lane_links'] = links
                # add the junction to dict if the roads are driving roads
                if junction_dict['incoming_road'] and junction_dict['connecting_road'] in list_ids:
                    self.junctions.append(junction_dict)

    def _create_road_mapping(self):
        """Fill the mapping with the position in the matrix and the point"""
        # init the mapping and the counter
        self.mapping = {}
        counter = 0

        self.point_dict = {}
        # iterate over all entries in the lane-lets list
        for road in self.lane_lets:
            # iterate over all links
            point_mapping = []
            for link in road['left_ids']:
                if len(road['geometry']) == 0:
                    continue
                else:
                    self.mapping[f"{road['road_id']}_0_{link}"] = counter
                    self.mapping[f"{road['road_id']}_1_{link}"] = counter + 1
                counter += 2
            for link in road['right_ids']:
                if len(road['geometry']) == 0:
                    continue
                else:
                    self.mapping[f"{road['road_id']}_0_{link}"] = counter
                    self.mapping[f"{road['road_id']}_1_{link}"] = counter + 1
                counter += 2
            for geo in road['geometry']:
                point_mapping.append(geo)
            self.point_dict[road['road_id']] = point_mapping

    def get_index(self, road: int, pos: int, link: int):
        """Get the index out of the mapping."""
        key = f"{road}_{pos}_{link}"
        if key in self.mapping:
            return self.mapping[key]
        return AttributeError

    @staticmethod
    def _get_linked_elements(element) -> tuple:
        element_id = int(element.get('elementId'))
        element_type = element.get('elementType')
        element_contact_point = element.get('contactPoint')

        return element_id, element_type, element_contact_point

    def _get_root(self, filepath: Path) -> eTree.Element or FileNotFoundError:
        # check if file exist
        if not os.path.isfile(filepath):
            return FileNotFoundError
        self.filepath = filepath

        # get the root of the xml file
        return eTree.parse(self.filepath).getroot()

    def read_xodr(self, filepath: Path):
        """Read the xodr file from the file path."""
        root = self._get_root(filepath)
        # parse all roads
        for road in root.findall('road'):
            self.road = road
            road_dict = {}
            road_dict['road_id'] = int(self.road.get('id'))
            road_dict['junction'] = int(self.road.get('junction'))

            # parse the link tag
            link = self.road.find('link')
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
            road_dict['left_ids'], road_dict['right_ids'] = self._get_lane_id()
            road_dict['line_type'] = self._get_line_type()

            # parse traffic signs
            road_dict['traffic_signs'] = self._get_traffic_signs()

            # parse all geometries
            road_dict['geometry'] = self._get_geometry()

            # add new dict to lane_lets
            if len(road_dict['left_ids']) + len(road_dict['right_ids']):
                # the start and end point are different nodes
                self.lane_lets.append(road_dict)

        # create the road mapping
        self._create_road_mapping()

        # parse all junctions
        self._get_junction_dic(root.findall('junction'))
        return None

    def link_roads(self):
        """Link the geometries between each other."""
        for index, road in enumerate(self.lane_lets):
            # TODO Cost function
            length = 0
            for geometry in road['geometry']:
                length += geometry[3]

            for link in road['left_ids']:
                identifier = self.get_index(road['road_id'], 0, link)
                self.matrix[identifier+1][identifier] = length

            for link in road['right_ids']:
                identifier = self.get_index(road['road_id'], 0, link)
                self.matrix[identifier][identifier+1] = length

    def road_id2index(self, road_id: int, first: bool) -> int or AttributeError:
        """Get the index to the corresponding road id."""
        if first:
            try:
                return self.mapping[f"{road_id}_0"][0]
            except KeyError:
                return AttributeError
        else:
            founded_keys = [v[0] for k, v in self.mapping.items() if k.startswith(f"{road_id}_")]
            # if no element in the list
            if not founded_keys:
                return AttributeError

            return founded_keys[-1]

    def junc_ids_entries(self, junction_id: int) -> List[int]:
        """Get the list of indices to the corresponding junction id"""
        founded_junc = []
        for index, junction in enumerate(self.junctions):
            if junction['junction_id'] == int(junction_id):
                founded_junc.append(index)

        return founded_junc

    def _road_connection(self, road, link: str):
        # link = {'pre', 'suc'}
        if road[link + '_contact_point'] == 'start':
            contact_link = 0
            sign = -1 if link == "pre" else 1
        else:
            contact_link = 1
            sign = 1 if link == "pre" else -1

        contact_road = 0 if link == "pre" else 1

        for lane_link in road['right_ids']:
            index_link = self.get_index(road[link + '_id'], contact_link, sign * lane_link)
            index_road = self.get_index(road['road_id'], contact_road, lane_link)

            self.matrix[index_link][index_road] = 1e-6

        for lane_link in road['left_ids']:
            index_link = self.get_index(road[link + '_id'], contact_link, sign * lane_link)
            index_road = self.get_index(road['road_id'], contact_road, lane_link)

            self.matrix[index_road][index_link] = 1e-6

    def _junction_connection(self, road, link: str):
        for i in self.junc_ids_entries(road[link+'_id']):
            if self.junctions[i]['incoming_road'] != road['road_id']:
                continue

            connecting_road = int(self.junctions[i]['connecting_road'])
            contact_point = self.junctions[i]['contact_point']
            incoming_road = int(self.junctions[i]['incoming_road'])

            contact_incoming = 0 if link == 'pre' else 1
            contact_connecting = 0 if contact_point == 'start' else 1

            for lane_links in self.junctions[i]['lane_links']:
                index_incoming = self.get_index(incoming_road, contact_incoming, lane_links[0])
                index_connecting = self.get_index(connecting_road, contact_connecting,
                                                  lane_links[1])

                self.matrix[index_incoming][index_connecting] = 1e-6

    def link_pre_suc(self):
        """Link the predecessor and successor in the weighted matrix."""
        links = ['pre', 'suc']
        for road in self.lane_lets:
            for link in links:
                if link+'_id' not in road.keys():
                    continue
                road_type = road[link+'_type']
                if road_type == 'road':
                    self._road_connection(road, link)
                elif road_type == 'junction':
                    self._junction_connection(road, link)

    def create_links(self):
        """Link geometry, predecessor and successor in the weighted matrix."""
        self.num_nodes = len(self.mapping)
        self.matrix = np.zeros(shape=(self.num_nodes, self.num_nodes))
        self.link_roads()
        self.link_pre_suc()


# if __name__ == "__main__":
#     filename = Path("./../../../xodr/Town01.xodr")
#     xodr = XODRConverter()
#     xodr.read_xodr(filename)
#     xodr.create_links()
#
#     gp = GlobalRoutePlanner()
#     gp.set_matrix(xodr.matrix)
#     gp.point_dict = xodr.point_dict
#     gp.set_mapping(xodr.mapping)
#     gp.road_dict = xodr.lane_lets
#     gp.compute_route()
