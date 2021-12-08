import math
import os.path
import numpy as np
import xml.etree.ElementTree as eTree
from global_route_planner import GlobalRoutePlanner


class XODRConverter:
    """A xodr converter based on xodr files."""
    def __init__(self):
        """Initialize the xodr converter."""
        self.filepath = ''
        self.lane_lets = []
        self.connections = []
        self.num_nodes = 0
        self.road = None
        self.junctions = []
        # matrix
        self.matrix = np.zeros(shape=(1, 1))
        # Mapping
        self.mapping = []

    # def add_node(self, node_id: str):
    #     if len(self.nodes) < self.num_nodes:
    #         # create new entry in dict
    #         self.nodes[node_id] = self.matrix_pos
    #         # add the matrix position up
    #         self.matrix_pos += 1
    #     else:
    #         print('Reached maximum of nodes!')
    #
    # def add_edge(self, start_id, target_id, weight):
    #     # check if start_id and target_id ar in the dict
    #     if start_id not in self.nodes:
    #         self.add_node(start_id)
    #     if target_id not in self.nodes:
    #         self.add_node(target_id)
    #
    #     start_pos = self.get_pos(start_id)
    #     target_pos = self.get_pos(target_id)
    #
    #     # set the weight in the graph
    #     self.graph[start_pos, target_pos] = weight
    #     # self.graph[target_pos, start_pos] = -1.0

    def get_lane_sec(self):
        """Get the lane section out of the road."""
        return self.road.find('lanes').find('laneSection')

    def get_line_type(self):
        """Get the line type out of the road."""
        lane_sec = self.get_lane_sec()
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    def get_lane_id(self):
        """Get the lane id."""
        directions = ['left', 'right']
        return_ids = [[], []]

        lane_sec = self.get_lane_sec()
        for i, direction in enumerate(directions):
            if lane_sec.find(direction) is not None:
                for lane in lane_sec.find(direction).findall('lane'):
                    if lane.get('type') == 'driving':
                        return_ids[i].append(lane.get('id'))
        return return_ids

    def get_traffic_signs(self):
        """Get the traffic signs out of the road."""
        objects = []
        if self.road.find('objects') is not None:
            for obj in self.road.find('objects').findall('object'):
                objects.append([obj.get('name'), obj.get('s'), obj.get('t')])
        return objects

    @staticmethod
    def calculate_end_point(start_point, angle, length, arc):
        """Calculate the end point based on the start point, angle, length and arc."""
        # ToDo check implementation and add arc
        return start_point[0] + math.cos(angle) * length, start_point[1] + math.sin(angle) * length

    def get_geometry(self):
        """Get the geometry out of the road."""
        objects = []
        if self.road.find('planView') is not None:
            for geometry in self.road.find('planView').findall('geometry'):
                start_point = [float(geometry.attrib['x']), float(geometry.attrib['y'])]
                angle = float(geometry.attrib['hdg'])
                length = float(geometry.attrib['length'])
                arc = geometry.find('arc')
                curvature = 0.0
                if arc is not None:
                    curvature = float(arc.get('curvature'))

                end_point = self.calculate_end_point(start_point, angle, length, curvature)
                objects.append([start_point, angle, length, end_point, curvature])

        return objects

    def get_junction_dic(self, all_junctions):
        """Get the junction dictionary."""
        for junction in all_junctions:
            junc_id = int(junction.attrib['id'])
            for connection in junction:
                if connection.tag == 'connection':
                    junction_dict = {
                        'junction_id': junc_id,
                        'connection_id': int(connection.attrib['id']),
                        'incomingRoad': int(connection.attrib['incomingRoad']),
                        'connectingRoad': int(connection.attrib['connectingRoad']),
                        'contactPoint': connection.attrib['contactPoint']
                    }
                    for lane_link in connection:
                        if 'lane_links' in junction_dict.keys():
                            tmp = junction_dict['lane_links']
                            tmp.append([int(lane_link.attrib['from']), int(lane_link.attrib['to'])])
                            junction_dict['lane_links'] = tmp
                        else:
                            junction_dict['lane_links'] = [
                                [int(lane_link.attrib['from']), int(lane_link.attrib['to'])]]

                    if junction_dict['incomingRoad'] and junction_dict['connectingRoad'] in [lane['road_id'] for
                                                                                             lane in
                                                                                             self.lane_lets]:
                        self.junctions.append(junction_dict)

        return self.junctions

    def read_xodr(self, filepath):
        """Read the xodr file from the file path."""
        # check if file exist
        if not os.path.isfile(filepath):
            return FileNotFoundError
        self.filepath = filepath

        # get the root of the xml file
        root = eTree.parse(self.filepath).getroot()

        # parse all roads
        for road in root.findall('road'):
            add_dic = False
            self.road = road
            road_dict = dict()
            road_dict['road_id'] = int(self.road.get('id'))
            road_dict['junction'] = int(self.road.get('junction'))

            # parse the successor
            suc = self.road.find('link').find('successor')
            if suc is not None:
                # ToDo: AttributeError: 'NoneType' object has no attribute 'get'
                road_dict['suc_id'] = int(suc.get('elementId'))
                road_dict['suc_type'] = suc.get('elementType')
                # road_dict['contact_point'] = suc.get('contactPoint')
                if suc.attrib['elementType'] == "road":
                    road_dict['contactPoint_suc'] = suc.attrib['contactPoint']

            # parse the predecessor
            pre = self.road.find('link').find('predecessor')
            if pre is not None:
                road_dict['pre_id'] = int(pre.get('elementId'))
                road_dict['pre_type'] = pre.get('elementType')
                # road_dict['contact_point'] = pre.get('contactPoint')
                if pre.attrib['elementType'] == "road":
                    road_dict['contactPoint_pre'] = pre.attrib['contactPoint']

            # get the lane ids and the line type
            ids = self.get_lane_id()
            road_dict['left_ids'] = ids[0]
            road_dict['right_ids'] = ids[1]
            if len(ids[0]) + len(ids[1]):
                add_dic = True
            road_dict['line_type'] = self.get_line_type()

            # parse objects
            road_dict['traffic_signs'] = self.get_traffic_signs()

            # parse all geometries
            geometry = self.get_geometry()
            road_dict['geometry'] = geometry

            # add new dict to lane_lets
            if add_dic:
                self.num_nodes += len(geometry) * 2
                self.lane_lets.append(road_dict)

        # parse all junctions
        self.get_junction_dic(root.findall('junction'))

    def link_geometry(self):
        """Link the geometries between each other."""
        index = 0
        mapping = []
        for road in self.lane_lets:
            last_geo_id = -1
            geo_id = 0
            last_geo = 999
            # TODO Check
            # TODO Cost function
            for geometry in road['geometry']:
                if last_geo_id == road['road_id'] and index > 0:
                    # connection from start to end point of the geometry
                    if len(road['left_ids']):
                        self.matrix[index - 1][index] = last_geo
                    if len(road['right_ids']):
                        self.matrix[index][index - 1] = last_geo
                last_geo = geometry[2]
                # connection from endpoint to start point
                # if road_id == last_geo_id --> 1e-4 else geometry[2]
                self.matrix[index][index + 1] = geometry[2]
                self.matrix[index + 1][index] = geometry[2]
                mapping.append((road['road_id'], geo_id, geometry[0]))
                mapping.append((road['road_id'], geo_id + 1, geometry[3]))
                index += 2
                geo_id += 2
                last_geo_id = road['road_id']
        self.mapping = mapping

    def find_mapping(self, road_id, first):
        index = 0
        mapping = self.mapping
        if first:
            for m in mapping:
                if m[0] == road_id:
                    return index
                index += 1
            return np.nan
        else:
            for m in mapping:
                if m[0] == road_id:
                    index += 1
            rec = self.find_mapping(road_id, True)
            if rec is None or rec == -1:
                return np.nan
            else:
                return index + rec - 1

    def find_mapping_connection(self, junction_id, first):
        index = 0
        junctions = self.junctions
        if first:
            for m in junctions:
                if int(m['junction_id']) == int(junction_id):
                    return index
                index += 1
            return -1
        else:
            for m in junctions:
                if int(m['junction_id']) == int(junction_id):
                    index += 1
            rec = self.find_mapping_connection(junction_id, True)
            if rec is None or rec == -1:
                return -1
            else:
                return index + rec - 1

    def link_pre(self):
        for road in self.lane_lets:
            if 'pre_id' in road.keys():

                pre = road['pre_id']
                pre_type = road['pre_type']
                index_pre = -1
                if pre_type == 'road':

                    if road['contactPoint_pre'] == 'start':
                        # last entry pre
                        index_pre = self.find_mapping(pre, True)
                    elif road['contactPoint_pre'] == 'end':
                        index_pre = self.find_mapping(pre, False)

                    # last entry road
                    index_id = self.find_mapping(road['road_id'], True)
                    # TODO check if both sides are necessary
                    if index_pre != index_id:
                        self.matrix[index_pre][index_id] = 0.0001
                        self.matrix[index_id][index_pre] = 0.0001

                elif pre_type == 'junction':
                    index_pre_first = self.find_mapping_connection(pre, True)
                    index_pre_last = self.find_mapping_connection(pre, False)
                    for i in range(index_pre_first, index_pre_last + 1):
                        if int(self.junctions[i]['incomingRoad']) == road['road_id']:

                            connecting_road = int(self.junctions[i]['connectingRoad'])
                            contact_point = self.junctions[i]['contactPoint']

                            index_id = self.find_mapping(road['road_id'], True)
                            if contact_point == 'start':
                                index_id2 = self.find_mapping(connecting_road, True)
                            else:
                                index_id2 = self.find_mapping(connecting_road, False)
                            # TODO check if both sides are necessary
                            if index_id != index_id2:
                                self.matrix[index_id2][index_id] = 0.0001
                                self.matrix[index_id][index_id2] = 0.0001

    def link_suc(self):
        for road in self.lane_lets:
            if 'suc_id' in road.keys():
                suc = road['suc_id']
                suc_type = road['suc_type']
                index_successor = -1
                if suc_type == 'road':
                    if road['contactPoint_suc'] == 'start':
                        # last entry suc
                        index_successor = self.find_mapping(suc, True)
                    elif road['contactPoint_suc'] == 'end':
                        index_successor = self.find_mapping(suc, False)

                    # last entry road
                    index_id = self.find_mapping(road['road_id'], False)
                    # TODO check if both sides are necessary
                    if index_successor != index_id:
                        self.matrix[index_successor][index_id] = 0.0001
                        self.matrix[index_id][index_successor] = 0.0001

                elif suc_type == 'junction':
                    index_successor_first = self.find_mapping_connection(suc, True)
                    index_successor_last = self.find_mapping_connection(suc, False)
                    for i in range(index_successor_first, index_successor_last + 1):
                        if int(self.junctions[i]['incomingRoad']) == road['road_id']:
                            connecting_road = int(self.junctions[i]['connectingRoad'])
                            contact_point = self.junctions[i]['contactPoint']
                            index_id = self.find_mapping(road['road_id'], False)
                            if contact_point == 'start':
                                index_id2 = self.find_mapping(connecting_road, True)
                            else:
                                index_id2 = self.find_mapping(connecting_road, False)
                            if index_id != index_id2:
                                self.matrix[index_id][index_id2] = 0.0001
                                self.matrix[index_id2][index_id] = 0.0001


if __name__ == "__main__":
    from pathlib import Path

    filename = Path("./../../../xodr/Town01.xodr")

    xodr = XODRConverter()
    xodr.read_xodr(filename)

    xodr.matrix = np.zeros(shape=(xodr.num_nodes, xodr.num_nodes))
    print(len(xodr.matrix))
    xodr.link_geometry()
    xodr.link_pre()
    xodr.link_suc()

    gp = GlobalRoutePlanner(xodr.num_nodes)
    gp.set_matrix(xodr.matrix)
    gp.set_mapping(xodr.mapping)
    gp.compute_route()
    # gp.set_gps()
