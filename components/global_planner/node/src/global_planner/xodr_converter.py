import numpy as np
import math
import os.path
import xml.etree.ElementTree as eTree
from global_route_planner import GlobalRoutePlanner


class XODRConverter:
    def __init__(self):
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

    def get_lane_sec(self):
        return self.road.find('lanes').find('laneSection')

    def get_line_type(self):
        lane_sec = self.get_lane_sec()
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    def get_lane_id(self):
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
        objects = []
        if self.road.find('objects') is not None:
            for obj in self.road.find('objects').findall('object'):
                objects.append([obj.get('name'), obj.get('s'), obj.get('t')])
        return objects

    def calculateEndPoint(self, startPoint, angle, length, arc):
        # ToDo Check Implementatiuon
        return (startPoint[0] + math.cos(angle) * length, startPoint[1] + math.sin(angle) * length)

    def get_geometry(self):
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

                end_point = self.calculateEndPoint(start_point, angle, length, curvature)
                objects.append([start_point, angle, length, end_point, curvature])

        return objects

    def get_junctionDic(self, allJunctions):

        for junction in allJunctions:
            id = int(junction.attrib['id'])
            for connection in junction:
                if connection.tag == 'connection':
                    junction_dict = {}

                    junction_dict['junction_id'] = id
                    junction_dict['connection_id'] = int(connection.attrib['id'])
                    junction_dict['incomingRoad'] = int(connection.attrib['incomingRoad'])
                    junction_dict['connectingRoad'] = int(connection.attrib['connectingRoad'])
                    junction_dict['contactPoint'] = connection.attrib['contactPoint']
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
        # check if file exist
        if not os.path.isfile(filepath):
            return FileNotFoundError
        self.filepath = filepath

        # get the root of the xml file
        root = eTree.parse(self.filepath).getroot()

        # parse all roads
        for road in root.findall('road'):
            addDic = False
            self.road = road
            road_dict = dict()
            road_dict['road_id'] = int(self.road.get('id'))
            road_dict['junction'] = int(self.road.get('junction'))

            # parse the successor
            suc = self.road.find('link').find('successor')
            road_dict['suc_id'] = int(suc.get('elementId'))
            road_dict['suc_type'] = suc.get('elementType')
            # road_dict['contact_point'] = suc.get('contactPoint')
            if suc.attrib['elementType'] == "road":
                road_dict['contactPoint_suc'] = suc.attrib['contactPoint']

            # parse the predecessor
            pre = self.road.find('link').find('predecessor')
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
                addDic = True
            road_dict['line_type'] = self.get_line_type()

            # parse objects
            road_dict['traffic_signs'] = self.get_traffic_signs()

            # parse all geometries
            geometry = self.get_geometry()
            road_dict['geometry'] = geometry

            # add new dict to lane_lets
            if addDic:
                self.num_nodes += len(geometry) * 2
                self.lane_lets.append(road_dict)

        # parse all junctions
        self.get_junctionDic(root.findall('junction'))

    def link_geometry(self):
        index = 0
        mapping = []
        for road in self.lane_lets:
            lastgeoID = -1
            geoID = 0
            lastgeo = 999
            # TODO Check
            # TODO Kostenfunktion
            for geometry in road['geometry']:
                if lastgeoID == road['road_id'] and index > 0:
                    # connection from start to end point of the geometry
                    if len(road['left_ids']):
                        self.matrix[index - 1][index] = lastgeo
                    if len(road['right_ids']):
                        self.matrix[index][index - 1] = lastgeo
                lastgeo = geometry[2]
                # connection from endpoint to start point
                # if road_id == lastgeoID --> 1e-4 else geometry[2]
                self.matrix[index][index + 1] = geometry[2]
                self.matrix[index + 1][index] = geometry[2]
                mapping.append((road['road_id'], geoID, geometry[0]))
                mapping.append((road['road_id'], geoID + 1, geometry[3]))
                index += 2
                geoID += 2
                lastgeoID = road['road_id']
        self.mapping = mapping

    def findMaping(self, roadId, first):
        index = 0
        mapping = self.mapping
        if first:
            for map in mapping:
                if map[0] == roadId:
                    return index
                index += 1
            return np.nan
        else:
            found = False
            for map in mapping:
                if map[0] == roadId:
                    index += 1
            rec = self.findMaping(roadId, True)
            if rec == None or rec == -1:
                return np.nan
            else:
                return index + rec - 1

    def findMapingConnectin(self, junction_id, first):
        index = 0
        junctions = self.junctions
        if first:
            for map in junctions:
                if int(map['junction_id']) == int(junction_id):
                    return index
                index += 1
            return -1
        else:
            for map in junctions:
                if int(map['junction_id']) == int(junction_id):
                    index += 1
            rec = self.findMapingConnectin(junction_id, True)
            if rec == None or rec == -1:
                return -1
            else:
                return index + rec - 1

    def linkPre(self):
        for road in self.lane_lets:
            if 'pre_id' in road.keys():

                pre = road['pre_id']
                pre_type = road['pre_type']
                index_pre = -1
                if pre_type == 'road':

                    if road['contactPoint_pre'] == 'start':
                        # Letzter Eintrag Pre
                        index_pre = self.findMaping(pre, True)
                    elif road['contactPoint_pre'] == 'end':
                        # index_id = findMaping(mapping, road['id'], False)
                        index_pre = self.findMaping(pre, False)

                    # Last Eintrag Road
                    index_id = self.findMaping(road['road_id'], True)
                    # TODO Prüfen ob beidseitig notwendig?
                    if index_pre != index_id:
                        self.matrix[index_pre][index_id] = 0.0001
                        self.matrix[index_id][index_pre] = 0.0001

                elif pre_type == 'junction':
                    index_pre_first = self.findMapingConnectin(pre, True)
                    index_pre_last = self.findMapingConnectin(pre, False)
                    for i in range(index_pre_first, index_pre_last + 1):
                        if int(self.junctions[i]['incomingRoad']) == road['road_id']:

                            connectingRoad = int(self.junctions[i]['connectingRoad'])
                            contactPoint = self.junctions[i]['contactPoint']

                            index_id = self.findMaping(road['road_id'], True)
                            if contactPoint == 'start':
                                index_id2 = self.findMaping(connectingRoad, True)
                            else:
                                index_id2 = self.findMaping(connectingRoad, False)
                            # TODO Prüfen ob beidseitig notwendig?
                            if index_id != index_id2:
                                self.matrix[index_id2][index_id] = 0.0001
                                self.matrix[index_id][index_id2] = 0.0001

    def linkSuc(self):
        for road in self.lane_lets:
            if 'suc_id' in road.keys():
                suc = road['suc_id']
                suc_type = road['suc_type']
                index_sucessor = -1
                if suc_type == 'road':
                    if road['contactPoint_suc'] == 'start':
                        # Letzter Eintrag Suc
                        index_sucessor = self.findMaping(suc, True)
                    elif road['contactPoint_suc'] == 'end':
                        # index_id = findMaping(mapping, road['id'], False)
                        index_sucessor = self.findMaping(suc, False)

                    # Last Eintrag Road
                    index_id = self.findMaping(road['road_id'], False)
                    # TODO Prüfen ob beidseitig notwendig?
                    if index_sucessor != index_id:
                        self.matrix[index_sucessor][index_id] = 0.0001
                        self.matrix[index_id][index_sucessor] = 0.0001

                elif suc_type == 'junction':
                    index_sucessor_first = self.findMapingConnectin(suc, True)
                    index_sucessor_last = self.findMapingConnectin(suc, False)
                    for i in range(index_sucessor_first, index_sucessor_last + 1):
                        if int(self.junctions[i]['incomingRoad']) == road['road_id']:
                            connectingRoad = int(self.junctions[i]['connectingRoad'])
                            contactPoint = self.junctions[i]['contactPoint']
                            index_id = self.findMaping(road['road_id'], False)
                            if contactPoint == 'start':
                                index_id2 = self.findMaping(connectingRoad, True)
                            else:
                                index_id2 = self.findMaping(connectingRoad, False)
                            if index_id != index_id2:
                                self.matrix[index_id][index_id2] = 0.0001
                                self.matrix[index_id2][index_id] = 0.0001


if __name__ == "__main__":

    filename = 'Town01.xodr'

    xodr = XODRConverter()
    xodr.read_xodr(filename)

    xodr.matrix = np.zeros(shape=(xodr.num_nodes, xodr.num_nodes))
    print(len(xodr.matrix))
    xodr.link_geometry()
    xodr.linkPre()
    xodr.linkSuc()

    gp = GlobalRoutePlanner(xodr.num_nodes)
    gp.set_matrix(xodr.matrix)
    gp.set_mapping(xodr.mapping)
    gp.compute_route()
    #gp.set_gps()
    #gp.set_targetDestionation()
    gp.find_nearest_road((163.0, 0.004))
