import os.path
import numpy as np
import networkx as nx
import xml.etree.ElementTree as eTree
import math

class GlobalPlanner:
    def __init__(self, num_nodes):
        self.num_nodes = num_nodes
        # dict with lane-let ids and matrix pos
        self.nodes = {}
        # counter for number of nodes
        self.matrix_pos = 0
        # graph with
        self.graph = np.zeros(shape=(num_nodes, num_nodes))
        # initialize all distances with inf.
        self.dist = np.ones(shape=(num_nodes,)) * np.inf
        # array for the parents to store shortest path tree
        self.parent = np.ones(shape=(num_nodes,)) * (-1)
        # path
        self.path = []

    def add_node(self, node_id: str):
        if len(self.nodes) < self.num_nodes:
            # create new entry in dict
            self.nodes[node_id] = self.matrix_pos
            # add the matrix position up
            self.matrix_pos += 1
        else:
            print('Reached maximum of nodes!')

    def get_pos(self, node_id):
        return self.nodes[node_id] if node_id in self.nodes else -1

    def add_edge(self, start_id, target_id, weight):
        # check if start_id and target_id ar in the dict
        if start_id not in self.nodes:
            self.add_node(start_id)
        if target_id not in self.nodes:
            self.add_node(target_id)

        start_pos = self.get_pos(start_id)
        target_pos = self.get_pos(target_id)

        # set the weight in the graph
        self.graph[start_pos, target_pos] = weight
        # self.graph[target_pos, start_pos] = -1.0

    def min_distance(self, queue):
        # initialize min value and min_index as -1
        minimum = np.inf
        min_index = 0

        # from the dist array, pick one which has min value and is till in queue
        for index in range(self.num_nodes):
            if self.dist[index] < minimum and index in queue:
                # set the new minimum
                minimum = self.dist[index]
                # set new index
                min_index = index
        # return the index
        return min_index

    def dijkstra(self, start_id: str):
        # distance of source to itself is 0
        start_pos = self.get_pos(start_id)
        self.dist[start_pos] = 0.0

        # add all nodes_id in queue
        queue = list(range(self.num_nodes))

        # find the shortest path for all nodes
        while queue:
            # pick the minimum dist node from the set of nodes
            index_min = self.min_distance(queue)
            # remove min element
            queue.remove(index_min)

            # update dist value and parent
            for num in range(self.num_nodes):
                # update dist[i] if it is in queue, there is an edge from index_min to i,
                if self.graph[index_min][num] and num in queue:
                    new_dist = self.dist[index_min] + self.graph[index_min][num]
                    if new_dist < self.dist[num]:
                        self.dist[num] = new_dist
                        self.parent[num] = index_min
        # return self.dist, self.parent

    def print_path(self, pos):
        # Base Case : If j is source
        if self.parent[pos] == -1:
            self.path.append(pos)
            return pos
        self.print_path(self.parent[pos])

        self.path.append(pos)
        return pos

    def print_solution(self, target_id):
        for num in range(1, self.num_nodes):
            if num == self.get_pos(target_id):
                self.print_path(num)
        return self.path

    def get_path_ids(self):
        key_list = list(self.nodes.keys())
        return [key_list[p] for p in self.path]

    def show_graph_with_labels(self):
        edges = np.where(self.graph > 0)
        gr = nx.Graph()
        gr.add_edges_from(edges)
        nx.draw(gr, node_size=500, labels=list(self.nodes.keys()), with_labels=True)


class XODRConverter:
    def __init__(self):
        self.filepath = ''
        self.lane_lets = []
        self.connections = []
        self.num_nodes = 0
        self.road = None
        self.junctions = []

    def get_lane_sec(self):
        return self.road.find('lanes').find('laneSection')

    def get_line_type(self):
        lane_sec = self.get_lane_sec()
        return lane_sec.find('center').find('lane').find('roadMark').get('type')

    def get_lane_id(self):
        directions = ['left', 'right']
        return_ids = [[],[]]

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

    def calculateEndPoint(self, startPoint, angle, length):
        return (startPoint[0] + math.cos(angle) * length, startPoint[1] + math.sin(angle) * length)

    def get_geometry(self):
        objects = []
        if self.road.find('planView') is not None:
            for obj in self.road.find('planView').findall('geometry'):
                start_point = [float(obj.attrib['x']), float(obj.attrib['y'])]
                angle = float(obj.attrib['hdg'])
                length = float(obj.attrib['length'])
                end_point = self.calculateEndPoint(start_point, angle, length)
                objects.append([start_point, angle, length, end_point])

        return objects

    def get_junctionDic(self, allJunctions):

        for junction in allJunctions:
            for connection in junction:
                if connection.tag == 'connection':
                    junction_dict = {}
                    junction_dict['junction_id'] = id
                    junction_dict['connection_id'] = int(connection.attrib['id'])
                    junction_dict['incomingRoad'] = int(connection.attrib['incomingRoad'])
                    junction_dict['connectingRoad'] = int(connection.attrib['connectingRoad'])
                    junction_dict['contactPoint'] = connection.attrib['contactPoint']
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
            self.road = road
            road_dict = dict()
            road_dict['road_id'] = int(self.road.get('id'))
            road_dict['junction'] = int(self.road.get('junction'))

            # parse the successor
            suc = self.road.find('link').find('successor')
            road_dict['suc_id'] = int(suc.get('elementId'))
            road_dict['suc_type'] = suc.get('elementType')
            #road_dict['contact_point'] = suc.get('contactPoint')
            if suc.attrib['elementType'] == "road":
                road_dict['contactPoint_suc'] = suc.attrib['contactPoint']

            # parse the predecessor
            pre = self.road.find('link').find('predecessor')
            road_dict['pre_id'] = int(pre.get('elementId'))
            road_dict['pre_type'] = pre.get('elementType')
            #road_dict['contact_point'] = pre.get('contactPoint')
            if pre.attrib['elementType'] == "road":
                road_dict['contactPoint_pre'] = pre.attrib['contactPoint']

            # get the lane ids and the line type
            ids = self.get_lane_id()
            road_dict['left_ids'] = ids[0]
            road_dict['right_ids'] = ids[1]
            road_dict['line_type'] = self.get_line_type()

            # parse objects
            road_dict['traffic_signs'] = self.get_traffic_signs()

            # parse all geometries
            geometry = self.get_geometry()
            road_dict['geometry'] = geometry
            self.num_nodes += len(geometry)*2
            #print(road_dict)
            # add new dict to lane_lets
            self.lane_lets.append(road_dict)

        # parse all junctions
        self.get_junctionDic(root.findall('junction'))
        print(self.junctions)


if __name__ == "__main__":
    filename = 'Town01.xodr'

    xodr = XODRConverter()
    xodr.read_xodr(filename)
